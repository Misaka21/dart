#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <iterator>
#include <optional>
#include <thread>
#include <type_traits>
#include <vector>

#include <fmt/format.h>
#include <opencv2/core/mat.hpp>

#include "hardware_node.hpp"
#include "hik_cam/hik_camera.hpp"
#include "plugin/debug/logger.hpp"
#include "plugin/param/static_config.hpp"
#include "plugin/stats/fps_stats.hpp"
#include "plugin/watchdog/watchdog_node.hpp"
#include "serial/serial_thread.hpp"
#include "umt/umt.hpp"

namespace hardware {

using namespace std::chrono_literals;
using SteadyClock = std::chrono::steady_clock;

struct TimestampedSerialData {
    int64_t timestamp_us;
    serial::SerialReceiveData data;
};

camera::CameraConfig load_camera_config(const toml::table& config) {
    camera::CameraConfig cam_config;

    cam_config.use_camera_sn = static_param::get_param<bool>(config, "Camera", "use_camera_sn");
    cam_config.camera_sn = static_param::get_param<std::string>(config, "Camera", "camera_sn");
    cam_config.use_mfs_config = static_param::get_param<bool>(config, "Camera", "use_config_from_file");

    std::string mfs_filename = static_param::get_param<std::string>(config, "Camera", "config_file_path");
    cam_config.mfs_config_path = std::string(CONFIG_DIR) + "/" + mfs_filename;

    cam_config.use_runtime_config = static_param::get_param<bool>(config, "Camera", "use_camera_config");

    auto param_table = static_param::get_param_table(config, "Camera.config");
    for (const auto& [key, value] : param_table) {
        std::visit([&](const auto& v) {
            using T = std::decay_t<decltype(v)>;
            if constexpr (!std::is_same_v<T, std::vector<int64_t>>) {
                cam_config.runtime_params.emplace_back(key, camera::CameraParam(v));
            }
        }, value);
    }

    return cam_config;
}

void drain_subscriber_to_buffer(umt::Subscriber<serial::SerialReceiveData>& subscriber,
                                std::deque<TimestampedSerialData>& buffer,
                                size_t max_buffer_size) {
    auto messages = subscriber.drain();
    for (auto& data : messages) {
        TimestampedSerialData ts_data;
        ts_data.data = data;
        ts_data.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
            SteadyClock::now().time_since_epoch()
        ).count();

        buffer.push_back(ts_data);
        while (buffer.size() > max_buffer_size) {
            buffer.pop_front();
        }
    }
}

std::optional<serial::SerialReceiveData> find_nearest_serial_data(
    const std::deque<TimestampedSerialData>& buffer,
    int64_t target_time_us,
    int64_t max_diff_us = 50000) {

    if (buffer.empty()) return std::nullopt;

    auto it_after = std::lower_bound(
        buffer.begin(), buffer.end(), target_time_us,
        [](const TimestampedSerialData& d, int64_t t) {
            return d.timestamp_us < t;
        }
    );

    const TimestampedSerialData* nearest_ptr = nullptr;

    if (it_after == buffer.begin()) {
        nearest_ptr = &buffer.front();
    } else if (it_after == buffer.end()) {
        nearest_ptr = &buffer.back();
    } else {
        const auto& before = *std::prev(it_after);
        const auto& after = *it_after;
        nearest_ptr = std::abs(before.timestamp_us - target_time_us) <=
                      std::abs(after.timestamp_us - target_time_us)
                          ? &before
                          : &after;
    }

    if (!nearest_ptr || std::abs(nearest_ptr->timestamp_us - target_time_us) > max_diff_us) {
        return std::nullopt;
    }

    return nearest_ptr->data;
}

void start_hardware_node() {
    if (debug::get_session_path().empty()) {
        debug::init_session();
    }

    debug::print(debug::PrintMode::INFO, "HardwareNode", "Starting hardware node...");
    debug::print(debug::PrintMode::INFO, "HardwareNode", "Session: {}", debug::get_session_path());

    try {
        auto config = static_param::parse_file("hardware.toml");

        int64_t delta_t_us = static_param::get_param<int64_t>(config, "TimeSync", "delta_t_us");

        bool use_fake_serial = static_param::get_param<bool>(config, "Serial", "use_fake_serial_data");

        serial::SerialReceiveData fake_data;
        if (use_fake_serial) {
            fake_data.should_detect =
                static_param::get_param<bool>(config, "Serial.fake_data", "should_detect");
            fake_data.dart_number = static_cast<uint8_t>(
                static_param::get_param<int64_t>(config, "Serial.fake_data", "dart_number"));
        }

        debug::print(debug::PrintMode::INFO, "HardwareNode", "Delta_t: {} us", delta_t_us);
        debug::print(debug::PrintMode::INFO, "HardwareNode", "Use fake serial: {}", use_fake_serial);

        if (!use_fake_serial) {
            serial::start_serial_communication();
            std::this_thread::sleep_for(100ms);
        } else {
            debug::print(debug::PrintMode::WARNING, "HardwareNode",
                "Using fake serial: should_detect={}, dart_number={}",
                fake_data.should_detect, fake_data.dart_number);
        }

        camera::CameraConfig cam_config = load_camera_config(config);
        camera::HikCam cam(cam_config);
        cam.open();

        umt::Publisher<SyncFrame> pub("sync_frame");
        umt::Subscriber<serial::SerialReceiveData> serial_subscriber("serial_receive", 300);
        auto current_should_detect = umt::BasicObjManager<bool>::find_or_create("current_should_detect", false);
        auto hardware_running = umt::BasicObjManager<bool>::find_or_create("hardware_running", false);
        auto app_running = umt::BasicObjManager<bool>::find_or_create("app_running", true);

        debug::print(debug::PrintMode::INFO, "HardwareNode", "Hardware node started");

        std::deque<TimestampedSerialData> serial_buffer;
        constexpr size_t max_buffer_size = 200;

        stats::FpsStats stats("HardwareNode", "synced");
        stats.set_extra_info([&serial_buffer]() {
            return fmt::format("buf: {}", serial_buffer.size());
        });

        int consecutive_errors = 0;
        const int max_consecutive_errors = 3;

        while (app_running->get()) {
            watchdog::heartbeat("hardware");
            try {
                cv::Mat& img = cam.capture();
                if (img.empty()) continue;

                consecutive_errors = 0;

                int64_t cam_time_us = std::chrono::duration_cast<std::chrono::microseconds>(
                    SteadyClock::now().time_since_epoch()
                ).count();

                SyncFrame frame;
                frame.image = img.clone();
                frame.frame_id = cam.frame_id;
                frame.timestamp_us = cam_time_us;

                bool synced = false;
                if (use_fake_serial) {
                    frame.serial_data = fake_data;
                    frame.serial_valid = true;
                    synced = true;
                } else {
                    drain_subscriber_to_buffer(serial_subscriber, serial_buffer, max_buffer_size);

                    int64_t target_time_us = cam_time_us - delta_t_us;
                    if (auto data = find_nearest_serial_data(serial_buffer, target_time_us)) {
                        frame.serial_data = *data;
                        frame.serial_valid = true;
                        synced = true;
                    }
                }

                if (frame.serial_valid) {
                    current_should_detect->store(frame.serial_data.should_detect);
                }

                pub.push(frame);

                if (frame.serial_valid && !hardware_running->get()) {
                    hardware_running->get() = true;
                    debug::print(debug::PrintMode::INFO, "HardwareNode", "Serial synced, hardware ready");
                }

                if (frame.serial_valid) {
                    watchdog::heartbeat_data("hardware");
                }

                stats.update(0, synced);
            } catch (const std::exception& e) {
                consecutive_errors++;
                debug::print(debug::PrintMode::ERROR, "HardwareNode",
                    "Loop error ({}/{}): {}", consecutive_errors, max_consecutive_errors, e.what());

                if (consecutive_errors >= max_consecutive_errors) {
                    debug::print(debug::PrintMode::FATAL, "HardwareNode",
                        "Too many consecutive errors, camera disconnected?");
                    std::exit(1);
                }
                std::this_thread::sleep_for(100ms);
            }
        }

        debug::print(debug::PrintMode::INFO, "HardwareNode", "Hardware node stopped");
    } catch (const std::exception& e) {
        debug::print(debug::PrintMode::FATAL, "HardwareNode", "Init failed: {}", e.what());
        std::exit(1);
    }
}

} // namespace hardware
