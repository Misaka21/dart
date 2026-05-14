#include <chrono>
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

void start_hardware_node() {
    if (debug::get_session_path().empty()) {
        debug::init_session();
    }

    debug::print(debug::PrintMode::INFO, "HardwareNode", "Starting hardware node...");
    debug::print(debug::PrintMode::INFO, "HardwareNode", "Session: {}", debug::get_session_path());

    try {
        auto config = static_param::parse_file("hardware.toml");

        bool use_fake_serial = static_param::get_param<bool>(config, "Serial", "use_fake_serial_data");

        serial::SerialReceiveData fake_data;
        if (use_fake_serial) {
            fake_data.should_detect =
                static_param::get_param<bool>(config, "Serial.fake_data", "should_detect");
            fake_data.dart_number = static_cast<uint8_t>(
                static_param::get_param<int64_t>(config, "Serial.fake_data", "dart_number"));
        }

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

        serial::SerialReceiveData latest_serial;
        bool has_serial = false;

        debug::print(debug::PrintMode::INFO, "HardwareNode", "Hardware node started");

        stats::FpsStats stats("HardwareNode", "synced");

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

                    // 可视化：伪造串口收发数据
                    auto vision_transmit = umt::BasicObjManager<serial::VisionData_t>::find("vision_transmit");
                    auto tx_debug = umt::BasicObjManager<std::string>::find_or_create(
                        "serial_tx_debug", std::string{});
                    auto rx_debug = umt::BasicObjManager<std::string>::find_or_create(
                        "serial_rx_debug", std::string{});
                    int yaw = vision_transmit ? vision_transmit->load().yaw_offset_px : 0;
                    tx_debug->store(fmt::format(
                        "TX: yaw={:d}  raw=[FAKE]", yaw));
                    rx_debug->store(fmt::format(
                        "RX: detect={:d}  dart={:d}  raw=[FAKE]",
                        fake_data.should_detect ? 1 : 0,
                        static_cast<int>(fake_data.dart_number)));
                } else {
                    auto messages = serial_subscriber.drain();
                    if (!messages.empty()) {
                        latest_serial = messages.back();
                        has_serial = true;
                    }
                    if (has_serial) {
                        frame.serial_data = latest_serial;
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
