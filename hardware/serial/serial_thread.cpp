#include <chrono>
#include <cmath>
#include <thread>

#include "crc16.hpp"
#include "plugin/debug/logger.hpp"
#include "plugin/param/static_config.hpp"
#include "plugin/rerun/rmcv_rerun.hpp"
#include "plugin/stats/fps_stats.hpp"
#include "protocol/uart_protocol.hpp"
#ifdef HAVE_LIBUSB_1_0
#include "protocol/usb_bulk_protocol.hpp"
#endif
#include "serial_thread.hpp"
#include "umt/umt.hpp"

namespace serial {

namespace umt = ::umt;
using namespace std::chrono_literals;

namespace {

constexpr float kDegToRad = static_cast<float>(M_PI / 180.0);

std::string reconnect_attempts_to_string(int64_t max_reconnect) {
    return max_reconnect < 0 ? "inf" : std::to_string(max_reconnect);
}

std::shared_ptr<ProtocolInterface> create_uart_protocol(const toml::table& config) {
    std::string port_name = static_param::get_param<std::string>(config, "Serial.uart", "port_name");
    int64_t baudrate = static_param::get_param<int64_t>(config, "Serial.uart", "baudrate");

    debug::print(debug::PrintMode::INFO, "SerialManager",
        "Creating UART: {} @ {}", port_name, baudrate);
    return std::make_shared<UartProtocol>(port_name, static_cast<int>(baudrate));
}

std::shared_ptr<ProtocolInterface> create_usb_bulk_protocol(const toml::table& config) {
#ifdef HAVE_LIBUSB_1_0
    std::string vendor_id = static_param::get_param<std::string>(config, "Serial.usb_bulk", "vendor_id");
    std::string product_id = static_param::get_param<std::string>(config, "Serial.usb_bulk", "product_id");
    std::string serial_number = static_param::get_param<std::string>(config, "Serial.usb_bulk", "serial_number");
    int64_t interface_number = static_param::get_param<int64_t>(config, "Serial.usb_bulk", "interface_number");
    std::string bulk_in = static_param::get_param<std::string>(config, "Serial.usb_bulk", "bulk_in_endpoint");
    std::string bulk_out = static_param::get_param<std::string>(config, "Serial.usb_bulk", "bulk_out_endpoint");
    int64_t timeout_ms = static_param::get_param<int64_t>(config, "Serial.usb_bulk", "timeout_ms");

    debug::print(debug::PrintMode::INFO, "SerialManager",
        "Creating USB Bulk: VID={} PID={}",
        vendor_id, product_id.empty() ? "(any)" : product_id);

    return UsbBulkProtocol::create_from_config(
        vendor_id,
        product_id,
        serial_number,
        static_cast<int>(interface_number),
        bulk_in,
        bulk_out,
        static_cast<int>(timeout_ms)
    );
#else
    (void)config;
    debug::print(debug::PrintMode::ERROR, "SerialManager",
        "USB bulk requested but this build has no libusb-1.0 support");
    return nullptr;
#endif
}

std::shared_ptr<ProtocolInterface> create_protocol_from_config(const toml::table& config) {
    std::string protocol_type = static_param::get_param<std::string>(config, "Serial", "protocol");

    if (protocol_type == "uart") {
        return create_uart_protocol(config);
    }
    if (protocol_type == "usb_bulk") {
        return create_usb_bulk_protocol(config);
    }

    debug::print(debug::PrintMode::ERROR, "SerialManager",
        "Unknown protocol type: {}", protocol_type);
    return nullptr;
}

class SerialManager {
public:
    SerialManager() = delete;

    static constexpr int MAX_RETRY_COUNT = 5;
    static constexpr int RETRY_INTERVAL_MS = 2000;

    static void start_serial_threads() {
        auto config = static_param::parse_file("hardware.toml");

        bool ignore_crc = static_param::get_param<bool>(config, "Serial", "ignore_crc");
        bool data_print_debug = static_param::get_param<bool>(config, "Serial", "data_print_debug");
        auto debug_print = umt::BasicObjManager<bool>::find_or_create("serial_debug_print", data_print_debug);
        debug_print->get() = data_print_debug;

        int64_t reconnect_interval_ms = RETRY_INTERVAL_MS;
        int64_t max_reconnect = MAX_RETRY_COUNT;
        std::string protocol_type = static_param::get_param<std::string>(config, "Serial", "protocol");
        if (protocol_type == "usb_bulk") {
            reconnect_interval_ms = static_param::get_param<int64_t>(
                config, "Serial.usb_bulk", "reconnect_interval_ms");
            max_reconnect = static_param::get_param<int64_t>(
                config, "Serial.usb_bulk", "max_reconnect_attempts");
        }

        std::shared_ptr<ProtocolInterface> protocol = nullptr;
        int retry_count = 0;

        while (max_reconnect < 0 || retry_count < max_reconnect) {
            try {
                protocol = create_protocol_from_config(config);
                if (protocol && protocol->open()) {
                    debug::print(debug::PrintMode::INFO, "SerialManager",
                        "Protocol opened successfully");
                    break;
                }

                if (protocol) {
                    debug::print(debug::PrintMode::WARNING, "SerialManager",
                        "Open failed ({}/{}): {}",
                        retry_count + 1,
                        reconnect_attempts_to_string(max_reconnect),
                        protocol->error_message());
                }
            } catch (const std::exception& e) {
                debug::print(debug::PrintMode::WARNING, "SerialManager",
                    "Exception ({}/{}): {}",
                    retry_count + 1,
                    reconnect_attempts_to_string(max_reconnect),
                    e.what());
            }

            retry_count++;
            debug::print(debug::PrintMode::INFO, "SerialManager",
                "Retry in {} ms...", reconnect_interval_ms);
            std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_interval_ms));
        }

        if (!protocol || !protocol->is_open()) {
            debug::print(debug::PrintMode::FATAL, "SerialManager",
                "Protocol open failed after {} retries, exiting", retry_count);
            std::exit(1);
        }

        try {
            auto transceiver = std::make_shared<TransceiverManager<32>>(protocol, ignore_crc);
            std::thread([transceiver]() { serial_sender_run(transceiver); }).detach();
            std::thread([transceiver]() { serial_receiver_run(transceiver); }).detach();
            debug::print(debug::PrintMode::INFO, "SerialManager", "TX/RX threads started");
        } catch (const std::exception& e) {
            debug::print(debug::PrintMode::FATAL, "SerialManager", "Start failed: {}", e.what());
            std::exit(1);
        }
    }
};

} // namespace

void serial_sender_run(std::shared_ptr<TransceiverManager<32>> transceiver) {
    try {
        auto config = static_param::parse_file("hardware.toml");
        const bool imu_yaw_negate = static_param::get_param<bool>(config, "Serial", "imu_yaw_negate");
        const bool imu_pitch_negate = static_param::get_param<bool>(config, "Serial", "imu_pitch_negate");

        auto vision_transmit = umt::BasicObjManager<VisionData_t>::find_or_create("vision_transmit");
        auto send_enabled = umt::BasicObjManager<bool>::find_or_create("serial_send_enabled", true);
        auto app_running = umt::BasicObjManager<bool>::find_or_create("app_running", true);
        auto debug_print = umt::BasicObjManager<bool>::find_or_create("serial_debug_print", false);

        debug::print(debug::PrintMode::INFO, "SerialSender", "Sender thread started");
        debug::print(debug::PrintMode::INFO, "SerialSender",
            "TX sign mapping: yaw_negate={} pitch_negate={}", imu_yaw_negate, imu_pitch_negate);

        stats::FpsStats fps_stats("SerialSender", "sent");
        auto next_tick = std::chrono::steady_clock::now();

        while (app_running->get()) {
            try {
                if (!send_enabled->get()) {
                    std::this_thread::sleep_for(10ms);
                    next_tick = std::chrono::steady_clock::now();
                    continue;
                }

                VisionData_t vision_data = vision_transmit->load();
                if (imu_yaw_negate) {
                    vision_data.yaw = -vision_data.yaw;
                }
                if (imu_pitch_negate) {
                    vision_data.pitch = -vision_data.pitch;
                }

                bool sent = false;
                FixedPacket<32> packet;
                if (SerialUtils::vision_data_to_packet(vision_data, packet)) {
                    if (debug_print->get()) {
                        std::string hex;
                        for (size_t i = 0; i < 32; ++i) {
                            hex += fmt::format("{:02X} ", packet.buffer()[i]);
                        }
                        debug::print(debug::PrintMode::DEBUG, "SerialTX", "{}", hex);
                    }

                    if (transceiver->send_packet(packet)) {
                        sent = true;
                    }
                }

                fps_stats.update(0, sent);
                next_tick += 1ms;
                std::this_thread::sleep_until(next_tick);
            } catch (const std::exception& e) {
                debug::print(debug::PrintMode::ERROR, "SerialSender", "Exception: {}", e.what());
                std::this_thread::sleep_for(100ms);
                next_tick = std::chrono::steady_clock::now();
            }
        }

        debug::print(debug::PrintMode::INFO, "SerialSender", "Sender thread stopped");
    } catch (const std::exception& e) {
        debug::print(debug::PrintMode::ERROR, "SerialSender", "Init failed: {}", e.what());
    }
}

void serial_receiver_run(std::shared_ptr<TransceiverManager<32>> transceiver) {
    try {
        umt::Publisher<SerialReceiveData> publisher("serial_receive");
        auto current_should_detect = umt::BasicObjManager<bool>::find_or_create("current_should_detect", false);
        auto current_should_detect_time_us =
            umt::BasicObjManager<int64_t>::find_or_create("current_should_detect_time_us", 0);
        auto recv_enabled = umt::BasicObjManager<bool>::find_or_create("serial_recv_enabled", true);
        auto app_running = umt::BasicObjManager<bool>::find_or_create("app_running", true);
        auto debug_print = umt::BasicObjManager<bool>::find_or_create("serial_debug_print", false);

        debug::print(debug::PrintMode::INFO, "SerialReceiver", "Receiver thread started");
        stats::FpsStats fps_stats("SerialReceiver", "received");

        while (app_running->get()) {
            try {
                if (!recv_enabled->get()) {
                    std::this_thread::sleep_for(10ms);
                    continue;
                }

                FixedPacket<32> packet;
                bool received = transceiver->recv_packet(packet);
                if (received) {
                    if (debug_print->get()) {
                        std::string hex;
                        for (size_t i = 0; i < 32; ++i) {
                            hex += fmt::format("{:02X} ", packet.buffer()[i]);
                        }
                        debug::print(debug::PrintMode::DEBUG, "SerialRX", "{}", hex);
                    }

                    auto recv_time = std::chrono::steady_clock::now();
                    int64_t recv_time_us = std::chrono::duration_cast<std::chrono::microseconds>(
                        recv_time.time_since_epoch()
                    ).count();

                    SerialReceiveData receive_data;
                    if (SerialUtils::packet_to_receive_data(packet, receive_data)) {
                        receive_data.recv_time_us = recv_time_us;
                        current_should_detect->store(receive_data.should_detect);
                        current_should_detect_time_us->store(recv_time_us);

                        rr::scalar("serial/should_detect", receive_data.should_detect);
                        rr::scalar("serial/dart_number", static_cast<int>(receive_data.dart_number));

                        publisher.push(receive_data);
                    }
                } else {
                    std::this_thread::sleep_for(1ms);
                }

                fps_stats.update(0, received);
            } catch (const std::exception& e) {
                debug::print(debug::PrintMode::ERROR, "SerialReceiver", "Exception: {}", e.what());
                std::this_thread::sleep_for(10ms);
            }
        }

        debug::print(debug::PrintMode::INFO, "SerialReceiver", "Receiver thread stopped");
    } catch (const std::exception& e) {
        debug::print(debug::PrintMode::ERROR, "SerialReceiver", "Init failed: {}", e.what());
    }
}

void start_serial_communication() {
    SerialManager::start_serial_threads();
}

bool SerialUtils::vision_data_to_packet(const VisionData_t& cmd, PacketType& packet) {
    try {
        packet.clear();

        const uint8_t control = cmd.is_found ? 1U : 0U;
        const uint8_t shoot = 0U;
        const float yaw_rad = cmd.yaw * kDegToRad;
        const float pitch_rad = cmd.pitch * kDegToRad;

        packet.load_data(control, 1);
        packet.load_data(shoot, 2);
        packet.load_data(yaw_rad, 3);
        packet.load_data(pitch_rad, 7);

        uint8_t* buf = const_cast<uint8_t*>(packet.buffer());
        crc16_append(buf + 1, 30);
        return true;
    } catch (const std::exception& e) {
        debug::print(debug::PrintMode::ERROR, "SerialUtils", "vision_data_to_packet: {}", e.what());
        return false;
    }
}

bool SerialUtils::packet_to_receive_data(const PacketType& packet, SerialReceiveData& data) {
    try {
        // Dart RX payload: byte 1 = should_detect, byte 2 = dart_number.
        uint8_t should_detect = 0;
        if (packet.unload_data(should_detect, 1)) {
            data.should_detect = (should_detect != 0);
        }

        uint8_t dart_number = 1;
        if (packet.unload_data(dart_number, 2)) {
            data.dart_number = dart_number;
        }

        data.aim_mode = data.should_detect ? 1U : 0U;
        data.aiming_lock = data.should_detect;
        data.allow_fire = data.should_detect;
        return true;
    } catch (const std::exception& e) {
        debug::print(debug::PrintMode::ERROR, "SerialUtils", "packet_to_receive_data: {}", e.what());
        return false;
    }
}

} // namespace serial
