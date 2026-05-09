//
// Test Serial Communication
//

#include <chrono>
#include <thread>

#include "hardware/serial/serial_thread.hpp"
#include "plugin/debug/logger.hpp"
#include "umt/umt.hpp"

int main() {
    using namespace std::chrono_literals;

    debug::init_session("test_serial");
    debug::print(debug::PrintMode::INFO, "TestSerial", "Serial communication test starting...");

    try {
        // Start serial communication from config/hardware.toml.
        serial::start_serial_communication();

        // Subscribe parsed electrical-control data.
        umt::Subscriber<serial::SerialReceiveData> subscriber("serial_receive");

        debug::print(debug::PrintMode::INFO, "TestSerial", "Serial started, monitoring receive...");

        int count = 0;
        int total = 0;
        serial::SerialReceiveData latest_data;
        bool has_data = false;
        auto start_time = std::chrono::steady_clock::now();

        while (true) {
            try {
                auto data = subscriber.pop_for(100);
                count++;
                total++;
                latest_data = data;
                has_data = true;

                if (total % 100 == 0) {
                    debug::print(debug::PrintMode::DEBUG, "TestSerial",
                        "[{}] should_detect: {}, dart_number: {}",
                        total, data.should_detect, static_cast<int>(data.dart_number));
                }
            } catch (const umt::MessageError_Timeout&) {
                // Timeout is normal when no serial data arrives in this 100 ms window.
            }

            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();
            if (elapsed >= 1) {
                if (has_data) {
                    debug::print(debug::PrintMode::INFO, "TestSerial",
                        "Received {} packets in last second, latest should_detect: {}, dart_number: {}",
                        count, latest_data.should_detect, static_cast<int>(latest_data.dart_number));
                } else {
                    debug::print(debug::PrintMode::INFO, "TestSerial",
                        "Received {} packets in last second", count);
                }

                count = 0;
                start_time = now;
            }
        }

    } catch (const std::exception& e) {
        debug::print(debug::PrintMode::ERROR, "TestSerial", "Error: {}", e.what());
        return 1;
    }

    return 0;
}
