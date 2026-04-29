/**
 * @file test_serial.cpp
 * @brief Run the serial receiver thread with a mock protocol and print parsed data.
 */

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "hardware/serial/crc16.hpp"
#include "hardware/serial/protocol/protocol_interface.hpp"
#include "hardware/serial/serial_thread.hpp"
#include "plugin/debug/logger.hpp"
#include "plugin/telemetry/telemetry.hpp"
#include "umt/umt.hpp"

namespace {

class MockProtocol final : public ProtocolInterface {
public:
    bool open() override {
        opened_ = true;
        return true;
    }

    void close() noexcept override {
        opened_ = false;
    }

    [[nodiscard]] bool is_open() const noexcept override {
        return opened_;
    }

    [[nodiscard]] int read(std::byte* buffer, std::size_t len) override {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!opened_) return -1;
        if (rx_chunks_.empty()) return 0;

        auto& chunk = rx_chunks_.front();
        const std::size_t n = std::min(len, chunk.size());
        std::copy_n(chunk.data(), n, buffer);

        if (n == chunk.size()) {
            rx_chunks_.pop_front();
        } else {
            chunk.erase(chunk.begin(), chunk.begin() + static_cast<std::ptrdiff_t>(n));
        }

        return static_cast<int>(n);
    }

    [[nodiscard]] int write(const std::byte* buffer, std::size_t len) override {
        std::lock_guard<std::mutex> lock(mutex_);
        tx_bytes_.insert(tx_bytes_.end(), buffer, buffer + len);
        return static_cast<int>(len);
    }

    [[nodiscard]] std::string error_message() const override {
        return {};
    }

    void push_chunk(std::vector<std::byte> bytes) {
        std::lock_guard<std::mutex> lock(mutex_);
        rx_chunks_.push_back(std::move(bytes));
    }

private:
    bool opened_ = false;
    std::deque<std::vector<std::byte>> rx_chunks_;
    std::vector<std::byte> tx_bytes_;
    std::mutex mutex_;
};

serial::SerialUtils::PacketType make_receive_packet(bool should_detect, uint8_t dart_number) {
    serial::SerialUtils::PacketType packet;
    packet.clear();

    const uint8_t should_detect_byte = should_detect ? 1U : 0U;
    packet.load_data(should_detect_byte, 1);
    packet.load_data(dart_number, 2);

    auto* buffer = const_cast<uint8_t*>(packet.buffer());
    serial::crc16_append(buffer + 1, 6);
    return packet;
}

std::vector<std::byte> to_bytes(const serial::SerialUtils::PacketType& packet) {
    std::vector<std::byte> bytes(8);
    const auto* buffer = packet.buffer();
    for (std::size_t i = 0; i < bytes.size(); ++i) {
        bytes[i] = static_cast<std::byte>(buffer[i]);
    }
    return bytes;
}

std::string hex_dump(const serial::SerialUtils::PacketType& packet) {
    std::ostringstream oss;
    const auto* buffer = packet.buffer();
    for (std::size_t i = 0; i < 8; ++i) {
        if (i > 0) oss << ' ';
        oss << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
            << static_cast<int>(buffer[i]);
    }
    return oss.str();
}

void print_receive_data(const serial::SerialReceiveData& data) {
    std::cout
        << "parsed: "
        << "should_detect=" << (data.should_detect ? "true" : "false")
        << ", dart_number=" << static_cast<int>(data.dart_number)
        << ", aim_mode=" << static_cast<int>(data.aim_mode)
        << ", allow_fire=" << (data.allow_fire ? "true" : "false")
        << ", aiming_lock=" << (data.aiming_lock ? "true" : "false")
        << ", recv_time_us=" << data.recv_time_us
        << '\n';
}

}  // namespace

int main() {
    debug::set_min_level(debug::PrintMode::DEBUG);
    telemetry::init_from_config();

    auto app_running = umt::BasicObjManager<bool>::find_or_create("app_running", true);
    auto recv_enabled = umt::BasicObjManager<bool>::find_or_create("serial_recv_enabled", true);
    auto debug_print = umt::BasicObjManager<bool>::find_or_create("serial_debug_print", true);
    app_running->set(true);
    recv_enabled->set(true);
    debug_print->set(true);

    auto protocol = std::make_shared<MockProtocol>();
    protocol->open();

    const auto packet1 = make_receive_packet(true, 2);
    const auto packet2 = make_receive_packet(false, 4);

    std::cout << "input packet 1: " << hex_dump(packet1) << '\n';
    std::cout << "input packet 2: " << hex_dump(packet2) << '\n';

    protocol->push_chunk(to_bytes(packet1));

    auto packet2_bytes = to_bytes(packet2);
    protocol->push_chunk(std::vector<std::byte>(packet2_bytes.begin(), packet2_bytes.begin() + 3));
    protocol->push_chunk(std::vector<std::byte>(packet2_bytes.begin() + 3, packet2_bytes.end()));

    auto transceiver = std::make_shared<serial::TransceiverManager<8>>(protocol);
    umt::Publisher<serial::SerialReceiveData> serial_pub_keep_alive("serial_receive");
    umt::Subscriber<serial::SerialReceiveData> serial_sub("serial_receive", 8);

    std::thread receiver_thread([transceiver]() {
        serial::serial_receiver_run(transceiver);
    });

    int received_count = 0;
    while (received_count < 2) {
        try {
            auto data = serial_sub.pop_for(1000);
            print_receive_data(data);
            ++received_count;
        } catch (const umt::MessageError_Timeout&) {
            std::cerr << "timeout waiting serial_receive output\n";
            app_running->set(false);
            receiver_thread.join();
            return 1;
        }
    }

    app_running->set(false);
    receiver_thread.join();
    telemetry::shutdown();
    return 0;
}
