#ifndef SERIAL_THREAD_HPP
#define SERIAL_THREAD_HPP

#include <cstdint>
#include <string>

#include "fixed_packet.hpp"
#include "transceiver_manager.hpp"

namespace serial {

struct VisionData_t {
    int yaw_offset_px;

    VisionData_t() : yaw_offset_px(0) {}
};

// Dart 发送协议: 视觉只上传当前像素偏差 yaw_offset_px。
// Dart 接收协议: 电控只下发是否检测和当前第几枚飞镖。
struct SerialReceiveData {
    bool should_detect;
    uint8_t dart_number;

    int64_t recv_time_us = 0;

    SerialReceiveData()
        : should_detect(false), dart_number(1)
        , recv_time_us(0) {}
};

void serial_sender_run(std::shared_ptr<TransceiverManager<8>> transceiver);

void serial_receiver_run(std::shared_ptr<TransceiverManager<8>> transceiver);

void start_serial_communication();

class SerialUtils {
public:
    using PacketType = FixedPacket<8>;

    static bool vision_data_to_packet(const VisionData_t& cmd, PacketType& packet);

    static bool packet_to_receive_data(const PacketType& packet, SerialReceiveData& data);
};

} // namespace serial

#endif // SERIAL_THREAD_HPP
