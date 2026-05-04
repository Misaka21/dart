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
    float yaw;
    float pitch;
    float roll;

    // 当前协议未传 robot_id，保留字段用于兼容旧代码/假数据。
    uint8_t robot_id;
    uint8_t enemy_color;

    float bullet_speed;

    uint8_t aim_mode;
    bool should_detect;
    uint8_t dart_number;
    bool allow_fire;
    bool aiming_lock;

    int64_t recv_time_us = 0;

    SerialReceiveData()
        : yaw(0.0f), pitch(0.0f), roll(0.0f)
        , robot_id(0), enemy_color(0)
        , bullet_speed(15.0f)
        , aim_mode(0), should_detect(false), dart_number(1), allow_fire(false), aiming_lock(false)
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
