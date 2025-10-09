//
// Created by rm_autoaim on 2025/10/1.
//

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>
#include  <cmath>

namespace rm_serial_driver {
    struct ReceiverPacket {
        uint8_t header = 0x5A;
        uint8_t detect_color: 1;
        uint8_t task_mode: 2;
        // bool reset_tracker :1;
        // uint8_t is_play: 1;
        // bool chang_target : 5;
        uint8_t reserved: 5;
        float roll;
        float pitch;
        float yaw;
        float aim_x;
        float aim_y;
        float aim_z;
        // uint16_t game_time;
        // uint32_t timestamp;
        uint16_t checksum = 0;
    }__attribute__((packed));

    struct SendPacket {
        uint8_t header = 0xA5;
        bool state: 1;          //
        uint8_t id: 3;
        uint8_t armors_num: 3;
        // uint8_t reserved: 1;
        float x;
        float y;
        float z;
        float yaw;

        float vx;
        float vy;
        float vz;
        float v_yaw;
        float r1;
        float r2;
        float dz;

        // uint32_t cap_timestamp;
        // uint16_t t_offset;

        // uint16_t chechsum = 0;
    }__attribute__((packed));


    inline ReceiverPacket fromVector(const std::vector<uint8_t> *data) {
        ReceiverPacket packet;
        std::copy(data->begin(), data->end(), reinterpret_cast<uint8_t *>(&packet));
        return packet;
    }

    inline std::vector<uint8_t> toVector(const SendPacket &data) {
        std::vector<uint8_t> packet(sizeof(SendPacket));
        std::copy(
            reinterpret_cast<const uint8_t *>(&data),
            reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacket), packet.begin());
        return packet;
    }
}

#endif //RM_SERIAL_DRIVER__PACKET_HPP_
