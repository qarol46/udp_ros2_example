#pragma once
#include <cstdint>
#include <cstring>

#pragma pack(push, 1)
struct Message {
    uint8_t number_device = 0;     // 0
    uint8_t operating_mode = 0;    // 1
    uint8_t work_device = 0;       // 2
    uint8_t null_array_1[11] = {}; // 3-13
    int16_t linear_vel = 0;        // 14-15
    int16_t angular_vel = 0;       // 16-17
    uint8_t null_array_2[46] = {}; // 18-63
    int16_t torque[6] = {};        // 64-75
    uint8_t __RES2[2] = {};        // 76-77
    int16_t velocity[6] = {};      // 78-89
    uint8_t __RES3[2] = {};        // 90-91
    int16_t odom[6] = {};          // 92-103
    uint8_t null_array_3[23] = {}; // 104-126
    uint8_t sender_addres = 0;     // 127

    Message() = default;
};
#pragma pack(pop)