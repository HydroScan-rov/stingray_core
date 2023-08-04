#ifndef STINGRAY_MESSAGES_STM_H
#define STINGRAY_MESSAGES_STM_H

#include "messages/common.h"

// cm4 -> stm
struct StmRequestMessage : public AbstractMessage
{
    StmRequestMessage();

    const static uint8_t length = 35; // 1(type) + 32(message) + 2(checksum) = 35 dyte

    const static uint8_t type = 0xA5;

    uint8_t connection_status;
    uint8_t flags;             // [0]thrusters_on, [1]rgb_light_on, [2]lower_light_on,
    uint16_t velocity[8];      // pwm to thrusters 8 pcs
    uint16_t tilt;             // 1000-2000
    uint8_t power_lower_light; // 0-255
    uint8_t r_rgb_light;       // 0-255
    uint8_t g_rgb_light;
    uint8_t b_rgb_light;

    uint16_t checksum;

    void pack(std::vector<uint8_t> &container) override; // STM to raspberry_cm4
};

// stm -> cm4
struct StmResponseMessage : public AbstractMessage
{
    StmResponseMessage();

    const static uint8_t length = 56; // 1(type) + 53(message) + 2(checksum) = 56 dyte

    const static uint8_t type = 0xA5;

    uint8_t connection_status;
    float_t current_logic_electronics; // from stm 
    float_t current_vma[8];
    float_t voltage_battery_cell[4];

    uint16_t checksum;

    bool parse(std::vector<uint8_t> &input) override; // STM to raspberry_cm4
};

#endif // STINGRAY_MESSAGES_STM_H
