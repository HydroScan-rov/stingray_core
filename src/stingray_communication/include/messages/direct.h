#ifndef STINGRAY_MESSAGES_DIRECT_H
#define STINGRAY_MESSAGES_DIRECT_H

#include "messages/common.h"

// pult -> cm4
struct RequestDirectMessage : public AbstractMessage
{
    RequestDirectMessage();

    const static uint8_t length = 142; // 1(type) + 139(message) + 2(checksum) = 142 dyte

    const static uint8_t type = 0xAA;
    uint8_t connection_status;
    uint8_t flags; // [0]thrusters_on, [1]reset_imu, [2]reset_depth, [3]rgb_light_on, [4]lower_light_on, [5]save_constants
    uint8_t reverse; // [0]reverse of 0 thruster, [1]reverse of 1st thruster
    uint8_t id[8]; // [0]id of horizontal-front-left, [1]id of horizontal-front-right....
    float_t target_force[8]; // newton
    float_t k_forward[8];
    float_t k_backward[8];
    uint16_t dPWM_forward[8]; // 0-500
    uint16_t dPWM_backward[8]; // 0-500

    uint16_t checksum;

    bool thrusters_on;
    bool reset_imu;
    bool reset_depth;
    bool rgb_light_on;
    bool lower_light_on;
    bool save_constants;

    bool enable_reverse[8];

    bool parse(std::vector<uint8_t>& input) override; // pult to raspberry_cm4
};

// cm4 -> pult
struct ResponseDirectMessage : public AbstractMessage
{
    ResponseDirectMessage();

    const static uint8_t length = 30; // 1(type) + 27(message) + 2(checksum) = 30 dyte

    const static uint8_t type = 0xAA;
    uint8_t connection_status;

    uint16_t current_logic_electronics; // from jetson + raspberry dc-dc
    uint16_t current_vma[8];
    uint16_t voltage_battery_cell[4]; // [0]: 1st sell; [1]: 1+2; [2]: 1+2+3; [3]: 1+2+3+4 (full battery)

    uint16_t checksum;

    void pack(std::vector<uint8_t>& container) override; // raspberry_cm4 to pult
};
#endif // STINGRAY_MESSAGES_DIRECT_H
