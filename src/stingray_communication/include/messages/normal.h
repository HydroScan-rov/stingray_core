#ifndef STINGRAY_MESSAGES_NORMAL_H
#define STINGRAY_MESSAGES_NORMAL_H

#include "messages/common.h"

// pult -> cm4
struct RequestNormalMessage : public AbstractMessage
{
    RequestNormalMessage();

    const static uint8_t length = 37; // 1(type) + 30(message) + 2(checksum) = 33 dyte

    const static uint8_t type = 0xA5;
    uint8_t connection_status;
    uint8_t flags;        // [0]thrusters_on, [1]reset_imu, [2]reset_depth, [3]rgb_light_on, [4]lower_light_on,
    uint8_t stab_flags;   // [0]march, [1]lag, [2]depth, [3]roll, [4]pitch, [5]yaw
    uint8_t control_mode; // [0]handle , [1]auto (set depth and yaw, pitch and roll = 0), [2]maneuverable (set depth, yaw, pitch and roll)
    float march;          // NED coordinate system
    float lag;
    float depth;
    float roll;
    float pitch;
    float yaw;
    uint16_t tilt;             // 1000-2000
    uint8_t power_lower_light; // 0-255
    uint8_t r_rgb_light;       // 0-255
    uint8_t g_rgb_light;
    uint8_t b_rgb_light;
    uint16_t checksum;

    bool thrusters_on;
    bool reset_imu;
    bool reset_depth;
    bool rgb_light_on;
    bool lower_light_on;

    bool stab_march;
    bool stab_lag;
    bool stab_depth;
    bool stab_roll;
    bool stab_pitch;
    bool stab_yaw;

    bool control_handle;
    bool control_auto;
    bool control_maneuverable;

    bool parse(std::vector<uint8_t>& input) override; // pult to raspberry_cm4
};

// cm4 -> pult
struct ResponseNormalMessage : public AbstractMessage
{
    ResponseNormalMessage();

    const static uint8_t length = 91; // 89(message) + 2(checksum) = 91 dyte

    uint8_t reseived_connection_status;

    float_t depth;
    float_t roll;
    float_t pitch;
    float_t yaw;

    float_t distance_l; // distance from laser rangefinder
    float_t distance_r;

    float_t speed_down; // speed signal from jetson
    float_t speed_right;

    float_t current_logic_electronics; // from jetson + raspberry dc-dc
    float_t current_vma[8];
    float_t voltage_battery_cell[4];
    float_t voltage_battery; // 56

    uint16_t checksum;

    void pack(std::vector<uint8_t>& container) override; // raspberry_cm4 to pult
};

#endif // STINGRAY_MESSAGES_NORMAL_H
