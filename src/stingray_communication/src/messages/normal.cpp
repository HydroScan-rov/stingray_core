#include "messages/normal.h"

RequestNormalMessage::RequestNormalMessage() : AbstractMessage()
{
    connection_status = 0;
    flags = 0;
    stab_flags = 0;
    control_mode = 0;
    march = 0;
    lag = 0;
    depth = 0;
    roll = 0;
    pitch = 0;
    yaw = 0;
    tilt = 0;
    power_lower_light = 0;
    r_rgb_light = 0;
    g_rgb_light = 0;
    b_rgb_light = 0;
    checksum = 0;
}

// pull message from byte-vector (pult to raspberry_cm4)
bool RequestNormalMessage::parse(std::vector<uint8_t> &input)
{
    // get checksum
    popFromVector(input, checksum, true);
    uint16_t checksum_calc = getChecksum16b(input);
    if (checksum_calc != checksum)
    {
        return false;
    }

    popFromVector(input, b_rgb_light);
    popFromVector(input, g_rgb_light);
    popFromVector(input, r_rgb_light);
    popFromVector(input, power_lower_light);
    popFromVector(input, tilt);
    popFromVector(input, yaw);
    popFromVector(input, pitch);
    popFromVector(input, roll);
    popFromVector(input, depth);
    popFromVector(input, lag);
    popFromVector(input, march);
    popFromVector(input, control_mode);
    popFromVector(input, stab_flags);
    popFromVector(input, flags);
    popFromVector(input, connection_status);

    thrusters_on = pickBit(flags, 0);
    reset_imu = pickBit(flags, 1);
    reset_depth = pickBit(flags, 2);
    rgb_light_on = pickBit(flags, 3);
    lower_light_on = pickBit(flags, 4);

    stab_march = pickBit(stab_flags, 0);
    stab_lag = pickBit(stab_flags, 1);
    stab_depth = pickBit(stab_flags, 2);
    stab_roll = pickBit(stab_flags, 3);
    stab_pitch = pickBit(stab_flags, 4);
    stab_yaw = pickBit(stab_flags, 5);

    control_handle = pickBit(control_mode, 0);
    control_auto = pickBit(control_mode, 1);
    control_maneuverable = pickBit(control_mode, 2);

    return true;
}

ResponseNormalMessage::ResponseNormalMessage()
{
    depth = 0;
    roll = 0;
    pitch = 0;
    yaw = 0;

    distance_l = 0;
    distance_r = 0;

    speed_down = 0;
    speed_right = 0;

    current_logic_electronics = 0;
    for (int i = 0; i < 4; i++)
    {
        current_vma[i] = 0;
    }
    for (int i = 0; i < 8; i++)
    {
        voltage_battery_cell[i] = 0;
    }
    voltage_battery = 0;

    checksum = 0;
}

// form byte-vector (raspberry_cm4 to pult)
void ResponseNormalMessage::pack(std::vector<uint8_t> &container)
{
    pushToVector(container, depth);
    pushToVector(container, roll);
    pushToVector(container, pitch);
    pushToVector(container, yaw);

    pushToVector(container, distance_l);
    pushToVector(container, distance_r);

    pushToVector(container, speed_down);
    pushToVector(container, speed_right);

    pushToVector(container, current_logic_electronics);
    for (int i = 0; i < 8; i++)
    {
        pushToVector(container, current_vma[i]);
    }
    for (int i = 0; i < 4; i++)
    {
        pushToVector(container, voltage_battery_cell[i]);
    }
    pushToVector(container, voltage_battery);

    uint16_t checksum = getChecksum16b(container);
    pushToVector(container, checksum); // do i need to revert bytes here?
};
