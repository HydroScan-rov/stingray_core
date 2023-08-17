#include "messages/direct.h"

// pult -> cm4 -> stm
RequestDirectMessage::RequestDirectMessage() : AbstractMessage() {
    connection_status = 0;
    flags = 0;
    for (int i = 0; i < 8; i++) {
        id[i] = 0;
        target_force[i] = 0;
        k_forward[i] = 0;
        k_backward[i] = 0;
        s_forward[i] = 0;
        s_backward[i] = 0;
    }
    reverse = 0;
    checksum = 0;

    thrusters_on = 0;
    reset_imu = 0;
    reset_depth = 0;
    rgb_light_on = 0;
    lower_light_on = 0;
    save_constants = 0;
}

// stm -> cm4 -> pult
ResponseDirectMessage::ResponseDirectMessage() : AbstractMessage() {
    connection_status = 0;
    current_logic_electronics = 0;
    for (int i = 0; i < 8; i++)
        current_vma[i] = 0;
    for (int i = 0; i < 4; i++)
        voltage_battery_cell[i] = 0;
    checksum = 0;
}

// pult to raspberry_cm4
bool RequestDirectMessage::parse(std::vector<uint8_t>& input) {
    popFromVector(input, checksum);
    uint16_t checksum_calc = getChecksum16b(input);
    if (checksum_calc != checksum) {
        return false;
    }

    for (int i = 7; i >= 0; i++)
        popFromVector(input, s_backward[i]);
    for (int i = 7; i >= 0; i++)
        popFromVector(input, s_forward[i]);
    for (int i = 7; i >= 0; i++)
        popFromVector(input, k_backward[i]);
    for (int i = 7; i >= 0; i++)
        popFromVector(input, k_forward[i]);
    for (int i = 7; i >= 0; i++)
        popFromVector(input, target_force[i]);
    for (int i = 7; i >= 0; i++)
        popFromVector(input, id[i]);

    popFromVector(input, reverse);
    popFromVector(input, flags);
    popFromVector(input, connection_status);

    thrusters_on = pickBit(flags, 0);
    reset_imu = pickBit(flags, 1);
    reset_depth = pickBit(flags, 2);
    rgb_light_on = pickBit(flags, 3);
    lower_light_on = pickBit(flags, 4);
    save_constants = pickBit(flags, 5);

    return true;
}

// form byte-vector (raspberry_cm4 to pult)
void ResponseDirectMessage::pack(std::vector<uint8_t>& container) {
    pushToVector(container, type);
    pushToVector(container, connection_status);
    pushToVector(container, current_logic_electronics);
    for (int i = 0; i < 8; i++)
        pushToVector(container, current_vma[i]);
    for (int i = 0; i < 4; i++)
        pushToVector(container, voltage_battery_cell[i]);

    uint16_t checksum = getChecksum16b(container);
    pushToVector(container, checksum);
}
