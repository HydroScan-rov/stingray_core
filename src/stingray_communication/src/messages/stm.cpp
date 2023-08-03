#include "messages/stm.h"

StmRequestMessage::StmRequestMessage() : AbstractMessage()
{
    connection_status = 0;
    flags = 0;
    for (int i = 0; i < 8; i++)
        velocity[i] = 0;
    tilt = 0;
    power_lower_light = 0;
    r_rgb_light = 0;
    g_rgb_light = 0;
    b_rgb_light = 0;
    checksum = 0;
}

// form byte-vector (raspberry_cm4 to pult)
void StmRequestMessage::pack(std::vector<uint8_t> &container)
{
    pushToVector(container, connection_status);
    pushToVector(container, flags);
    for (int i = 0; i < 8; i++)
        pushToVector(container, velocity[i]);
    pushToVector(container, tilt);
    pushToVector(container, power_lower_light);
    pushToVector(container, r_rgb_light);
    pushToVector(container, g_rgb_light);
    pushToVector(container, b_rgb_light);

    uint16_t checksum = getChecksum16b(container);
    pushToVector(container, checksum); // do i need to revert bytes here?
};

StmResponseMessage::StmResponseMessage()
{
    connection_status = 0;
    current_logic_electronics = 0;
    for (int i = 0; i < 4; i++)
        current_vma[i] = 0;
    for (int i = 0; i < 8; i++)
        voltage_battery_cell[i] = 0;

    checksum = 0;
}

// pull message from byte-vector (pult to raspberry_cm4)
bool StmResponseMessage::parse(std::vector<uint8_t> &input)
{
    // get checksum
    popFromVector(input, checksum, true);
    uint16_t checksum_calc = getChecksum16b(input);
    if (checksum_calc != checksum)
        return false;

    for (int i = 0; i < 8; i++)
        popFromVector(input, voltage_battery_cell[8 - i]);
    for (int i = 0; i < 8; i++)
        popFromVector(input, current_vma[8 - i]);
    popFromVector(input, current_logic_electronics);
    popFromVector(input, connection_status);

    return true;
}
