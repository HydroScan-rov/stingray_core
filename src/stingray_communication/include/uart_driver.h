#ifndef STINGRAY_COMMUNICATION_UART_BRIDGE_H
#define STINGRAY_COMMUNICATION_UART_BRIDGE_H

#include <serial/serial.h>

#include <sstream>
#include <string>
#include <vector>

#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

using json = nlohmann::json;
using std::placeholders::_1;

class UartDriver : public rclcpp::Node {
public:
    UartDriver();

private:
    void toStmMessage_callback(const std_msgs::msg::UInt8MultiArray& msg);
    void portInitialize();
    bool sendData();
    bool receiveData();

    // ROS publishers
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr fromStmMessage_pub;
    // ROS subscribers
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr toStmMessage_sub;
    // Other
    serial::Serial port;  // serial port
    // Message containers
    std::vector<uint8_t> toStmVector;   // Hardware bridge -> Protocol_driver
    std_msgs::msg::UInt8MultiArray fromStmMessage;  // Protocol_driver -> Hardware bridge
    // get json config
    json ros_config;
    json com_config;
};

#endif  // STINGRAY_COMMUNICATION_UART_DRIVER_NODELET_H
