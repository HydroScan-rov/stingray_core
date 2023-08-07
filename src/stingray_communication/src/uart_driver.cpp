/**
 * This node:
 * - receives data from STM32 and publishes it
 * - receives byte array from hardware_bridge and send it to STM32 via UART
 */

#include <fstream>
#include "uart_driver.h"
#include "messages/stm.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

UartDriver::UartDriver() : Node("UartDriver")
{
    std::string config_directory = ament_index_cpp::get_package_share_directory("stingray_config");
    ros_config = json::parse(std::ifstream(config_directory + "/ros.json"));
    com_config = json::parse(std::ifstream(config_directory + "/communication.json"));
    // Serial port initialization
    portInitialize();
    // ROS publishers
    this->fromStmMessage_pub = this->create_publisher<std_msgs::msg::UInt8MultiArray>(ros_config["topics"]["from_driver_parcel"], 1);
    // ROS subscribers
    this->toStmMessage_sub = this->create_subscription<std_msgs::msg::UInt8MultiArray>(ros_config["topics"]["to_driver_parcel"], 1,
                                                                                       std::bind(
                                                                                           &UartDriver::toStmMessage_callback,
                                                                                           this, _1));
    // Outnput message container
    fromStmMessage.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    fromStmMessage.layout.dim[0].size = StmResponseMessage::length;
    fromStmMessage.layout.dim[0].stride = StmResponseMessage::length;
    fromStmMessage.layout.dim[0].label = "fromStmMessage";
    fromStmMessage.data = {0};
}
/**
 * Initialasing serial port
 * Closes port if it is closed, initialized it
 * with given parameter and DOES NOT OPEN IT.
 */
void UartDriver::portInitialize()
{
    std::string device = com_config["drivers"]["uart"]["device"];
    int baudrate = com_config["drivers"]["uart"]["baudrate"];
    int dataBytesInt = com_config["drivers"]["uart"]["data_bytes"];
    serial::bytesize_t dataBytes;
    switch (dataBytesInt)
    {
    case 5:
        dataBytes = serial::bytesize_t::fivebits;
        break;
    case 6:
        dataBytes = serial::bytesize_t::sixbits;
        break;
    case 7:
        dataBytes = serial::bytesize_t::sevenbits;
        break;
    case 8:
        dataBytes = serial::bytesize_t::eightbits;
        break;
    default:
        RCLCPP_ERROR(this->get_logger(), "Forbidden data bytes size %d, available sizes: 5, 6, 7, 8", dataBytesInt);
        return;
    }
    std::string parityStr = com_config["drivers"]["uart"]["parity"];
    std::transform(parityStr.begin(), parityStr.end(), parityStr.begin(), ::tolower);
    serial::parity_t parity;
    if (parityStr == "even")
        parity = serial::parity_t::parity_even;
    else if (parityStr == "odd")
        parity = serial::parity_t::parity_odd;
    else if (parityStr == "none")
        parity = serial::parity_t::parity_none;
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Unrecognised parity \"%s\", available parities: \"none\", \"odd\", \"even\"",
                     parityStr.c_str());
        return;
    }
    int stopBitsInt = com_config["drivers"]["uart"]["stop_bits"];
    serial::stopbits_t stopBits;
    switch (stopBitsInt)
    {
    case 1:
        stopBits = serial::stopbits_t::stopbits_one;
        break;
    case 2:
        stopBits = serial::stopbits_t::stopbits_two;
        break;
    default:
        RCLCPP_ERROR(this->get_logger(), "Forbidden stop bits size %d, available sizes: 1, 2", stopBitsInt);
        return;
    }
    RCLCPP_INFO(this->get_logger(),
                "UART settings: Device: %s, Baudrate: %d, Data bytes: %d, Parity: %s, Stop bits: %d",
                device.c_str(), baudrate, dataBytes, parityStr.c_str(), stopBitsInt);
    if (port.isOpen())
        port.close();
    port.setPort(device);
    serial::Timeout serialTimeout = serial::Timeout::simpleTimeout(com_config["drivers"]["uart"]["serial_timeout"]);
    port.setTimeout(serialTimeout);
    port.setBaudrate(baudrate);
    port.setBytesize(dataBytes);
    port.setParity(parity);
    port.setStopbits(stopBits);
}

bool UartDriver::sendData()
{
    size_t toWrite = sizeof(uint8_t) * toStmVector.size();
    RCLCPP_INFO(this->get_logger(), "Size: %d", int(toWrite));

    try
    {
        port.flush();
        size_t written = port.write(toStmVector);
        return written == toWrite;
    }
    catch (serial::IOException &ex)
    {
        RCLCPP_ERROR(this->get_logger(), "Serial exception, when trying to flush and send. Error: %s", ex.what());
        return false;
    }
}

bool UartDriver::receiveData()
{
    if (port.available() < StmResponseMessage::length)
    {
        RCLCPP_ERROR(this->get_logger(), "Port not avaliable. Error: %s", port.available());
        return false;
    }
    std::vector<uint8_t> answer;
    port.read(answer, StmResponseMessage::length);
    fromStmMessage.data.clear();
    for (int i = 0; i < StmResponseMessage::length; i++)
        fromStmMessage.data.push_back(answer[i]);
    RCLCPP_INFO(this->get_logger(), "RECEIVE FROM STM");

    return true;
}

/** @brief Parse string bitwise correctly into ResponseNormalMessage and check 16bit checksum.
 *
 * @param[in]  &input String to parse.
 */
void UartDriver::toStmMessage_callback(const std_msgs::msg::UInt8MultiArray &msg)
{
    RCLCPP_INFO(this->get_logger(), "toStmMessage_callback");

    toStmVector.clear();
    for (int i = 0; i < StmRequestMessage::length; i++)
        toStmVector.push_back(msg.data[i]);

    try
    {
        if (!port.isOpen())
        {
            port.open();
            if (!port.isOpen())
                RCLCPP_ERROR(this->get_logger(), "Unable to open UART port");
        }
    }
    catch (serial::IOException &ex)
    {
        RCLCPP_ERROR(this->get_logger(), "Serial exception when trying to open. Error: %s", ex.what());
        return;
    }
    if (!sendData())
    {
        RCLCPP_ERROR(this->get_logger(), "Unable to send message to STM32");
        return;
    }
    else
        RCLCPP_ERROR(this->get_logger(), "Successfully sent to STM32");

    if (receiveData())
        fromStmMessage_pub->publish(fromStmMessage);
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Unable to receive message from STM32");
        return;
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<UartDriver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
