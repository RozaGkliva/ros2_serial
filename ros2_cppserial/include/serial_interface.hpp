#ifndef SERIAL_INTERFACE_HPP_
#define SERIAL_INTERFACE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"
#include "ros2_serial_interfaces/msg/serial_string.hpp"

class SerialInterface : public rclcpp::Node
{
public:
    SerialInterface();

private:
    void readSerial();

    rclcpp::Publisher<ros2_serial_interfaces::msg::SerialString>::SharedPtr publisher_;
    std::string frame_id_;
    std::shared_ptr<serial::Serial> serial_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif /* SERIAL_INTERFACE_HPP_ */
