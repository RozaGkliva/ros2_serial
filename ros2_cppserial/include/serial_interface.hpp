#ifndef SERIAL_INTERFACE_HPP_
#define SERIAL_INTERFACE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>

#include "serial/serial.h"
#include "ros2_serial_interfaces/msg/serial_string.hpp"

class SerialInterface : public rclcpp::Node
{
public:
    serial::Serial serial_device;
    std::string device_port;

    SerialInterface();

private:
    std::string scan_ports();
    void init_serial(std::string device_port_);
    void readSerial();

    rclcpp::Publisher<ros2_serial_interfaces::msg::SerialString>::SharedPtr publisher_;
    std::string frame_id_;
    std::string device_hwid_;
    int baud_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif /* SERIAL_INTERFACE_HPP_ */
