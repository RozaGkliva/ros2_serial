// Copyright 2024 Roza Gkliva

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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

    ros2_serial_interfaces::msg::SerialString message_;
    rclcpp::Publisher<ros2_serial_interfaces::msg::SerialString>::SharedPtr publisher_;
    std::string frame_id_;
    std::string device_hwid_;
    int baud_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif /* SERIAL_INTERFACE_HPP_ */
