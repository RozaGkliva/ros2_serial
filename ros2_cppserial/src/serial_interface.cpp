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

#include "serial_interface.hpp"

#include <string>
#include <vector>

// TODO: find a unique device identifier, this only identifies the USB-Serial adapter

SerialInterface::SerialInterface()
    : Node("serial_interface")
{
    // get parameters
    this->declare_parameter("serial_params.use_port", false);
    bool use_port = this->get_parameter("serial_params.use_port").as_bool();

    if (use_port)
    {
        this->declare_parameter("serial_params.port", "default");
        std::string port = this->get_parameter("serial_params.port").as_string();
        RCLCPP_INFO(this->get_logger(), "Port provided: %s", port.c_str());
        device_port = port;
    }
    else
    {
        // RCLCPP_INFO(this->get_logger(), "Port not provided, trying to find device with hardware ID");
        this->declare_parameter("serial_params.device_hwid", "id");
        device_hwid_ = this->get_parameter("serial_params.device_hwid").as_string();
       RCLCPP_INFO(this->get_logger(), "Port not provided. Trying to find device with hardware ID: %s", device_hwid_.c_str());
        device_port = scan_ports();

    }

    this->declare_parameter("serial_params.baud", 9600);
    baud_ = this->get_parameter("serial_params.baud").as_int();

    // RCLCPP_INFO(this->get_logger(), "Device port: %s", device_port.c_str());
    if (device_port.compare("nan") != 0)
    {
        SerialInterface::init_serial(device_port);
    }
    else
    {
        throw std::runtime_error("Device port not found!");
    }

    this->declare_parameter("incoming_message.header", "start");
    this->declare_parameter("incoming_message.packet_size", 0);
    this->declare_parameter("incoming_message.footer", "end");
    this->declare_parameter("incoming_message.rate_hz", 0);
    this->declare_parameter("publish_topic.name", "name");
    this->declare_parameter("publish_topic.frame_id", "fid");

    std::string header = this->get_parameter("incoming_message.header").as_string();
    std::string footer = this->get_parameter("incoming_message.footer").as_string();
    // int8_t packet_size = this->get_parameter("incoming_message.packet_size").as_int();
    int8_t read_rate = this->get_parameter("incoming_message.rate_hz").as_int();
    std::string topic_name = this->get_parameter("publish_topic.name").as_string();
    frame_id_ = this->get_parameter("publish_topic.frame_id").as_string();

    // create a timer to read from serial port
    auto timer_period_ = int((1.0/read_rate) * 1000);  // calculate sampling period and convert to milliseconds
    timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_period_), std::bind(&SerialInterface::readSerial, this));

    // create messag type and publisher
    message_ = ros2_serial_interfaces::msg::SerialString();
    message_.header.frame_id = frame_id_;
    publisher_ = this->create_publisher<ros2_serial_interfaces::msg::SerialString>(topic_name, 10); 
}

// private member functions

/**
 * Scans all ports for a device with a given hardware ID. Finds port where device is connected.
 * 
 * @return string(port), or 'nan' if port not found.
 */
std::string SerialInterface::scan_ports()
{
    std::string port_;

    std::vector<serial::PortInfo> devices_found = serial::list_ports();

    std::vector<serial::PortInfo>::iterator iter = devices_found.begin();

    while( iter != devices_found.end() )
    {
        serial::PortInfo device = *iter++;
        std::string device_ = device.hardware_id.c_str();
        std::size_t device_found = device_.find(device_hwid_);

        // printf( "(port: %s, device description: %s, device hardware_id: %s)\n", device.port.c_str(), device.description.c_str(), device.hardware_id.c_str() );

        if(device_found!=std::string::npos)
        {   
            RCLCPP_INFO(this->get_logger(), "device_found: %s\n", device_.c_str());
            port_ = device.port.c_str();
            RCLCPP_INFO(this->get_logger(), "Found device_port: %s\n", port_.c_str());
        }
        // else
        // {
        //     RCLCPP_INFO(this->get_logger(), "Device not found, returning port as nan");  // TODO: figure out why this runs even when device is found
        //     port_ = "nan";
        // }
    }
    return port_;
}

/**
 * Initializes serial port.
 * 
 * @param string_port_ String denoting the port where WMS is connected.
 * 
 * @throws serial::IOException Thrown if unable to open port
 */
void SerialInterface::init_serial(std::string port_)
{
    // std::int32_t baud_ = 115200;            // TODO: baud rate from yaml

    // try connecting to serial port
    try
    {
        serial_device.setPort(port_);
        serial_device.setBaudrate(baud_);
        serial::Timeout to_ = serial::Timeout(200, 200, 0, 200, 0);
        serial_device.setTimeout(to_);
        serial_device.open();
    }
    catch(serial::IOException& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Unable to open port %s ", port_.c_str());
    }
}

void SerialInterface::readSerial()
{
    if (serial_device.available() > 1)
    {   
        message_.header.stamp = this->get_clock()->now();
        // use frame_id from node
        message_.header.frame_id = frame_id_;

        std::string data = serial_device.readline();

        message_.data = data;
        // RCLCPP_INFO(this->get_logger(), "Read from serial: %s", data.c_str());

        // publish the message
        publisher_->publish(message_);
    }
}

/**
 * Main entry point for the node.
 */
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialInterface>());
    rclcpp::shutdown();
    return 0;
}
