#include "serial_interface.hpp"

#include <string>
#include <vector>

// TODO: find a unique device identifier, this only identifies the USB-Serial adapter

SerialInterface::SerialInterface()
    : Node("serial_interface")
{
    // RCLCPP_INFO(this->get_logger(), "SerialInterface Node has been created");

    this->declare_parameter("serial_params.port", "/dev/ttyUSB0");
    this->declare_parameter("serial_params.baud", 9600);
    this->declare_parameter("incoming_message.header", "start");
    this->declare_parameter("incoming_message.packet_size", 0);
    this->declare_parameter("incoming_message.footer", "end");
    this->declare_parameter("incoming_message.rate_hz", 0);
    this->declare_parameter("publish_topic.name", "name");
    this->declare_parameter("publish_topic.frame_id", "fid");

    std::string port = this->get_parameter("serial_params.port").as_string();
    int baud = this->get_parameter("serial_params.baud").as_int();
    RCLCPP_INFO(this->get_logger(), "Trying to use serial port: %s, at baud:%d", port.c_str(), baud);  // TODO: get port from scan_ports(), with device_id from yaml

    device_port = scan_ports();
    if (device_port.compare("nan") != 0)               // if the port is not 'nan' then try connecting
    {
        SerialInterface::init_serial(device_port);           // setup and connect to serial port
    }
    else
    {
        throw std::runtime_error("Device port not found!");
    }

    // try
    // {
    //     serial_ = std::make_shared<serial::Serial>(port, baud, serial::Timeout::simpleTimeout(1000));
    // }
    // catch(const std::exception& e)
    // {
    //     RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
    //     throw;
    // }

    std::string header = this->get_parameter("incoming_message.header").as_string();
    std::string footer = this->get_parameter("incoming_message.footer").as_string();
    // int8_t packet_size = this->get_parameter("incoming_message.packet_size").as_int();
    int8_t read_rate = this->get_parameter("incoming_message.rate_hz").as_int();
    std::string topic_name = this->get_parameter("publish_topic.name").as_string();
    frame_id_ = this->get_parameter("publish_topic.frame_id").as_string();

    // create a timer to read from serial port
    auto timer_period_ = 1/read_rate;
    timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_period_), std::bind(&SerialInterface::readSerial, this));

    // create publisher
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

        // find serial device and get port
        std::string device_ = device.hardware_id.c_str();

        // find the desired device
        std::size_t device_found = device_.find("0403:6001");   // TODO: hardware_id from yaml

        if(device_found!=std::string::npos )
        {   
            // RCLCPP_INFO(this->get_logger(), "device_found: %s\n", device_.c_str());
            port_ = device.port.c_str();
            RCLCPP_INFO(this->get_logger(), "Found device_port: %s\n", port_.c_str());
        }
        else
        {
            port_ = "nan";
        }
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
    std::int32_t baud_ = 115200;            // TODO: baud rate from yaml

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
        auto message = ros2_serial_interfaces::msg::SerialString();
        message.header.stamp = this->get_clock()->now();
        // use frame_id from node
        message.header.frame_id = frame_id_;

        std::string data = serial_device.readline();

        message.data = data;
        RCLCPP_INFO(this->get_logger(), "Read from serial: %s", data.c_str());

        // publish the message
        publisher_->publish(message);
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
