#include "serial_interface.hpp"

SerialInterface::SerialInterface()
    : Node("serial_interface")
{
    RCLCPP_INFO(this->get_logger(), "SerialInterface Node has been created");

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
    RCLCPP_INFO(this->get_logger(), "Using serial port: %s, at baud:%d", port.c_str(), baud);

    try
    {
        serial_ = std::make_shared<serial::Serial>(port, baud, serial::Timeout::simpleTimeout(1000));
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
        throw;
    }

    std::string header = this->get_parameter("incoming_message.header").as_string();
    std::string footer = this->get_parameter("incoming_message.footer").as_string();
    int8_t packet_size = this->get_parameter("incoming_message.packet_size").as_int();
    int8_t read_rate = this->get_parameter("incoming_message.rate_hz").as_int();
    std::string topic_name = this->get_parameter("publish_topic.name").as_string();
    frame_id_ = this->get_parameter("publish_topic.frame_id").as_string();

    // create a timer to read from serial port
    auto timer_period_ = 1/read_rate;
    timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_period_), std::bind(&SerialInterface::readSerial, this));

    // create publisher
    publisher_ = this->create_publisher<ros2_serial_interfaces::msg::SerialString>(topic_name, 10); 
}

void SerialInterface::readSerial()
{
    if (serial_->available() > 1)
    {
        auto message = ros2_serial_interfaces::msg::SerialString();
        message.header.stamp = this->get_clock()->now();
        // use frame_id from node
        message.header.frame_id = frame_id_;
        std::string data = serial_->readline();

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
