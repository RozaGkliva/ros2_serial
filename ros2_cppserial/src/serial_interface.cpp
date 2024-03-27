

#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"

#include <variant>

#include "ros2_serial_interfaces/msg/serial_string.hpp"


class SerialInterface : public rclcpp::Node
{
    public:
        SerialInterface()
        : Node("serial_interface")
        {
            RCLCPP_INFO(this->get_logger(), "SerialInterface Node has been created");

            this->declare_parameter("serial_params.port", "/dev/ttyUSB0");
            this->declare_parameter("serial_params.baud", 9600);
            this->declare_parameter("incoming_message.header", "start");
            this->declare_parameter("incoming_message.packet_size", 0);
            this->declare_parameter("incoming_message.footer", "end");

            std::string port = this->get_parameter("serial_params.port").as_string();
            int baud = this->get_parameter("serial_params.baud").as_int();
            // RCLCPP_INFO(this->get_logger(), "Using serial port: %s, at baud:%d", port.c_str(), baud);

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

            

            // create a timer to read from serial port
            timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&SerialInterface::readSerial, this));
            // create publisher 
            publisher_ = this->create_publisher<ros2_serial_interfaces::msg::SerialString>("serial_data", 10);  // TODO: topic name from yaml

            
        }

    private:
        // define a struct to hold the parsed data
        struct PayloadData
        {
            std::vector<std::string> content;
            std::vector<int> varLens;
            std::vector<std::variant<float, short>> variables;  // std::variant is a type-safe union to hold different types of variables
        }; 

        rclcpp::Publisher<ros2_serial_interfaces::msg::SerialString>::SharedPtr publisher_;
        
        void readSerial()
        {
            if (serial_->available() > 1)
            {
                auto message = ros2_serial_interfaces::msg::SerialString();
                message.header.stamp = this->get_clock()->now();
                message.header.frame_id = "serial_data";        // TODO: frame_id from yaml
                std::string data = serial_->readline();

                message.data = data;
                RCLCPP_INFO(this->get_logger(), "Read from serial: %s", data.c_str());
                // print out the entire packet
                // RCLCPP_INFO(this->get_logger(), "Read from serial: ");
                // for (uint8_t byte: data) {
                //     RCLCPP_INFO(this->get_logger(), "0x%02x ", byte);
                // }

                // take the first 8 bytes and convert them to  a string
                // std::string syncstr = data.substr(0, 16);
                // RCLCPP_INFO(this->get_logger(), "Read from serial: %s", syncstr.c_str()); 

                // publish the message
                publisher_->publish(message);
            }
        }

        std::shared_ptr<serial::Serial> serial_;
        rclcpp::TimerBase::SharedPtr timer_;


};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialInterface>());
    rclcpp::shutdown();
    return 0;
}
