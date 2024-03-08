

#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"


class SerialInterface : public rclcpp::Node
{
    public:
        SerialInterface()
        : Node("serial_interface")
        {
            RCLCPP_INFO(this->get_logger(), "SerialInterface Node has been created");

            this->declare_parameter("serial_params.port", "/dev/ttyUSB0");
            this->declare_parameter("serial_params.baud", 9600);
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

            // create a timer to read from serial port
            timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&SerialInterface::readSerial, this));
            
        }

    private:
        void readSerial()
        {
            if (serial_->available() > 0)
            {
                std::string data = serial_->readline();
                // RCLCPP_INFO(this->get_logger(), "Read from serial: %s", data.c_str());
                // print out the entire packet
                RCLCPP_INFO(this->get_logger(), "Read from serial: ");
                for (uint8_t byte: data) {
                    RCLCPP_INFO(this->get_logger(), "0x%02x ", byte);
                }

                // take the first 8 bytes and convert them to  a string
                std::string syncstr = data.substr(0, 16);
                RCLCPP_INFO(this->get_logger(), "Read from serial: %s", syncstr.c_str()); 
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
