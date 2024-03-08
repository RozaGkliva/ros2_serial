

#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"

// #include <chrono>

class SerialInterface : public rclcpp::Node
{
    public:
        SerialInterface()
        : Node("serial_interface")
        {
            RCLCPP_INFO(this->get_logger(), "SerialInterface Node has been created");

            try
            {
                serial_ = std::make_shared<serial::Serial>("/dev/ttyACM0", 115200, serial::Timeout::simpleTimeout(1000));
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
                RCLCPP_INFO(this->get_logger(), "Read from serial: %s", data.c_str());
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
