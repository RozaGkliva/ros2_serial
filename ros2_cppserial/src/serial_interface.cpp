

#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"

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
            
        }

    private:
        std::shared_ptr<serial::Serial> serial_;


};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialInterface>());
    rclcpp::shutdown();
    return 0;
}
