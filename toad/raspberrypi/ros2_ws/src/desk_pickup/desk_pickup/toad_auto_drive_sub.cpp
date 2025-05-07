#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/bool.hpp>
#include <serial/serial.h>
#include <iostream>
#include <string>
#include "toad_auto_drive/msg/toad_drive_msg.hpp"

using std::placeholders::_1;

class ToadAutoDriveSub : public rclcpp::Node{
    public:
    ToadAutoDriveSub() : Node("auto_drive_sub_node"){

        subscription_ = this->create_subscription<toad_auto_drive::msg::ToadDriveMsg>(
            "/auto_drive", 10,
            std::bind(&ToadAutoDriveSub::driveCallback, this, _1)
        );

        try{
            serial_.setPort("/dev/serial0");
            //serial_.setPort("/dev/ttyUSB0");
            serial_.setBaudrate(115200);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            serial_.setTimeout(to);
            serial_.open();
        }catch(serial::IOException& e){
            RCLCPP_ERROR(this->get_logger(), "unable to open serial port");
            rclcpp::shutdown();
        }

      
    }
    private:
    float right_pwm = 0.0;
    float left_pwm = 0.0;
    serial::Serial serial_;
    rclcpp::Subscription<toad_auto_drive::msg::ToadDriveMsg>::SharedPtr subscription_;

    void driveCallback(const toad_auto_drive::msg::ToadDriveMsg::SharedPtr msg){
        
        right_pwm = msg->right_motor;
        left_pwm = msg->left_motor;

        std::string message = "L" + std::to_string(right_pwm) + "R" + std::to_string(left_pwm) + "\n";
        serial_.write(message);
        RCLCPP_INFO(this->get_logger(), "sent : %f, %f", right_pwm, left_pwm);
    }
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ToadAutoDriveSub>());
    rclcpp::shutdown();
    return 0;
}