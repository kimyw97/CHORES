#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/bool.hpp>
#include <serial/serial.h>
#include <iostream>
#include <string>
#include "toad_auto_drive/msg/toad_drive_msg.hpp"
#include "toad_auto_drive/msg/trash_info.hpp"

using std::placeholders::_1;

class ToadControllerNode : public rclcpp::Node{
    public :
    ToadControllerNode() : Node("toad_controller_node"){
        cap_subscription_ = tihs->create_subscription<toad_auto_drive::msg:::TrashInfo>(
            "/trash_detect", 10,
            std::bind(&ToadControllerNode::trashDetectCallback, tihs, _1);
        );

        drive_subscription_ = this->create_subscription<toad_auto_drive::msg::ToadDriveMsg>(
            "/auto_drive", 10,
            std::bind(&ToadAutoDriveSub::driveCallback, this, _1)
        );


        try{
            serial_.setPort("/dev/ttyUSB0");
            serial_.setBaurdrate(115200);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            serial_.setTimeouw(to);
            serial_.open();
        }catch(serial::IOException e){
            RCLCPP_ERROR(this->get_logger(), "unable to open serial port");
            rclcpp::shutdown();
        }
    }
    private :
    serial::Serial serial_;
    int right_pwm = 0;
    int left_pwm = 0;
    rclcpp::Subscription<toad_auto_drive::msg:::TrashInfo>::SharedPtr cap_subscription_;
    rclcpp::Subscription<toad_auto_drive::msg:::TrashInfo>::SharedPtr drive_subscription_;


    void trashDetectCallback(){

    }

    void driveCallback(const toad_auto_drive::msg::ToadDriveMsg::SharedPtr msg){
        right_pwm = msg->right_motor;
        left_pwm = msg->left_motor;
        std::string message = "L" + std::to_string(left_pwm) + "R" + std::to_string(right_pwm) + "\n";
    }

};

int main(int argc, char * argv[]){

}