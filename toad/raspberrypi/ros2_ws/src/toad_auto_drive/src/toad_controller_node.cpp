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
        cap_subscription_ = this->create_subscription<toad_auto_drive::msg::TrashInfo>(
            "/trash_detect", 10,
            std::bind(&ToadControllerNode::trashDetectCallback, this, _1)
        );

        drive_subscription_ = this->create_subscription<toad_auto_drive::msg::ToadDriveMsg>(
            "/auto_drive", 10,
            std::bind(&ToadControllerNode::driveCallback, this, _1)
        );


        try{
            serial_.setPort("/dev/ttyUSB0");
            serial_.setBaudrate(115200);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            serial_.setTimeout(to);
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
    bool is_trash;
    float distance_cm;
    int center;
    rclcpp::Subscription<toad_auto_drive::msg::TrashInfo>::SharedPtr cap_subscription_;
    rclcpp::Subscription<toad_auto_drive::msg::ToadDriveMsg>::SharedPtr drive_subscription_;


    void trashDetectCallback(const toad_auto_drive::msg::TrashInfo::SharedPtr msg){
        is_trash = msg->is_trash;
        distance_cm = msg->distance_cm;
        center = msg->center;

        controller();
    }

    void driveCallback(const toad_auto_drive::msg::ToadDriveMsg::SharedPtr msg){
        right_pwm = msg->right_motor;
        left_pwm = msg->left_motor;

        controller();
    }

    void controller(){
        if(is_trash){
            if(distance_cm > 30.0){
                std::string message = "L" + std::to_string(left_pwm) + "R" + std::to_string(right_pwm) + "\n";
                serial_.write(message);
            }else if(distance_cm > 0 && distance_cm <= 30){
                std::string message = "L0R0";
                serial_.write(message);
                message = ""; // 로봇팔 각도값 입력
                serial_.write(message);
            }
        }else{
            std::string message = "L" + std::to_string(left_pwm) + "R" + std::to_string(right_pwm) + "\n";
            serial_.write(message);
        }
    }

};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ToadControllerNode>());
    rclcpp::shutdown();
    return 0;
}
