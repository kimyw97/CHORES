#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/bool.hpp>
#include <serial/serial.h>
#include <iostream>
#include <string>
#include "toad_auto_drive/msg/toad_drive_msg.hpp"
#include "trash_camera/msg/trash_info.hpp"
#include "geometry_msgs/msg/point.hpp"

using std::placeholders::_1;

class ToadControllerNode : public rclcpp::Node{
    public :
    ToadControllerNode() : Node("toad_controller_node"){
        cap_subscription_ = this->create_subscription<trash_camera::msg::TrashInfo>(
            "trash_info", 10,
            std::bind(&ToadControllerNode::trashDetectCallback, this, _1)
        );

        drive_subscription_ = this->create_subscription<toad_auto_drive::msg::ToadDriveMsg>(
            "/auto_drive", 10,
            std::bind(&ToadControllerNode::driveCallback, this, _1)
        );

        try{
            serial_.setPort("/dev/serial0");
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
    bool right_sensor = false;
    bool left_sensor = false;
    bool is_trash = false;
    float distance_cm = 0.0;
    geometry_msgs::msg::Point center;

    rclcpp::Subscription<trash_camera::msg::TrashInfo>::SharedPtr cap_subscription_;
    rclcpp::Subscription<toad_auto_drive::msg::ToadDriveMsg>::SharedPtr drive_subscription_;

    int right_pwm = 0;
    int left_pwm = 0;
    bool edge_detect = false;
    bool middle_clean = false;
    int cnt = 0;
    int middle_cnt = 0;

    void trashDetectCallback(const trash_camera::msg::TrashInfo::SharedPtr msg){
        is_trash = msg->is_trash;
        distance_cm = msg->distance_cm;
	center = msg->center;

        controller();
    }

    void driveCallback(const toad_auto_drive::msg::ToadDriveMsg::SharedPtr msg){
        right_sensor = msg->right_sensor;
        left_sensor = msg->left_sensor;

        controller();
    }
    
    void controller(){

        if(is_trash){
            cnt = 0;
            middle_cnt = 0;
            edge_detect = false;
            middle_clean = false;
            if(distance_cm > 15){
                if(right_sensor && left_sensor){
                    go_straight();
                    std::string message = "L" + std::to_string(left_pwm) + "R" + std::to_string(right_pwm) + "\n";
                    serial_.write(message);
                    RCLCPP_INFO(this->get_logger(), "쓰레기 감지, 쓰레기와의 거리 : %2.f", distance_cm);
                }
                RCLCPP_INFO(this->get_logger(), "쓰레기 감지, 쓰레기와의 거리 : %2.f", distance_cm);
            }else if(distance_cm > 0 && distance_cm <= 15){
                std::string message = "L0R0";
                serial_.write(message);
                RCLCPP_INFO(this->get_logger(), "쓰레기 수거 중");
                //로봇팔
                
            }
        }else{
            if (edge_detect && !middle_clean) {
                RCLCPP_INFO(this->get_logger(), "모서리 발견");
                if (!left_sensor && !right_sensor) {
                    turn_left();
                } else if (left_sensor && right_sensor) {
                    go_straight();
                    edge_detect = false;
                    cnt++;
                    if (cnt == 4) {
                        cnt = 0;
                        middle_clean = true;
                    }
                } else if (left_sensor && !right_sensor) {
                    turn_left();
                } else if (!left_sensor && right_sensor) {
                    turn_right();
                }
            } else if (!edge_detect && !middle_clean) {
                if (left_sensor && right_sensor) {
                    go_straight();
                } else if (left_sensor && !right_sensor) {
                    turn_left();
                } else if (!left_sensor && right_sensor) {
                    turn_right();
                } else if (!left_sensor && !right_sensor) {
                    turn_left();
                    edge_detect = true;
                }
            } else if (!edge_detect && middle_clean) {
                if (middle_cnt == 0) {
                    go_straight();
                    middle_cnt++;
                } else if (middle_cnt == 1) {
                    turn_left();
                    middle_cnt++;
                } else if (middle_cnt >= 2) {
                    if (left_sensor && right_sensor) {
                        go_straight();
                        middle_cnt++;
                    } else if (!left_sensor && !right_sensor) {
                        middle_cnt = 0;
                        middle_clean = false;
                        stop();
                    }
                }

                std::string message = "L" + std::to_string(left_pwm) + "R" + std::to_string(right_pwm) + "\n";
            }
        } 
    }

    void go_straight(){
        right_pwm = 25;
        left_pwm = 25;
        RCLCPP_INFO(this->get_logger(), "직진");
    }

    void stop(){
        right_pwm = 0;
        left_pwm = 0;
        RCLCPP_INFO(this->get_logger(), "정지");
    }
    void back(){
        right_pwm = -25;
        left_pwm = -25;
        RCLCPP_INFO(this->get_logger(), "후진");
    }
    void turn_left(){
        right_pwm = 25;
        left_pwm = -25;
        RCLCPP_INFO(this->get_logger(), "좌회전");
    }
    void turn_right(){
        right_pwm = -25;
        left_pwm = 25;
        RCLCPP_INFO(this->get_logger(), "우회전");
    }

};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ToadControllerNode>());
    rclcpp::shutdown();
    return 0;
}
