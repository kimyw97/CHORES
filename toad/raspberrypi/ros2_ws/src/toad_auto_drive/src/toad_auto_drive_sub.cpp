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
            // serial_.setPort("/dev/ttyUSB0");
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
    int right_motor = 0;
    int left_motor = 0;
    bool L = false;
    bool R = false;

    bool edge_detect = false;
    bool midle_clean = false;
    bool is_turn = false;
    int cnt = 0;
    int midle_cnt = 0;

    serial::Serial serial_;
    rclcpp::Subscription<toad_auto_drive::msg::ToadDriveMsg>::SharedPtr subscription_;
    

    void driveCallback(const toad_auto_drive::msg::ToadDriveMsg::SharedPtr msg){
        
        L = msg->left_sensor;
        R = msg->right_sensor;
        std::string message;

        if (edge_detect && !midle_clean) {
            RCLCPP_INFO(this->get_logger(), "모서리 회전 판단");
                if (!L && !R) {
                    stop();
                    message = "L" + std::to_string(left_motor) + "R" + std::to_string(right_motor) + "\n";
                    serial_.write(message);
                    turn_left();
                    RCLCPP_INFO(this->get_logger(), "회전");

                } else if (L && R) {
                    go_straight();
                    edge_detect = false;
                    RCLCPP_INFO(this->get_logger(), "회전 완료");
                    cnt++;
                    if (cnt == 4) {
                        cnt = 0;
                        midle_clean = true;
                    }
                } else if (L && !R) {
                    turn_left();
                } else if (!L && R) {
                    turn_right();
                    
                }
            } else if (!edge_detect && !midle_clean) {
                if (L && R) {
                    go_straight();
                } else if (L && !R) {
                    turn_left();
                } else if (!L && R) {
                    turn_right();
                } else if (!L && !R) {
                    turn_left();
                    edge_detect = true;
                }
            } else if (!edge_detect && midle_clean) {
                if (midle_cnt == 0) {
                    go_straight();
                    midle_cnt++;
                } else if (midle_cnt == 1) {
                    turn_left();
                    midle_cnt++;
                } else if (midle_cnt >= 2) {
                    if (L && R) {
                        go_straight();
                        midle_cnt++;
                    } else if (!L && !R) {
                        midle_cnt = 0;
                        midle_clean = false;
                        left_motor = 0;
                        right_motor = 0;
                    }
                }
            }
            message = "L" + std::to_string(left_motor) + "R" + std::to_string(right_motor) + "\n";
            serial_.write(message);

    }

    void go_straight() {
        left_motor = 25;
        right_motor = 25;
        RCLCPP_INFO(this->get_logger(), "직진");
        
    }

    void turn_left() {
        left_motor = -25;
        right_motor = 25;
        RCLCPP_INFO(this->get_logger(), "좌회전");
        
    }

    void turn_right() {
        left_motor = 25;
        right_motor = -25;
        RCLCPP_INFO(this->get_logger(), "우회전");
        
    }

    void stop(){
        left_motor = 0;
        right_motor = 0;
        RCLCPP_INFO(this->get_logger(), "정지");
    }

    void back(){
        left_motor = -25;
        right_motor = -25;
        RCLCPP_INFO(this->get_logger(), "후진");
    }
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ToadAutoDriveSub>());
    rclcpp::shutdown();
    return 0;
}
