#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/bool.hpp>
#include <serial/serial.h>
#include <iostream>
#include <string>
#include "toad_auto_drive/msg/toad_drive_msg.hpp"
#include <regex>

class ToadAutoDrivePubNode : public rclcpp::Node{
    public:
    ToadAutoDrivePubNode(): Node("auto_drive_node"){
        publisher_ = this-> create_publisher<toad_auto_drive::msg::ToadDriveMsg>("/auto_drive", 10);
        right_motor_ = 0.0;
        left_motor_ = 0.0;
        edge_detect = false;
        midle_clean = false;
        is_turn = false;
        cnt = 0;
        midle_cnt = 0;
        
        try {
            serial_.setPort("/dev/serial0");
            serial_.setBaudrate(115200);
            serial::Timeout to = serial::Timeout::simpleTimeout(100);
            serial_.setTimeout(to);
            serial_.open();
        } catch (serial::IOException &e) {
            RCLCPP_ERROR(this->get_logger(), "Unable to open serial port.");
        }

        if (serial_.isOpen()) {
            RCLCPP_INFO(this->get_logger(), "Serial port opened successfully.");
        }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ToadAutoDrivePubNode::check_go_possible_, this)
        );

    }
    private:
    rclcpp::TimerBase::SharedPtr timer_;
    float right_motor_;
    float left_motor_;
    bool edge_detect;
    bool midle_clean;
    bool is_turn;
    float time;
    rclcpp::Publisher<toad_auto_drive::msg::ToadDriveMsg>::SharedPtr publisher_;
    serial::Serial serial_;
    int cnt;
    int midle_cnt;
    
    void check_go_possible_(){

        auto msg = toad_auto_drive::msg::ToadDriveMsg();
        if (serial_.available()) {
            std::string data = serial_.readline(1024, "\n");
            RCLCPP_INFO(this->get_logger(), "Received: %s", data.c_str());
            
            bool L = false, R = false;
            float distance = 0.0;

            std::regex s1_regex("s1(\\d+)");
            std::regex s2_regex("s2(\\d+)");
            std::smatch match;
            if (std::regex_search(data, match, s1_regex) && match.size() > 1) {
                L = (std::stoi(match[1]) == 1);
            }
            if (std::regex_search(data, match, s2_regex) && match.size() > 1) {
                R = (std::stoi(match[1]) == 1);
            }

            if(edge_detect && !midle_clean){
                RCLCPP_INFO(this->get_logger(), "모서리 회전 판단");
                if(!L && !R){
                    msg = turn_left(msg);
                    RCLCPP_INFO(this->get_logger(), "회전");
                }else if(L && R){
                    msg = go_straight(msg);
                    edge_detect = false;
                    RCLCPP_INFO(this->get_logger(), "회전 완료");
                    cnt++;

                    if(cnt == 4){
                        RCLCPP_INFO(this->get_logger(), "청소 완료");
                        cnt = 0;
                        midle_clean = true;
                    }
                    
                    RCLCPP_INFO(this->get_logger(), "cnt : %d", cnt);
                    
                }else if(L && !R){
                    msg = turn_left(msg);
                    RCLCPP_INFO(this->get_logger(), "좌회전");
                }else if(!L && R){
                    msg = turn_right(msg);
                    RCLCPP_INFO(this->get_logger(), "우회전");
                }
            }else if(!edge_detect && !midle_clean){
            if(L && R){
                msg = go_straight(msg);
                RCLCPP_INFO(this->get_logger(), "직진");
            }else if(L && !R){
                msg = turn_left(msg);
                RCLCPP_INFO(this->get_logger(), "좌회전");
            }else if(!L && R){
                msg = turn_right(msg);
                RCLCPP_INFO(this->get_logger(), "우회전");
            }else if (!L && !R) {
                msg = turn_left(msg);
                edge_detect = true;
                RCLCPP_INFO(this->get_logger(), "책상 모서리 발견");
                publisher_->publish(msg);
            }
        }else if(!edge_detect && midle_clean){
            RCLCPP_INFO(this->get_logger(), "책상 가운데 부분 청소");
            if(midle_cnt == 0){
                msg = go_straight(msg);
                midle_cnt++;
            }else if(midle_cnt == 1){
                msg = turn_left(msg);
                midle_cnt++;
            }else if(midle_cnt == 2){
                msg = go_straight(msg);
            }
        }
            RCLCPP_INFO(this->get_logger(), "Recive : left : %f, right : %f",msg.left_motor, msg.right_motor);
            publisher_->publish(msg);

    }


            
    }

    toad_auto_drive::msg::ToadDriveMsg go_straight(toad_auto_drive::msg::ToadDriveMsg msg){
        msg.left_motor = 30.0;
        msg.right_motor = 30.0;
        return msg;
    }
    
    toad_auto_drive::msg::ToadDriveMsg turn_left(toad_auto_drive::msg::ToadDriveMsg msg){
        msg.left_motor = -30.0;
        msg.right_motor = 30.0;
        return msg;
    }

    toad_auto_drive::msg::ToadDriveMsg turn_right(toad_auto_drive::msg::ToadDriveMsg msg){
        msg.left_motor = 30.0;
        msg.right_motor = -30.0;
        return msg;
    }

    toad_auto_drive::msg::ToadDriveMsg go_back(toad_auto_drive::msg::ToadDriveMsg msg){
        msg.left_motor = -30.0;
        msg.right_motor = -30.0;
        return msg;
    }

};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ToadAutoDrivePubNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}