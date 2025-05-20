#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/bool.hpp>
#include <serial/serial.h>
#include <iostream>
#include <string>
#include "toad_auto_drive/msg/toad_drive_msg.hpp"
#include <regex>

class ToadAutoDrivePubNode : public rclcpp::Node {
public:
    ToadAutoDrivePubNode() : Node("auto_drive_node") {
        publisher_ = this->create_publisher<toad_auto_drive::msg::ToadDriveMsg>("/auto_drive", 10);
        right_motor_ = 0;
        left_motor_ = 0;
        edge_detect = false;
        midle_clean = false;
        is_turn = false;
        cnt = 0;
        midle_cnt = 0;

   
        try {
            //serial_.setPort("/dev/ttyUSB0");
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

    ~ToadAutoDrivePubNode() {
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    long int right_motor_;
    long 
    int left_motor_;
    bool edge_detect;
    bool midle_clean;
    bool is_turn;
    rclcpp::Publisher<toad_auto_drive::msg::ToadDriveMsg>::SharedPtr publisher_;
    serial::Serial serial_;
    int cnt;
    int midle_cnt;
    void check_go_possible_() {
        auto msg = toad_auto_drive::msg::ToadDriveMsg();

        if (serial_.available()) {
            std::string data = serial_.readline(1024, "\n");
            RCLCPP_INFO(this->get_logger(), "Received: %s", data.c_str());

            bool L = false, R = false;
            std::regex s1_regex("s1(\\d+)");
            std::regex s2_regex("s2(\\d+)");
            std::smatch match;
            if (std::regex_search(data, match, s1_regex) && match.size() > 1) {
                L = (std::stoi(match[1]) == 1);
            }
            if (std::regex_search(data, match, s2_regex) && match.size() > 1) {
                R = (std::stoi(match[1]) == 1);
            }
            
            msg.left_sensor = L;
            msg.right_sensor = R;

        publisher_->publish(msg);
        }

        
    }

    
};

// 메인 함수
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ToadAutoDrivePubNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

