#include "rclcpp/rclcpp.hpp"
#include "toad_auto_drive/msg/toad_teleop_msg.hpp"
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <iostream>
#include <unordered_map>
#include <std_msgs/msg/string.hpp>

class TeleopDriveNode : public rclcpp::Node
{
public:
    TeleopDriveNode() : Node("teleop_drive_node")
    {
        pub_ = this->create_publisher<toad_auto_drive::msg::ToadTeleopMsg>("/auto_drive", 10);
        publisher_ = this->create_publisher<std_msgs::msg::String>("arm_command", 10);
        RCLCPP_INFO(this->get_logger(), "Teleop node started. Use WASD to control, Q to quit.");

        configureTerminal();
        run();
        restoreTerminal();
    }

private:
    rclcpp::Publisher<toad_auto_drive::msg::ToadTeleopMsg>::SharedPtr pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    struct termios old_tio_;

    std::unordered_map<char, std::string> key_map = {
        {'1', "pose1"},
        {'2', "pose2"},
        {'3', "pose3"},
        {'4', "pose4"},
        {'5', "pose5"},
	{'6', "pose6"},
        {'7', "gripper_open"},
        {'8', "gripper_close"},
	{'9', "gripper_catch"},
    };

    void configureTerminal()
    {
        struct termios new_tio;
        tcgetattr(STDIN_FILENO, &old_tio_);
        new_tio = old_tio_;
        new_tio.c_lflag &= ~(ICANON | ECHO); // 비표준 입력, 에코 끄기
        tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);
    }

    void restoreTerminal()
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &old_tio_);
    }

    void run()
    {
        char c;
        int speed = 25;
        std_msgs::msg::String arm_msg;
        

        while (rclcpp::ok())
        {
            c = getchar();
            auto msg = toad_auto_drive::msg::ToadTeleopMsg();


            if(c == 'u'){
                speed += 1;
                if(speed > 100){
                    speed = 100;
                }
            }
            if(c == 'j'){
                speed -= 1;
                if(speed < 1){
                    speed = 1;
                }
            }
            if (c == 'w') {        // 앞으로
                msg.left_motor = speed;
                msg.right_motor = speed;
            }
            else if (c == 's') {   // 뒤로
                msg.left_motor = -speed;
                msg.right_motor = -speed;
            }
            else if (c == 'a') {   // 좌회전
                msg.left_motor = -speed;
                msg.right_motor = speed;
            }
            else if (c == 'd') {   // 우회전
                msg.left_motor = speed;
                msg.right_motor = -speed;
            }
            else if (c == 'x') {   // 정지
                msg.left_motor = 0;
                msg.right_motor = 0;

            }else if(c == '1'){
                arm_msg.data = key_map['1'];
                publisher_->publish(arm_msg);

            }
            else if(c == '2'){
                arm_msg.data = key_map['2'];
                publisher_->publish(arm_msg);

            }
            else if(c == '3'){
                arm_msg.data = key_map['3'];
                publisher_->publish(arm_msg);

            }
            else if(c == '4'){
                arm_msg.data = key_map['4'];
                publisher_->publish(arm_msg);

            }
            else if(c == '5'){
                arm_msg.data = key_map['5'];
                publisher_->publish(arm_msg);

            }
            else if(c == '6'){
                arm_msg.data = key_map['6'];
                publisher_->publish(arm_msg);

            }
            else if(c == '7'){
                arm_msg.data = key_map['7'];
                publisher_->publish(arm_msg);

            }
            else if(c == '8'){
                arm_msg.data = key_map['8'];
                publisher_->publish(arm_msg);

            }
            else if(c == '9'){
                arm_msg.data = key_map['9'];
                publisher_->publish(arm_msg);

            }
            else if (c == 'q') {   // 종료
                break;
            }
            else {
                continue;
            }

            pub_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Sent: left: %d, right: %d", msg.left_motor, msg.right_motor);
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::make_shared<TeleopDriveNode>();  // run()은 생성자에서 호출
    rclcpp::shutdown();
    return 0;
}

