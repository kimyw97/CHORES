#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <unordered_map>

char getch()
{
    struct termios oldt, newt;
    char ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

class ArmCommandPublisher : public rclcpp::Node
{
public:
    ArmCommandPublisher() : Node("arm_control_pub_node")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("arm_command", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ArmCommandPublisher::keyLoop, this));
   
    }

private:
void keyLoop()
{
    char c = getch();
    std::unordered_map<char, std::string> key_map = {
        {'1', "pose1"},
        {'2', "pose2"},
        {'3', "pose3"},
        {'4', "pose4"},
        {'5', "pose5"},
        {'6', "gripper_open"},
        {'7', "gripper_close"},
    };

    if (key_map.find(c) != key_map.end())
    {
        std_msgs::msg::String msg;
        msg.data = key_map[c];
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published: '%s'", msg.data.c_str());
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "알 수 없는 입력: '%c'", c);
    }
}

rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
rclcpp::TimerBase::SharedPtr timer_;
}; 


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmCommandPublisher>());
    rclcpp::shutdown();
    return 0;
}
