#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "serial/serial.h"
#include <unordered_map>
#include <vector>

class ArmControlNode : public rclcpp::Node
{
public:
    ArmControlNode();  // 생성자 선언만

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg);
    void send_joint_angles(const std::vector<int> &angles);
    void gripper_callback(const std_msgs::msg::String::SharedPtr msg);
    void send_gripper_angle(int angle);
    std::string format_joint_angles(const std::vector<int>& angles);

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gripper_subscription_;
    serial::Serial serial_;
    std::unordered_map<std::string, std::vector<int>> joint_angles_map_;
    std::unordered_map<std::string, std::vector<int>> gripper_angles_map_;
};
