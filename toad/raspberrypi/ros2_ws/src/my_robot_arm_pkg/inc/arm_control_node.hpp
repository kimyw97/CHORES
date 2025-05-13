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

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    serial::Serial serial_;
    std::unordered_map<std::string, std::vector<int>> joint_angles_map_;
};
