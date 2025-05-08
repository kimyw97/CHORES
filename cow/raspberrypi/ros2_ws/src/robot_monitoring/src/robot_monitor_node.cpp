#include "rclcpp/rclcpp.hpp"
#include "robot_monitoring/msg/robot_status.hpp"

#include <fstream>
#include <regex>
#include <string>

class RobotMonitorNode : public rclcpp::Node {
public:
  RobotMonitorNode()
  : Node("robot_monitor_node")
  {
    status_pub_ = this->create_publisher<robot_monitoring::msg::RobotStatus>("robot_status", 10);
    serial_.open("/dev/serial0");
    serial_.setf(std::ios::skipws);
    if (!serial_.is_open()) {
      RCLCPP_FATAL(this->get_logger(), "Failed to open /dev/serial0");
      rclcpp::shutdown();
    } else {
      RCLCPP_INFO(this->get_logger(), "Serial port /dev/serial0 opened successfully.");
    }

    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&RobotMonitorNode::readSerial, this));
  }

private:
  void readSerial() {
    std::string line;
    if (!std::getline(serial_, line)) {
      RCLCPP_WARN(this->get_logger(), "No line received from serial.");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Raw line: '%s'", line.c_str());

    std::regex regex("SPEED:L(-?\\d+),R(-?\\d+);TRASH:(\\d);EMERGENCY:(\\d);ENCODER:L(-?\\d+),R(-?\\d+)");
    std::smatch match;

    if (std::regex_search(line, match, regex)) {
      auto msg = robot_monitoring::msg::RobotStatus();
      msg.left_speed = std::stoi(match[1]);
      msg.right_speed = std::stoi(match[2]);
      msg.trash_full = std::stoi(match[3]) == 1;
      msg.emergency = std::stoi(match[4]) == 1;
      msg.left_encoder = std::stoi(match[5]);
      msg.right_encoder = std::stoi(match[6]);

      RCLCPP_INFO(this->get_logger(), "Parsed data - LSpeed: %d, RSpeed: %d, Trash: %d, Emergency: %d, LEnc: %d, REnc: %d",
        msg.left_speed, msg.right_speed,
        msg.trash_full, msg.emergency,
        msg.left_encoder, msg.right_encoder
      );

      status_pub_->publish(msg);
      RCLCPP_INFO(this->get_logger(), "Published robot status.");
    } else {
      RCLCPP_WARN(this->get_logger(), "Regex match failed for line: '%s'", line.c_str());
    }
  }

  std::ifstream serial_;
  rclcpp::Publisher<robot_monitoring::msg::RobotStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotMonitorNode>());
  rclcpp::shutdown();
  return 0;
}