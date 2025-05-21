#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <fstream>
#include <string>

class SerialSender : public rclcpp::Node {
public:
  SerialSender() : Node("serial_command_sender") {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "robot_cmd", 10,
        std::bind(&SerialSender::callback, this, std::placeholders::_1));

    serial_.open("/dev/serial0", std::ios::out | std::ios::binary);
    if (!serial_.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open /dev/serial0");
      rclcpp::shutdown();
    }

    RCLCPP_INFO(this->get_logger(), "Serial Command Sender Started");
  }

  ~SerialSender() {
    if (serial_.is_open())
      serial_.close();
  }

private:
  void callback(const std_msgs::msg::String::SharedPtr msg) {
    if (serial_.is_open()) {
      serial_ << msg->data << "\n";
      serial_.flush();
      RCLCPP_INFO(this->get_logger(), "Sent command: %s", msg->data.c_str());
    }
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  std::ofstream serial_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SerialSender>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

