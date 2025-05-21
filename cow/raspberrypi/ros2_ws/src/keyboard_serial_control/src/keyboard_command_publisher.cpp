#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>
#include <string>

class KeyboardPublisher : public rclcpp::Node {
public:
  KeyboardPublisher() : Node("keyboard_command_publisher") {
    publisher_ = this->create_publisher<std_msgs::msg::String>("robot_cmd", 10);
    RCLCPP_INFO(this->get_logger(), "Keyboard Command Publisher Started");

    // 비동기 루프 (main에서 spin 호출 시 loop 안 돌게 하기 위함)
    thread_ = std::thread([this]() { input_loop(); });
  }

  ~KeyboardPublisher() {
    if (thread_.joinable())
      thread_.join();
  }

private:
  void input_loop() {
    std::string input;
    while (rclcpp::ok()) {
      std::cout << "Enter command (e.g. U100, D50, S1): ";
      std::getline(std::cin, input);

      if (input.empty())
        continue;

      if (input[0] == 'U' || input[0] == 'D' || input[0] == 'S') {
        auto msg = std_msgs::msg::String();
        msg.data = input;
        publisher_->publish(msg);
      } else {
        std::cout << "Invalid command format.\n";
      }
    }
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  std::thread thread_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KeyboardPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
