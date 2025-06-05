#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

class IsTrashPublisher : public rclcpp::Node {
public:
  IsTrashPublisher()
  : Node("is_trash_publisher"), is_trash_(false) {
    publisher_ = this->create_publisher<std_msgs::msg::Bool>("is_trash", 10);
    timer_ = this->create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&IsTrashPublisher::timer_callback, this));
  }

private:
  void timer_callback() {
    auto msg = std_msgs::msg::Bool();
    msg.data = is_trash_;
    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published is_trash: %s", is_trash_ ? "true" : "false");
    is_trash_ = !is_trash_;  // true <-> false toggle
  }

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool is_trash_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IsTrashPublisher>());
  rclcpp::shutdown();
  return 0;
}
