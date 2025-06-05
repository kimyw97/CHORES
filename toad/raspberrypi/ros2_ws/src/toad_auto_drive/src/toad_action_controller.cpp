#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <toad_auto_drive/action/toad_auto_drive_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <serial/serial.h>
#include <string>
#include <thread>
#include <utility>
#include <regex>

class DriveByIRActionServer : public rclcpp::Node {
public:
  using DriveByIR = toad_auto_drive::action::ToadAutoDriveAction;
  using GoalHandleDriveByIR = rclcpp_action::ServerGoalHandle<DriveByIR>;

  DriveByIRActionServer()
      : Node("drive_by_ir_action_server"), is_trash_detected_(false) {

    action_server_ = rclcpp_action::create_server<DriveByIR>(
        this,
        "drive_by_ir",
        std::bind(&DriveByIRActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&DriveByIRActionServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&DriveByIRActionServer::handle_accepted, this, std::placeholders::_1));

    is_trash_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "is_trash",
        10,
        std::bind(&DriveByIRActionServer::isTrashCallback, this, std::placeholders::_1));

    try {
      serial_.setPort("/dev/ttyUSB0");
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
  }

private:
  rclcpp_action::Server<DriveByIR>::SharedPtr action_server_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_trash_sub_;
  serial::Serial serial_;
  bool is_trash_detected_;

  void isTrashCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    is_trash_detected_ = msg->data;
  }

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &,
      std::shared_ptr<const DriveByIR::Goal>) {
    RCLCPP_INFO(this->get_logger(), "Received goal request.");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleDriveByIR>) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal.");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleDriveByIR> goal_handle) {
    std::thread{std::bind(&DriveByIRActionServer::execute, this, goal_handle)}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleDriveByIR> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal...");
    auto feedback = std::make_shared<DriveByIR::Feedback>();
    auto result = std::make_shared<DriveByIR::Result>();

    while (rclcpp::ok() && !goal_handle->is_canceling()) {
      if (is_trash_detected_) {
        feedback->status = "Trash detected. Stopping.";
        goal_handle->publish_feedback(feedback);
        stop();
        result->success = true;
        goal_handle->succeed(result);
        return;
      }

      auto [left_ir, right_ir] = readIRPair();
      driveWithIR(left_ir, right_ir);

      feedback->status = "IR values: L=" + std::to_string(left_ir) + ", R=" + std::to_string(right_ir);
      goal_handle->publish_feedback(feedback);

      rclcpp::sleep_for(std::chrono::milliseconds(200));
    }

    if (goal_handle->is_canceling()) {
      stop();
      result->success = false;
      goal_handle->canceled(result);
      return;
    }

    result->success = true;
    goal_handle->succeed(result);
  }

  std::pair<int, int> readIRPair() {
    if (serial_.available()) {
      std::string data = serial_.readline(1024, "\n");
      RCLCPP_INFO(this->get_logger(), "Received: %s", data.c_str());

      bool left = false, right = false;
      std::regex s1_regex("s1(\\d+)");
      std::regex s2_regex("s2(\\d+)");
      std::smatch match;

      if (std::regex_search(data, match, s1_regex) && match.size() > 1) {
        left = (std::stoi(match[1]) == 1);
      }
      if (std::regex_search(data, match, s2_regex) && match.size() > 1) {
        right = (std::stoi(match[1]) == 1);
      }

      return {left, right};
    }
    return {0, 0};
  }

  void driveWithIR(int left_ir, int right_ir) {
    std::string cmd;

    if (left_ir == 1 && right_ir == 1) {
      RCLCPP_INFO(this->get_logger(), "Go Forward");
      cmd = "L30R30\n";
    } else if (left_ir == 0 && right_ir == 0) {
      RCLCPP_INFO(this->get_logger(), "Stop");
      cmd = "L0R0\n";
    } else if (left_ir == 1 && right_ir == 0) {
      RCLCPP_INFO(this->get_logger(), "Turn Right");
      cmd = "L30R0\n";
    } else if (left_ir == 0 && right_ir == 1) {
      RCLCPP_INFO(this->get_logger(), "Turn Left");
      cmd = "L0R30\n";
    } else {
      cmd = "L0R0\n";
    }

    writeSerial(cmd);
  }

  void stop() {
    RCLCPP_INFO(this->get_logger(), "STOP");
    writeSerial("L0R0\n");
  }

  void writeSerial(const std::string &cmd) {
    if (serial_.isOpen()) {
      serial_.write(cmd);
    } else {
      RCLCPP_WARN(this->get_logger(), "Serial port not open");
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DriveByIRActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
