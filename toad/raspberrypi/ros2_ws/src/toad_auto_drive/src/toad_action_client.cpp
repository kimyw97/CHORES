#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <toad_auto_drive/action/toad_auto_drive_action.hpp>
#include <serial/serial.h>
#include <regex>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class DriveByIRActionClient : public rclcpp::Node {
public:
  using DriveByIR = toad_auto_drive::action::ToadAutoDriveAction;
  using GoalHandleDriveByIR = rclcpp_action::ClientGoalHandle<DriveByIR>;

  DriveByIRActionClient()
  : Node("drive_by_ir_action_client") {
    client_ptr_ = rclcpp_action::create_client<DriveByIR>(this, "drive_by_ir");

    // 시리얼 설정
    try {
      serial_.setPort("/dev/ttyUSB0");  // 포트는 환경에 따라 수정
      serial_.setBaudrate(115200);
      serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
      serial_.setTimeout(timeout);  // ✅ OK

      serial_.open();
    } catch (serial::IOException &e) {
      RCLCPP_ERROR(this->get_logger(), "Unable to open serial port.");
    }

    if (serial_.isOpen()) {
      RCLCPP_INFO(this->get_logger(), "Serial port opened successfully.");
    }

    // 타이머 설정: 주기적으로 goal 전송
    this->timer_ = this->create_wall_timer(
      1s, std::bind(&DriveByIRActionClient::send_goal, this));
  }

private:
  rclcpp_action::Client<DriveByIR>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  serial::Serial serial_;

  void send_goal() {
    if (!client_ptr_->wait_for_action_server(2s)) {
      RCLCPP_WARN(this->get_logger(), "Action server not available");
      return;
    }

    int s1_val = 0, s2_val = 0;

    if (serial_.available()) {
      std::string line = serial_.readline(1024, "\n");
      RCLCPP_INFO(this->get_logger(), "Serial received: %s", line.c_str());

      std::regex s1_regex("s1(\\d+)");
      std::regex s2_regex("s2(\\d+)");
      std::smatch match;

      if (std::regex_search(line, match, s1_regex) && match.size() > 1)
        s1_val = std::stoi(match[1]);

      if (std::regex_search(line, match, s2_regex) && match.size() > 1)
        s2_val = std::stoi(match[1]);
    }

    DriveByIR::Goal goal_msg;
    goal_msg.s1 = s1_val;
    goal_msg.s2 = s2_val;

    RCLCPP_INFO(this->get_logger(), "Sending goal: s1=%d, s2=%d", s1_val, s2_val);

    auto send_goal_options = rclcpp_action::Client<DriveByIR>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&DriveByIRActionClient::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
      std::bind(&DriveByIRActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
      std::bind(&DriveByIRActionClient::result_callback, this, std::placeholders::_1);

    client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

  void goal_response_callback(GoalHandleDriveByIR::SharedPtr goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server");
    }
  }

  void feedback_callback(
    GoalHandleDriveByIR::SharedPtr,
    const std::shared_ptr<const DriveByIR::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(), "Feedback: %s", feedback->status.c_str());
  }

  void result_callback(const GoalHandleDriveByIR::WrappedResult &result) {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(), "Goal was canceled");
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        break;
    }

    RCLCPP_INFO(this->get_logger(), "Result: success=%s", result.result->success ? "true" : "false");
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DriveByIRActionClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
