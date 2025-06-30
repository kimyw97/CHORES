#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <toad_auto_drive/action/toad_auto_drive_action.hpp>
#include <serial/serial.h>
#include <regex>
#include <chrono>

using namespace std::chrono_literals;

class DriveByIRActionClient : public rclcpp::Node {
public:
  using DriveByIR = toad_auto_drive::action::ToadAutoDriveAction;
  using GoalHandleDriveByIR = rclcpp_action::ClientGoalHandle<DriveByIR>;

  DriveByIRActionClient() : Node("drive_by_ir_action_client") {
    client_ptr_ = rclcpp_action::create_client<DriveByIR>(this, "drive_by_ir");

    try {
      serial_.setPort("/dev/ttyUSB0");
      serial_.setBaudrate(115200);
      serial::Timeout to = serial::Timeout::simpleTimeout(100);
      serial_.setTimeout(to);
      serial_.open();
    } catch (serial::IOException &e) {
      RCLCPP_ERROR(this->get_logger(), "Unable to open serial port");
    }

    timer_ = this->create_wall_timer(500ms, std::bind(&DriveByIRActionClient::read_and_send_goal, this));
  }

private:
  rclcpp_action::Client<DriveByIR>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  serial::Serial serial_;

  void read_and_send_goal() {
    if (!serial_.available()) return;

    std::string data = serial_.readline(1024, "\n");
    std::smatch match;
    std::regex s1_regex("s1(\\d+)");
    std::regex s2_regex("s2(\\d+)");

    int s1 = 0, s2 = 0;
    if (std::regex_search(data, match, s1_regex)) s1 = std::stoi(match[1]);
    if (std::regex_search(data, match, s2_regex)) s2 = std::stoi(match[1]);

    if (!client_ptr_->wait_for_action_server(2s)) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available");
      return;
    }

    auto goal_msg = DriveByIR::Goal();
    goal_msg.s1 = s1;
    goal_msg.s2 = s2;

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
      RCLCPP_ERROR(this->get_logger(), "Goal rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted");
    }
  }

  void feedback_callback(GoalHandleDriveByIR::SharedPtr,
                         const std::shared_ptr<const DriveByIR::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(), "Feedback: %s", feedback->status.c_str());
  }

  void result_callback(const GoalHandleDriveByIR::WrappedResult &result) {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(this->get_logger(), "Command: %s", result.result->command.c_str());
      serial_.write(result.result->command + "\n");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Action failed or canceled");
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DriveByIRActionClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}