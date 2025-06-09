#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <toad_auto_drive/action/toad_auto_drive_action.hpp>
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

    this->timer_ = this->create_wall_timer(
      1s, std::bind(&DriveByIRActionClient::send_goal, this));
  }

private:
  rclcpp_action::Client<DriveByIR>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  void send_goal() {
    this->timer_->cancel();

    if (!client_ptr_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
      return;
    }

    auto goal_msg = DriveByIR::Goal();  // 필요시 goal 내부 값 설정
    RCLCPP_INFO(this->get_logger(), "Sending goal");

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
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
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
    rclcpp::shutdown();
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DriveByIRActionClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
