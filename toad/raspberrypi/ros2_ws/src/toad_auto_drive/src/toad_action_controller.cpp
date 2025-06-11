// ===== action_server.cpp =====
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <toad_auto_drive/action/toad_auto_drive_action.hpp>
#include <memory>
#include <thread>
#include <string>

class DriveByIRActionServer : public rclcpp::Node {
public:
  using DriveByIR = toad_auto_drive::action::ToadAutoDriveAction;
  using GoalHandleDriveByIR = rclcpp_action::ServerGoalHandle<DriveByIR>;

  DriveByIRActionServer() : Node("drive_by_ir_action_server") {
    action_server_ = rclcpp_action::create_server<DriveByIR>(
        this, "drive_by_ir",
        std::bind(&DriveByIRActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&DriveByIRActionServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&DriveByIRActionServer::handle_accepted, this, std::placeholders::_1));
  }

private:
  rclcpp_action::Server<DriveByIR>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &,
                                          std::shared_ptr<const DriveByIR::Goal>) {
    RCLCPP_INFO(this->get_logger(), "Goal received");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleDriveByIR>) {
    RCLCPP_INFO(this->get_logger(), "Goal canceled");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleDriveByIR> goal_handle) {
    std::thread{std::bind(&DriveByIRActionServer::execute, this, goal_handle)}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleDriveByIR> goal_handle) {
    auto goal = goal_handle->get_goal();
    auto result = std::make_shared<DriveByIR::Result>();
    auto feedback = std::make_shared<DriveByIR::Feedback>();

    int s1 = goal->s1;
    int s2 = goal->s2;

    std::string command;
    std::string speed = "29";
    if (s1 == 1 && s2 == 1) command = "L" + speed + "R" + speed;
    else if (s1 == 1 && s2 == 0) command = "L-" + speed + "R" + speed;
    else if (s1 == 0 && s2 == 1) command = "L"+ speed + "R-" + speed;
    else command = "L0R0";

    feedback->status = "Processed IR values: s1=" + std::to_string(s1) + ", s2=" + std::to_string(s2);
    goal_handle->publish_feedback(feedback);

    result->command = command;
    goal_handle->succeed(result);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DriveByIRActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}