#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <toad_auto_drive/action/toad_auto_drive_action.hpp>
#include <serial/serial.h>
#include <string>
#include <thread>

class DriveByIRActionServer : public rclcpp::Node {
public:
  using DriveByIR = toad_auto_drive::action::ToadAutoDriveAction;
  using GoalHandleDriveByIR = rclcpp_action::ServerGoalHandle<DriveByIR>;

  DriveByIRActionServer()
      : Node("drive_by_ir_action_server") {

    action_server_ = rclcpp_action::create_server<DriveByIR>(
        this,
        "drive_by_ir",
        std::bind(&DriveByIRActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&DriveByIRActionServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&DriveByIRActionServer::handle_accepted, this, std::placeholders::_1));

    try {
      serial_.setPort("/dev/ttyUSB0");  // 환경에 따라 변경 필요
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
  serial::Serial serial_;

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

    int s1 = goal_handle->get_goal()->s1;
    int s2 = goal_handle->get_goal()->s2;

    std::string cmd;

    if (s1 == 1 && s2 == 1) {
      RCLCPP_INFO(this->get_logger(), "Go Forward");
      cmd = "L30R30\n";
    } else if (s1 == 0 && s2 == 1) {
      RCLCPP_INFO(this->get_logger(), "Turn Right");
      cmd = "L30R-30\n";
    } else if (s1 == 1 && s2 == 0) {
      RCLCPP_INFO(this->get_logger(), "Turn Left");
      cmd = "L-30R30\n";
    } /*else {
      RCLCPP_INFO(this->get_logger(), "Stop");
      cmd = "L0R0\n";
    }
*/
    writeSerial(cmd);

    feedback->status = "IR received: s1=" + std::to_string(s1) +
                       ", s2=" + std::to_string(s2) +
                       ", command=" + cmd;
    goal_handle->publish_feedback(feedback);

    rclcpp::sleep_for(std::chrono::milliseconds(500));

    stop();

    result->success = true;
    goal_handle->succeed(result);
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
