#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "toad_auto_drive/action/toad_auto_drive_action.hpp"

#include <std_msgs/msg/bool.hpp>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <string>
#include <thread>
#include <utility>

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

    openSerial("/dev/ttyUSB0", B115200);
  }

private:
  rclcpp_action::Server<DriveByIR>::SharedPtr action_server_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_trash_sub_;
  int serial_fd_;
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
    char buffer[16];
    int n = read(serial_fd_, buffer, sizeof(buffer) - 1);
    if (n > 0) {
      buffer[n] = '\0';
      std::string data(buffer);

      size_t first = data.find("s");
      size_t second = data.find("s", first + 1);

      if (first != std::string::npos && second != std::string::npos && second + 2 < data.size()) {
        int left = data[first + 1] - '0';
        int right = data[second + 1] - '0';
        RCLCPP_INFO(this->get_logger(), "recive s1%ds2%d", left, right);
        return {left, right};
      }
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

  void openSerial(const std::string &port, int baudrate) {
    serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY);
    if (serial_fd_ == -1) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s", port.c_str());
      return;
    }

    struct termios tty;
    tcgetattr(serial_fd_, &tty);
    cfsetispeed(&tty, baudrate);
    cfsetospeed(&tty, baudrate);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tcsetattr(serial_fd_, TCSANOW, &tty);
  }

  void writeSerial(const std::string &cmd) {
    if (serial_fd_ != -1) {
      write(serial_fd_, cmd.c_str(), cmd.size());
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
