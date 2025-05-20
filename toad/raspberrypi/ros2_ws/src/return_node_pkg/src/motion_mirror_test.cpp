#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "motion_interfaces/msg/serial_cmd.hpp"
#include <chrono>
#include <string>
#include <vector>

using namespace std::chrono;
using std::placeholders::_1;
using motion_interfaces::msg::SerialCmd;

struct MotionEntry {
  std::string command;
  double duration;
  double stop_duration;

  MotionEntry(std::string cmd, double dur, double stop = 0.0)
    : command(std::move(cmd)), duration(dur), stop_duration(stop) {}

  std::string reversed_command() const {
    try {
      size_t l_idx = command.find("L");
      size_t r_idx = command.find("R");\
      int l_val = std::stoi(command.substr(l_idx + 1, r_idx - l_idx - 1));
      int r_val = std::stoi(command.substr(r_idx + 1));
      return "L" + std::to_string(-l_val) + "R" + std::to_string(-r_val);
    } catch (...) {
      return "L0R0";
    }
  }
};

class ReturnNode : public rclcpp::Node {
public:
  ReturnNode() : Node("return_node"), replay_step_(0) {
    cmd_pub_ = this->create_publisher<SerialCmd>("/serial_cmd", 10);
    return_signal_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/return_signal", 10, std::bind(&ReturnNode::trigger_replay, this, _1));
    motion_cmd_sub_ = this->create_subscription<SerialCmd>(
      "/motion_cmd", 10, std::bind(&ReturnNode::record_command, this, _1));
    pickup_done_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/pickup_done", 10, std::bind(&ReturnNode::record_stop_duration_on_pickup_done, this, _1));
    throw_done_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/throw_done", 10, std::bind(&ReturnNode::on_throw_done, this, _1));

    throw_start_pub_ = this->create_publisher<std_msgs::msg::Bool>("/start_throw", 10);
    return_done_pub_ = this->create_publisher<std_msgs::msg::Bool>("/return_done", 10);

    last_time_ = now();
  }

private:
  rclcpp::Publisher<SerialCmd>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr throw_start_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr return_done_pub_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr return_signal_sub_;
  rclcpp::Subscription<SerialCmd>::SharedPtr motion_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pickup_done_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr throw_done_sub_;

  std::vector<MotionEntry> motion_cmd_;
  std::string last_cmd_;
  time_point<steady_clock> last_time_;
  size_t replay_step_;

  time_point<steady_clock> now() { return steady_clock::now(); }

  void send_command(const std::string& cmd_str) {
    SerialCmd msg;
    msg.command = cmd_str;
    cmd_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "[SEND] %s", cmd_str.c_str());
  }

  void record_command(const SerialCmd::SharedPtr msg) {
    auto now_time = now();
    double duration = duration_cast<duration<double>>(now_time - last_time_).count();

    if (!last_cmd_.empty() && last_cmd_ == "L0R0" && !motion_cmd_.empty()) {
      motion_cmd_.back().stop_duration = duration;
    }

    motion_cmd_.emplace_back(msg->command, duration);
    last_time_ = now_time;
    last_cmd_ = msg->command;
    RCLCPP_INFO(this->get_logger(), "[RECORD] %s (%.2fs)", msg->command.c_str(), duration);
  }

  void record_stop_duration_on_pickup_done(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data && !motion_cmd_.empty()) {
      double stop_duration = duration_cast<duration<double>>(now() - last_time_).count();
      motion_cmd_.back().stop_duration = stop_duration;
      RCLCPP_INFO(this->get_logger(), "[RECORD] 팔 동작 정지 시간 기록: %.2fs", stop_duration);
    }
  }

  void trigger_replay(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
      RCLCPP_INFO(this->get_logger(), "[REPLAY] Motion Mirror Return 시작");
      replay_step_ = 0;
      execute_next();
    }
  }

  /* 수정해야 하는 부분 */
  void execute_next() {
    if (replay_step_ >= motion_cmd_.size()) {
      send_command("L0R0");
      RCLCPP_INFO(this->get_logger(), "[REPLAY] 복귀 완료. 기록 초기화됨");
      motion_cmd_.clear();
      publish_return_done();
      return;
    }

    size_t idx = motion_cmd_.size() - 1 - replay_step_;
    auto& entry = motion_cmd_[idx];

    send_command(entry.reversed_command());
    rclcpp::sleep_for(std::chrono::duration<double>(entry.duration));

    if (entry.stop_duration > 0.0) {
      send_command("L0R0");
      RCLCPP_INFO(this->get_logger(), "[REPLAY] 정지 유지 %.2fs", entry.stop_duration);
      rclcpp::sleep_for(std::chrono::duration<double>(entry.stop_duration));

      if (replay_step_ == 0) {
        RCLCPP_INFO(this->get_logger(), "[REPLAY] 소 앞 도착 → 투입 명령 송신");
        publish_start_throw();
        return;
      }
    }

    ++replay_step_;
    execute_next();
  }

  void on_throw_done(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
      RCLCPP_INFO(this->get_logger(), "[REPLAY] 쓰레기 투입 완료 → 마지막 주차 시작");
      ++replay_step_;
      execute_next();
    }
  }

  void publish_start_throw() {
    std_msgs::msg::Bool msg;
    msg.data = true;
    throw_start_pub_->publish(msg);
  }

  void publish_return_done() {
    std_msgs::msg::Bool msg;
    msg.data = true;
    return_done_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "[REPLAY] 전체 복귀 완료 → return_done 송신");
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ReturnNode>());
  rclcpp::shutdown();
  return 0;
}
