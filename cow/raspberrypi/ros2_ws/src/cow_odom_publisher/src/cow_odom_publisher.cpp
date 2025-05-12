#include "rclcpp/rclcpp.hpp"
#include "robot_monitoring/msg/robot_status.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>

class OdomFromStatusNode : public rclcpp::Node {
public:
  OdomFromStatusNode()
  : Node("cow_odom_publisher"),
    x_(0.0), y_(0.0), th_(0.0),
    last_left_ticks_(0), last_right_ticks_(0), first_reading_(true)
  {
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    sub_ = this->create_subscription<robot_monitoring::msg::RobotStatus>(
      "robot_status", 10,
      std::bind(&OdomFromStatusNode::statusCallback, this, std::placeholders::_1)
    );

    last_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "Odometry publisher started (TF removed, joint_states added).");
  }

private:
  void statusCallback(const robot_monitoring::msg::RobotStatus::SharedPtr msg) {
    int left_ticks = msg->left_encoder;
    int right_ticks = msg->right_encoder;

    rclcpp::Time current_time = this->now();

    if (first_reading_) {
      last_left_ticks_ = left_ticks;
      last_right_ticks_ = right_ticks;
      last_time_ = current_time;
      first_reading_ = false;
      return;
    }

    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    int delta_left = left_ticks - last_left_ticks_;
    int delta_right = right_ticks - last_right_ticks_;
    last_left_ticks_ = left_ticks;
    last_right_ticks_ = right_ticks;

    const double TICKS_PER_REV = 660.0;
    const double WHEEL_RADIUS = 0.045;
    const double WHEEL_BASE = 0.30;

    double dist_per_tick = 2 * M_PI * WHEEL_RADIUS / TICKS_PER_REV;
    double d_left = delta_left * dist_per_tick;
    double d_right = delta_right * dist_per_tick;

    double d_center = (d_left + d_right) / 2.0;
    double d_theta = (d_right - d_left) / WHEEL_BASE;

    x_ += d_center * std::cos(th_ + d_theta / 2.0);
    y_ += d_center * std::sin(th_ + d_theta / 2.0);
    th_ += d_theta;

    tf2::Quaternion q;
    q.setRPY(0, 0, th_);
    q.normalize();

    // Publish Odometry
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.orientation = tf2::toMsg(q);
    odom.twist.twist.linear.x = d_center / dt;
    odom.twist.twist.angular.z = d_theta / dt;
    for (int i = 0; i < 36; ++i) {
      odom.pose.covariance[i] = 0.0;
      odom.twist.covariance[i] = 0.0;
    }
    odom.pose.covariance[0] = 0.01;   // x
    odom.pose.covariance[7] = 0.01;   // y
    odom.pose.covariance[35] = 0.05;  // yaw

    odom.twist.covariance[0] = 0.01;  // vx
    odom.twist.covariance[35] = 0.05; // vyaw
    odom_pub_->publish(odom);

    // Publish JointState (include fixed joints too)
    double left_angle = left_ticks * (2.0 * M_PI / TICKS_PER_REV);
    double right_angle = right_ticks * (2.0 * M_PI / TICKS_PER_REV);

    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = current_time;
    joint_state.name = {
      "left_wheel_joint",
      "right_wheel_joint",
      "base_footprint_to_base_link",
      "base_link_to_laser"
    };
    joint_state.position = {
      left_angle,
      right_angle,
      0.0,  // fixed joint
      0.0   // fixed joint
    };
    joint_pub_->publish(joint_state);

    // Logging
    RCLCPP_INFO(this->get_logger(),
      "x: %.3f, y: %.3f, th: %.3f | L_angle: %.2f rad, R_angle: %.2f rad",
      x_, y_, th_, left_angle, right_angle);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Subscription<robot_monitoring::msg::RobotStatus>::SharedPtr sub_;

  rclcpp::Time last_time_;
  double x_, y_, th_;
  int last_left_ticks_, last_right_ticks_;
  bool first_reading_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomFromStatusNode>());
  rclcpp::shutdown();
  return 0;
}
