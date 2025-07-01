#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_monitoring/msg/robot_status.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>

class OdomFromStatusNode : public rclcpp::Node {
public:
  OdomFromStatusNode()
      : Node("cow_odom_publisher"), x_(0.0), y_(0.0), th_(0.0),
        last_left_ticks_(0), last_right_ticks_(0), first_reading_(true) {
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "joint_states", 10);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);

    sub_ = this->create_subscription<robot_monitoring::msg::RobotStatus>(
        "robot_status", 10,
        std::bind(&OdomFromStatusNode::statusCallback, this,
                  std::placeholders::_1));

    last_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "Odometry + IMU publisher started.");
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
    const double WHEEL_BASE = 0.165;

    double dist_per_tick = 2 * M_PI * WHEEL_RADIUS / TICKS_PER_REV;
    double d_left = delta_left * dist_per_tick;
    double d_right = delta_right * dist_per_tick;

    double d_center = (d_left + d_right) / 2.0;
    // 사실 sin(d_theta)임
    double d_theta = (d_right - d_left) / WHEEL_BASE;

    // 회전 누적 방지 로직
    if (std::abs(delta_left - delta_right) > 20) {
      d_theta = 0.0;
    }
    const int ENCODER_DEADBAND = 10;
    if (std::abs(delta_left) < ENCODER_DEADBAND) {
      delta_left = 0;
    }
    if (std::abs(delta_right) < ENCODER_DEADBAND) {
      delta_right = 0;
    }

    x_ += d_center * std::cos(th_ + d_theta / 2.0);
    y_ += d_center * std::sin(th_ + d_theta / 2.0);
    th_ += d_theta;

    tf2::Quaternion q;
    q.setRPY(0, 0, th_);
    q.normalize();

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.orientation = tf2::toMsg(q);
    odom.twist.twist.linear.x = d_center / dt;
    odom.twist.twist.angular.z = d_theta / dt;
    odom.pose.covariance[0] = 0.1;
    odom.pose.covariance[7] = 0.1;
    odom.pose.covariance[35] = 0.2;
    odom.twist.covariance[0] = 0.1;
    odom.twist.covariance[35] = 0.2;
    odom_pub_->publish(odom);

    // double left_angle = left_ticks * (2.0 * M_PI / TICKS_PER_REV);
    // double right_angle = right_ticks * (2.0 * M_PI / TICKS_PER_REV);
    // sensor_msgs::msg::JointState joint_state;
    // joint_state.header.stamp = current_time;
    // joint_state.name = {"left_wheel_joint", "right_wheel_joint",
    //                     "base_footprint_to_base_link", "base_link_to_base_scan"};
    // joint_state.position = {left_angle, right_angle, 0.0, 0.0};
    // joint_pub_->publish(joint_state);

    sensor_msgs::msg::Imu imu;
    imu.header.stamp = current_time;
    imu.header.frame_id = "base_footprint";

    // 가속도 단위 변환 (m/s²)
    double acc_scale = 9.81 / 16384.0;
    imu.linear_acceleration.x = msg->acc_x * acc_scale;
    imu.linear_acceleration.y = msg->acc_y * acc_scale;
    imu.linear_acceleration.z = msg->acc_z * acc_scale;

    // 자이로 단위 변환 (rad/s)
    double gyro_scale = (M_PI / 180.0) / 131.0;
    imu.angular_velocity.x = msg->gyro_x * gyro_scale;
    imu.angular_velocity.y = msg->gyro_y * gyro_scale;
    imu.angular_velocity.z = msg->gyro_z * gyro_scale;

    // orientation은 신뢰 불가 시 생략 (EKF와 충돌 방지)
    // tf2::Quaternion imu_q;
    // imu_q.setRPY(msg->ori_roll, msg->ori_pitch, msg->ori_yaw);
    // imu_q.normalize();
    // imu.orientation = tf2::toMsg(imu_q);

    // covariance 설정
    imu.angular_velocity_covariance[0] = 0.05;
    imu.angular_velocity_covariance[4] = 0.05;
    imu.angular_velocity_covariance[8] = 0.1;

    imu.linear_acceleration_covariance[0] = 0;
    imu.linear_acceleration_covariance[4] = 0;
    imu.linear_acceleration_covariance[8] = 0;

    imu_pub_->publish(imu);

    RCLCPP_INFO(this->get_logger(),
                "x: %.3f, y: %.3f, th: %.3f | ACC[%.2f %.2f %.2f] | GYRO[%.2f "
                "%.2f %.2f]",
                x_, y_, th_, imu.linear_acceleration.x,
                imu.linear_acceleration.y, imu.linear_acceleration.z,
                imu.angular_velocity.x, imu.angular_velocity.y,
                imu.angular_velocity.z);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
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