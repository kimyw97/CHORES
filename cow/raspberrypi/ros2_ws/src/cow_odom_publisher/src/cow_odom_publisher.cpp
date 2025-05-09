#include "rclcpp/rclcpp.hpp"
#include "robot_monitoring/msg/robot_status.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


class OdomFromStatusNode : public rclcpp::Node {
public:
  OdomFromStatusNode()
  : Node("cow_odom_publisher"),
    x_(0.0), y_(0.0), th_(0.0),
    last_left_ticks_(0), last_right_ticks_(0), first_reading_(true)
  {
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    sub_ = this->create_subscription<robot_monitoring::msg::RobotStatus>(
      "robot_status", 10,
      std::bind(&OdomFromStatusNode::statusCallback, this, std::placeholders::_1)
    );

    last_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "Odometry from status node started.");
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

      tf2::Quaternion q;
      q.setRPY(0, 0, 0);
      q.normalize();

      // 초기 Odometry 퍼블리시
      nav_msgs::msg::Odometry odom;
      odom.header.stamp = current_time;
      odom.header.frame_id = "odom";
      odom.child_frame_id = "base_footprint";
      odom.pose.pose.position.x = 0.0;
      odom.pose.pose.position.y = 0.0;
      odom.pose.pose.orientation = tf2::toMsg(q);
      odom_pub_->publish(odom);

      // 초기 TF 퍼블리시
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = current_time;
      tf.header.frame_id = "odom";
      tf.child_frame_id = "base_footprint";
      tf.transform.translation.x = 0.0;
      tf.transform.translation.y = 0.0;
      tf.transform.translation.z = 0.0;
      tf.transform.rotation = tf2::toMsg(q);
      tf_broadcaster_->sendTransform(tf);
      return;
    }

    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    int delta_left = left_ticks - last_left_ticks_;
    int delta_right = right_ticks - last_right_ticks_;
    last_left_ticks_ = left_ticks;
    last_right_ticks_ = right_ticks;

    const double TICKS_PER_REV = 500.0;
    const double WHEEL_RADIUS = 0.033;
    const double WHEEL_BASE = 0.16;

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

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.orientation = tf2::toMsg(q);
    odom.twist.twist.linear.x = d_center / dt;
    odom.twist.twist.angular.z = d_theta / dt;
    odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = current_time;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_footprint";
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = tf2::toMsg(q);
    tf_broadcaster_->sendTransform(tf);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
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
