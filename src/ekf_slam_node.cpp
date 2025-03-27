// Copyright (c) 2025 Richard Armstrong
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"

# include "ekf_localizer.hpp"
using Ekf = ekf_localizer::Ekf;

constexpr size_t QOS_HISTORY_DEPTH = 10;

class EkfNode final : public rclcpp::Node
{
public:
  EkfNode()
    : Node("ekf_localizer")
  {
    // A tag array contains all currently-detected TagDetections.
    tag_detections_sub_ = this->create_subscription<TagArray>(
      "tag_detections", QOS_HISTORY_DEPTH, [this](const TagArray::SharedPtr msg){ tag_detection_cb(msg); });

    // Odometry messages, from which we will use velocities and treat them as control.
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", QOS_HISTORY_DEPTH, [this](const nav_msgs::msg::Odometry::SharedPtr msg){ odom_cb(msg); });

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

private:
  Ekf filter_;
  time_point last_odom_time_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<TagArray>::SharedPtr tag_detections_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr& msg)
  {
    // If this is our first odom message, we can't calculate dt, so
    // save the current time and return.
    if (0 == last_odom_time_.time_since_epoch().count())
    {
      last_odom_time_ = std::chrono::steady_clock::now();
      return;
    }

    ekf_localizer::TwistCmd u{msg->twist.twist.linear.x, msg->twist.twist.angular.z};
    double dt = double_seconds(steady_clock::now() - last_odom_time_).count();
    auto next = filter_.predict(u, dt);
    last_odom_time_ = steady_clock::now();

    RCLCPP_INFO_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,  "EKF pose estimate:  ("
        << next.state(0) << ", "
        << next.state(1) << ", "
        << next.state(2) << ") ");
  }

  void tag_detection_cb(const TagArray::SharedPtr& msg)
  {
    RCLCPP_INFO_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,  "TagArray frame_id:  " << msg->header.frame_id);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<EkfNode>();
  RCLCPP_INFO(node->get_logger(), "EKF Localizer node starting.");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
