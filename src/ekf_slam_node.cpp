// Copyright (c) 2025 Richard Armstrong
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
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

  void broadcast_pose_as_tf(const Pose2D& pose) const
  {
    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link_ekf";

    t.transform.translation.x = pose(0);
    t.transform.translation.y = pose(1);
    t.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, pose(2));
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    // Send the transformation
    tf_broadcaster_->sendTransform(t);
  }

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

    broadcast_pose_as_tf(next.state(Eigen::seqN(0, 3)));

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
