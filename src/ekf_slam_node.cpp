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

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("ekf_pose", QOS_HISTORY_DEPTH);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

private:
  Ekf filter_;
  time_point last_odom_time_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<TagArray>::SharedPtr tag_detections_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  void broadcast_pose_as_tf(const ekf_localizer::Pose2D& pose) const
  {
    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "map";
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

    // Publish.
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

    publish_pose(next.mean(Eigen::seqN(0, 3)), next.covariance.block<3, 3>(0, 0));
    broadcast_pose_as_tf(next.mean(Eigen::seqN(0, 3)));

    RCLCPP_INFO_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,  "EKF pose estimate:  ("
        << next.mean(0) << ", "
        << next.mean(1) << ", "
        << next.mean(2) << ") ");
  }

  // Convert a ekf_localizer::Pose2D to a ROS message and publish it.
  void publish_pose(const ekf_localizer::Pose2D& pose, const Eigen::Matrix3d& cov) const
  {
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "map";
    msg.pose.pose.position.x = pose(0);
    msg.pose.pose.position.y = pose(1);
    msg.pose.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, pose(2));
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();
    msg.pose.covariance[0] = cov(0, 0);
    msg.pose.covariance[1] = cov(0, 1);
    msg.pose.covariance[2] = cov(0, 2);
    msg.pose.covariance[3] = cov(1, 0);
    msg.pose.covariance[4] = cov(1, 1);
    msg.pose.covariance[5] = cov(1, 2);
    msg.pose.covariance[6] = cov(2, 0);
    msg.pose.covariance[7] = cov(2, 1);
    msg.pose.covariance[8] = cov(2, 2);

    pose_pub_->publish(msg);
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
