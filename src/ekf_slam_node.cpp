// Copyright (c) 2025 Richard Armstrong
#include <format>
#include <mutex>
#include<nav_msgs/msg/odometry.hpp>
#include<tf2_ros/transform_broadcaster.h>
#include<tf2_ros/buffer.h>
#include<tf2_ros/transform_listener.h>
#include<tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include<geometry_msgs/msg/point_stamped.hpp>
#include<geometry_msgs/msg/transform_stamped.hpp>
#include<visualization_msgs/msg/marker_array.hpp>

#include "ekf_localizer.h"
#include "measurement.h"
#include "utility.hpp"
using Ekf = ekf_localizer::Ekf;

constexpr size_t QOS_HISTORY_DEPTH = 10;
constexpr double LM_MARKER_SCALE = 0.25;  // RViz markers representing landmarks.

const char* BASE_FRAME = "base_link";
const char* SENSOR_FRAME = "oakd_rgb_camera_optical_frame";

class EkfNode final : public rclcpp::Node
{
public:
  EkfNode()
    : Node("ekf_localizer")
  {
    // A tag array contains all currently-detected TagDetections.
    tag_detections_sub_ = this->create_subscription<ekf_localizer::TagArray>(
      "tag_detections", QOS_HISTORY_DEPTH, [this](const ekf_localizer::TagArray::SharedPtr msg){ tag_detection_cb(msg); });

    // Odometry messages, from which we will use velocities and treat them as control.
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", QOS_HISTORY_DEPTH, [this](const nav_msgs::msg::Odometry::SharedPtr msg){ odom_cb(msg); });

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("ekf_pose", QOS_HISTORY_DEPTH);
    lm_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("landmarks", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

private:
  Ekf filter_;

  // Odom messages.
  bool first_odom_msg_ = true;
  rclcpp::Time last_odom_time_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Tag detection messages.
  rclcpp::Subscription<ekf_localizer::TagArray>::SharedPtr tag_detections_sub_;
  std::queue<ekf_localizer::TagArray> tag_msg_queue_;
  std::mutex tag_msg_queue_mutex_;

  // Publishers.
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr lm_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // tf listener.
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

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

  /**
   * @brief Odometry message handler that drives the predict/correct cycle.
   *
   * Our odometry updates are the highest-rate message, so we'll use them as our "clock signal".
   *
   * @param[in] msg  nav_msgs::msg::Odometry::SharedPtr&, a velocity command with
   * linear and angular components.
   */
  // TODO: we do too much work in this callback; it should just push the message into a queue and return.
  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr& msg)
  {
    rclcpp::Time msg_timestamp = msg->header.stamp;

    // If this is our first odom message, we can't calculate dt, so
    // save the current time and return.
    if (first_odom_msg_)
    {
      last_odom_time_ = msg_timestamp;
      first_odom_msg_ = false;
      RCLCPP_INFO(this->get_logger(), "EkfNode::odom_cb(): first odom message received.");
      return;
    }

    // Read-in our velocity command.
    const ekf_localizer::TwistCmd u{msg->twist.twist.linear.x, msg->twist.twist.angular.z};

    // Calculate the time delta and predict our next state based solely on our motion model.
    double dt = (msg_timestamp - last_odom_time_).seconds();
    auto predicted_state = filter_.predict(filter_.get_state(), u, dt);
    last_odom_time_ = msg_timestamp;

    // TODO: collapse this to a function.
    // Check for a new set of tag observations.
    ekf_localizer::TagArray latest_tag_array;
    bool new_observation = false;
    {
      std::scoped_lock lock(tag_msg_queue_mutex_);
      if (!tag_msg_queue_.empty())
      {
        if (!tag_msg_queue_.front().detections.empty())
        {
          // Make a copy of the latest, so we can let go of our mutex.
          latest_tag_array = tag_msg_queue_.front();
          new_observation = true;
        }
        tag_msg_queue_.pop();
      }
    }

    // TODO: collapse this to a function.
    if (new_observation)
    {
      // Use tf to transform detections from the sensor frame into the base frame.
      std::string target_frame = BASE_FRAME;
      std::string source_frame = SENSOR_FRAME;
      ekf_localizer::TagArray tag_array_base_frame;  // latest tag detection array, base frame.
      for (const ekf_localizer::TagDetection& d: latest_tag_array.detections)
      {
        // Position of the detected tag, in the sensor frame.
        geometry_msgs::msg::PointStamped p_sensor;
        p_sensor.header.frame_id = source_frame;
        p_sensor.header.stamp = this->get_clock()->now();
        p_sensor.point = d.pose.pose.pose.position;

        // Transform the position to base frame. We assume that the sensor->base
        // transformation is a static transform and always available.
        geometry_msgs::msg::PointStamped p_base_frame;
        try
        {
          p_base_frame = tf_buffer_->transform(p_sensor, target_frame);
        }
        catch (tf2::TransformException& ex)
        {
          // Nothing we can do at this point.
          RCLCPP_ERROR(this->get_logger(), "EkfNode::odom_cb(): transform failed: %s", ex.what());
          return;
        }

        // Use the transformed position to create a new detection and save it.
        ekf_localizer::TagDetection d_base_frame;
        d_base_frame.id =  d.id;
        d_base_frame.pose.pose.pose.position = p_base_frame.point;
        tag_array_base_frame.detections.push_back(d_base_frame);
      }

      // Do a correction using the latest observation.
      std::list<ekf_localizer::Measurement> z_k = to_measurements(tag_array_base_frame);
      predicted_state.init_new_landmarks(z_k);
      filter_.correct(predicted_state, z_k);

      // Notify on stdout, for debugging purposes.
      std::stringstream ss;
      ss << "odom_cb(): observed tag ids:  ";
      for (const ekf_localizer::TagDetection& d: tag_array_base_frame.detections)
      {
        ss << d.id[0] << ", ";
      }
      RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, ss.str());
    }
    else
    {
      filter_.correct(predicted_state, std::list<ekf_localizer::Measurement>());
    }

    publish_pose(filter_.get_state().mean.head(ekf_localizer::POSE_DIMS), filter_.get_state().covariance.block<3, 3>(0, 0));
    publish_landmarks(filter_.get_landmarks());
    broadcast_pose_as_tf(filter_.get_state().mean.head(ekf_localizer::POSE_DIMS));
  }

  void publish_landmarks(const Eigen::Matrix<double, ekf_localizer::LANDMARKS_KNOWN, ekf_localizer::LM_DIMS>& landmarks)
  {
    visualization_msgs::msg::MarkerArray marker_array;
    for (uint i = 0; i < ekf_localizer::LANDMARKS_KNOWN; ++i)
    {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = this->now();
      marker.ns = "landmarks";
      marker.id = i;
      marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.text = std::format("{}", i);
      marker.pose.position.x = landmarks.row(i)[0];
      marker.pose.position.y = landmarks.row(i)[1];
      marker.pose.position.z = LM_MARKER_SCALE;
      marker.scale.x = LM_MARKER_SCALE;
      marker.scale.y = LM_MARKER_SCALE;
      marker.scale.z = LM_MARKER_SCALE;
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0f;
      marker.lifetime = rclcpp::Duration(0, 0);;
      marker_array.markers.push_back(marker);
    }
    lm_pub_->publish(marker_array);

  }

  // Convert a ekf_localizer::Pose2D to a ROS message and publish it.
  void publish_pose(const ekf_localizer::Pose2D& pose, const Eigen::Matrix3d& cov) const
  {
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "map";

    // Pose.
    msg.pose.pose.position.x = pose(0);
    msg.pose.pose.position.y = pose(1);
    msg.pose.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, pose(2));
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();

    // 2D position covariances, writing to 2x2 block from (0, 0).
    msg.pose.covariance[0] = cov(0, 0);
    msg.pose.covariance[1] = cov(0, 1);
    msg.pose.covariance[6] = cov(1, 0);
    msg.pose.covariance[7] = cov(1, 1);

    // Orientation (theta) covariance (rotation about z only).
    msg.pose.covariance[35] = cov(2, 2);
    pose_pub_->publish(msg);
  }

  void tag_detection_cb(const ekf_localizer::TagArray::SharedPtr& msg)
  {
    // Lock and put a copy of the message in the queue.
    {
      std::scoped_lock lock(tag_msg_queue_mutex_);
      tag_msg_queue_.emplace(*msg);
    }

    // Warn if tag bundles are present.
    for (const ekf_localizer::TagDetection& d: msg->detections)
    {
      if (d.id.size() > 1)
      {
        RCLCPP_WARN_STREAM_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,  "Warning: tag bundle detected. Tag bundles are not supported "
          "and may result in erroneous measurements.");
      }
    }
  }
};

class DeadReckoner final : public rclcpp::Node
{
public:
  DeadReckoner()
    : Node("dead_reckoner")
  {
    // Odometry messages, from which we will use velocities and treat them as control.
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", QOS_HISTORY_DEPTH, [this](const nav_msgs::msg::Odometry::SharedPtr msg){ odom_cb(msg); });

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("ekf_pose", QOS_HISTORY_DEPTH);
  }

  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr& msg)
  {
    ekf_localizer::TwistCmd u(0., 0.);
    double dt;
    ekf_localizer::Pose2D x0;
    ekf_localizer::Pose2D x1 = ekf_localizer::g(u, x0, dt);
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<EkfNode>();
  RCLCPP_INFO(node->get_logger(), "EKF Localizer node started.");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
