// Copyright (c) 2025 Richard Armstrong
#ifndef TYPES_HPP
#define TYPES_HPP
#include <Eigen/Core>
#include <apriltag_ros_interfaces/msg/april_tag_detection.hpp>
#include <apriltag_ros_interfaces/msg/april_tag_detection_array.hpp>

namespace ekf_localizer
{
  using TagDetection = apriltag_ros_interfaces::msg::AprilTagDetection;
  using TagArray = apriltag_ros_interfaces::msg::AprilTagDetectionArray;
  using Pose2D = Eigen::Vector3d;  // x, y, theta
}

#endif //TYPES_HPP