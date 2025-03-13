// Copyright (c) 2025 Richard Armstrong
#ifndef EKF_LOCALIZER_HPP
#define EKF_LOCALIZER_HPP
#include <iostream>
#include <string>

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "apriltag_ros_interfaces/msg/april_tag_detection.hpp"
#include "apriltag_ros_interfaces/msg/april_tag_detection_array.hpp"

// Aliases to reduce the name noise.
using TagDetection = apriltag_ros_interfaces::msg::AprilTagDetection;
using TagArray = apriltag_ros_interfaces::msg::AprilTagDetectionArray;

constexpr size_t POSE_DIMS = 3;     // x, y, theta.
constexpr size_t LM_DIMS = 2;       // x, y.
constexpr size_t LANDMARKS_KNOWN = 16;  // The number of tags defined in config/tags.yaml.
constexpr size_t STATE_DIMS = POSE_DIMS + LM_DIMS * LANDMARKS_KNOWN;

class Measurement;
typedef std::list<Measurement> MeasurementList;

class State final {
public:
  State() = default;
  ~State() = default;
  [[nodiscard]] std::string to_string() const { return std::string("State"); }
};

struct Twist
{
  Eigen::Vector3d linear;
  Eigen::Vector3d angular;
};

class Ekf final
{
public:
  Ekf() = default;
  ~Ekf() = default;

  State predict(const Twist& u, const State& prev_state)
  {
    std::cout << u.angular.size() << std::endl;
    std::cout << prev_state.to_string() << std::endl;
    return Ekf::state;
  }

  State correct(MeasurementList z)
  {
    std::cout << z.size();
    return Ekf::state;
  }

private:
  inline static auto state = State();
};

class Measurement final
{
public:
  Measurement() = default;
  ~Measurement() = default;
};

#endif //EKF_LOCALIZER_HPP
