// Copyright (c) 2025 Richard Armstrong
#ifndef EKF_LOCALIZER_HPP
#define EKF_LOCALIZER_HPP
#include <cmath>
#include <chrono>
using duration = std::chrono::steady_clock::duration;
using time_point = std::chrono::steady_clock::time_point;

#include <Eigen/Dense>
using Pose2D = Eigen::Vector3d;  // x, y, theta

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
using MeasurementList = std::list<Measurement>;

struct EkfState {
  EkfState()
    :state(STATE_DIMS), covariance(STATE_DIMS, STATE_DIMS)
  {
    state.setZero();
    covariance.setZero();
  }

  Eigen::VectorXd state;
  Eigen::MatrixXd covariance;
  time_point timestamp;
};

struct Twist
{
  double linear;
  double angular;
};

inline
Pose2D g(const Twist& u, const Pose2D& x0, double delta_t)
{
  double v_t = u.linear;
  double omega_t = u.angular;
  double theta = x0(2);

  // The control command u represents a circular trajectory, whose radius is abs(v_t / omega_t).
  // Here we're calling the signed ration v/omega "r" for convenience, even though it's not exactly /that/.
  double r = v_t / omega_t;

  // Pose delta.
  Pose2D delta_x{
    -r * sin(theta) + r * sin(theta + (omega_t * delta_t)),
    r * cos(theta) - r * cos(theta + (omega_t * delta_t)),
    omega_t * delta_t
  };

  return x0 + delta_x;
}

class Ekf final
{
public:
  Ekf() = default;
  ~Ekf() = default;

  EkfState predict(const Twist& u, duration dt)
  {
    // Silence unused var warnings for now.
    (void) u;
    (void) dt;

    return Ekf::state;
  }

  EkfState correct(MeasurementList z)
  {
    // Silence unused var warnings for now.
    (void) z;
    return Ekf::state;
  }

private:
  inline static auto state = EkfState();

};

class Measurement final
{
public:
  Measurement() = default;
  ~Measurement() = default;
};

#endif //EKF_LOCALIZER_HPP
