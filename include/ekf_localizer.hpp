// Copyright (c) 2025 Richard Armstrong
#ifndef EKF_LOCALIZER_HPP
#define EKF_LOCALIZER_HPP
#include <cmath>
#include <chrono>

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"

#include "apriltag_ros_interfaces/msg/april_tag_detection.hpp"
#include "apriltag_ros_interfaces/msg/april_tag_detection_array.hpp"

// Aliases to reduce the name noise.
using double_seconds = std::chrono::duration<double>;
using steady_clock = std::chrono::steady_clock;
using time_point = steady_clock::time_point;
using Pose2D = Eigen::Vector3d;  // x, y, theta
using TagDetection = apriltag_ros_interfaces::msg::AprilTagDetection;
using TagArray = apriltag_ros_interfaces::msg::AprilTagDetectionArray;

namespace ekf_localizer {

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

struct TwistCmd
{
  TwistCmd(double linear, double angular)
    :linear(linear), angular(angular) {}
  double linear;
  double angular;
};

inline
Pose2D g(const TwistCmd& u, const Pose2D& x0, double delta_t)
{
  double v_t = u.linear;
  double omega_t = u.angular;
  double theta = x0(2);

  // The control command u represents a circular trajectory, whose radius is abs(v_t / omega_t).
  // Here we're calling the signed ratio v/omega "r" for de-cluttering convenience, even though it's not exactly /that/.
  double r = v_t / omega_t;

  // Pose delta.
  Pose2D delta_x = {
    -r * sin(theta) + r * sin(theta + (omega_t * delta_t)),
    r * cos(theta) - r * cos(theta + (omega_t * delta_t)),
    omega_t * delta_t
  };

  // Normalize theta to [-pi, pi].
  delta_x(2) = atan2(sin(delta_x(2)), cos(delta_x(2)));
  return x0 + delta_x;
}

// EKF Landmark SLAM with a fixed number of easily-identifiable landmarks.
// We manage the global EKF state vector using the Borg (aka Monostate) pattern.
class Ekf final
{
public:
  Ekf() = default;
  ~Ekf() = default;

  // Mutates the global EKF state.
  EkfState predict(const TwistCmd& u, double dt)
  {
    // Noise-free motion estimate.
    Pose2D predicted_pose = g(u, estimated_state.state(Eigen::seqN(0, 3)), dt);

    // Update current state.
    estimated_state.state(Eigen::seqN(0, 3)) = predicted_pose;

    return estimated_state;
  }

  // Mutates the global EKF state.
  EkfState correct(MeasurementList z)
  {
    // Silence unused var warnings for now.
    (void) z;
    return Ekf::estimated_state;
  }

private:
  inline static auto estimated_state = EkfState();
  int x;
};

class Measurement final
{
public:
  Measurement() = default;
  ~Measurement() = default;
};

}
#endif //EKF_LOCALIZER_HPP
