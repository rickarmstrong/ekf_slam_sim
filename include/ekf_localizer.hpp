// Copyright (c) 2025 Richard Armstrong
#ifndef EKF_LOCALIZER_HPP
#define EKF_LOCALIZER_HPP
#include <cmath>
#include <chrono>
#include <limits>

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"

#include "apriltag_ros_interfaces/msg/april_tag_detection.hpp"
#include "apriltag_ros_interfaces/msg/april_tag_detection_array.hpp"

// Aliases to reduce the name noise.
using double_seconds = std::chrono::duration<double>;
using steady_clock = std::chrono::steady_clock;
using time_point = steady_clock::time_point;
using TagDetection = apriltag_ros_interfaces::msg::AprilTagDetection;
using TagArray = apriltag_ros_interfaces::msg::AprilTagDetectionArray;

namespace ekf_localizer {
using Pose2D = Eigen::Vector3d;  // x, y, theta

constexpr size_t POSE_DIMS = 3;     // x, y, theta.
constexpr size_t LM_DIMS = 2;       // x, y.
constexpr size_t LANDMARKS_KNOWN = 16;  // The number of tags defined in config/tags.yaml.
constexpr size_t STATE_DIMS = POSE_DIMS + LM_DIMS * LANDMARKS_KNOWN;

// Process noise params, expressed as control noise. Will be mapped to state space at runtime.
constexpr double CMD_VEL_LIN_STDEV_MS = 0.1;  // One-sigma Linear velocity error, meters/s.
constexpr double CMD_VEL_ANG_STDEV_RADS = 0.1; // One-sigma angular velocity error, radians/s.
const Eigen::Matrix2d M_t{
  {CMD_VEL_LIN_STDEV_MS * CMD_VEL_LIN_STDEV_MS, 0.0},
  {0.0, CMD_VEL_LIN_STDEV_MS * CMD_VEL_LIN_STDEV_MS},
};

class Measurement;
using MeasurementList = std::list<Measurement>;

struct EkfState {
  EkfState()
    :mean(STATE_DIMS), covariance(STATE_DIMS, STATE_DIMS)
  {
    mean.setZero();
    covariance.setZero();
  }

  Eigen::VectorXd mean;
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
  double omega_t = std::fmax(u.angular, std::numeric_limits<double>::epsilon());
  double theta = x0(2);

  // // The control command u represents a circular trajectory, whose radius is abs(v_t / omega_t).
  // // Here we're calling the signed ratio v/omega "r" for de-cluttering convenience, even though it's not exactly /that/.
  double r = v_t / omega_t;

  // Pose delta.
  Pose2D delta_x = {
    -r * sin(theta) + r * sin(theta + (omega_t * delta_t)),
    r * cos(theta) - r * cos(theta + (omega_t * delta_t)),
    omega_t * delta_t
  };

  // Normalize theta to [-pi, pi].
  Pose2D x_next = x0 + delta_x;
  x_next(2) = atan2(sin(x_next(2)), cos(x_next(2)));
  return x_next;
}

// Return the 3x3 Jacobian of the motion model function g().
inline
Eigen::Matrix3d G_t_x(const TwistCmd& u, const Pose2D& x0, double delta_t)
{
  double v_t = u.linear;
  double omega_t = std::fmax(u.angular, std::numeric_limits<double>::epsilon());
  double theta = x0(2);

  // The control command u represents a circular trajectory, whose radius is abs(v_t / omega_t).
  // Here we're calling the signed ratio v/omega "r" for de-cluttering convenience, even though it's not exactly /that/.
  double r = v_t / omega_t;

  Eigen::Matrix3d G_t_x;
  G_t_x <<  1.0, 0.0, -r * cos(theta) + r * cos(theta + (omega_t * delta_t)),
            0.0, 1.0, -r * sin(theta) + r * sin(theta + (omega_t * delta_t)),
            0.0, 0.0, 1.0;
  return G_t_x;
}

inline
Eigen::Matrix<double, 3, 2> V_t_x(const TwistCmd& u, const Pose2D& x0, double delta_t)
{
  double v_t = u.linear;
  double w_t = std::fmax(u.angular, std::numeric_limits<double>::epsilon());
  double theta = x0(2);

  double s_t = sin(theta);
  double c_t = cos(theta);
  double s_w_t = sin(theta + (w_t * delta_t));
  double c_w_t = cos(theta + (w_t * delta_t));

  double V_0_0 = (1. / w_t) + (-s_t + s_w_t);
  double V_0_1 = (v_t / w_t * w_t) * s_t  - s_w_t + (v_t / w_t) * c_w_t * delta_t;
  double V_1_0 = (1. / w_t) * (c_t - c_w_t);
  double V_1_1 = -(v_t / w_t * w_t) * c_t - c_w_t + (v_t / w_t) * s_w_t * delta_t;
  Eigen::MatrixXd V_t(3, 2);
  V_t <<  V_0_0, V_0_1,
          V_1_0, V_1_1,
          0.0, delta_t;
  return V_t;
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
    // Current pose estimate.
    auto x0 = estimated_state_.mean(Eigen::seqN(0, 3));

    // Noise-free motion estimate.
    Pose2D predicted_pose = g(u, x0, dt);

    // Update pose.
    estimated_state_.mean(Eigen::seqN(0, 3)) = predicted_pose;

    // Update pose covariance.
    Eigen::Matrix<double, STATE_DIMS, STATE_DIMS> G_t;
    G_t.setIdentity();
    G_t.block<3, 3>(0, 0) = G_t_x(u, x0, dt);
    auto V_t = V_t_x(u, x0, dt); // 3x2 matrix that maps control space noise to state space.
    auto R_t = V_t * M_t * V_t.transpose();
    Eigen::Matrix<double, STATE_DIMS, STATE_DIMS> cov_next = G_t * estimated_state_.covariance * G_t.transpose();
    cov_next.block<3, 3>(0, 0) += R_t;  // Only update pose covariance.
    estimated_state_.covariance = cov_next;

    return estimated_state_;
  }

  // Mutates the global EKF state.
  EkfState correct(MeasurementList z)
  {
    // Silence unused var warnings for now.
    (void) z;
    return Ekf::estimated_state_;
  }

private:
  inline static auto estimated_state_ = EkfState();
};

class Measurement final
{
public:
  Measurement() = default;
  ~Measurement() = default;
};

}
#endif //EKF_LOCALIZER_HPP
