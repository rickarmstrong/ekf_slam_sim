// Copyright (c) 2025 Richard Armstrong
#ifndef EKF_LOCALIZER_HPP
#define EKF_LOCALIZER_HPP
#include <cmath>
#include <chrono>
#include <list>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include "ekf_state.h"
#include "measurement.h"

namespace ekf_localizer {

// Process noise params, expressed as control noise. Will be mapped to state space at runtime.
constexpr double CMD_VEL_LIN_STDEV_MS = 0.001;  // One-sigma Linear velocity error, meters/s.
constexpr double CMD_VEL_ANG_STDEV_RADS = 0.01; // One-sigma angular velocity error, radians/s.
const Eigen::Matrix2d M_t{
  {pow(CMD_VEL_LIN_STDEV_MS, 2.), 0.0},
  {0.0, pow(CMD_VEL_ANG_STDEV_RADS, 2.)},
};

// Measurement noise, stdev of cartesian (x, y) measurement noise.
constexpr double MEASUREMENT_STDEVX_M = 0.001;
constexpr double MEASUREMENT_STDEVY_M = 0.001;
const Eigen::Matrix2d Q_t{
  {pow(MEASUREMENT_STDEVX_M, 2.), 0.0},
  {0.0, pow(MEASUREMENT_STDEVY_M, 2.)},
};

// We frequently find ourselves dividing by angular velocity, and
// we frequently find ourselves moving in a straight line, i.e.
// with an angular velocity of zero. We mitigate this problem by
// setting a small minimum angular velocity.
constexpr double MIN_ANG_VEL = 1e-4;

// Need to know this for scaling covariance with velocity.
constexpr double MAX_VEL_X = 0.25;  // m/s.

using SensorJacobian = Eigen::Matrix<double, LM_DIMS, POSE_DIMS + LM_DIMS>;

struct TwistCmd
{
  TwistCmd(double linear, double angular)
    :linear(linear), angular(angular) {}
  double linear;
  double angular;
};

// TODO: need a docstring here.
// TODO: have a look at Gazebo's implementation here: https://github.com/gazebosim/gz-math/blob/gz-math9/src/DiffDriveOdometry.cc#L226.
// Might handle low angular velocity better.
inline
Pose2D g(const TwistCmd& u, const Pose2D& x0, double delta_t)
{
  double v_t = u.linear;
  double w_t = u.angular;
  double theta = x0(2);

  Pose2D delta_x;

  // Straight-line motion (limiting case as w_t -> 0).
  if (fabs(w_t) < MIN_ANG_VEL) {
    delta_x = {
      v_t * cos(theta) * delta_t,
      v_t * sin(theta) * delta_t,
      0.0  // No rotation
    };
  }
  // Curved motion.
  else {
    double r = v_t / w_t;
    delta_x = {
      -r * sin(theta) + r * sin(theta + w_t * delta_t),
       r * cos(theta) - r * cos(theta + w_t * delta_t),
       w_t * delta_t
    };
  }

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
  double omega_t = u.angular;
  double theta = x0(2);

  // Avoid div/zero for straight-line trajectories or no motion.
  if (fabs(omega_t) < MIN_ANG_VEL)
  {
    omega_t = MIN_ANG_VEL;
  }

  // The control command u represents a circular trajectory, whose radius is abs(v_t / omega_t).
  // Here we're calling the signed ratio v/omega "r" for de-cluttering convenience, even though it's not exactly /that/.
  double r = v_t / omega_t;

  Eigen::Matrix3d G_t_x;
  G_t_x <<  1.0, 0.0, -r * cos(theta) + r * cos(theta + (omega_t * delta_t)),
            0.0, 1.0, -r * sin(theta) + r * sin(theta + (omega_t * delta_t)),
            0.0, 0.0, 1.0;
  return G_t_x;
}

/**
 * @brief Return the Jacobian of the (cartesian) sensor model h(x), w.r.t the robot pose and a landmark.
 * @param x_t pose of the robot at time t.
 * @param lm  position of a landmark.
 * @return Matrix with dimensions (LM_DIMS) x (POSE_DIMS + LM_DIMS).
 */
inline
SensorJacobian H_i_t(Pose2D x_t, Eigen::Vector2d lm)
{
  double theta = x_t(2);
  double H_0_0 = -cos(theta);
  double H_0_1 = -sin(theta);
  double H_0_2 = -lm[0] * sin(theta) + lm[1] * cos(theta) + x_t[0] * sin(theta) - x_t[1] * cos(theta);
  double H_0_3 =  cos(theta);
  double H_0_4 =  sin(theta);
  double H_1_0 = sin(theta);
  double H_1_1 = -cos(theta);
  double H_1_2 =  -lm[0] * cos(theta) - lm[1] * sin(theta) + x_t[0] * cos(theta) + x_t[1] * sin(theta);
  double H_1_3 =  -sin(theta);
  double H_1_4 =  cos(theta);
  SensorJacobian H;
  H <<  H_0_0, H_0_1, H_0_2, H_0_3, H_0_4,
        H_1_0, H_1_1, H_1_2, H_1_3, H_1_4;
  return H;
}

/**
 * @brief Return the Jacobian of the function that maps control space noise (v_t, omega_t) to state space (x, y, theta).
 * From Probabilistic Robotics, ch. 7.4, eq. 7.11: this is "the derivative of the motion function g w.r.t. the motion
 * parameters, evaluated at u_t, and mu_t-1.".
 *
 * @param u current control input (linear and angular velocity).
 * @param x0 current robot pose, of which we use only theta.
 * @param delta_t time step.
 * @return 3x2 matrix.
 */
inline
Eigen::Matrix<double, 3, 2> V_t_x(const TwistCmd& u, const Pose2D& x0, double delta_t)
{
  double v_t = u.linear;
  double w_t = u.angular;
  double theta = x0(2);

  Eigen::MatrixXd V_t(3, 2);

  // Avoid div/zero for straight-line trajectories or no motion.
  if (fabs(w_t) < MIN_ANG_VEL)
  {
    double c_t = cos(theta);
    double s_t = sin(theta);

    V_t << c_t * delta_t,  0,
           s_t * delta_t,  0,
           0,              delta_t;
  }
  else
  {
    double s_t = sin(theta);
    double c_t = cos(theta);
    double s_w_t = sin(theta + (w_t * delta_t));
    double c_w_t = cos(theta + (w_t * delta_t));

    double V_0_0 = (1. / w_t) * (-s_t + s_w_t);
    double V_0_1 = (v_t / w_t * w_t) * (s_t  - s_w_t) + (v_t / w_t) * c_w_t * delta_t;
    double V_1_0 = (1. / w_t) * (c_t - c_w_t);
    double V_1_1 = -(v_t / w_t * w_t) * (c_t - c_w_t) + (v_t / w_t) * s_w_t * delta_t;
    V_t <<  V_0_0, V_0_1,
            V_1_0, V_1_1,
            0.0, delta_t;
  }
  return V_t;
}

// EKF Landmark SLAM with a fixed number of easily-identifiable landmarks.
class Ekf final
{
public:
  EkfState predict(const EkfState& prev_state, const TwistCmd& u, double dt);
  EkfState predict(const EkfState& prev_state, const TwistCmd& u, double dt, const Eigen::Matrix2d& M_t);
  EkfState correct(const EkfState& predicted_state, const std::list<Measurement>& z_k);

  [[nodiscard]] Eigen::Vector<double, POSE_DIMS> get_pose() const;
  [[nodiscard]] Eigen::Matrix<double, LANDMARKS_KNOWN, LM_DIMS> get_landmarks() const;
  [[nodiscard]] EkfState get_state() const;

  void init_new_landmarks(const std::list<Measurement>& z_k);

private:
  /**
  * @brief Returns a writable view on the (flat) state vector, reshaped to look
  * like a matrix of shape LANDMARKS_KNOWN x LM_DIMS.
  */
  auto get_landmarks_()
  {
    return estimated_state_.mean.tail(STATE_DIMS - POSE_DIMS).reshaped<Eigen::RowMajor>(LANDMARKS_KNOWN, LM_DIMS);
  }

  /**
  * @brief Returns a read-only view on the (flat) state vector, reshaped to look
  * like a matrix of shape LANDMARKS_KNOWN x LM_DIMS.
  */
  auto get_landmarks_() const
  {
    return estimated_state_.mean.tail(STATE_DIMS - POSE_DIMS).reshaped<Eigen::RowMajor>(LANDMARKS_KNOWN, LM_DIMS);
  }

  EkfState estimated_state_;
};

}  // namespace ekf_localizer
#endif //EKF_LOCALIZER_HPP