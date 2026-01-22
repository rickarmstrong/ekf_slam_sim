// Copyright (c) 2025 Richard Armstrong
#include "ekf_localizer.h"
#include "ekf_state.h"
#include "utility.hpp"

// TODO: reorder definitions by name (i.e. alphabetize).
namespace ekf_localizer {

EkfState Ekf::predict(const EkfState& prev_state, const TwistCmd& u, double dt)
{
  // Use the global noise config matrix M_t.
  return predict(prev_state, u, dt, M_t);
}

EkfState Ekf::predict(const EkfState& prev_state, const TwistCmd& u, double dt, const Eigen::Matrix2d& M_t)
{
  // Noise-free motion estimate.
  Pose2D x0 = prev_state.mean.head(POSE_DIMS);
  Pose2D x1 = g(u, x0, dt);

  // Update pose.
  EkfState predicted_state;
  predicted_state.mean = prev_state.mean;
  predicted_state.mean.head(POSE_DIMS) = x1;

  // Update pose covariance.
  Eigen::Matrix<double, STATE_DIMS, STATE_DIMS> G_t;
  G_t.setIdentity();
  G_t.block<3, 3>(0, 0) = G_t_x(u, x0, dt);
  Eigen::Matrix<double, 3, 2> V_t = V_t_x(u, x0, dt); // 3x2 matrix that maps control space noise to state space.
  Eigen::Matrix3d R_t = V_t * M_t * V_t.transpose();
  Eigen::Matrix<double, STATE_DIMS, STATE_DIMS> P0 = prev_state.covariance;
  Eigen::Matrix<double, STATE_DIMS, STATE_DIMS> P1 = G_t * P0 * G_t.transpose();
  P1.block<3, 3>(0, 0) += R_t * (u.linear / MAX_VEL_X) * dt; // Only update pose covariance.
  predicted_state.covariance = P1;

  return predicted_state;
}

Eigen::Matrix<double, LANDMARKS_KNOWN, LM_DIMS> Ekf::get_landmarks() const
{
  return get_landmarks_().eval();
}

EkfState Ekf::get_state() const
{
  return estimated_state_;
}

void Ekf::init_new_landmarks(const std::list<Measurement>& z_k)
{
  estimated_state_.init_new_landmarks(z_k);
}

// TODO: need docstrings for this and predict().
EkfState Ekf::correct(const EkfState& predicted_state, const std::list<Measurement>& z_k)
{
  if (z_k.empty())
  {
    estimated_state_ = predicted_state;
    return estimated_state_;
  }

  for (Measurement z: z_k)
  {
    // Get the low-dimensional Jacobian of the sensor model.
    SensorJacobian h = H_i_t(predicted_state.mean.head(POSE_DIMS), get_landmarks().row(z.id));

    // "Promote" h to the full-size Jacobian, H_t.
    Eigen::Matrix<double, LM_DIMS, STATE_DIMS> H_t;
    H_t.setZero();
    H_t.block<LM_DIMS, POSE_DIMS>(0, 0) = h.block<LM_DIMS, POSE_DIMS>(0, 0);
    H_t.block<LM_DIMS, LM_DIMS>(0, POSE_DIMS + z.id * LM_DIMS) = h.block<LM_DIMS, LM_DIMS>(0, POSE_DIMS);

    // Kalman gain, dimensions (2N+3, 2), where N is the number of landmarks.
    // (2N+3, 2) = (2N+3,2N+3) @ (2N+3, 2) @ ((2, 2N+3) @ (2N+3, 2N+3) @ (2N+3, 2) + (2, 2))^-1
    Eigen::Matrix<double, 2 * LANDMARKS_KNOWN + POSE_DIMS, LM_DIMS> K_i_t;
    K_i_t.setZero();
    K_i_t = (predicted_state.covariance * H_t.transpose()) * (H_t * predicted_state.covariance * H_t.transpose() + Q_t).inverse();

    /////////////////////////////////////////////////////////////////////////
    // Update mean state and covariance estimates for this observation.
    /////////////////////////////////////////////////////////////////////////
    //
    // Mean.
    Eigen::Vector2d z_hat = map_to_sensor(get_landmarks().row(z.id), predicted_state.mean.head(POSE_DIMS));  // Expected measurement.
    estimated_state_.mean = predicted_state.mean + K_i_t * (Eigen::Vector2d(z.x, z.y) - z_hat);
    double theta = estimated_state_.mean[2];
    estimated_state_.mean[2] = atan2(sin(theta), cos(theta)); // Normalize theta.

    // Covariance (Joseph form).
    Eigen::Matrix<double, STATE_DIMS, STATE_DIMS> I = Eigen::Matrix<double, STATE_DIMS, STATE_DIMS>::Identity();
    Eigen::Matrix<double, STATE_DIMS, STATE_DIMS> I_KH = I - K_i_t * H_t;
    estimated_state_.covariance = I_KH * estimated_state_.covariance * I_KH.transpose() + K_i_t * Q_t * K_i_t.transpose();
  }

  return estimated_state_;
}

}  // namespace ekf_localizer

