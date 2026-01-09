// Copyright (c) 2025 Richard Armstrong
#include "ekf_localizer.h"

namespace ekf_localizer {

EkfState Ekf::predict(const TwistCmd& u, double dt)
{
  // Use the global noise config matrix M_t.
  return predict(u, dt, M_t);
}

EkfState Ekf::predict(const TwistCmd& u, double dt, const Eigen::Matrix2d& M_t)
{
  // Current pose estimate.
  Eigen::Vector3d x0 = estimated_state_.mean(Eigen::seqN(0, 3));

  // Noise-free motion estimate.
  Pose2D predicted_pose = g(u, x0, dt);

  // Update pose.
  estimated_state_.mean.head(POSE_DIMS) = predicted_pose;

  // Update pose covariance.
  Eigen::Matrix<double, STATE_DIMS, STATE_DIMS> G_t;
  G_t.setIdentity();
  G_t.block<3, 3>(0, 0) = G_t_x(u, x0, dt);
  Eigen::Matrix<double, 3, 2> V_t = V_t_x(u, x0, dt); // 3x2 matrix that maps control space noise to state space.
  Eigen::Matrix3d R_t = V_t * M_t * V_t.transpose();
  Eigen::Matrix<double, STATE_DIMS, STATE_DIMS> cov_next = G_t * estimated_state_.covariance * G_t.transpose();
  cov_next.block<3, 3>(0, 0) += R_t * (u.linear / MAX_VEL_X) * dt; // Only update pose covariance.
  estimated_state_.covariance = cov_next;

  return estimated_state_;
}

Eigen::Vector<double, POSE_DIMS> Ekf::get_pose() const
{
  return get_pose_();
}

Eigen::Matrix<double, LANDMARKS_KNOWN, LM_DIMS> Ekf::get_landmarks() const
{
  return get_landmarks_().eval();
}

void Ekf::set_landmark(unsigned id, const Eigen::Vector2d& landmark)
{
  get_landmarks_().row(id) = landmark;
  landmarks_seen_.push_back(id);
}

EkfState Ekf::correct(const MeasurementList& z_k)
{
  for (Measurement z: z_k)
  {
    // If this is the first time we've seen this tag,
    // initialize our landmark estimate with this measurement.
    Eigen::MatrixXd landmarks = get_landmarks();
    if (std::find(landmarks_seen_.begin(), landmarks_seen_.end(), z.id) == landmarks_seen_.end())
    {
      set_landmark(z.id, sensor_to_map(Eigen::Vector2d(z.x, z.y), get_pose()));
    }

    // Get the low-dimensional Jacobian of the sensor model.
    SensorJacobian h = H_i_t(get_pose(), get_landmarks().row(z.id));

    // "Promote" h to the full-size Jacobian, H_t.
    Eigen::Matrix<double, LM_DIMS, STATE_DIMS> H_t;
    H_t.setZero();
    H_t.block<LM_DIMS, POSE_DIMS>(0, 0) = h.block<LM_DIMS, POSE_DIMS>(0, 0);
    H_t.block<LM_DIMS, LM_DIMS>(0, POSE_DIMS + z.id * LM_DIMS) = h.block<LM_DIMS, LM_DIMS>(0, POSE_DIMS);

    // Kalman gain, dimensions (2N+3, 2), where N is the number of landmarks.
    // (2N+3, 2) = (2N+3,2N+3) @ (2N+3, 2) @ ((2, 2N+3) @ (2N+3, 2N+3) @ (2N+3, 2) + (2, 2))^-1
    const Eigen::Matrix<double, STATE_DIMS, STATE_DIMS>& S_bar = estimated_state_.covariance;  // Reduce clutter a little.
    Eigen::Matrix<double, 2 * LANDMARKS_KNOWN + POSE_DIMS, LM_DIMS> K_i_t;
    K_i_t.setZero();
    K_i_t = (estimated_state_.covariance * H_t.transpose()) * (H_t * estimated_state_.covariance * H_t.transpose() + Q_t).inverse();

    /////////////////////////////////////////////////////////////////////////
    // Update mean state and covariance estimates for this observation.
    /////////////////////////////////////////////////////////////////////////

    // Mean.
    Eigen::Vector2d z_hat = map_to_sensor(get_landmarks().row(z.id), get_pose());  // Expected measurement.
    estimated_state_.mean += K_i_t * (Eigen::Vector2d(z.x, z.y) - z_hat);
    double theta = estimated_state_.mean[2];
    estimated_state_.mean[2] = atan2(sin(theta), cos(theta)); // Normalize theta.

    // Covariance (Joseph form).
    Eigen::Matrix<double, STATE_DIMS, STATE_DIMS> I = Eigen::Matrix<double, STATE_DIMS, STATE_DIMS>::Identity();
    Eigen::Matrix<double, STATE_DIMS, STATE_DIMS> I_KH = I - K_i_t * H_t;
    estimated_state_.covariance = I_KH * estimated_state_.covariance * I_KH.transpose() + K_i_t * Q_t * K_i_t.transpose();
  }

  return estimated_state_;
}

Eigen::VectorBlock<Eigen::Matrix<double, -1, 1>> Ekf::get_pose_()
{
  return estimated_state_.mean.head(POSE_DIMS);
}

}  // namespace ekf_localizer
