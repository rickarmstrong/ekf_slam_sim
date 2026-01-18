// Copyright (c) 2026 Richard Armstrong
#include <Eigen/Core>
#include "ekf_state.h"
#include "measurement.h"
#include "utility.hpp"
// TODO: reorder definitions by name (i.e. alphabetize).
namespace ekf_localizer
{

EkfState::EkfState()
    :mean(STATE_DIMS), covariance(STATE_DIMS, STATE_DIMS)
  {
    // We assume we start at (0, 0), so this makes sense for our pose.
    // We set landmarks to (0, 0) to mark them as uninitialized.
    mean.setZero();

    // Pose covariance starts at zero, because we know our initial location.
    // Landmark variances start at "infinity", which we fake by initializing
    // them to arbitrary "big number".
    covariance.setZero();
    covariance.block<LANDMARKS_KNOWN * LM_DIMS, LANDMARKS_KNOWN * LM_DIMS>(3, 3) =
      Eigen::MatrixXd::Identity(LANDMARKS_KNOWN * LM_DIMS, LANDMARKS_KNOWN * LM_DIMS) * 1e6;
  }

void EkfState::init_new_landmarks(const std::list<Measurement>& z_k)
{
  for (Measurement z: z_k)
  {
    // If this is the first time we've seen this tag,
    // initialize our landmark estimate with this measurement.
    Eigen::MatrixXd landmarks = get_landmarks();
    if (std::find(landmarks_seen.begin(), landmarks_seen.end(), z.id) == landmarks_seen.end())
    {
      set_landmark(z.id, sensor_to_map(Eigen::Vector2d(z.x, z.y), mean.head(POSE_DIMS)));
    }
  }
}

Eigen::Matrix<double, LANDMARKS_KNOWN, LM_DIMS> EkfState::get_landmarks() const
{
  return mean.tail(STATE_DIMS - POSE_DIMS).reshaped<Eigen::RowMajor>(LANDMARKS_KNOWN, LM_DIMS).eval();
}

void EkfState::set_landmark(unsigned id, const Eigen::Vector2d& landmark)
{
  mean.tail(STATE_DIMS - POSE_DIMS)
    .reshaped<Eigen::RowMajor>(LANDMARKS_KNOWN, LM_DIMS)  // view landmarks as a list of 1x2 vectors.
    .row(id) = landmark;
  landmarks_seen.push_back(id);
}
} // namespace ekf_localizer
