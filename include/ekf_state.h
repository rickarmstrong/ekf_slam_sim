// Copyright (c) 2026 Richard Armstrong
#ifndef EKF_SLAM_SIM_EKF_STATE_H
#define EKF_SLAM_SIM_EKF_STATE_H
#include <list>

namespace ekf_localizer {

constexpr size_t POSE_DIMS = 3;     // x, y, theta.
constexpr size_t LM_DIMS = 2;       // x, y.
constexpr size_t LANDMARKS_KNOWN = 16;  // The number of tags defined in config/tags.yaml.
constexpr size_t STATE_DIMS = POSE_DIMS + LM_DIMS * LANDMARKS_KNOWN;

class Measurement;
class EkfState
{
public:
  EkfState();
  void init_new_landmarks(const std::list<Measurement>& z_k);
  void set_landmark(unsigned id, const Eigen::Vector2d& landmark);

  [[nodiscard]] Eigen::Matrix<double, LANDMARKS_KNOWN, LM_DIMS> get_landmarks() const;

  Eigen::VectorXd mean;
  Eigen::MatrixXd covariance;
  std::vector<unsigned> landmarks_seen;
};

} // namespace ekf_localizer
#endif //EKF_SLAM_SIM_EKF_STATE_H