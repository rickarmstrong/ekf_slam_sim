// Copyright (c) 2025 Richard Armstrong
#include <cmath>
#include <gtest/gtest.h>
#include "ekf_localizer.h"

namespace ekf_localizer {

TEST(EKFLocalizerTest, SmokeTest)
{
  auto filter = Ekf();
  Eigen::Vector<double, POSE_DIMS> pose = filter.get_pose();
  std::cout << pose << std::endl;
}

TEST(EKFLocalizerTest, MotionModelTest)
{
  TwistCmd u{1.0, 1.0};
  Pose2D x0{0.0, 0.0, 0.0};
  double delta_t = 1.0;
  Pose2D x1 = g(u, x0, delta_t);
  EXPECT_NEAR(x1(0), sin(1.0), 1e-6);
  EXPECT_NEAR(x1(1), 1.0 - cos(1.0), 1e-6);
  EXPECT_NEAR(x1(2), 1.0, 1e-6);
}

TEST(EKFLocalizerTest, FilterTest)
{
  TwistCmd u{1.0, 1.0};
  auto filter = Ekf();
  EkfState predicted = filter.predict(u, 1.0);
  Pose2D new_pose(predicted.mean(Eigen::seqN(0, 3)));
  ASSERT_NEAR(new_pose(0), sin(1.0), 1e-6);
  ASSERT_NEAR(new_pose(1), 1.0 - cos(1.0), 1e-6);
  ASSERT_NEAR(new_pose(2), 1.0, 1e-6);

  // TODO: write a test the verifies that pose.theta unwrapping is correct.
}

// This test was created to troubleshoot an issue at the start of
// the project, when we were having trouble with misbehavior in the
// covariance update. Namely, the pose covariance was blowing up rapidly
// due to numerical precision problems in the function that calculates
// the mapping of noise from control space to state space.
TEST(EKFLocalizerTest, CovPropagationTest)
{
  auto filter = Ekf();
  TwistCmd u{0.25, 0.0};
  double dt = 0.033;  // 30 Hz.
  double covx;
  const Eigen::Matrix2d control_noise{
    // Arbitrarily chosen.
    {0.01, 0.0},  // Linear velocity stdev.
    {0.0, 0.01},  // Angular velocity stdev.
  };

  // Run the prediction step through a few iterations
  // and check that it's not gone insane
  for (auto i = 0; i < 10; ++i)
  {
    EkfState next_state = filter.predict(u, dt, control_noise);
    covx = next_state.covariance(0, 0); // Pose covariance in x.
  }
  EXPECT_LT(covx, 1.0);  // Not blowing-up.
}

TEST(EkfLocalizerTest, SensorJacobianTest)
{
  Pose2D x_t{0.0, 0.0, 0.0};
  Eigen::Vector2d lm{1.0, 0.0};
  uint j = 1;  // Hypothetical landmark index.
  SensorJacobian h = H_i_t(x_t, lm);

  // h is the low-dimensional Jacobian of the sensor model, w.r.t. the pose and one landmark.
  // Here, we demonstrate how to 'promote' this to the higher-dimensional version of H_t used in
  // calculating the Kalman gain.
  Eigen::Matrix<double, LM_DIMS, STATE_DIMS> H_t;
  H_t.setZero();
  H_t.block<LM_DIMS, POSE_DIMS>(0, 0) = h.block<LM_DIMS, POSE_DIMS>(0, 0);
  H_t.block<LM_DIMS, LM_DIMS>(0, POSE_DIMS + j * LM_DIMS) = h.block<LM_DIMS, LM_DIMS>(0, POSE_DIMS);

  // Landmark component in the right place? Redundant test, but gives a spot for a breakpoint.
  EXPECT_EQ(H_t(0, POSE_DIMS + j * LM_DIMS), h(0, POSE_DIMS));
  EXPECT_EQ(H_t(1, POSE_DIMS + j * LM_DIMS + 1), h(1, POSE_DIMS + 1));
}

}  // namespace ekf_localizer
