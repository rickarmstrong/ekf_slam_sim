// Copyright (c) 2025 Richard Armstrong
#include <cmath>
#include "gtest/gtest.h"
#include "ekf_localizer.hpp"


TEST(EKFLocalizerTest, SmokeTest)
{
  auto filter = ekf_localizer::Ekf();
}

TEST(EKFLocalizerTest, MotionModelTest)
{
  ekf_localizer::TwistCmd u{1.0, 1.0};
  ekf_localizer::Pose2D x0{0.0, 0.0, 0.0};
  double delta_t = 1.0;
  ekf_localizer::Pose2D x1 = g(u, x0, delta_t);
  EXPECT_NEAR(x1(0), sin(1.0), 1e-6);
  EXPECT_NEAR(x1(1), 1.0 - cos(1.0), 1e-6);
  EXPECT_NEAR(x1(2), 1.0, 1e-6);
}

TEST(EKFLocalizerTest, FilterTest)
{
  ekf_localizer::TwistCmd u{1.0, 1.0};
  auto filter = ekf_localizer::Ekf();
  ekf_localizer::EkfState predicted = filter.predict(u, 1.0);
  ekf_localizer::Pose2D new_pose(predicted.mean(Eigen::seqN(0, 3)));
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
  auto filter = ekf_localizer::Ekf();
  ekf_localizer::TwistCmd u{0.25, 0.0};
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
    ekf_localizer::EkfState next_state = filter.predict(u, dt, control_noise);
    covx = next_state.covariance(0, 0); // Pose covariance in x.
  }
  EXPECT_LT(covx, 1.0);  // Not blowing-up.
}