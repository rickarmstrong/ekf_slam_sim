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
