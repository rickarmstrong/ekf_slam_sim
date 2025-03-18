// Copyright (c) 2025 Richard Armstrong
#include <cmath>
#include "gtest/gtest.h"
#include "ekf_localizer.hpp"


TEST(EKFLocalizerTest, SmokeTest)
{
    auto filter = Ekf();
}

TEST(EKFLocalizerTest, MotionModelTest)
{
    ekf_localizer::TwistCmd u{1.0, 1.0};
    Pose2D x0{0.0, 0.0, 0.0};
    double delta_t = 1.0;
    Pose2D x1 = g(u, x0, delta_t);
    EXPECT_NEAR(x1(0), sin(1.0), 1e-6);
    EXPECT_NEAR(x1(1), 1.0 - cos(1.0), 1e-6);
}
