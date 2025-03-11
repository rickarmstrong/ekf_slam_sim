// Copyright (c) 2025 Richard Armstrong
#include <vector>
#include "gtest/gtest.h"
#include "ekf_localizer.hpp"

TEST(EKFLocalizerTest, SmokeTest)
{
    auto filter = Ekf();
    EXPECT_EQ(0, 0);
}
