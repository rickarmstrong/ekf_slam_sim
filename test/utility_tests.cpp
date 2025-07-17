// Copyright (c) 2025 Richard Armstrong
#include "gtest/gtest.h"
#include "ekf_localizer.hpp"
#include "utility.hpp"


TEST(UtilityTest, ToMeasurementSmokeTest)
{
  const size_t TAG_ARRAY_SIZE = 100;
  TagArray ta;
  for (size_t i = 0; i < TAG_ARRAY_SIZE; ++i) { ta.detections.emplace_back(); }

  ekf_localizer::MeasurementList measurements = to_measurements(ta);

  ASSERT_EQ(measurements.size(), TAG_ARRAY_SIZE);
}
