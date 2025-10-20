// Copyright (c) 2025 Richard Armstrong
#include <limits>
#include "gtest/gtest.h"
#include <Eigen/Dense>
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

TEST(UtilityTest, EigenSandbox)
{
  Eigen::VectorXd flat = Eigen::VectorXd::Random(6);
  auto landmarks = flat.tail(4).reshaped(2, 2);
  Eigen::Vector2d lm(0., 1.);
  landmarks.row(0) = lm;

  ASSERT_EQ(1, 1);
}

TEST(UtilityTest, SensorToMapTest)
{
  // Sensor at (1, 1), no rotation.
  ekf_localizer::Pose2D x_t(1., 1., 0);

  // Landmark measurement in the sensor frame.
  Eigen::Vector2d p_sensor;
  Eigen::Vector2d expected;  // Output that we will check.

  // Landmark at the sensor origin.
  p_sensor = Eigen::Vector2d(0., 0.);
  expected = Eigen::Vector2d(1., 1.);
  Eigen::Vector2d measured = sensor_to_map(p_sensor, x_t);
  ASSERT_DOUBLE_EQ(measured[0], expected[0]);
  ASSERT_DOUBLE_EQ(measured[1], expected[1]);

  // Landmark at the map origin.
  p_sensor = Eigen::Vector2d(-1., -1.);
  expected = Eigen::Vector2d(0., 0.);
  measured = sensor_to_map(p_sensor, x_t);
  ASSERT_DOUBLE_EQ(measured[0], expected[0]);
  ASSERT_DOUBLE_EQ(measured[1], expected[1]);

  // Landmark at the map origin, with sensor pointed at the origin.
  x_t[2] = 5. * M_PI / 4.;
  p_sensor = Eigen::Vector2d(sqrt(2.), 0.);
  expected = Eigen::Vector2d(0., 0.);
  measured = sensor_to_map(p_sensor, x_t);
  ASSERT_NEAR(measured[0], expected[0], std::numeric_limits<double>::epsilon());
  ASSERT_NEAR(measured[1], expected[1], std::numeric_limits<double>::epsilon());
}
