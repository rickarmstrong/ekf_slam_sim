// Copyright (c) 2025 Richard Armstrong
#include <limits>
#include "gtest/gtest.h"
#include <Eigen/Dense>
#include "ekf_localizer.hpp"
#include "utility.hpp"


TEST(UtilityTest, ToMeasurementSmokeTest)
{
  // Generate an array of tag detections.
  const int TAG_ARRAY_SIZE = 100;
  ekf_localizer::TagArray ta;
  for (int i = 0; i < TAG_ARRAY_SIZE; ++i)
  {
    ekf_localizer::TagDetection d;
    d.id =  std::vector<int> {i};
    d.pose.pose.pose.position.x = i;
    d.pose.pose.pose.position.y = i;
    d.pose.pose.pose.position.z = i;
    ta.detections.push_back(d);
  }

  ekf_localizer::MeasurementList measurements = to_measurements(ta);
  ASSERT_EQ(measurements.size(), TAG_ARRAY_SIZE);
}

TEST(UtilityTest, EigenSandbox)
{
  Eigen::VectorXd flat = Eigen::VectorXd::Random(6);
  Eigen::MatrixXd landmarks = flat.tail(4).reshaped(2, 2);
  Eigen::Vector2d lm(0., 1.);
  landmarks.row(0) = lm;

  ASSERT_EQ(1, 1);
}

TEST(UtilityTest, MapToSensorTest)
{
  // A point on the map y-axis.
  Eigen::Vector2d p_map(0., 1.);

  // Sensor at origin, looking down the x-axis.
  ekf_localizer::Pose2D x_t(0., 0., 0);
  Eigen::Vector2d expected(0., 1.);  // Expected sensor frame result.
  Eigen::Vector2d actual = map_to_sensor(p_map, x_t);
  ASSERT_EQ(expected, actual);

  // Sensor at (1, 1, pi), looking at the point down the sensor x-axis.
  x_t = ekf_localizer::Pose2D(1., 1., std::numbers::pi);
  expected = Eigen::Vector2d(1., 0.);  // Expected sensor frame result.
  actual = map_to_sensor(p_map, x_t);
  ASSERT_NEAR(actual[0], expected[0], std::numeric_limits<double>::epsilon());
  ASSERT_NEAR(actual[1], expected[1], std::numeric_limits<double>::epsilon());

  // Sensor at (1, 0, 0), no rotation.
  x_t = ekf_localizer::Pose2D(1., 0., 0.);
  expected = Eigen::Vector2d(-1., 1.);
  actual = map_to_sensor(p_map, x_t);
  ASSERT_NEAR(actual[0], expected[0], std::numeric_limits<double>::epsilon());
  ASSERT_NEAR(actual[1], expected[1], std::numeric_limits<double>::epsilon());

  // Sensor at (-1, 0, pi/4), looking at the point.
  x_t = ekf_localizer::Pose2D(-1., 0., std::numbers::pi / 4.);
  expected = Eigen::Vector2d(sqrt(2.), 0.);
  actual = map_to_sensor(p_map, x_t);
  ASSERT_NEAR(actual[0], expected[0], std::numeric_limits<double>::epsilon());
  ASSERT_NEAR(actual[1], expected[1], std::numeric_limits<double>::epsilon());
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
