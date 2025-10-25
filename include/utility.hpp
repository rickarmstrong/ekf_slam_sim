// Copyright (c) 2025 Richard Armstrong
#ifndef UTILITY_HPP
#define UTILITY_HPP
#include <chrono>
#include <list>
#include "measurement.hpp"

using steady_clock = std::chrono::steady_clock;
using time_point = steady_clock::time_point;

inline Eigen::Vector2d
sensor_to_map(Eigen::Vector2d p, ekf_localizer::Pose2D x_t)
{
  // Construct the homogeneous transformation.
  double theta = x_t[2];
  double ct = cos(theta);
  double st = sin(theta);
  Eigen::Matrix3d m_T_b;
  m_T_b <<  ct, -st, x_t[0],
            st, ct, x_t[1],
            0., 0., 1.;

  // Homogeneous representation of p.
  Eigen::Vector3d p_homo;
  p_homo << p, 1.;

  return (m_T_b * p_homo).head(2).eval();
}

/**
 * @brief Convenience function to turn a message header timestamp to a std::chrono::time_point.
 * @param ros_time timestamp from a ROS2 message header.
 * @return std::chrono::time_point<std::chrono::system_clock>
 */
inline time_point
to_std_time(const builtin_interfaces::msg::Time& ros_time)
{
  const auto duration = std::chrono::seconds(ros_time.sec) + std::chrono::nanoseconds(ros_time.nanosec);
  return time_point(duration);
}

/**
 * @brief Convenience function to convert TagArray into a list of Measurements.
 * @param ta reference to a TagArray.
 * @return
 */
inline std::list<ekf_localizer::Measurement>
to_measurements(const ekf_localizer::TagArray& ta)
{
  std::list<ekf_localizer::Measurement> measurements;
  std::transform(
    ta.detections.begin(),
    ta.detections.end(),
    std::back_inserter(measurements),
    [](const auto& d){ return ekf_localizer::Measurement(d); });
  return measurements;;
}
#endif //UTILITY_HPP
