// Copyright (c) 2025 Richard Armstrong
#ifndef UTILITY_HPP
#define UTILITY_HPP
#include <chrono>
#include <list>
#include "ekf_localizer.hpp"

using steady_clock = std::chrono::steady_clock;
using time_point = steady_clock::time_point;

using Measurement = ekf_localizer::Measurement;
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
to_measurements(const TagArray& ta)
{
  std::list<Measurement> measurements;
  std::transform(
    ta.detections.begin(),
    ta.detections.end(),
    std::back_inserter(measurements),
    [](const auto& d){ return Measurement(d); });
  return measurements;;
}
#endif //UTILITY_HPP
