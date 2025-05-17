// Copyright (c) 2025 Richard Armstrong
#ifndef UTILITY_HPP
#define UTILITY_HPP
#include <rclcpp/time.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <chrono>

using steady_clock = std::chrono::steady_clock;
using time_point = steady_clock::time_point;


/**
 * @brief Convenience function to turn a message header timestamp to a std::chrono::time_point.
 * @param ros_time timestamp from a ROS2 message header.
 * @return std::chrono::time_point<std::chrono::system_clock>
 */
inline time_point
to_std_time(const builtin_interfaces::msg::Time& ros_time) {
  const auto duration = std::chrono::seconds(ros_time.sec) + std::chrono::nanoseconds(ros_time.nanosec);
  return time_point(duration);
}
#endif //UTILITY_HPP
