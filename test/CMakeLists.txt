set(TEST_NAMES
  ekf_localizer_tests utility_tests
)

foreach(name IN LISTS TEST_NAMES)
  ament_add_gtest(${name} ${name}.cpp)
  ament_target_dependencies(${name} rclcpp apriltag_ros_interfaces nav_msgs)

  if(${TEST_DEBUG_INFO})
    target_compile_definitions(${name} PUBLIC -DTEST_DEBUG_INFO)
  endif()
endforeach()
