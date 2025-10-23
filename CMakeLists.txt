cmake_minimum_required(VERSION 3.8)
project(ekf_slam_sim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(apriltag_ros_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(ekf_slam src/ekf_slam_node.cpp
        include/utility.hpp
        src/measurement.cpp
        include/measurement.hpp
        include/types.hpp
)
ament_target_dependencies(ekf_slam PUBLIC
        rclcpp
        std_msgs
        nav_msgs
        apriltag_ros_interfaces
        Eigen3
        tf2_ros)

install(TARGETS ekf_slam
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY maps DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()

  ament_find_gtest()

  add_subdirectory(test)
endif()

ament_package()
