#!/bin/bash
CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:/opt/ros/jazzy /usr/bin/cmake --build /home/ubuntu/src/ekf_slam_sim_ws/build/ekf_slam_sim -- -j32 -l32
CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:/opt/ros/jazzy /usr/bin/cmake --install /home/ubuntu/src/ekf_slam_sim_ws/build/ekf_slam_sim
