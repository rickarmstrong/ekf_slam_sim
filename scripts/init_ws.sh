#!/bin/bash
# Sets up the ROS environment in our lxc container. To kick it off
# at login, do this in .bash_profile:
# . ~/.profile
# . ~/init_ws.sh

cd ~/src/ekf_slam_sim_ws/ || exit
. install/setup.bash
export GZ_SIM_RESOURCE_PATH="$HOME/.gazebo/models/harmonic/apriltag/"
