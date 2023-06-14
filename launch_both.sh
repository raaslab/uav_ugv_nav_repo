#!/bin/bash

# Script to launch PX4
gnome-terminal -- bash -c '
cd
cd PX4-Autopilot/
DONT_RUN=1 make px4_sitl_default gazebo
source ~/catkin_ws/devel/setup.bash
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
source /opt/ros/melodic/setup.bash
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch px4 mavros_posix_sitl.launch'

# Script to launch Husky
gnome-terminal -- bash -c '
sleep 20
cd
export HUSKY_GAZEBO_DESCRIPTION=$(rospack find husky_gazebo)/urdf/description.gazebo.xacro
roslaunch husky_gazebo husky_empty_world.launch'




