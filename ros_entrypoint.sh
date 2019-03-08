#!/bin/bash
set -e

# setup ROS environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

# building ROS packages
cd ~/simbotic_catkin_workspace
catkin_make
echo "source $HOME/simbotic_catkin_workspace/devel/setup.bash" > ~/.bashrc
roscore

exec "$@"