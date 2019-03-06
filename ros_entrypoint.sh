#!/bin/bash
set -e

# setup ROS environment
source "$HOME"/.bashrc

# building ROS packages
cd ~/simbotic_catkin_workspace
catkin_make
echo "source $HOME/simbotic_catkin_workspace/devel/setup.bash" > ~/.bashrc
roscore > /dev/null 2>&1 &
exec "$@"