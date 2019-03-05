#!/bin/bash
set -e

# setup ROS environment

echo "source /opt/ros/$ROS_DISTRO/setup.bash" > ~/.bashrc
source "$HOME"/.bashrc

# building ROS packages
pushd ~/simbotic_catkin_workspace
catkin_make
popd
source "$HOME"/simbotic_catkin_workspace/devel/setup.bash
ls

exec "$@"