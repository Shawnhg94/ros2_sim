#!/bin/bash
# set -e 
export DISPLAY=host.docker.internal:0.0
export LIBGL_ALWAYS_INDIRECT=0

source /opt/ros/jazzy/setup.bash
colcon build
source /workspaces/gazebo_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

exec "$@"