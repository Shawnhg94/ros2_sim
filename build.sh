#!/usr/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
colcon build
source ./install/setup.bash