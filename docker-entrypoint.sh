#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash
source /home/ros2/ros2_ws/install/setup.bash

exec "$@"
