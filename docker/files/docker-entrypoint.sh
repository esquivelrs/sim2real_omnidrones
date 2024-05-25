#!/usr/bin/env bash
set -e

sudo sudo udevadm control --reload-rules && sudo udevadm trigger

source /opt/ros/humble/setup.bash
source /home/ros2_ws/install/setup.bash


# Run the command
exec "$@"