#!/bin/bash
# Wrapper so coral_cmd.py has ROS / domain / partition set.
set -e
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-84}"
export GZ_PARTITION="${GZ_PARTITION:-aquatic}"
export PYTHONUNBUFFERED=1
set +u
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
set -u
ROOT="/root/udrone_boat/behaviours"
exec python3 -u "$ROOT/cmd.py" "$@"
