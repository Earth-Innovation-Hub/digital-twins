#!/bin/bash
# xudrone fixed nadir camera -> /xudrone/camera_down/{image,camera_info}
# Flock-guarded: start_boat and start_udrone --secondary may both try to start it.
set -e
exec 9> /tmp/xudrone_down_camera_bridge.lock
if ! flock -n 9; then
  echo "$(date -u +%FT%TZ) xudrone down camera bridge already running; not starting another"
  exit 0
fi
source /opt/ros/humble/setup.bash
[ -f /root/ros2_ws/install/setup.bash ] && source /root/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-84}
export GZ_PARTITION=${GZ_PARTITION:-aquatic}

WORLD=${PX4_GZ_WORLD:-kiwanis_lake}
MODEL=${GZ_MODEL_NAME:-xudrone_1}
C=/world/$WORLD/model/$MODEL/link/down_camera_link/sensor/down_camera

exec ros2 run ros_gz_bridge parameter_bridge \
  "$C/image@sensor_msgs/msg/Image[gz.msgs.Image" \
  "$C/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo" \
  --ros-args -r __node:=xudrone_down_camera_bridge -r __ns:=/xudrone \
  -r "$C/image:=/xudrone/camera_down/image" \
  -r "$C/camera_info:=/xudrone/camera_down/camera_info"
