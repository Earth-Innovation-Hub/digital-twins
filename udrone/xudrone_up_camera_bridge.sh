#!/bin/bash
# xudrone upward camera -> /xudrone/camera_up/{image,camera_info}
# Looks at the AprilTag under the Heron deck. Flock-guarded.
set -e
exec 9> /tmp/xudrone_up_camera_bridge.lock
if ! flock -n 9; then
  echo "$(date -u +%FT%TZ) xudrone up camera bridge already running; not starting another"
  exit 0
fi
source /opt/ros/humble/setup.bash
[ -f /root/ros2_ws/install/setup.bash ] && source /root/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-84}
export GZ_PARTITION=${GZ_PARTITION:-aquatic}

WORLD=${PX4_GZ_WORLD:-kiwanis_lake}
MODEL=${GZ_MODEL_NAME:-xudrone_1}
C=/world/$WORLD/model/$MODEL/link/up_camera_link/sensor/up_camera

exec ros2 run ros_gz_bridge parameter_bridge \
  "$C/image@sensor_msgs/msg/Image[gz.msgs.Image" \
  "$C/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo" \
  --ros-args -r __node:=xudrone_up_camera_bridge -r __ns:=/xudrone \
  -r "$C/image:=/xudrone/camera_up/image" \
  -r "$C/camera_info:=/xudrone/camera_up/camera_info"
