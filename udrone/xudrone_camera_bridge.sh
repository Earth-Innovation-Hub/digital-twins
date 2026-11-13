#!/bin/bash
# xudrone front camera (Gazebo) -> ROS 2 /xudrone/camera/{image,camera_info}
# Started by start_udrone.sh; the host watchdog restarts it if it dies. Only
# one copy runs: the lock is inherited by the bridge and freed when it exits.
set -e
exec 9> /tmp/xudrone_camera_bridge.lock
if ! flock -n 9; then
  echo "$(date -u +%FT%TZ) camera bridge already running; not starting another"
  exit 0
fi
source /opt/ros/humble/setup.bash
[ -f /root/ros2_ws/install/setup.bash ] && source /root/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-84}
export GZ_PARTITION=${GZ_PARTITION:-aquatic}

WORLD=${PX4_GZ_WORLD:-kiwanis_lake}
MODEL=${GZ_MODEL_NAME:-xudrone_0}
CAM=/world/$WORLD/model/$MODEL/link/front_camera_link/sensor/front_camera

exec ros2 run ros_gz_bridge parameter_bridge \
  "$CAM/image@sensor_msgs/msg/Image[gz.msgs.Image" \
  "$CAM/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo" \
  --ros-args -r __node:=xudrone_camera_bridge -r __ns:=/xudrone \
  -r "$CAM/image:=/xudrone/camera/image" \
  -r "$CAM/camera_info:=/xudrone/camera/camera_info"
