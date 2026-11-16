#!/bin/bash
# xudrone front camera (Gazebo) -> ROS 2 /xudrone/camera_front/{image,camera_info}
# The nadir bridge uses /xudrone/camera_down. Both names carry the view.
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
MODEL=${GZ_MODEL_NAME:-}
if [ -z "$MODEL" ]; then
  # CoRAL dual spawn uses xudrone_1; solo SITL uses xudrone_0.
  # Never lock onto a default that is not in the world yet — wait for a real model.
  MODEL=
  for _ in $(seq 1 90); do
    listed=$(gz model --list 2>/dev/null || true)
    if echo "$listed" | grep -q "xudrone_1"; then
      MODEL=xudrone_1
      break
    fi
    if echo "$listed" | grep -q "heron_0"; then
      # Boat up: keep waiting for secondary AUV (do not fall through to xudrone_0).
      sleep 2
      continue
    fi
    if echo "$listed" | grep -q "xudrone_0"; then
      MODEL=xudrone_0
      break
    fi
    sleep 2
  done
fi
if [ -z "$MODEL" ]; then
  echo "$(date -u +%FT%TZ) xudrone front camera bridge: no xudrone_* model yet; exiting" >&2
  exit 1
fi
CAM=/world/$WORLD/model/$MODEL/link/front_camera_link/sensor/front_camera
echo "$(date -u +%FT%TZ) xudrone front camera bridge MODEL=$MODEL CAM=$CAM"

exec ros2 run ros_gz_bridge parameter_bridge \
  "$CAM/image@sensor_msgs/msg/Image[gz.msgs.Image" \
  "$CAM/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo" \
  --ros-args -r __node:=xudrone_camera_bridge -r __ns:=/xudrone \
  -r "$CAM/image:=/xudrone/camera_front/image" \
  -r "$CAM/camera_info:=/xudrone/camera_front/camera_info"
