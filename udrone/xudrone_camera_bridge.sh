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
MODEL=${GZ_MODEL_NAME:-}
if [ -z "$MODEL" ]; then
  # CoRAL dual spawn uses xudrone_1; solo SITL uses xudrone_0.
  # start_boat can launch this before xudrone_1 exists — wait when heron is up.
  MODEL=xudrone_0
  for _ in $(seq 1 40); do
    listed=$(gz model --list 2>/dev/null || true)
    if echo "$listed" | grep -q "xudrone_1"; then
      MODEL=xudrone_1
      break
    fi
    if echo "$listed" | grep -q "heron_0"; then
      sleep 2
      continue
    fi
    # No boat yet / solo world — accept xudrone_0 if present.
    if echo "$listed" | grep -q "xudrone_0"; then
      MODEL=xudrone_0
      break
    fi
    sleep 2
  done
fi
CAM=/world/$WORLD/model/$MODEL/link/front_camera_link/sensor/front_camera
echo "$(date -u +%FT%TZ) xudrone front camera bridge MODEL=$MODEL CAM=$CAM"

exec ros2 run ros_gz_bridge parameter_bridge \
  "$CAM/image@sensor_msgs/msg/Image[gz.msgs.Image" \
  "$CAM/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo" \
  --ros-args -r __node:=xudrone_camera_bridge -r __ns:=/xudrone \
  -r "$CAM/image:=/xudrone/camera/image" \
  -r "$CAM/camera_info:=/xudrone/camera/camera_info"
