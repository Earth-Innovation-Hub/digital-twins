#!/bin/bash
# xudrone EchoRanger (down) + surface ranger → ROS LaserScan.
# Bottom beam also feeds PX4 via GZBridge (lidar_sensor_link); this bridge is
# for behaviours / logging.
set -e
exec 9> /tmp/xudrone_range_bridge.lock
if ! flock -n 9; then
  echo "$(date -u +%FT%TZ) xudrone range bridge already running; not starting another"
  exit 0
fi
source /opt/ros/humble/setup.bash
[ -f /root/ros2_ws/install/setup.bash ] && source /root/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-84}
export GZ_PARTITION=${GZ_PARTITION:-aquatic}

WORLD=${PX4_GZ_WORLD:-kiwanis_lake}
MODEL=${GZ_MODEL_NAME:-}
if [ -z "$MODEL" ]; then
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
    if echo "$listed" | grep -q "xudrone_0"; then
      MODEL=xudrone_0
      break
    fi
    sleep 2
  done
fi

BOT=/world/$WORLD/model/$MODEL/link/lidar_sensor_link/sensor/lidar/scan
SUR=/world/$WORLD/model/$MODEL/link/surface_range_link/sensor/surface_lidar/scan
echo "$(date -u +%FT%TZ) xudrone range bridge MODEL=$MODEL bot=$BOT"

exec ros2 run ros_gz_bridge parameter_bridge \
  "$BOT@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan" \
  "$SUR@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan" \
  --ros-args -r __node:=xudrone_range_bridge -r __ns:=/xudrone \
  -r "$BOT:=/xudrone/range/bottom" \
  -r "$SUR:=/xudrone/range/surface"
