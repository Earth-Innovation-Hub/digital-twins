#!/bin/bash
# xudrone mapping: OctoMap (3D, from the forward sonar) + xudrone_mapper (TF,
# odometry, global 2D grid, local cloud, map image). One copy per container.
set -e
exec 9> /tmp/xudrone_mapping.lock
if ! flock -n 9; then
  echo "$(date -u +%FT%TZ) xudrone mapping already running; not starting another"
  exit 0
fi
source /opt/ros/humble/setup.bash
for ws in /root/ws_px4/install/setup.bash /root/px4_ros_ws/install/setup.bash /root/ros2_ws/install/setup.bash; do
  [ -f "$ws" ] && source "$ws"
done
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-84}
export GZ_PARTITION=${GZ_PARTITION:-aquatic}
export UDRONE_SRC=${UDRONE_SRC:-/root/udrone_boat/src}
export UDRONE_GTSAM_PATH=${UDRONE_GTSAM_PATH:-/opt/udrone_py}
mkdir -p /root/log /root/udrone_boat/maps

if ros2 pkg prefix octomap_server >/dev/null 2>&1; then
  ros2 run octomap_server octomap_server_node --ros-args \
    -r __node:=xudrone_octomap -r __ns:=/xudrone/octomap \
    -r cloud_in:=/xudrone/sonar/points \
    -r octomap_point_cloud_centers:=/xudrone/octomap/points \
    -p frame_id:=map \
    -p base_frame_id:=xudrone/base_link \
    -p resolution:=0.25 \
    -p sensor_model.max_range:=30.0 \
    -p filter_ground_plane:=false \
    -p latch:=false \
    -p publish_free_space:=false \
    -p occupancy_min_z:=-12.0 \
    -p occupancy_max_z:=1.0 \
    >> /root/log/xudrone_octomap.log 2>&1 &
  echo "$(date -u +%FT%TZ) octomap_server started (pid $!)"
else
  echo "$(date -u +%FT%TZ) octomap_server not installed; 2D map and TF only"
fi

echo "$(date -u +%FT%TZ) xudrone mapper starting"
exec python3 -u /root/udrone_boat/behaviours/xudrone_mapper.py
