#!/bin/bash
# badger, the Aion R1 rover with a forward camera: PX4 instance 1 in the twin's
# running Gazebo world (instance 0 is kestrel, the x500_gimbal). Every topic is
# rooted at its callsign. Also starts, and stops with it:
#   px4_relay (ns /badger)  /badger/fmu/out/* -> /badger/px4/*   (routed by zenoh-locutus)
#   badger camera           gz camera -> /badger/camera/image, /badger/camera/camera_info (lazy)
#   ground_executor (ns /badger)  /badger/mission_supervisor/command -> PX4 repositions
#                           (log /root/log/ground_executor.log)
ROBOT=badger
if pgrep -f "[b]in/px4 -i 1" >/dev/null; then
  echo "rover px4 already running"
  exit 0
fi
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
cd /root/PX4-Autopilot/build/px4_sitl_default/rootfs
. ./gz_env.sh
ready=0
for _ in $(seq 1 90); do
  if gz topic -l 2>/dev/null | grep -q '^/world/default/clock$'; then
    ready=1
    break
  fi
  sleep 1
done
if [ "$ready" != 1 ]; then
  echo "ERROR rover: /world/default/clock never appeared" >&2
  exit 1
fi

MODEL=r1_rover_cam_1
gz service -s /world/default/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean \
  --timeout 3000 --req "name: \"$MODEL\" type: MODEL" >/dev/null 2>&1

export ROS_DOMAIN_ID=53
children=()
cleanup() {
  for pid in "${children[@]}"; do kill "$pid" 2>/dev/null; done
  pkill -f "[b]in/px4 -i 1" 2>/dev/null
}
trap cleanup EXIT INT TERM

ros2 run mission_supervisor px4_relay --ros-args -r __node:=px4_relay -r __ns:=/$ROBOT &
children+=($!)

mkdir -p /root/log
python3 -m mission_supervisor.ground_executor --ros-args -r __ns:=/$ROBOT >> /root/log/ground_executor.log 2>&1 &
children+=($!)

CAM=/world/default/model/$MODEL/link/front_camera_link/sensor/camera
ros2 run ros_gz_bridge parameter_bridge \
  "$CAM/image@sensor_msgs/msg/Image[gz.msgs.Image" \
  "$CAM/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo" \
  --ros-args -r __node:=camera_bridge -r __ns:=/$ROBOT -p lazy:=true \
  -r "$CAM/image:=/$ROBOT/camera/image" -r "$CAM/camera_info:=/$ROBOT/camera/camera_info" &
children+=($!)

# The twin launch exports PX4_UXRCE_DDS_NS=kestrel for instance 0; this one is badger
export PX4_UXRCE_DDS_NS=$ROBOT
export PX4_SYS_AUTOSTART=4009
export PX4_SIM_MODEL=gz_r1_rover_cam
export PX4_GZ_WORLD=default
export PX4_GZ_MODEL_POSE="-8,6,0.35,0,0,0"
../bin/px4 -i 1 -d
