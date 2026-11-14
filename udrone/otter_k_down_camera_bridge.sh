#!/bin/bash
# otter_k fixed nadir camera -> /otter_k/camera_down/{image,camera_info}
# Borrowed from Tempe heron_camera_bridge.sh (gimbal path kept separate).
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-84}
export GZ_PARTITION=${GZ_PARTITION:-aquatic}
C=/world/kiwanis_lake/model/heron_0/link/down_camera_link/sensor/down_camera
exec ros2 run ros_gz_bridge parameter_bridge \
  $C/image@sensor_msgs/msg/Image[gz.msgs.Image \
  $C/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
  --ros-args -r __node:=otter_k_down_camera_bridge -r __ns:=/otter_k \
  -r $C/image:=/otter_k/camera_down/image \
  -r $C/camera_info:=/otter_k/camera_down/camera_info
