#!/bin/bash
# apriltag_ros + tag_detector for otter_k or xudrone.
# Installed as /usr/local/bin/coral_tags.sh
#
# Usage: coral_tags.sh [otter_k|xudrone]
set -e
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash 2>/dev/null || true
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-84}
export GZ_PARTITION=${GZ_PARTITION:-aquatic}

VEHICLE=${1:-${CORAL_TAG_VEHICLE:-otter_k}}
ROOT="/root/udrone_boat/behaviours"
if [ ! -f "$ROOT/tag_detector.py" ]; then
  ROOT="$(cd "$(dirname "$0")" && pwd)"
fi
CFG=${CORAL_APRILTAG_YAML:-$ROOT/apriltag.yaml}
DET=${CORAL_TAG_DETECTOR:-$ROOT/tag_detector.py}

PIDS=""
cleanup() { kill $PIDS 2>/dev/null || true; }
trap cleanup EXIT

wait_topic() {
  local topic=$1
  for _ in $(seq 1 90); do
    if ros2 topic list 2>/dev/null | grep -qx "$topic"; then
      return 0
    fi
    sleep 1
  done
  echo "coral_tags: timeout waiting for $topic" >> /root/log/coral_tags.log
  return 1
}

start_apriltag() {
  local ns=$1 image=$2 info=$3
  shift 3
  # shellcheck disable=SC2086
  ros2 run apriltag_ros apriltag_node --ros-args \
    -r __ns:=$ns \
    -r image_rect:=$image \
    -r camera_info:=$info \
    "$@" \
    --params-file "$CFG" \
    >> /root/log/apriltag_${LOG_TAG}.log 2>&1 &
  PIDS="$PIDS $!"
}

case "$VEHICLE" in
  otter_k|boat)
    VEHICLE=otter_k
    LOG_TAG=otter_k
    wait_topic /otter_k/camera_down/image || true
    start_apriltag /otter_k /otter_k/camera_down/image /otter_k/camera_down/camera_info
    python3 -u "$DET" --ros-args \
      -r __ns:=/otter_k \
      -p supervisor:=/coral \
      -p gimbal_pitch_rad:=0.0 \
      -p camera_yaw_deg:=0.0 \
      -p max_range_m:=25.0 \
      -p settle_sec:=0.6 \
      -p min_samples:=2 \
      -r fmu/out/vehicle_local_position:=/otter_k/fmu/out/vehicle_local_position \
      -r detections:=detections \
      >> /root/log/tag_detector_${LOG_TAG}.log 2>&1 &
    PIDS="$PIDS $!"
    ;;
  xudrone|auv)
    VEHICLE=xudrone
    LOG_TAG=xudrone
    GZ_MODEL=${GZ_MODEL_NAME:-xudrone_1}

    # FRONT camera → /xudrone/detections (until-stop). During pitched coverage the
    # bed tag appears "in front"; otter-style nadir geometry does not apply here,
    # so this stream is for raw stop only (TF isolated).
    wait_topic /xudrone/camera/image || true
    start_apriltag /xudrone_front /xudrone/camera/image /xudrone/camera/camera_info \
      -r detections:=/xudrone/detections \
      -r /tf:=/xudrone/tf_front

    # DOWN camera → /xudrone/detections_down + tag_detector ENU (otter-k pattern).
    if wait_topic /xudrone/camera_down/image; then
      start_apriltag /xudrone /xudrone/camera_down/image /xudrone/camera_down/camera_info \
        -r detections:=/xudrone/detections_down \
        -r /tf:=/xudrone/tf
      # tracks_hz=2 keeps /coral/tracks live for approach_tag PBVS refine.
      python3 -u "$DET" --ros-args \
        -r __ns:=/xudrone \
        -p supervisor:=/coral \
        -p gimbal_pitch_rad:=0.0 \
        -p camera_yaw_deg:=0.0 \
        -p max_range_m:=25.0 \
        -p settle_sec:=0.4 \
        -p min_samples:=2 \
        -p tracks_hz:=2.0 \
        -p gz_model:=$GZ_MODEL \
        -r fmu/out/vehicle_local_position:=/xudrone/fmu/out/vehicle_local_position \
        -r detections:=detections_down \
        -r /tf:=/xudrone/tf \
        >> /root/log/tag_detector_${LOG_TAG}.log 2>&1 &
      PIDS="$PIDS $!"
    else
      echo "coral_tags(xudrone): camera_down missing; front-only until-stop" \
        >> /root/log/coral_tags.log
    fi
    ;;
  *)
    echo "usage: $0 otter_k|xudrone" >&2
    exit 2
    ;;
esac

echo "coral_tags($VEHICLE): pids=$PIDS cfg=$CFG" >> /root/log/coral_tags.log
wait
