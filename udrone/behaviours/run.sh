#!/bin/bash
# Start the CoRAL behaviour node inside digital-twin-aquatic.
#   /usr/local/bin/coral_behaviours.sh
# or:  bash /root/udrone_boat/behaviours/run.sh
set -e
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-84}"
export GZ_PARTITION="${GZ_PARTITION:-aquatic}"
export PYTHONUNBUFFERED=1
export UDRONE_SRC="${UDRONE_SRC:-/root/udrone_boat/src}"
export UDRONE_HELIX_RUNNER="${UDRONE_HELIX_RUNNER:-/root/udrone_boat/behaviours/run_helix.py}"
export UDRONE_COV_RUNNER="${UDRONE_COV_RUNNER:-/root/udrone_boat/behaviours/run_coverage.py}"

set +u
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
set -u

# Prefer the pack install path; fall back to this script's directory (dev).
ROOT="/root/udrone_boat/behaviours"
if [ ! -f "$ROOT/coral_behaviours.py" ]; then
  ROOT="$(cd "$(dirname "$0")" && pwd)"
fi
export PYTHONPATH="$ROOT${PYTHONPATH:+:$PYTHONPATH}"
exec python3 -u "$ROOT/coral_behaviours.py"
