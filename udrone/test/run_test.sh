#!/bin/bash
# Run one of the aquatic-twin tests inside the container with the right
# ROS / Gazebo environment.
#   run_test.sh latch      dock latch transitions and gates (no thrust)
#   run_test.sh transit    otter_k drives a leg carrying the docked xudrone
#   run_test.sh undock     docked helix, DETACH, free helix (CoRAL)
#   run_test.sh helix      helix controller on a free xudrone
#   run_test.sh redock     teleport xudrone back under the boat and latch it
# Extra environment (UDRONE_HELIX_*, UDRONE_UNDOCK_AT, BOAT_LEG_M, ...) passes through.
# Exit status is the test's: 0 pass, 1 fail.
here="$(cd "$(dirname "$0")" && pwd)"
case "${1:-}" in
  helix)   script=udrone_sinusoidal_controller.py ;;
  undock)  script=udrone_undock_helix_test.py ;;
  transit) script=boat_transit_test.py ;;
  latch)   script=latch_test.py ;;
  redock)  script=redock.py ;;
  *) echo "usage: $0 latch|transit|undock|helix|redock" >&2; exit 2 ;;
esac
set +u
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-84}"
export GZ_PARTITION="${GZ_PARTITION:-aquatic}"
export PYTHONUNBUFFERED=1
exec python3 "$here/$script"
