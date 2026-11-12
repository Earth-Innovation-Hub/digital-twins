#!/bin/bash
# Closed-loop leg test through the surface executor (the same commands the supervisor
# sends), measured against Gazebo's pose. Run on dreamslab. Detached; output in the
# container at /tmp/closed_NAME.out and .csv.
#   start:       run_closed.sh NAME leg [leg ...]
#   wait/print:  run_closed.sh -w NAME [seconds]
# legs: goto:<bearing relative to the bow, deg>:<metres>[:timeout_s]   orbit:<radius>:<seconds>[:ccw]
#       rtl[:timeout_s]   wait:<seconds>   land (stop and disarm; end every run with it)
#   e.g. run_closed.sh f1 goto:0:25 goto:135:20 goto:90:5 orbit:6:40 rtl:150 goto:45:12 land
# Start with the boat disarmed if the run has an rtl leg (PX4 takes home where it arms).
C=digital-twin-water-tempe-lake
if [ "$1" = "-w" ]; then
  NAME=$2; T=${3:-300}
  for i in $(seq 1 $T); do
    docker exec $C pgrep -f "closed_test.py $NAME " >/dev/null || break
    sleep 1
  done
  docker exec $C pgrep -f "closed_test.py $NAME " >/dev/null && echo "(still running)"
  docker exec $C bash -c "grep -v -E '^\[(INFO|WARN)\]' /tmp/closed_$NAME.out"
  exit 0
fi
HERE=$(cd "$(dirname "$0")" && pwd)
NAME=$1; shift
docker exec $C pgrep -f "python3 -u /tmp/closed_test.py" >/dev/null && { echo "a closed_test is already running"; exit 1; }
tr -d '\r' < "$HERE/closed_test.py" > /tmp/closed_test.$$ && docker cp /tmp/closed_test.$$ $C:/tmp/closed_test.py; rm -f /tmp/closed_test.$$
docker exec -d $C bash -c "
  source /opt/ros/humble/setup.bash
  source /root/ros2_ws/install/setup.bash
  export ROS_DOMAIN_ID=27 GZ_PARTITION=tempe_lake PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
  python3 -u /tmp/closed_test.py $NAME $* > /tmp/closed_$NAME.out 2>&1
"
echo "started $NAME"
