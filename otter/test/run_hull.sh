#!/bin/bash
# Open-loop hull test: pause the thrust adapter (it publishes 0 N at 100 Hz), drive the jets
# directly, then resume the adapter. Run on dreamslab; refuses while the boat is armed.
# usage: run_hull.sh NAME port_N,starboard_N,seconds [...]
#   e.g. run_hull.sh h1 33.6,33.6,14 0,0,6 15,-15,10 0,0,5 -19.88,-19.88,10 0,0,6 33.6,10,14 0,0,6
# Prints surge/sway speed, yaw rate, pitch, roll and the speed time constant per segment.
C=digital-twin-water-tempe-lake
HERE=$(cd "$(dirname "$0")" && pwd)
NAME=$1; shift
tr -d '\r' < "$HERE/hull_test.py" > /tmp/hull_test.$$ && docker cp /tmp/hull_test.$$ $C:/tmp/hull_test.py; rm -f /tmp/hull_test.$$
docker exec $C bash -c "
  armed=\$(simctl status | grep -c ' armed')
  if [ \"\$armed\" != 0 ]; then echo 'boat is armed; not running an open-loop test'; exit 1; fi
  pid=\$(pgrep -f '^/usr/local/bin/heron_thrust_adapter\$')
  kill -STOP \$pid
  trap 'kill -CONT '\$pid EXIT
  export GZ_PARTITION=tempe_lake PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
  python3 /tmp/hull_test.py $NAME $*
"
