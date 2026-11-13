#!/bin/bash
# Container half of install.sh. Run as a file (never through `bash -c "<text>"`):
# the process patterns below must not appear on this shell's own command line.
#   install_inner.sh build     compile the adapter and the latch plugin
#   install_inner.sh armed     exit 1 if any running PX4 instance is armed
#   install_inner.sh install   put everything in place (simulator stopped)
set -e
N=/tmp/udrone_new
P=/root/PX4-Autopilot
B=$P/build/px4_sitl_default
SFX=bak-$(date +%Y%m%d)

case "${1:-}" in
build)
  cd $N
  g++ -O2 -std=c++17 udrone_thrust_adapter.cpp -o udrone_thrust_adapter \
    $(pkg-config --cflags --libs gz-transport13 gz-msgs10)
  g++ -O2 -std=c++17 -shared -fPIC -o libgz-sim-link-attacher-system.so \
    LinkAttacher.cc $(pkg-config --cflags --libs gz-sim8 gz-plugin2)
  echo "built adapter + latch plugin"
  ;;

armed)
  armed=0
  for pid in $(pgrep -x px4 || true); do
    inst=$(tr '\0' ' ' < /proc/$pid/cmdline | sed -n 's/.* -i \([0-9]*\).*/\1/p')
    state=$(timeout 5 $B/bin/px4-listener --instance "${inst:-0}" vehicle_status -n 1 2>/dev/null |
            awk '/^ *arming_state:/ {print $2; exit}')
    echo "px4 instance ${inst:-0}: arming_state ${state:-unknown}"
    if [ "$state" = 2 ]; then armed=1; fi
  done
  [ "$armed" = 0 ]
  ;;

install)
  if pgrep -x px4 >/dev/null || pgrep -f '^gz sim' >/dev/null; then
    echo "simulator still running; stop it first" >&2
    exit 1
  fi
  bk() { [ ! -e "$1" ] || [ -e "$1.$SFX" ] || cp -a "$1" "$1.$SFX"; }
  Mdir=$P/Tools/simulation/gz/models/xudrone
  Hdir=$P/Tools/simulation/gz/models/heron
  Kdir=$P/Tools/simulation/gz/models/kiwanis_lake
  W=$P/Tools/simulation/gz/worlds/kiwanis_lake.sdf
  A1=$P/ROMFS/px4fmu_common/init.d-posix/airframes/60003_gz_xudrone
  A2=$B/etc/init.d-posix/airframes/60003_gz_xudrone
  mkdir -p $Mdir $Hdir $Kdir /root/udrone_boat/src /usr/local/lib/gz-sim-8/plugins
  for f in $Mdir/model.sdf $Mdir/model.config $Hdir/model.sdf $Hdir/model.config $W $A1 \
           /root/udrone_thrust_adapter.cpp /usr/local/bin/udrone_thrust_adapter \
           /root/start_udrone.sh /root/start_boat.sh /root/stop_sim.sh \
           /usr/local/bin/xudrone_camera_bridge.sh /usr/local/bin/dock_command.py \
           /usr/local/bin/coral_behaviours.sh /usr/local/bin/coral_cmd.py; do
    bk "$f"
  done

  cp -a $N/xudrone_model.sdf $Mdir/model.sdf
  cp -a $N/xudrone_model.config $Mdir/model.config
  cp -a $N/heron_model.sdf $Hdir/model.sdf
  cp -a $N/heron_model.config $Hdir/model.config
  cp -a $N/kiwanis_lake.sdf $W
  rm -rf $Kdir
  cp -a $N/kiwanis_lake $Kdir
  cp -a $N/60003_gz_xudrone $A1
  mkdir -p $(dirname $A2)
  cp -a $N/60003_gz_xudrone $A2
  grep -q '60003_gz_xudrone' $P/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt \
    || echo 'px4_add_romfs_files(60003_gz_xudrone)' >> $P/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt

  # Helpers from an earlier boot (anchored to the executable path).
  pkill -f '^/usr/local/bin/udrone_thrust_adapter' 2>/dev/null || true
  pkill -f '^/usr/local/bin/udrone_dock_lock' 2>/dev/null || true
  pkill -f 'lib/ros_gz_bridge/parameter_bridge .*front_camera' 2>/dev/null || true
  sleep 1

  cp -a $N/udrone_thrust_adapter.cpp /root/udrone_thrust_adapter.cpp
  install -m 755 $N/udrone_thrust_adapter /usr/local/bin/udrone_thrust_adapter
  for d in /usr/local/lib/gz-sim-8/plugins /usr/lib/x86_64-linux-gnu/gz-sim-8/plugins; do
    install -m 755 $N/libgz-sim-link-attacher-system.so $d/libgz-sim-link-attacher-system.so
  done
  install -m 755 $N/start_udrone.sh /root/start_udrone.sh
  install -m 755 $N/start_boat.sh /root/start_boat.sh
  install -m 755 $N/stop_sim.sh /root/stop_sim.sh
  install -m 755 $N/xudrone_camera_bridge.sh /usr/local/bin/xudrone_camera_bridge.sh
  install -m 755 $N/dock_command.py /usr/local/bin/dock_command.py
  for f in $N/test/*; do install -m 755 "$f" /root/udrone_boat/src/; done
  mkdir -p /root/udrone_boat/behaviours
  for f in $N/behaviours/*; do install -m 755 "$f" /root/udrone_boat/behaviours/; done
  install -m 755 $N/behaviours/run.sh /usr/local/bin/coral_behaviours.sh
  install -m 755 $N/behaviours/cmd.sh /usr/local/bin/coral_cmd.py
  if [ -f $N/KIWANIS.md ]; then cp -a $N/KIWANIS.md /root/KIWANIS.md; fi

  # Superseded latch experiments.
  rm -f /usr/local/bin/udrone_dock_lock /usr/local/bin/udrone_dock_attach.sh

  bash $N/patch_simctl_coral.sh
  echo installed
  ;;

*)
  echo "usage: install_inner.sh build|armed|install" >&2
  exit 2
  ;;
esac
