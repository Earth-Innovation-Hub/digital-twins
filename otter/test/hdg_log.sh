#!/bin/bash
# Log PX4's heading against the simulator's, with the estimator's compass and gyro bias.
#   hdg_log.sh start [seconds]   run detached in the container, output /tmp/hdg_log.txt
#   hdg_log.sh show              summary + every 5th line
#   hdg_log.sh params            the estimator's timing parameters
C=digital-twin-water-tempe-lake
case "$1" in
start)
    docker exec -d $C bash -c '
      cd /root/PX4-Autopilot/build/px4_sitl_default
      end=$(( $(date +%s) + '"${2:-120}"' ))
      : > /tmp/hdg_log.txt
      while [ $(date +%s) -lt $end ]; do
        e=$(./bin/px4-listener vehicle_attitude 1 | sed -n "s/.*Yaw: \(.*\) deg.*/\1/p")
        t=$(./bin/px4-listener vehicle_attitude_groundtruth 1 | sed -n "s/.*Yaw: \(.*\) deg.*/\1/p")
        b=$(./bin/px4-listener estimator_sensor_bias 1 | sed -n "s/.*mag_bias: \[\(.*\)\]/\1/p;s/.*gyro_bias: \[\(.*\)\]/\1/p" | tr "\n" " ")
        r=$(./bin/px4-listener vehicle_angular_velocity_groundtruth 1 | sed -n "s/.*xyz: \[.*, \(.*\)\]/\1/p")
        echo "$(date +%s.%N | cut -c1-14) $e $t | $r | $b" >> /tmp/hdg_log.txt
        sleep 0.3
      done'
    echo started
    ;;
show)
    docker exec $C python3 -c '
import math
rows = []
for line in open("/tmp/hdg_log.txt"):
    p = line.replace("|", " ").replace(",", " ").split()
    try:
        rows.append([float(x) for x in p])
    except ValueError:
        pass
t0 = rows[0][0]
errs = []
for i, r in enumerate(rows):
    err = (r[1] - r[2] + 180) % 360 - 180
    errs.append(err)
    if i % 5 == 0:
        print("%6.1f s  px4 %7.1f  truth %7.1f  err %+5.1f  yaw rate %+5.1f deg/s  gyro bias z %+.4f  mag bias %+.3f %+.3f %+.3f" % (
            r[0] - t0, r[1], r[2], err, math.degrees(r[3]), r[6], r[7], r[8], r[9]))
a = [abs(e) for e in errs]
print("heading error: mean %+.1f  rms %.1f  max %.1f deg over %d samples" % (sum(errs) / len(errs), math.sqrt(sum(e * e for e in errs) / len(errs)), max(a), len(a)))
'
    ;;
params)
    docker exec $C bash -c 'cd /root/PX4-Autopilot/build/px4_sitl_default; for p in EKF2_GPS_DELAY EKF2_MAG_DELAY EKF2_BARO_DELAY EKF2_GPS_CTRL EKF2_GPS_V_NOISE EKF2_GPS_P_NOISE EKF2_GYR_B_NOISE EKF2_HEAD_NOISE EKF2_MAG_NOISE EKF2_MAG_TYPE EKF2_PREDICT_US IMU_INTEG_RATE EKF2_GPS_CHECK EKF2_REQ_HDRIFT; do ./bin/px4-param show $p 2>/dev/null | grep -E "^[x +*]" | sed "s/ \[[0-9,]*\]//"; done; grep -n "EKF2_\|GPS_DELAY" /root/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/px4-rc.simulator /root/PX4-Autopilot/ROMFS/px4fmu_common/init.d/rc.rover_differential_defaults /root/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/rcS | head -20; grep -n "navsat" -A 8 /root/PX4-Autopilot/src/modules/simulation/gz_bridge/GZBridge.cpp | grep -n "timestamp\|delay" | head'
    ;;
esac
