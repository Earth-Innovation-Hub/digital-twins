# udrone — uDrone AUV on Kiwanis Lake

Callsign **udrone** (DDS NS `udrone`), aquatic sandbox container `digital-twin-aquatic`
(host panel **:40077**). Isolated from production otter (`:40099`) via `ROS_DOMAIN_ID=84`
and `GZ_PARTITION=aquatic`.

## Layout

| path | installs to |
|---|---|
| `model/xudrone/` | `PX4-Autopilot/Tools/simulation/gz/models/xudrone/` |
| `world/kiwanis_lake.sdf` + `world/kiwanis_lake/` | `…/gz/worlds/` and `…/gz/models/kiwanis_lake/` |
| `airframe/60003_gz_xudrone` | PX4 ROMFS + build-tree airframes |
| `adapter/udrone_thrust_adapter.cpp` | `/root` → `/usr/local/bin/udrone_thrust_adapter` |
| `start_udrone.sh` | `/root/start_udrone.sh` |
| `test/` | sinusoidal offboard controller (ROS2 / px4_msgs) |

## Install

On dreamslab, with the aquatic sim **stopped**:

```bash
git clone https://github.com/Earth-Innovation-Hub/digital-twins.git
cd digital-twins/udrone
C=digital-twin-aquatic bash install.sh
```

Then: `simctl stop; vglrun /root/start_udrone.sh` (or your usual aquatic launcher).

## Test

```bash
cd test
ROS_DOMAIN_ID=84 GZ_PARTITION=aquatic \
  UDRONE_SINE_DUR=30 UDRONE_SINE_AMP=1.5 UDRONE_SINE_OMEGA=0.28 \
  python3 udrone_sinusoidal_controller.py
```

Requires `source /opt/ros/humble/setup.bash` and the lab `px4_msgs` workspace.

## Design notes

- Gazebo `Thruster` plugins + `udrone_thrust_adapter` (PWM/`motor_speed` → `cmd_thrust`).
  Adapter subscribes to `/xudrone_0/command/motor_speed` (PX4 SIM_GZ_EC topic).
  Idle `velocity=0` maps to **0 N** (not reverse).
- Control allocation: `CA_AIRFRAME=7` (UUV), `CA_ROTORn_CT=1`, `UUV_SKIP_CTRL=1`.
  Default CT 6.5 put equal-surge cmds under the 2% stop-motor threshold (all-NaN motors).
- Kiwanis world: OSM shoreline, lake-center origin, tiled-box bed collision (ODE mesh
  collision segfaults). See `docs/KIWANIS.md`.
- Near-neutral displacement on `xudrone`. Spawn pose `0,0,-1.0`.
- Sinusoidal controller is a ROS2 port of legacy `udrone_boat` MAVROS code; uses Gazebo
  pose for feedback (PX4 local position on this twin is unreliable).
