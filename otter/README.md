# otter — Heron ASV on Tempe Town Lake

Callsign **otter**, container `digital-twin-water-tempe-lake`, ROS domain 27, panel port 40099.

## Layout

| path | installs to |
|---|---|
| `model/` | `PX4-Autopilot/Tools/simulation/gz/models/heron/` |
| `world/tempe_lake.sdf` | `…/gz/worlds/tempe_lake.sdf` |
| `airframe/4023_gz_heron[.post]` | PX4 ROMFS + build-tree airframes |
| `adapter/heron_thrust_adapter.cpp` | `/root/…` → `/usr/local/bin/heron_thrust_adapter` |
| `px4/px4_rover_differential_patch.py` | patches `DifferentialPosVelControl.cpp`, then `ninja px4` |
| `start_boat.sh` | `/root/start_boat.sh` |

## Install

On the twin host, with the boat **disarmed**:

```bash
git clone https://github.com/Earth-Innovation-Hub/digital-twins.git
cd digital-twins/otter
bash install.sh          # C=digital-twin-water-tempe-lake by default
```

About 90 s. Idempotent. See `install.sh` for the checks it prints afterwards.

## Test

```bash
cd test
bash run_hull.sh h1 33.6,33.6,12 0,0,6 15,-15,8
bash run_closed.sh c1 goto:0:20 orbit:10:30 rtl:120 land
bash run_closed.sh -w c1 300
bash hdg_log.sh start 60; bash hdg_log.sh show
```

## Design notes

- Graded buoyancy: eight boxes per hull (one long box had no pitch restoring moment).
- Sway drag `yVabsV` −120 is an estimate from hull shape, not a tank measurement.
- No Gazebo magnetometer/barometer in the model; PX4 uses `SENS_EN_MAGSIM` / `BAROSIM`.
- `.post` stops online `gyro_calibration` (it treated a steady disarmed turn as sensor bias).
- The PX4 patch makes the boat stop at a goal with no next waypoint, and ignores the
  `_mission_finished` latch outside Mission/Return. Upstream PR to PX4 is welcome; until
  then this repo pins the patch to `v1.16.0-alpha2-441` (`9ac03f03eb`).
