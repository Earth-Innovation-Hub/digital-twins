# badger — Aion R1 with forward camera

Callsign **badger**, shares container `digital-twin-locutus` with kestrel
(PX4 instance 1; kestrel is instance 0 — same two-robot pattern as aquatic
**otter_k** + **xudrone**), ROS domain 53.

## Layout

| path | installs to |
|---|---|
| `models/r1_rover_cam/model.sdf` + `model.config` | `PX4-Autopilot/Tools/simulation/gz/models/r1_rover_cam/` |
| (meshes) | copied from stock `models/r1_rover/meshes/` at install time — not vendored |
| `start_rover.sh` | `/root/start_rover.sh` |

The SDF is the upstream `r1_rover` model renamed to `r1_rover_cam`, plus a
`front_camera_link` (pose `0.2 0 0.3`, pitch 0.15 rad, 640×480, hfov 1.274, 5 Hz).
Airframe stays stock `4009_gz_r1_rover`; `PX4_SIM_MODEL=gz_r1_rover_cam` selects this model.

## Install

```bash
cd digital-twins/badger
bash install.sh          # C=digital-twin-locutus by default
```

Does not restart the twin. After install, restart the rover process (or the
container) so instance 1 picks up the model.

## Notes

- `start_rover.sh` also starts `px4_relay`, `ground_executor`, and a lazy
  `ros_gz_bridge` camera under the `/badger` namespace. Those ROS packages come
  from `dreams_llm_ros2`, not this repo.
- Mesh licence remains Aion's (`aion_io`); we only ship the camera SDF delta.
