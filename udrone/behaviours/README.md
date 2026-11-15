# CoRAL behaviour library

Typed command → dock / boat goto / survey / helix / coverage / approach_tag → event.
Runs inside `digital-twin-aquatic` on `/coral/{command,event,status,detections,tracks}`.

| runner | command | role |
|---|---|---|
| (in-process) | `dock`, `goto`, `survey`, `hold`, `abort` | boat + latch |
| `run_helix.py` | `helix` | AUV forward helix |
| `run_swim.py` | `swim` | AUV straight leg (compass, world, or along heading) |
| `run_hoop.py` | `hoop` | Front-camera search, then a forward sweep through the orange ring |
| `run_coverage.py` | `coverage` | AUV 3D lawnmower (`patterns.volume_lawnmower`) |
| `run_approach_tag.py` | `approach_tag` | Expanding spiral search → front-cam IBVS rates |
| `run_tags.sh` | (launch) | apriltag_ros + `tag_detector` for otter_k / xudrone |

Helix, swim, hoop, and coverage share `UdroneHelixController` (force → point-hull-axis).
`swim` is one polyline leg. `hoop` (`hoop_geom.py`) covers a 2 m dock-centred
grid until the front camera's orange pixels form a ring, then flies a forward
sweep to a standoff and, once three centre fixes agree, freezes the line
through the opening. Pitch from `rpy_from_q` is positive nose-down.
`/xudrone/hoop_debug/image` overlays the ring, any AprilTag, and the command
arrow; tags on that image are not used for steering. The pass is scored
against the estimated centre, not the authored (6, 0, −1) pose.
`approach_tag` subclasses the same offboard path: **search** = attitude-rate
expanding spiral around an ENU seed / last lock until `/xudrone/detections`
locks, then IBVS pitch/yaw/surge into `VehicleRatesSetpoint`; lost track
returns to search (radius grows). Clearance uses `/xudrone/range/{bottom,surface}`
(EchoRanger + upward beam) plus pose depth to keep off the bed / free surface.
Downward EchoRanger also feeds PX4 `distance_sensor` via GZBridge
(`EKF2_RNG_CTRL=1`; baro remains `EKF2_HGT_REF`). See the pack
[README](../README.md#behaviour-library-coral) for schema and CLI.

LLM path (tesseract): `coral_bridge` + `coral_instructions` map operator /
`instruction` text onto these commands (e.g. “short survey until tag, then go to
the tag”).
