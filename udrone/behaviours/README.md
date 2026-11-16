# CoRAL behaviour library

Typed command → dock / boat goto / survey / helix / swim / hoop / track / coverage / approach_tag → event.
Runs inside `digital-twin-aquatic` on `/coral/{command,event,status,detections,tracks}`.

| runner | command | role |
|---|---|---|
| (in-process) | `dock`, `goto`, `survey`, `hold`, `abort` | boat + latch |
| `run_helix.py` | `helix` | AUV forward helix |
| `run_swim.py` | `swim` | AUV straight leg (compass, world, or along heading) |
| `run_hoop.py` | `hoop` | Front-camera search, then one join onto the centerline through the orange ring |
| `run_track_otter.py` | `track` | Hold under otter_k on deck tag 10 (`track_geom.py`) |
| `run_coverage.py` | `coverage` | AUV 3D lawnmower (`patterns.volume_lawnmower`) |
| `run_approach_tag.py` | `approach_tag` | Expanding spiral search → front-cam IBVS rates |
| `run_tags.sh` | (launch) | apriltag_ros + `tag_detector` for otter_k / xudrone |

Helix, swim, hoop, track, and coverage share `UdroneHelixController` (force → point-hull-axis).
`swim` is one polyline leg. `hoop` (`hoop_geom.py`) covers a 2 m dock-centred
grid until the front camera's orange pixels form a ring, then flies one Hermite
join onto the centerline. Once three centre fixes agree, that line is kept.
A pass needs the ring around the optical axis and the hull inside the opening
at ENU (6, 0, −1). Pitch from `rpy_from_q` is positive nose-down.
`/xudrone/hoop_debug/image` overlays the ring, the planned path, and the command
arrow. `track` holds the upward camera about 1 m under AprilTag 10 on the
Heron deck. The method is Harish Anand's ASU thesis
([PDF](https://keep.lib.asu.edu/_flysystem/fedora/c7/233928/Anand_asu_0010N_20290.pdf));
Kiwanis uses a 1 m standoff because the lake is about 2.4 m deep.
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
the tag”, or `xudrone: track otter_k`).
