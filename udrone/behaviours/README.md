# CoRAL behaviour library

Typed command → dock / boat goto / survey / helix / swim / hoop / track / rendezvous / shape / coverage / approach_tag → event.
Runs inside `digital-twin-aquatic` on `/coral/{command,event,status,detections,tracks}`.

| runner | command | role |
|---|---|---|
| (in-process) | `dock`, `goto`, `survey`, `hold`, `abort` | boat + latch |
| `run_helix.py` | `helix` | AUV forward helix |
| `run_swim.py` | `swim` | AUV straight leg (compass, world, or along heading) |
| `run_hoop.py` | `hoop` | Front-camera search, then one join onto the centerline through the orange ring |
| `run_track_otter.py` | `track` | Hold under otter_k on deck tag 10 (`track_geom.py`) |
| `run_rendezvous.py` | `rendezvous` | Nest-axis spiral → tag 10 → servo onto boat −Z → latch (`rendezvous_geom.py`) |
| `run_shape.py` | `shape` | Infinity / orbit / vertical loop (`shape_geom.py`) |
| `run_coverage.py` | `coverage` | AUV 3D lawnmower (`patterns.volume_lawnmower`) |
| `run_approach_tag.py` | `approach_tag` | Expanding spiral search → front-cam IBVS rates |
| `run_hold.py` | (between steps) | Holds the armed AUV where the last step ended until the next step takes it over (baton in `/tmp`) |
| `run_tags.sh` | (launch) | apriltag_ros + `tag_detector` for otter_k / xudrone |
| `xudrone_mapper.py` + `map_geom.py` | (launch, `xudrone_mapping.sh`) | TF, odom, 2D grid, costmap, OctoMap cloud, map image, coverage plans |
| `factor_graph.py` | (used by mapper) | GTSAM iSAM2 pose graph (keyframes + tag landmarks) |
| `coverage_plan.py` | (used by mapper / `coverage --plan map`) | Map-aware boustrophedon over costmap gaps |

Helix, swim, hoop, track, and coverage share `UdroneHelixController` (force → point-hull-axis).
`swim` is one polyline leg. `hoop` (`hoop_geom.py`) covers a 2 m dock-centred
grid until the front camera's orange pixels form a ring, then flies one Hermite
join onto the centerline. Once three centre fixes agree, that line is kept.
A pass needs the ring around the optical axis and the hull inside the opening
at ENU (6, 0, −1). Pitch from `rpy_from_q` is positive nose-down.
Every runner on `UdroneHelixController` publishes one HUD image,
`/xudrone/hud/image` (latest frame also at `/tmp/xudrone_hud.jpg`, once a
second; `UDRONE_HUD=0` turns it off). `hud_overlay.py` draws the task's camera
(front for swim, shape, coverage, helix, hold, and hoop; up for track and
rendezvous; front or down as `approach_tag` switches eyes). On that camera it
adds the AprilTag boxes and the projected reference, targets, and boat. The front
camera also gets the horizon, the commanded hull direction, and the command arrow.
Around the edges sit a telemetry panel, the other two cameras, and a north-up
map that carries the bed depth and obstacles from `xudrone_mapper` (see the pack
[README](../README.md#mapping)). A runner adds its own layer with `_hud_layers()`: the hoop adds the ring
box, planned path, end, and fix; the rendezvous adds the nest-axis acceptance
cone, nest, lateral/tilt, and dock state. Geometry is in `rendezvous_geom`
(nest between pontoons, approach along boat −Z, plates parallel). `track`
holds the upward camera about 1 m under AprilTag 10 on the
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
the tag”, `xudrone: track otter_k`, or “undock, search for and approach tag and
when complete, rendezvous with otter_k” → detach → `approach_tag` → `rendezvous`).
