# Kiwanis Lake aquatic world

Used by `digital-twin-aquatic` (`PX4_GZ_WORLD=kiwanis_lake`). Production otter
on Tempe Town Lake is a different world and container.

## Geography

- Origin: **33.374047 N, 111.939580 W** (lake centre); elev ≈ 335.28 m; ~12.7 acres
- Shoreline: OSM way `29210987` (`kiwanis_shore_enu.json`); checked against satellite —
  NW lobe, NE peninsula pinch (~96 m), SE tip, NW cove; long-axis ~457 m ENU
- Visuals: earcut water mesh (~half transparent, ripple albedo
  `materials/textures/water_ripples.png`, planar UVs ~12 m/tile); depth-dish bed
  (shore ~1 m → centre 2.44 m) with tiled silt/sand albedo
  (`materials/textures/bed_sand.png`, ~8 m/tile); park mesh with lake hole; west
  pier stub; AprilTag 36h11 id 0 on a UV plane on the lake bed at ENU
  (5 E, 4 N, z≈−2.08); water uses classic transparency (no PBR) so the
  down-camera can see the bed/tag; static bottom
  debris and surface floats clear of the `0,0` spawn. Regenerate textures/UVs
  with `python3 mk_bed_texture.py` in this model folder.
- Sensors / perception: otter_k nadir (`/otter_k/camera_down/*`); xudrone front
  (`/xudrone/camera/*`, primary until-stop while pitched) + nadir
  (`/xudrone/camera_down/*`). `apriltag_ros` + `tag_detector` →
  `/coral/detections` and `/coral/tracks`. Boat `survey` / AUV `coverage` can
  stop on `until: tag_detected`; `approach_tag` swims closed-loop to the tag
  ENU (PBVS). Screenshots: [img/](img/).
- Collision: tiled bed boxes + bank walls; park AABB slab visuals (ODE triangle-mesh
  collision is not used — it segfaults)

## Vehicles in this world

| callsign | model | spawn | notes |
|---|---|---|---|
| **otter_k** | `heron_0` | `0,0,0.25` | surface boat; DDS `/otter_k` |
| **xudrone** (CoRAL) | `xudrone_1` | at the boat, then nest `0 0 0` boat-frame | docked under the catamaran |
| **xudrone** (solo) | `xudrone_0` | `0,0,-1.0` | AUV only |

uDrone mass 13.5 kg, displacement 0.01370 m³ → awash at z≈−0.11 with the hull top
at the waterline. See [../README.md](../README.md) for start/stop and demos.
