# Kiwanis Lake aquatic world

Used by `digital-twin-aquatic` (`PX4_GZ_WORLD=kiwanis_lake`). Production otter
on Tempe Town Lake is a different world and container.

## Geography

- Origin: **33.374047 N, 111.939580 W** (lake centre); elev ≈ 335.28 m; ~12.7 acres
- Shoreline: OSM way `29210987` (`kiwanis_shore_enu.json`); checked against satellite —
  NW lobe, NE peninsula pinch (~96 m), SE tip, NW cove; long-axis ~457 m ENU
- Visuals: see-through wavy water (classic material so the cameras still
  see the bed; chop and sun glint are in the mesh normals; an underside sheet
  for the upward view; a light colour veil below the nadir cameras; a short
  foam band on the bank); bowl bed
  (shore 1 m, deepest point 10 m, steepest at the bank) with a tiled coral-reef
  albedo (`materials/textures/bed_sand.png`, ~8 m/tile; cream, purple, pink,
  teal, and green heads on sand, kept off the hoop orange gate); park mesh with lake hole; west
  pier stub; AprilTag 36h11 id 0 (0.40 m), one plate flat on the bed at
  ENU (5 E, 4 N, z=−9.44), facing up, just above the bed; a 1 m
  hoop at ENU (6 E, 0 N, z=−4.63), the midpoint of the water column there;
  AprilTag 36h11 id 10 (0.40 m) centered on the underside of the Heron deck,
  facing down (nest search / track / rendezvous); ids 11 and 12 (0.14 m on
  0.16 m plates) inside the 0.24 m pontoon stern faces, facing aft; static bottom debris and
  surface floats clear of the `0,0` spawn. Regenerate textures/UVs with
  `python3 mk_bed_texture.py` in this model folder.
- Sensors / perception: otter_k nadir (`/otter_k/camera_down/*`); xudrone front
  (`/xudrone/camera_front/*`, primary until-stop while pitched), nadir
  (`/xudrone/camera_down/*`), and upward (`/xudrone/camera_up/*`, pitched −90°
  on the hull crown so it sees deck tag 10). `apriltag_ros` + `tag_detector` →
  `/coral/detections` and `/coral/tracks`. Front cam → `/xudrone/detections`;
  up cam → `/xudrone/detections_up` (deck tag 10). `track` holds a metre under
  otter_k (`xudrone: track otter_k`; detach first). `search_otter` reaches under
  the nest and locks deck tag 10; `rendezvous` servos on that tag and latches
  (`xudrone: rendezvous with otter-k`). Boat `survey` / AUV `coverage` can
  stop on `until: tag_detected`; `approach_tag` front-cam IBVS (attitude rates
  + thrust) to a bed tag. Screenshots: [img/](img/).
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
