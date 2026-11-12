# Kiwanis Lake aquatic world

- Origin: 33.374047 N, 111.939580 W (lake center); elev 335.28 m; ~12.7 acres (OSM 29210987)
- Visuals: earcut water mesh; depth-dish bed (shore ~1 m → center 2.44 m); park mesh with lake hole
- Collision: 38 tiled bed boxes (36 m, inside shoreline only) + 44 bank walls; park AABB slab visuals
  (ODE triangle-mesh collision segfaults — not used)
- No lawn plane under the water
- uDrone displacement ~0.013 m³ (near-neutral)
- Starts: uDrone 0,0,-1.0; surface craft 0,0,0.25; PX4_GZ_WORLD=kiwanis_lake
