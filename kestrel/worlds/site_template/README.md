# site_template

Gazebo world template for AI Pilot `create_twin`.

`twin_builder.bake` copies this folder into a job directory, writes
`textures/heightmap.png` and `textures/ortho.png` from DEM/nadir capture, fills
`site.sdf.in` → `site.sdf` with spherical_coordinates at the Cesium look-at, and
splices VLM prop models into `{{PROPS}}`.

Do not point production Locutus at this template; session containers use a unique
`ktwin-*` callsign, ROS domain, and GZ partition.
