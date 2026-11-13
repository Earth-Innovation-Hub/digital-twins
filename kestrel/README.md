# kestrel — x500 with gimbal on the Bishop scarp

Callsign **kestrel**, shares container `digital-twin-locutus` with badger, ROS domain 53, panel port 40088.

## What this folder ships

Nothing beyond this note. The vehicle itself is the stock PX4 Gazebo model
`x500_gimbal` (airframe `4019_gz_x500_gimbal`) from
[PX4/PX4-gazebo-models](https://github.com/PX4/PX4-gazebo-models).

Lab-specific pieces live elsewhere:

| piece | home |
|---|---|
| Twin image (ROS, relays, entrypoint) | [`Open-UAV/openuav-turbovnc`](https://github.com/Open-UAV/openuav-turbovnc) `Dockerfile.twin`, `start_twin.sh` |
| Bishop fault-scarp terrain + GPS origin | same repo, `terrain/bishop_scarp/` (generated assets, ~360 MB, not in git) |
| Callsign / topic roots (`PX4_UXRCE_DDS_NS=kestrel`) | [`DREAMS-lab/dreams_llm_ros2`](https://github.com/DREAMS-lab/dreams_llm_ros2) |
| Surface / mission executor | `dreams_robot_agent` in that repo |

If a future change makes the airframe or model diverge from upstream, put the
delta here the same way `../badger/` holds `r1_rover_cam`.

## Site template (AI Pilot create_twin)

`worlds/site_template/` is the Gazebo world shell filled by
[`../twin_builder`](../twin_builder) for on-demand twins at a Cesium look-at.
Session callsigns are `ktwin-*` and never replace Locutus.
