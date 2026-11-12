# digital-twins

PX4 Gazebo SITL vehicle packs for the Earth Innovation Hub / DREAMS Lab digital twins.

| folder | callsign | vehicle | world |
|---|---|---|---|
| [`otter/`](otter/) | otter | Clearpath Heron ASV | Tempe Town Lake |
| [`badger/`](badger/) | badger | Aion R1 + forward camera | Bishop fault scarp (Locutus) |
| [`kestrel/`](kestrel/) | kestrel | PX4 `x500_gimbal` (stock) | Bishop fault scarp (Locutus) |

Each vehicle folder is what must be installed into a twin container so an image rebuild does not lose the lab's setup. Ops scripts that know private addressing (watchdogs, zenoh configs) stay in the private lab infra repo, not here.

**Pinned PX4 for otter:** `v1.16.0-alpha2-441` (`9ac03f03eb`). The rover_differential guidance patch under `otter/px4/` applies to that tree.

## Aquatic sandbox (dreamslab)

A second water-world container, `digital-twin-aquatic` (image of the same name, committed
from the Tempe Heron twin), listens on host port **40077**. It runs otter for local SITL
tests (`ROS_DOMAIN_ID=84`, `GZ_PARTITION=aquatic`) and is the place to develop the next
vehicle — an underwater drone — without touching production otter on `:40099`. No zenoh
or DeepGIS enrollment yet; add a new callsign under this repo when that vehicle exists.

```bash
# install otter packs into the sandbox (on dreamslab)
C=digital-twin-aquatic bash otter/install.sh
```

**Related:**

- Twin image recipe (Locutus / kestrel): [`Open-UAV/openuav-turbovnc`](https://github.com/Open-UAV/openuav-turbovnc) `Dockerfile.twin` and `terrain/bishop_scarp/`
- On-robot ROS (executors, relays): [`DREAMS-lab/dreams_llm_ros2`](https://github.com/DREAMS-lab/dreams_llm_ros2)
- Globe / AI pilot: [`Earth-Innovation-Hub/deepgis-xr`](https://github.com/Earth-Innovation-Hub/deepgis-xr)
