# digital-twins

PX4 Gazebo SITL vehicle packs for the Earth Innovation Hub / DREAMS Lab digital twins.

Each vehicle folder is what must be installed into a twin container so an image
rebuild does not lose the lab's setup. Ops scripts that know private addressing
(watchdogs, zenoh configs, pose tokens) stay in the private lab infra repo, not here.

## Packs

| folder | callsign(s) | vehicle | container | panel | ROS / Gazebo | world |
|---|---|---|---|---|---|---|
| [`otter/`](otter/) | **otter** | Clearpath Heron ASV | `digital-twin-water-tempe-lake` | :40099 | domain 27 | Tempe Town Lake |
| [`udrone/`](udrone/) | **otter_k** + **xudrone** | Heron ASV + uDrone AUV (CoRAL pair) | `digital-twin-aquatic` | :40077 | domain 84 / `GZ_PARTITION=aquatic` | Kiwanis Lake |
| [`badger/`](badger/) | **badger** | Aion R1 + forward camera | `digital-twin-locutus` | (with kestrel) | domain 53 | Bishop scarp |
| [`kestrel/`](kestrel/) | **kestrel** | PX4 `x500_gimbal` (stock) | `digital-twin-locutus` | :40088 | domain 53 | Bishop scarp |
| [`twin_builder/`](twin_builder/) | `ktwin-*` (on demand) | x500_gimbal session | `digital-twin-session-*` | (ephemeral) | (per session) | Cesium look-at patch |

**Pinned PX4 for otter / aquatic:** `v1.16.0-alpha2-441` (`9ac03f03eb`). The
rover_differential guidance patch under `otter/px4/` applies to that tree.

## How the packs relate

- **otter** is the production surface boat. Install only into `:40099`.
- **udrone** is the aquatic *sandbox*. It installs a CoRAL two-robot mode into
  `:40077` (boat callsign **otter_k**, AUV callsign **xudrone**) and never
  touches the production otter. The Heron model/physics come from the same
  lineage as `otter/`; the AUV, Kiwanis world, latch plugin, and tests live here.
- **kestrel** + **badger** share Locutus (same two-robot PX4-instance pattern the
  aquatic pack copies).
- **twin_builder** is the AI Pilot on-demand twin worker; it never replaces
  Locutus, otter, or aquatic.

## Aquatic sandbox (quick start)

```bash
# on dreamslab, from a checkout of this repo
C=digital-twin-aquatic bash udrone/install.sh   # sim must be stopped / disarmed
docker exec digital-twin-aquatic simctl start    # otter_k + docked xudrone
docker exec digital-twin-aquatic simctl status
docker exec digital-twin-aquatic /root/udrone_boat/src/run_test.sh latch
docker exec digital-twin-aquatic simctl stop
```

Full start/stop, demos, and design notes: [`udrone/README.md`](udrone/README.md).
Production boat install/tests: [`otter/README.md`](otter/README.md).

## Related

- Twin image recipe (Locutus / kestrel): [`Open-UAV/openuav-turbovnc`](https://github.com/Open-UAV/openuav-turbovnc) `Dockerfile.twin` and `terrain/bishop_scarp/`
- On-robot ROS (executors, relays): [`DREAMS-lab/dreams_llm_ros2`](https://github.com/DREAMS-lab/dreams_llm_ros2)
- Globe / AI pilot: [`Earth-Innovation-Hub/deepgis-xr`](https://github.com/Earth-Innovation-Hub/deepgis-xr)
- CoRAL concept (field): [`Earth-Innovation-Hub/coral`](https://github.com/Earth-Innovation-Hub/coral)
