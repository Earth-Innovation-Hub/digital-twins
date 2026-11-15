# badger — Earth Rover in the Locutus world

Callsign **badger**, shares container `digital-twin-locutus` with kestrel
(PX4 instance 1; kestrel is instance 0), ROS domain 53.

The vehicle in the world is the [Earth Rover](https://github.com/Earth-Innovation-Hub/earth-rover):
a 26 inch trike, 65 kg, with the sensor mast at 1.2 m. That repository is the
field stack and has no Gazebo model, so the shape here is built from those
dimensions. The callsign stays **badger** so the existing `/badger` topics,
pose agent, and ground executor do not move.

## Layout

| path | installs to |
|---|---|
| `models/earth_rover/` | `PX4-Autopilot/Tools/simulation/gz/models/earth_rover/` |
| `airframe/4024_gz_earth_rover` | PX4 posix airframes, source and the sitl build tree |
| `start_rover.sh` | `/root/start_rover.sh` |

Airframe is `4024_gz_earth_rover` (PX4 Ackermann). `PX4_SIM_MODEL=gz_earth_rover`
selects this model. The front wheel is driven and steered. The rear wheels roll
freely. With one front wheel the Ackermann controller is the bicycle model:
there is no second front wheel to split the steer angle.
`front_camera_link` is on the mast (640×480, hfov 1.274, 5 Hz).

The previous Aion R1 delta is still in `models/r1_rover_cam/` and is no longer spawned.

## Install

```bash
cd digital-twins/badger
bash install.sh          # C=digital-twin-locutus by default
```

Does not restart the twin. After install, stop PX4 instance 1 and run
`/root/start_rover.sh` again. The script removes a leftover `r1_rover_cam_1`
before spawning `earth_rover_1`.

## Notes

- `start_rover.sh` also starts `px4_relay`, `ground_executor`, and a lazy
  `ros_gz_bridge` camera under the `/badger` namespace. Those ROS packages come
  from `dreams_llm_ros2`, not this repo.
