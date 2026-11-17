"""Start a digital-twin-session container for a baked world; enroll pose.

Never touches protected production names (locutus, otter, aquatic, …).
"""
from __future__ import annotations

import hashlib
import hmac
import json
import os
import re
import secrets
import shutil
import subprocess
import threading
import time
from pathlib import Path

PROTECTED = frozenset({
    "locutus", "otter", "aquatic", "heron", "kestrel", "badger", "xudrone",
    "digital-twin-locutus", "digital-twin-water-tempe-lake", "digital-twin-aquatic",
    "zenoh-locutus", "zenoh-heron-tempe",
})

# The lab image that actually exists and already serves noVNC on 40001.
# openuav/twin:latest was the planned session image and is not built.
DEFAULT_IMAGE = os.environ.get("DEEPGIS_TWIN_SESSION_IMAGE", "digital-twin-locutus:llm")
# Public ingest. A session container is on the docker bridge, which can reach this
# host. Override with DEEPGIS_POSE_URL / DEEPGIS_FRAME_URL when a private ingest is used.
DEFAULT_POSE_URL = "https://deepgis.org/label/api/vehicles/pose/"
DEFAULT_FRAME_URL = "https://deepgis.org/label/api/vehicles/frame/"
# First x500_gimbal PX4 spawns. The gimbal camera is a Gazebo sensor only until bridged.
GIMBAL_CAMERA = "/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera"
# The session image serves noVNC on this port; the host side is the registry's host_port.
NOVNC_PORT = int(os.environ.get("DEEPGIS_TWIN_NOVNC_PORT", "40001"))
# Lab desktops. The simulation registry never assigns these to a session.
LAB_PORTS = frozenset({40077, 40088, 40099})
SESSIONS_ROOT = Path(os.environ.get("DEEPGIS_TWIN_SESSIONS_DIR", "/var/twin-sessions"))
IDLE_TTL_SEC = int(os.environ.get("DEEPGIS_TWIN_IDLE_TTL_SEC", str(2 * 3600)))
DISK_QUOTA_MB = int(os.environ.get("DEEPGIS_TWIN_DISK_QUOTA_MB", "2048"))


def _assert_safe_name(name):
    low = (name or "").lower()
    if not low or low in PROTECTED:
        raise ValueError(f"Refusing protected or empty twin name: {name!r}")
    for p in PROTECTED:
        if p in low and not low.startswith("ktwin-"):
            raise ValueError(f"Refusing name that collides with protected twin: {name!r}")
    if not all(c.isalnum() or c in "-_" for c in low):
        raise ValueError(f"Invalid twin name: {name!r}")


def _docker(*args, check=True):
    return subprocess.run(["docker", *args], check=check, capture_output=True, text=True)


def publish_args(host_port):
    """Docker publish flags for a registry host_port onto the image's noVNC port."""
    if isinstance(host_port, bool) or not isinstance(host_port, int):
        raise ValueError("host_port must be an integer from the simulation registry")
    if host_port in LAB_PORTS or not 40000 <= host_port <= 40999:
        raise ValueError(f"refusing host_port {host_port}")
    return ["-p", f"{host_port}:{NOVNC_PORT}"]


def _domain_id(callsign):
    """Stable ROS_DOMAIN_ID in 100–189, above the hostname hash used by lab twins."""
    h = int(hashlib.sha256(callsign.encode()).hexdigest()[:4], 16)
    return 100 + (h % 90)  # 100..189


def _dds_ns(callsign):
    """ROS 2 / PX4 namespace: hyphens are not legal in topic names."""
    ns = "".join(c if c.isalnum() or c == "_" else "_" for c in callsign)
    if not ns or not ns[0].isalpha():
        ns = "ktwin_" + ns
    return ns


def _camera_bridge_script():
    """One gimbal bridge for a session kestrel.

    The lab kestrel bridge (the course launch inside the locutus image) remaps
    x500_gimbal_0's camera, gimbal commands, and odometry under the robot name.
    A session callsign contains hyphens, which ROS 2 rejects in a topic name, so
    the root is PX4_UXRCE_DDS_NS (underscores). parameter_bridge stays up when
    the Gazebo topic is still missing, so this waits until the image topic
    exists and holds a lock so the session script and start_frames cannot
    publish the same image twice.
    """
    script = """#!/bin/bash
exec 9>/tmp/camera-bridge.lock
if ! flock -n 9; then
  exit 0
fi
mkdir -p /root/log
exec >> /root/log/camera.log 2>&1
if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi
NS="${PX4_UXRCE_DDS_NS:-}"
if [ -z "$NS" ]; then
  echo "camera bridge: no DDS namespace"
  exit 1
fi
CAM=/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera
while true; do
  while ! gz topic -l 2>/dev/null | grep -q "${CAM}/image"; do
    sleep 2
  done
  ros2 run ros_gz_bridge parameter_bridge \\
    "$CAM/image@sensor_msgs/msg/Image[gz.msgs.Image" \\
    "$CAM/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo" \\
    "/model/x500_gimbal_0/command/gimbal_roll@std_msgs/msg/Float64@gz.msgs.Double" \\
    "/model/x500_gimbal_0/command/gimbal_pitch@std_msgs/msg/Float64@gz.msgs.Double" \\
    "/model/x500_gimbal_0/command/gimbal_yaw@std_msgs/msg/Float64@gz.msgs.Double" \\
    "/model/x500_gimbal_0/odometry_with_covariance@nav_msgs/msg/Odometry[gz.msgs.Odometry" \\
    --ros-args -r __node:=camera_bridge \\
    -r "$CAM/image:=/$NS/camera/image" \\
    -r "$CAM/camera_info:=/$NS/camera/camera_info" \\
    -r "/model/x500_gimbal_0/command/gimbal_roll:=/$NS/gimbal/roll" \\
    -r "/model/x500_gimbal_0/command/gimbal_pitch:=/$NS/gimbal/pitch" \\
    -r "/model/x500_gimbal_0/command/gimbal_yaw:=/$NS/gimbal/yaw" \\
    -r "/model/x500_gimbal_0/odometry_with_covariance:=/$NS/odometry" \\
    || true
  sleep 2
done
"""
    if GIMBAL_CAMERA not in script:
        raise RuntimeError("camera bridge lost the gimbal sensor path")
    return script


def _session_script():
    """Desktop, one x500, and the DDS agent. Does not start the course rover."""
    return """#!/bin/bash
mkdir -p /root/log
echo "[session] callsign=${DEEPGIS_CALLSIGN} domain=${ROS_DOMAIN_ID} partition=${GZ_PARTITION}" | tee -a /root/log/twin.log
export DISPLAY=:1
export VGL_DISPLAY="${VGL_DISPLAY:-:0.0}"
export PX4_GZ_WORLD="${PX4_GZ_WORLD:-default}"
export PX4_GZ_MODEL="${PX4_GZ_MODEL:-x500_gimbal}"
export PX4_GZ_MODEL_POSE="${PX4_GZ_MODEL_POSE:-0,0,0.1,0,0,0}"
# The image still names the gimbal /drone_camera. Use this vehicle instead.
# Replace the _info name first so it does not become camera/image_info.
if [ -n "${PX4_UXRCE_DDS_NS:-}" ]; then
  roots=""
  for d in /root/ros2_ws /usr/local/bin /root/ses598-space-robotics-and-ai-2026/assignments/terrain_mapping_drone_control; do
    [ -d "$d" ] && roots="$roots $d"
  done
  if [ -n "$roots" ]; then
    find $roots -type f \( -name '*.py' -o -name '*.yaml' -o -name '*.yml' -o -name '*.sh' \) -print0 2>/dev/null | while IFS= read -r -d '' f; do
      grep -q '/drone_camera' "$f" || continue
      sed -i \
        -e "s|/drone_camera_info|/${PX4_UXRCE_DDS_NS}/camera/camera_info|g" \
        -e "s|/drone_camera|/${PX4_UXRCE_DDS_NS}/camera/image|g" \
        "$f"
    done
  fi
fi
if [ -n "${PX4_UXRCE_DDS_NS:-}" ] && [ -f /usr/local/bin/camera_bridge.sh ]; then
  bash /usr/local/bin/camera_bridge.sh &
fi
/usr/local/bin/start_desktop.sh >> /root/log/desktop.log 2>&1 &
for _ in $(seq 1 90); do
  [ -S /tmp/.X11-unix/X1 ] && break
  sleep 1
done
if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi
if [ -x /root/run_qgroundcontrol.sh ]; then
  ( while true; do /root/run_qgroundcontrol.sh >> /root/log/qgc.log 2>&1 || true; sleep 15; done ) &
fi
if [ -x /usr/local/bin/MicroXRCEAgent ]; then
  ( while true; do /usr/local/bin/MicroXRCEAgent udp4 -p 8888 >> /root/log/agent.log 2>&1 || true; sleep 5; done ) &
fi
cd /root/PX4-Autopilot
exec vglrun make px4_sitl gz_x500_gimbal >> /root/log/twin.log 2>&1
"""


_SPHERICAL_RE = re.compile(r"<spherical_coordinates>.*?</spherical_coordinates>", re.S)


def _origin_block(lat, lon, elev, surface="EARTH_WGS84"):
    return (
        "    <spherical_coordinates>\n"
        f"      <surface_model>{surface}</surface_model>\n"
        "      <world_frame_orientation>ENU</world_frame_orientation>\n"
        f"      <latitude_deg>{float(lat):.8f}</latitude_deg>\n"
        f"      <longitude_deg>{float(lon):.8f}</longitude_deg>\n"
        f"      <elevation>{float(elev):.3f}</elevation>\n"
        "      <heading_deg>0</heading_deg>\n"
        "    </spherical_coordinates>"
    )


def apply_site(text, look, props_xml="", surface="EARTH_WGS84"):
    """Force the world GPS origin to the viewport and splice generated models.

    Replaces an existing spherical_coordinates block. The Locutus image ships
    with Bishop Scarp in default.sdf, and PX4/QGC read that block as home.
    A lunar session uses MOON_SCS so the home latitude is on the Moon.
    """
    lat, lon = look.get("lat"), look.get("lon")
    if not (isinstance(lat, (int, float)) and isinstance(lon, (int, float))):
        return text
    elev = look.get("elev_m") or 0
    block = _origin_block(lat, lon, elev, surface)
    if _SPHERICAL_RE.search(text):
        text = _SPHERICAL_RE.sub(block, text, count=1)
    else:
        needle = '<world name="default">'
        if needle in text:
            text = text.replace(needle, needle + "\n" + block, 1)
    if props_xml and 'name="prop_' not in text and "</world>" in text:
        text = text.replace("</world>", props_xml + "  </world>", 1)
    return text


_GROUND_RE = re.compile(r'<model name="ground_plane">.*?</model>', re.S)


def apply_moon_terrain(text, width_m, length_m, z_min, z_span,
                       uri="/root/moon_heightmap.png", texture_uri="", normal_uri=""):
    """Replace the infinite ground plane with a heightmap.

    Pixel 0 is z_min and a full white pixel is z_min + z_span. The patch is
    centred on the spawn. texture_uri, when set together with normal_uri, is
    the satellite PNG draped on the visual. The normal is required: the SDF
    schema otherwise inserts ``__default__`` and Gazebo refuses the world.
    The collision is the heightmap either way.
    """
    texture = ""
    if texture_uri and normal_uri:
        cover = max(float(width_m), float(length_m))
        texture = (
            "              <texture>\n"
            f"                <size>{cover:.2f}</size>\n"
            f"                <diffuse>{texture_uri}</diffuse>\n"
            f"                <normal>{normal_uri}</normal>\n"
            "              </texture>\n"
        )
    heightmap = (
        "            <heightmap>\n"
        f"              <uri>{uri}</uri>\n"
        f"              <size>{width_m:.2f} {length_m:.2f} {z_span:.3f}</size>\n"
        f"              <pos>0 0 {z_min:.3f}</pos>\n"
        "{visual_extra}"
        "            </heightmap>\n"
    )
    model = (
        '<model name="ground_plane">\n'
        "      <static>true</static>\n"
        "      <pose>0 0 0 0 0 0</pose>\n"
        '      <link name="link">\n'
        '        <collision name="collision">\n'
        "          <geometry>\n"
        + heightmap.format(visual_extra="")
        + "          </geometry>\n"
        "        </collision>\n"
        '        <visual name="visual">\n'
        "          <geometry>\n"
        + heightmap.format(visual_extra=texture)
        + "          </geometry>\n"
        "          <material>\n"
        "            <ambient>0.45 0.42 0.38 1</ambient>\n"
        "            <diffuse>0.62 0.58 0.52 1</diffuse>\n"
        "            <specular>0.05 0.05 0.05 1</specular>\n"
        "          </material>\n"
        "        </visual>\n"
        "      </link>\n"
        "    </model>"
    )
    if _GROUND_RE.search(text):
        return _GROUND_RE.sub(model, text, count=1)
    if "</world>" in text:
        return text.replace("</world>", model + "\n  </world>", 1)
    return text


def _patched_world(artifact, dest, image):
    """PX4's default world, viewport origin, plus any synthesized prop models."""
    look = artifact.get("look_at") or {}
    try:
        proc = _docker(
            "run", "--rm", "--entrypoint", "cat", image,
            "/root/PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf",
        )
    except (OSError, subprocess.CalledProcessError):
        return None
    from .sdf_props import props_xml_fragment
    from .satellite_ground import apply_ground
    lunar = artifact.get("body") == "moon"
    lat, lon = look.get("lat"), look.get("lon")
    fragment = ""
    if isinstance(lat, (int, float)) and isinstance(lon, (int, float)) and not lunar:
        fragment, _clean = props_xml_fragment(artifact.get("props") or [], lat, lon)
    text = apply_site(proc.stdout, look, fragment, "MOON_SCS" if lunar else "EARTH_WGS84")
    hm = artifact.get("heightmap_image")
    if hm and Path(hm).is_file():
        mount = artifact.get("heightmap_mount") or (
            "/root/moon_heightmap.png" if lunar else "/root/earth_heightmap.png")
        tex = ""
        normal = ""
        diffuse = artifact.get("diffuse_image")
        normal_image = artifact.get("normal_image")
        if (not lunar and diffuse and normal_image
                and Path(diffuse).is_file() and Path(normal_image).is_file()):
            tex = artifact.get("diffuse_mount") or "/root/earth_diffuse.png"
            normal = artifact.get("normal_mount") or "/root/earth_normal.png"
        text = apply_moon_terrain(
            text,
            float(artifact.get("width_m") or 150),
            float(artifact.get("length_m") or 150),
            float(artifact.get("heightmap_z_min") or 0),
            float(artifact.get("heightmap_span") or 1),
            mount,
            tex,
            normal,
        )
    else:
        size = artifact.get("ground_size_m") or []
        ground = artifact.get("ground_image")
        if ground and Path(ground).is_file() and len(size) >= 2:
            text = apply_ground(text, float(size[0]), float(size[1]))
    dest.write_text(text, encoding="utf-8")
    return dest


def session_dir(callsign):
    return SESSIONS_ROOT / callsign


def _require_session(callsign):
    """Container names for one ktwin session. Lab names raise."""
    _assert_safe_name(callsign)
    if not str(callsign).startswith("ktwin-"):
        raise ValueError(f"Refusing a name that is not a session twin: {callsign!r}")
    return f"digital-twin-session-{callsign}", f"deepgis-frames-{callsign}"


def _container_state(name):
    proc = _docker("inspect", "-f", "{{.State.Status}}", name, check=False)
    if proc.returncode != 0:
        return "missing"
    return (proc.stdout or "").strip() or "unknown"


def _started_at(name):
    proc = _docker("inspect", "-f", "{{.State.StartedAt}}", name, check=False)
    if proc.returncode != 0:
        return ""
    return (proc.stdout or "").strip()


def _docker_time_key(value):
    """Sortable timestamp. Docker omits a zero fraction, so text order is wrong.

    ``2026-11-15T20:00:01.5Z`` is later than ``2026-11-15T20:00:01Z``, but ``.``
    sorts before ``Z``.
    """
    text = (value or "").strip()
    if not text or text.startswith("0001-"):
        return ""
    if text.endswith("Z"):
        text = text[:-1]
    elif len(text) > 19 and "+" in text[19:]:
        text = text[:text.rfind("+")]
    if "." in text:
        head, frac = text.split(".", 1)
        digits = "".join(ch for ch in frac if ch.isdigit())
        return f"{head}.{(digits + '000000000')[:9]}"
    return text + ".000000000"


def _sidecar_older_than_session(container, frames):
    """True when the sidecar started before this run of the session container.

    The sidecar uses the session's network namespace. A restart of the session
    creates a new namespace, and a sidecar from the previous run never sees
    the camera topic.
    """
    session_at = _docker_time_key(_started_at(container))
    frames_at = _docker_time_key(_started_at(frames))
    if not session_at or not frames_at:
        return False
    return frames_at < session_at


def _session_callsigns():
    """ktwin callsigns with a session directory or a session/frames container."""
    found = set()
    if SESSIONS_ROOT.is_dir():
        for path in SESSIONS_ROOT.iterdir():
            if path.is_dir() and path.name.startswith("ktwin-"):
                found.add(path.name)
    proc = _docker("ps", "-a", "--format", "{{.Names}}", check=False)
    for line in (proc.stdout or "").splitlines():
        name = line.strip()
        for prefix in ("digital-twin-session-", "deepgis-frames-"):
            if name.startswith(prefix + "ktwin-"):
                found.add(name[len(prefix):])
    return sorted(found)


_FRAMES_LOCK = threading.Lock()


def stop_session(callsign):
    """Stop the session container and its frames sidecar. The container stays."""
    container, frames = _require_session(callsign)
    _docker("stop", "-t", "15", frames, check=False)
    if _container_state(container) == "missing":
        return {"status": "missing", "container": container}
    if _container_state(container) != "running":
        return {"status": "down", "container": container}
    proc = _docker("stop", "-t", "20", container, check=False)
    if proc.returncode != 0 and _container_state(container) == "running":
        raise RuntimeError("container did not stop")
    return {"status": "down", "container": container}


def start_session(callsign):
    """Start a stopped session and attach a new frames sidecar.

    The sidecar shares the container's network namespace, so a start (a new
    namespace) needs a new sidecar. The camera bridge comes back with the
    session script, and start_frames starts one immediately if it is not up.
    """
    container, _frames = _require_session(callsign)
    if _container_state(container) == "missing":
        raise RuntimeError("no container")
    if _container_state(container) != "running":
        proc = _docker("start", container, check=False)
        if proc.returncode != 0:
            raise RuntimeError("container did not start")
    frames = start_frames(callsign)
    return {"status": "up", "container": container, "frames": frames.get("status")}


def purge_session(callsign):
    """Remove the session container, its sidecar, and its files."""
    container, frames = _require_session(callsign)
    _docker("rm", "-f", frames, check=False)
    _docker("rm", "-f", container, check=False)
    folder = session_dir(callsign)
    root = SESSIONS_ROOT.resolve()
    if folder.exists():
        target = folder.resolve()
        target.relative_to(root)
        if target != root:
            shutil.rmtree(target)
    return {"status": "purged", "container": container}


def enforce_quota(callsign):
    """Delete idle session dirs over TTL; refuse if this session would exceed quota."""
    SESSIONS_ROOT.mkdir(parents=True, exist_ok=True)
    now = time.time()
    for path in SESSIONS_ROOT.iterdir():
        if not path.is_dir():
            continue
        meta_path = path / "session.json"
        if not meta_path.is_file():
            continue
        try:
            meta = json.loads(meta_path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            continue
        name = meta.get("callsign") or path.name
        if name in PROTECTED or path.name in PROTECTED:
            continue
        idle = meta.get("idle_ttl_sec", IDLE_TTL_SEC)
        updated = meta.get("updated", meta.get("created", 0))
        if now - updated > idle and meta.get("status") in ("ready", "stopped", "failed"):
            container = f"digital-twin-session-{path.name}"
            frames = f"deepgis-frames-{path.name}"
            state = _container_state(container)
            # Keep a stopped container so Sims Start can bring its camera back.
            # A running session is left untouched. Only a sidecar with no
            # session container is removed.
            if state == "running":
                continue
            try:
                if state == "missing":
                    _docker("rm", "-f", frames, check=False)
                elif _container_state(frames) == "running":
                    _docker("stop", "-t", "15", frames, check=False)
            except (OSError, subprocess.SubprocessError):
                pass
    # Disk use for this callsign
    target = session_dir(callsign)
    if target.exists():
        total = sum(f.stat().st_size for f in target.rglob("*") if f.is_file())
        if total > DISK_QUOTA_MB * 1024 * 1024:
            raise RuntimeError(f"Session disk quota exceeded ({DISK_QUOTA_MB} MB)")


def write_session_meta(callsign, artifact, extra=None):
    d = session_dir(callsign)
    d.mkdir(parents=True, exist_ok=True)
    meta = {
        "callsign": callsign,
        "created": time.time(),
        "updated": time.time(),
        "idle_ttl_sec": IDLE_TTL_SEC,
        "disk_quota_mb": DISK_QUOTA_MB,
        "status": "starting",
        "world_sdf": artifact.get("world_sdf"),
        "look_at": artifact.get("look_at"),
        "airframe": artifact.get("airframe", "x500_gimbal"),
        "ros_domain_id": _domain_id(callsign),
        "gz_partition": callsign,
        "dds_ns": _dds_ns(callsign),
        "host_port": artifact.get("host_port"),
        "body": artifact.get("body") or "earth",
    }
    if extra:
        meta.update(extra)
    (d / "session.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
    return meta


def spawn_session(artifact, dry_run=False):
    """Start docker digital-twin-session-{callsign} with the baked world.

    dry_run: write meta and env only (CI / when docker is unavailable).
    """
    callsign = artifact["callsign"]
    _assert_safe_name(callsign)
    host_port = artifact.get("host_port")
    dry = dry_run or os.environ.get("DEEPGIS_TWIN_SPAWN_DRY_RUN") == "1"
    if host_port is not None:
        publish_args(host_port)
    elif not dry:
        raise ValueError("host_port from the simulation registry is required")
    enforce_quota(callsign)
    meta = write_session_meta(callsign, artifact)
    env_path = session_dir(callsign) / "session.env"
    env_path.write_text(
        "\n".join([
            f"PX4_UXRCE_DDS_NS={_dds_ns(callsign)}",
            f"ROS_DOMAIN_ID={meta['ros_domain_id']}",
            f"GZ_PARTITION={callsign}",
            "PX4_GZ_MODEL=x500_gimbal",
            "PX4_GZ_WORLD=default",
            f"DEEPGIS_CALLSIGN={callsign}",
            f"DEEPGIS_LAT={artifact['look_at']['lat']}",
            f"DEEPGIS_LON={artifact['look_at']['lon']}",
        ] + ([
            f"PX4_GZ_MODEL_POSE=0,0,{float(artifact.get('spawn_z') or 15):.2f},0,0,0",
        ] if artifact.get("body") == "moon" or artifact.get("heightmap_image") else [])) + "\n",
        encoding="utf-8",
    )
    script_path = session_dir(callsign) / "start_session.sh"
    script_path.write_text(_session_script(), encoding="utf-8")
    bridge_path = session_dir(callsign) / "camera_bridge.sh"
    bridge_path.write_text(_camera_bridge_script(), encoding="utf-8")
    container = f"digital-twin-session-{callsign}"
    if dry:
        meta["status"] = "ready"
        meta["dry_run"] = True
        meta["container"] = container
        meta["updated"] = time.time()
        (session_dir(callsign) / "session.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
        return meta

    # Remove any prior session with this callsign only
    _docker("rm", "-f", container, check=False)
    image = os.environ.get("DEEPGIS_TWIN_SESSION_IMAGE", DEFAULT_IMAGE)
    world_file = _patched_world(artifact, session_dir(callsign) / "default.sdf", image)
    args = [
        "run", "-d", "--name", container,
        "--runtime", "nvidia",
        "--restart", "unless-stopped",
        "--shm-size", "1g",
        "--hostname", _dds_ns(callsign),
        "--env-file", str(env_path),
        "-e", "VGL_DISPLAY=:0.0",
        "-e", "NVIDIA_VISIBLE_DEVICES=all",
        "-e", "NVIDIA_DRIVER_CAPABILITIES=all",
        "-v", "/tmp/.X11-unix/X0:/tmp/.X11-unix/X0:rw",
        "-v", f"{script_path}:/usr/local/bin/start_session.sh:ro",
        "-v", f"{bridge_path}:/usr/local/bin/camera_bridge.sh:ro",
    ]
    if world_file is not None:
        args += ["-v", f"{world_file}:/root/PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf:ro"]
    heightmap = artifact.get("heightmap_image")
    has_heightmap = bool(heightmap and Path(heightmap).is_file())
    ground = artifact.get("ground_image")
    if ground and Path(ground).is_file() and not has_heightmap:
        args += ["-v", f"{ground}:/root/ground.jpg:ro"]
    if has_heightmap:
        mount = artifact.get("heightmap_mount") or (
            "/root/moon_heightmap.png" if artifact.get("body") == "moon" else "/root/earth_heightmap.png")
        args += ["-v", f"{heightmap}:{mount}:ro"]
        diffuse = artifact.get("diffuse_image")
        normal_image = artifact.get("normal_image")
        if (diffuse and normal_image and Path(diffuse).is_file()
                and Path(normal_image).is_file()):
            diffuse_mount = artifact.get("diffuse_mount") or "/root/earth_diffuse.png"
            normal_mount = artifact.get("normal_mount") or "/root/earth_normal.png"
            args += [
                "-v", f"{diffuse}:{diffuse_mount}:ro",
                "-v", f"{normal_image}:{normal_mount}:ro",
            ]
    args += [
        *publish_args(host_port),
        image,
        "bash", "/usr/local/bin/start_session.sh",
    ]
    try:
        _docker(*args)
        meta["status"] = "ready"
        meta["container"] = container
    except subprocess.CalledProcessError as e:
        meta["status"] = "failed"
        meta["error"] = (e.stderr or e.stdout or str(e))[:400]
    meta["updated"] = time.time()
    (session_dir(callsign) / "session.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
    return meta


def fleet_hmac(secret, callsign):
    """The token deepgis-xr accepts for a ktwin- id: HMAC-SHA256(secret, callsign)."""
    return hmac.new(secret.encode(), callsign.encode(), hashlib.sha256).hexdigest()


def enroll_pose(callsign, look_at, pose_url=None, token=None):
    """Write a pose-agent env fragment.

    Does not rotate production VEHICLE_POSE_TOKENS. When DEEPGIS_VEHICLE_FLEET_SECRET
    is set the token is HMAC-SHA256(secret, callsign), which is what the vehicles
    API checks for a ktwin- id.
    """
    _assert_safe_name(callsign)
    secret = os.environ.get("DEEPGIS_VEHICLE_FLEET_SECRET", "")
    if secret:
        token = fleet_hmac(secret, callsign)
    else:
        token = token or secrets.token_hex(16)
    env = session_dir(callsign) / "pose.env"
    env.parent.mkdir(parents=True, exist_ok=True)
    env.write_text(
        "\n".join([
            f"VEHICLE_ID={callsign}",
            f"VEHICLE_POSE_TOKEN={token}",
            f"POSE_LAT={look_at['lat']}",
            f"POSE_LON={look_at['lon']}",
            f"POSE_URL={pose_url or os.environ.get('DEEPGIS_POSE_URL', '') or DEFAULT_POSE_URL}",
        ]) + "\n",
        encoding="utf-8",
    )
    os.chmod(env, 0o600)
    return {"vehicle": callsign, "pose_env": str(env), "enrolled": bool(secret)}


def ensure_camera_bridge(callsign):
    """Start the gimbal bridge. A second start exits; the script holds the lock."""
    _assert_safe_name(callsign)
    container = f"digital-twin-session-{callsign}"
    proc = _docker(
        "exec", "-d", container, "bash", "/usr/local/bin/camera_bridge.sh",
        check=False,
    )
    if proc.returncode != 0:
        return {"status": "failed"}
    return {"status": "started"}


_UDP_PROFILE = """<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor><transport_id>udp</transport_id><type>UDPv4</type></transport_descriptor>
  </transport_descriptors>
  <participant profile_name="udp_only" is_default_profile="true">
    <rtps>
      <userTransports><transport_id>udp</transport_id></userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
    </rtps>
  </participant>
</profiles>
"""


def _udp_profile(folder):
    """FastDDS profile the frames sidecar must use.

    The sidecar shares the session network namespace and not its memory, so the
    default shared-memory transport never delivers the camera image and the
    frame agent logs nothing. UDP-only matches the lab profile and contains no
    addresses. DEEPGIS_FRAME_UDP_XML overrides this when that file exists.
    """
    override = os.environ.get("DEEPGIS_FRAME_UDP_XML", "")
    if override and Path(override).is_file():
        return Path(override)
    path = Path(folder) / "dds_udp.xml"
    path.write_text(_UDP_PROFILE, encoding="utf-8")
    return path


_FMU_TOPICS = (
    "vehicle_global_position",
    "vehicle_local_position",
    "vehicle_status",
    "vehicle_land_detected",
)


def _frames_command(dds_ns):
    """Pose and frame agents, remapped onto this session's PX4 namespace."""
    remaps = []
    for topic in _FMU_TOPICS:
        for suffix in ("", "_v1"):
            remaps.append(f"-r /fmu/out/{topic}{suffix}:=/{dds_ns}/fmu/out/{topic}{suffix}")
    ros = "--ros-args " + " ".join(remaps)
    return (
        "source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash && "
        f"(python3 /opt/deepgis/deepgis_pose_agent.py {ros} &) && "
        f"exec python3 /opt/deepgis/deepgis_frame_agent.py {ros}"
    )


def _session_vehicle_name(callsign):
    """Name posted with the pose. The Moon page follows only a Moon session."""
    try:
        meta = json.loads((session_dir(callsign) / "session.json").read_text(encoding="utf-8"))
    except (OSError, ValueError):
        meta = {}
    if meta.get("body") == "moon":
        return "Moon session kestrel"
    return "Session kestrel"


def _start_frames(callsign):
    """Share the session gimbal with DeepGIS and register the vehicle by posting pose.

    The sidecar joins the session container's network namespace, so it sees that
    twin's ROS domain and no other. It does not share the session's memory, so
    it always loads a UDP-only DDS profile; otherwise the image stays on the
    session's shared-memory transport and the frame agent never posts.
    The token stays in a mode-600 env file.
    """
    _assert_safe_name(callsign)
    bridge = ensure_camera_bridge(callsign)
    agent = os.environ.get("DEEPGIS_FRAME_AGENT_DIR", "")
    if not agent or not (Path(agent) / "deepgis_frame_agent.py").is_file():
        return {"status": "skipped", "reason": "frame agent dir is not configured", "camera": bridge}
    secret = os.environ.get("DEEPGIS_VEHICLE_FLEET_SECRET", "")
    if not secret:
        return {"status": "skipped", "reason": "fleet secret is not configured", "camera": bridge}
    dds = _dds_ns(callsign)
    pose_url = os.environ.get("DEEPGIS_POSE_URL", "") or DEFAULT_POSE_URL
    frame_url = os.environ.get("DEEPGIS_FRAME_URL", "") or DEFAULT_FRAME_URL
    env_path = session_dir(callsign) / "frames.env"
    session_dir(callsign).mkdir(parents=True, exist_ok=True)
    env_path.write_text(
        "\n".join([
            f"DEEPGIS_POSE_TOKEN={fleet_hmac(secret, callsign)}",
            f"DEEPGIS_FRAME_TOKEN={fleet_hmac(secret, callsign)}",
        ]) + "\n",
        encoding="utf-8",
    )
    os.chmod(env_path, 0o600)
    name = f"deepgis-frames-{callsign}"
    _docker("rm", "-f", name, check=False)
    args = [
        "run", "-d", "--name", name, "--restart", "unless-stopped",
        "--network", f"container:digital-twin-session-{callsign}",
        "--user", "root", "-w", "/root",
        "-v", f"{agent}:/opt/deepgis:ro",
        "--env-file", str(env_path),
        "-e", "PYTHONDONTWRITEBYTECODE=1",
        "-e", f"ROS_DOMAIN_ID={_domain_id(callsign)}",
        "-e", f"DEEPGIS_VEHICLE={callsign}",
        "-e", f"DEEPGIS_VEHICLE_NAME={_session_vehicle_name(callsign)}",
        "-e", f"DEEPGIS_POSE_URL={pose_url}",
        "-e", f"DEEPGIS_FRAME_URL={frame_url}",
        "-e", f"DEEPGIS_FRAME_TOPIC=/{dds}/camera/image",
        "-e", f"DEEPGIS_CAMERA_INFO_TOPIC=/{dds}/camera/camera_info",
        "-v", f"{_udp_profile(session_dir(callsign))}:/udp.xml:ro",
        "-e", "FASTRTPS_DEFAULT_PROFILES_FILE=/udp.xml",
    ]
    args += [
        "--entrypoint", "bash",
        os.environ.get("DEEPGIS_TWIN_SESSION_IMAGE", DEFAULT_IMAGE),
        "-lc", _frames_command(dds),
    ]
    try:
        _docker(*args)
    except subprocess.CalledProcessError as e:
        return {"status": "failed", "error": (e.stderr or e.stdout or "docker failed")[:200], "camera": bridge}
    return {"status": "up", "container": name, "camera": bridge}


def start_frames(callsign):
    """Start or replace the frames sidecar. One start runs at a time."""
    with _FRAMES_LOCK:
        return _start_frames(callsign)


def reconcile_session_frames():
    """Keep each running session's camera sidecar on that session's network.

    A sidecar started before the session container's current start is still
    joined to the previous network namespace, so pose and JPEG posts stop.
    A sidecar left running after the session is down is stopped. Lab frames
    containers are not ktwin names and are left alone.
    """
    with _FRAMES_LOCK:
        changed = []
        for callsign in _session_callsigns():
            try:
                _assert_safe_name(callsign)
            except ValueError:
                continue
            if not callsign.startswith("ktwin-"):
                continue
            container, frames = f"digital-twin-session-{callsign}", f"deepgis-frames-{callsign}"
            session_state = _container_state(container)
            frames_state = _container_state(frames)
            if session_state == "running" and (
                frames_state != "running" or _sidecar_older_than_session(container, frames)
            ):
                _start_frames(callsign)
                changed.append(callsign)
            elif session_state != "running" and frames_state == "running":
                _docker("stop", "-t", "15", frames, check=False)
                changed.append(callsign)
        return changed
