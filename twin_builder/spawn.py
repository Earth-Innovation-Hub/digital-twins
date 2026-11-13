"""Start a digital-twin-session container for a baked world; enroll pose.

Never touches protected production names (locutus, otter, aquatic, …).
"""
from __future__ import annotations

import hashlib
import json
import os
import secrets
import subprocess
import time
from pathlib import Path

PROTECTED = frozenset({
    "locutus", "otter", "aquatic", "heron", "kestrel", "badger", "xudrone",
    "digital-twin-locutus", "digital-twin-water-tempe-lake", "digital-twin-aquatic",
    "zenoh-locutus", "zenoh-heron-tempe",
})

DEFAULT_IMAGE = os.environ.get("DEEPGIS_TWIN_SESSION_IMAGE", "openuav/twin:latest")
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


def _domain_id(callsign):
    """Stable ROS_DOMAIN_ID in 100–200 excluding known lab domains."""
    h = int(hashlib.sha256(callsign.encode()).hexdigest()[:4], 16)
    return 100 + (h % 90)  # 100..189


def session_dir(callsign):
    return SESSIONS_ROOT / callsign


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
            try:
                _docker("rm", "-f", f"digital-twin-session-{path.name}", check=False)
            except (OSError, subprocess.SubprocessError):
                pass
            # Leave files for audit but mark expired
            meta["status"] = "expired"
            meta_path.write_text(json.dumps(meta, indent=2), encoding="utf-8")
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
        "dds_ns": callsign,
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
    enforce_quota(callsign)
    meta = write_session_meta(callsign, artifact)
    env_path = session_dir(callsign) / "session.env"
    env_path.write_text(
        "\n".join([
            f"PX4_UXRCE_DDS_NS={callsign}",
            f"ROS_DOMAIN_ID={meta['ros_domain_id']}",
            f"GZ_PARTITION={callsign}",
            f"PX4_GZ_MODEL=x500_gimbal",
            f"PX4_GZ_WORLD={artifact.get('world_sdf', '')}",
            f"DEEPGIS_CALLSIGN={callsign}",
            f"DEEPGIS_LAT={artifact['look_at']['lat']}",
            f"DEEPGIS_LON={artifact['look_at']['lon']}",
        ]) + "\n",
        encoding="utf-8",
    )
    container = f"digital-twin-session-{callsign}"
    if dry_run or os.environ.get("DEEPGIS_TWIN_SPAWN_DRY_RUN") == "1":
        meta["status"] = "ready"
        meta["dry_run"] = True
        meta["container"] = container
        meta["updated"] = time.time()
        (session_dir(callsign) / "session.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
        return meta

    # Remove any prior session with this callsign only
    _docker("rm", "-f", container, check=False)
    world_host = str(Path(artifact["world_sdf"]).resolve().parent)
    args = [
        "run", "-d", "--name", container,
        "--restart", "unless-stopped",
        "--env-file", str(env_path),
        "-v", f"{world_host}:/twin/world:ro",
        DEFAULT_IMAGE,
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


def enroll_pose(callsign, look_at, pose_url=None, token=None):
    """Write a pose-agent env fragment; optional one-shot HTTP enroll if URL set.

    Does not rotate production VEHICLE_POSE_TOKENS; session tokens are fleet-prefix
    style (HMAC) when DEEPGIS_VEHICLE_FLEET_SECRET is set.
    """
    _assert_safe_name(callsign)
    secret = os.environ.get("DEEPGIS_VEHICLE_FLEET_SECRET", "")
    if secret:
        token = hashlib.sha256(f"{secret}:{callsign}".encode()).hexdigest()
    else:
        token = token or secrets.token_hex(16)
    env = session_dir(callsign) / "pose.env"
    env.write_text(
        "\n".join([
            f"VEHICLE_ID={callsign}",
            f"VEHICLE_POSE_TOKEN={token}",
            f"POSE_LAT={look_at['lat']}",
            f"POSE_LON={look_at['lon']}",
            f"POSE_URL={pose_url or os.environ.get('DEEPGIS_POSE_URL', '')}",
        ]) + "\n",
        encoding="utf-8",
    )
    return {"vehicle": callsign, "pose_env": str(env), "enrolled": bool(pose_url or os.environ.get("DEEPGIS_POSE_URL"))}
