"""Authenticated twin-builder HTTP worker for dreamslab.

POST /jobs  — start bake+spawn+enroll (Bearer token)
GET  /jobs/<id> — status
POST /sessions/<callsign>/stop|start|purge — session containers only

Bind to localhost / overlay only. Do not expose without auth.
"""
from __future__ import annotations

import base64
import json
import os
import secrets
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

from . import bake, capture, osm_buildings, rasters, satellite_ground, spawn, vlm_props

JOBS = {}
JOBS_LOCK = threading.Lock()
TOKEN = os.environ.get("DEEPGIS_TWIN_BUILDER_TOKEN", "")
JOB_ROOT = Path(os.environ.get("DEEPGIS_TWIN_JOBS_DIR", "/var/twin-jobs"))
HOST = os.environ.get("DEEPGIS_TWIN_BUILDER_HOST", "127.0.0.1")
PORT = int(os.environ.get("DEEPGIS_TWIN_BUILDER_PORT", "8769"))


def session_route(path):
    """(callsign, action) for /sessions/<callsign>/<stop|start|purge>, else None."""
    parts = [part for part in (path or "").split("/") if part]
    if len(parts) != 3 or parts[0] != "sessions":
        return None
    callsign, action = parts[1], parts[2]
    if action not in ("stop", "start", "purge"):
        return None
    return callsign, action


def _auth(handler):
    hdr = handler.headers.get("Authorization", "")
    if not TOKEN or hdr != f"Bearer {TOKEN}":
        return False
    return True


def _run_job(job_id):
    with JOBS_LOCK:
        job = JOBS.get(job_id)
    if not job:
        return
    try:
        def set_status(status, message, **extra):
            with JOBS_LOCK:
                job["status"] = status
                job["message"] = message
                job["updated"] = time.time()
                job.update(extra)

        set_status("capturing", "Capturing DEM and nadir")
        nadir_path = None
        encoded = job.pop("nadir_b64", "") or ""
        if encoded.startswith("data:image/jpeg;base64,"):
            encoded = encoded.split(",", 1)[1]
        if encoded:
            try:
                raw = base64.b64decode(encoded, validate=False)
            except (ValueError, TypeError):
                raw = b""
            if raw:
                nadir_path = JOB_ROOT / job_id / "viewport.jpg"
                nadir_path.parent.mkdir(parents=True, exist_ok=True)
                nadir_path.write_bytes(raw)
        if job.get("body") == "moon":
            set_status("baking", "Writing the lunar elevation heightmap")
            try:
                artifact = bake.bake_moon(
                    job, job_root=JOB_ROOT / job_id,
                    nadir_jpeg=str(nadir_path) if nadir_path else None,
                )
            except Exception as moon_error:
                set_status(
                    "failed",
                    f"Lunar terrain: {type(moon_error).__name__}: {str(moon_error)[:160]}",
                )
                return
        else:
            artifact = None
            if job.get("dem"):
                # The page sampled Cesium World Terrain. That grid is the ground.
                # A failure here must not fall through to a flat satellite plane.
                set_status("baking", "Writing the terrain heightmap")
                try:
                    artifact = bake.bake_earth(
                        job, job_root=JOB_ROOT / job_id,
                        nadir_jpeg=str(nadir_path) if nadir_path else None,
                    )
                except Exception as dem_error:
                    set_status(
                        "failed",
                        f"Terrain: {type(dem_error).__name__}: {str(dem_error)[:160]}",
                    )
                    return
            if artifact is None:
                set_status("baking", "Reading OpenStreetMap buildings for the patch")
                try:
                    artifact = bake.bake_job(
                        job, job_root=JOB_ROOT / job_id,
                        nadir_jpeg=str(nadir_path) if nadir_path else None,
                    )
                except Exception as bake_error:
                    # Older clients send no grid. GPS and props still apply to the
                    # world PX4 loads. Open-Elevation is not the ground of a new page.
                    artifact = {
                        "callsign": job["callsign"],
                        "look_at": job["look_at"],
                        "bbox": job.get("bbox") or {},
                        "airframe": "x500_gimbal",
                        "world_sdf": "",
                        "props": [],
                        "bake_error": f"{type(bake_error).__name__}: {str(bake_error)[:160]}",
                    }
            else:
                set_status("baking", "Reading OpenStreetMap buildings for the patch")
            if not artifact.get("props") and nadir_path:
                artifact["props"] = vlm_props.detect_props(
                    nadir_path, job["look_at"], job.get("bbox") or {},
                )
            osm = osm_buildings.buildings_in_job(job)
            artifact["props"] = osm_buildings.merge_buildings(artifact.get("props") or [], osm)
            artifact["osm_buildings"] = len(osm)
            set_status("baking", "Fetching the satellite ground for the patch")
            ground = satellite_ground.fetch_ground(job, JOB_ROOT / job_id / "ground.jpg")
            if ground:
                artifact["ground_image"] = ground["path"]
                artifact["ground_size_m"] = [ground["width_m"], ground["length_m"]]
                # A heightmap photo has to be a PNG with a normal beside it.
                # The JPEG stays available for a flat-plane job that has no grid.
                if artifact.get("heightmap_image") and artifact.get("normal_image"):
                    diffuse = rasters.satellite_png(
                        ground["path"], Path(ground["path"]).with_name("earth_diffuse.png"))
                    if diffuse:
                        artifact["diffuse_image"] = diffuse
                        artifact["diffuse_mount"] = "/root/earth_diffuse.png"
        if job.get("host_port") is None and os.environ.get("DEEPGIS_TWIN_SPAWN_DRY_RUN") != "1":
            set_status("failed", "host_port from the simulation registry is required")
            return
        if job.get("host_port") is not None:
            artifact["host_port"] = int(job["host_port"])
        set_status("spawning", "Starting session container", artifact=artifact)
        meta = spawn.spawn_session(artifact)
        if meta.get("status") == "failed":
            set_status("failed", meta.get("error") or "Spawn failed")
            return
        set_status("enrolling", "Enrolling pose agent")
        enroll = spawn.enroll_pose(job["callsign"], job["look_at"])
        try:
            enroll["frames"] = spawn.start_frames(job["callsign"])
        except Exception as frame_error:
            enroll["frames"] = {"status": "failed", "error": type(frame_error).__name__}
        set_status(
            "ready",
            "Twin ready",
            vehicle=job["callsign"],
            follow_view="chase",
            airframe="x500_gimbal",
            session=meta,
            enroll=enroll,
        )
    except Exception as e:
        with JOBS_LOCK:
            job["status"] = "failed"
            job["message"] = f"{type(e).__name__}: {str(e)[:200]}"
            job["updated"] = time.time()


class Handler(BaseHTTPRequestHandler):
    def _json(self, code, obj):
        body = json.dumps(obj).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if not _auth(self):
            return self._json(401, {"error": "unauthorized"})
        path = urlparse(self.path).path
        if path.startswith("/jobs/"):
            job_id = path[len("/jobs/"):].strip("/")
            with JOBS_LOCK:
                job = JOBS.get(job_id)
            if not job:
                return self._json(404, {"error": "unknown job"})
            keys = ("id", "status", "message", "vehicle", "follow_view", "airframe",
                    "callsign", "created", "updated")
            return self._json(200, {k: job[k] for k in keys if k in job})
        if path in ("/health", "/"):
            return self._json(200, {"ok": True})
        return self._json(404, {"error": "not found"})

    def do_POST(self):
        if not _auth(self):
            return self._json(401, {"error": "unauthorized"})
        path = urlparse(self.path).path
        route = session_route(path)
        if route is not None:
            return self._session(route)
        if path.rstrip("/") != "/jobs":
            return self._json(404, {"error": "not found"})
        length = int(self.headers.get("Content-Length") or 0)
        try:
            data = json.loads(self.rfile.read(length).decode("utf-8"))
        except ValueError:
            return self._json(400, {"error": "invalid json"})
        if not isinstance(data, dict):
            return self._json(400, {"error": "invalid json"})
        job_id = str(data.get("job_id") or secrets.token_hex(16))
        callsign = str(data.get("callsign") or "")
        try:
            spawn._assert_safe_name(callsign)
        except ValueError as e:
            return self._json(400, {"error": str(e)})
        protected = set(data.get("protected_names") or []) | spawn.PROTECTED
        if callsign in protected:
            return self._json(400, {"error": "protected name"})
        raw_port = data.get("host_port")
        host_port = None
        if raw_port is not None:
            try:
                host_port = raw_port if isinstance(raw_port, int) and not isinstance(raw_port, bool) else int(raw_port)
                spawn.publish_args(host_port)
            except (TypeError, ValueError) as e:
                return self._json(400, {"error": str(e)})
        body = data.get("body") or "earth"
        if body not in ("earth", "moon"):
            return self._json(400, {"error": "body must be earth or moon"})
        dem = None
        if body == "moon" or data.get("dem") is not None:
            try:
                dem = capture.clean_dem(data.get("dem"))
            except ValueError as exc:
                return self._json(400, {"error": str(exc)})
        job = {
            "id": job_id,
            "callsign": callsign,
            "vehicle_class": data.get("vehicle_class") or "kestrel",
            "count": 1,
            "width_m": data.get("width_m") or data.get("radius_m") or 150,
            "length_m": data.get("length_m") or data.get("radius_m") or 150,
            "body": body,
            "dem": dem,
            "nadir_b64": data.get("nadir_b64") or "",
            "look_at": data.get("look_at") or {},
            "bbox": data.get("bbox") or {},
            "user": data.get("user"),
            "host_port": host_port,
            "status": "queued",
            "message": "Queued on twin builder",
            "created": time.time(),
            "updated": time.time(),
        }
        if not (isinstance(job["look_at"].get("lat"), (int, float))
                and isinstance(job["look_at"].get("lon"), (int, float))):
            return self._json(400, {"error": "look_at required"})
        with JOBS_LOCK:
            JOBS[job_id] = job
        threading.Thread(target=_run_job, args=(job_id,), daemon=True).start()
        return self._json(200, {"id": job_id, "status": "queued", "callsign": callsign})

    def _session(self, route):
        callsign, action = route
        try:
            spawn._assert_safe_name(callsign)
            if not callsign.startswith("ktwin-"):
                raise ValueError("not a session twin")
        except ValueError as exc:
            return self._json(400, {"error": str(exc)[:200]})
        try:
            if action == "stop":
                result = spawn.stop_session(callsign)
            elif action == "start":
                result = spawn.start_session(callsign)
            else:
                result = spawn.purge_session(callsign)
        except RuntimeError as exc:
            return self._json(400, {"error": str(exc)[:200]})
        return self._json(200, result)

    def log_message(self, fmt, *args):
        # Keep stdout quiet; ops can wrap with a logger
        pass


def _frames_loop():
    """Heal camera sidecars after a session container restart."""
    while True:
        try:
            spawn.reconcile_session_frames()
        except Exception:
            pass
        time.sleep(20)


def main():
    if not TOKEN:
        raise SystemExit("DEEPGIS_TWIN_BUILDER_TOKEN is required")
    JOB_ROOT.mkdir(parents=True, exist_ok=True)
    spawn.SESSIONS_ROOT.mkdir(parents=True, exist_ok=True)
    threading.Thread(target=_frames_loop, daemon=True).start()
    server = ThreadingHTTPServer((HOST, PORT), Handler)
    print(f"twin-builder listening on {HOST}:{PORT}", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
