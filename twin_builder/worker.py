"""Authenticated twin-builder HTTP worker for dreamslab.

POST /jobs  — start bake+spawn+enroll (Bearer token)
GET  /jobs/<id> — status

Bind to localhost / overlay only. Do not expose without auth.
"""
from __future__ import annotations

import json
import os
import secrets
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

from . import bake, spawn

JOBS = {}
JOBS_LOCK = threading.Lock()
TOKEN = os.environ.get("DEEPGIS_TWIN_BUILDER_TOKEN", "")
JOB_ROOT = Path(os.environ.get("DEEPGIS_TWIN_JOBS_DIR", "/var/twin-jobs"))
HOST = os.environ.get("DEEPGIS_TWIN_BUILDER_HOST", "127.0.0.1")
PORT = int(os.environ.get("DEEPGIS_TWIN_BUILDER_PORT", "8769"))


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
        set_status("baking", "Baking heightmap world and VLM props")
        artifact = bake.bake_job(job, job_root=JOB_ROOT / job_id)
        set_status("spawning", "Starting session container", artifact=artifact)
        meta = spawn.spawn_session(artifact)
        if meta.get("status") == "failed":
            set_status("failed", meta.get("error") or "Spawn failed")
            return
        set_status("enrolling", "Enrolling pose agent")
        enroll = spawn.enroll_pose(job["callsign"], job["look_at"])
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
        job = {
            "id": job_id,
            "callsign": callsign,
            "vehicle_class": data.get("vehicle_class") or "kestrel",
            "radius_m": data.get("radius_m") or 150,
            "look_at": data.get("look_at") or {},
            "bbox": data.get("bbox") or {},
            "user": data.get("user"),
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

    def log_message(self, fmt, *args):
        # Keep stdout quiet; ops can wrap with a logger
        pass


def main():
    if not TOKEN:
        raise SystemExit("DEEPGIS_TWIN_BUILDER_TOKEN is required")
    JOB_ROOT.mkdir(parents=True, exist_ok=True)
    spawn.SESSIONS_ROOT.mkdir(parents=True, exist_ok=True)
    server = ThreadingHTTPServer((HOST, PORT), Handler)
    print(f"twin-builder listening on {HOST}:{PORT}", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
