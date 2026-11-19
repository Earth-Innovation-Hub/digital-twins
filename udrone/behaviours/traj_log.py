"""Per-run trajectory records for the xudrone runners.

Every run gets its own directory, so a later run never overwrites an earlier
one:

  <root>/<YYYYmmdd_HHMMSS>_<skill>_<pid>/
      traj.csv       one row per control tick, world ENU, header line first
      events.jsonl   one JSON object per event (thrust on, path, arrive, fail...)
      summary.json   written when the run ends
  <root>/index.jsonl one summary line per finished run

The root is UDRONE_RUN_LOG_DIR (default /root/log/udrone_runs). Only the
newest KEEP_RUNS directories are kept. UDRONE_RUN_LOG=0 turns it off.
"""
from __future__ import annotations

import json
import math
import os
import shutil
import time

ROOT = os.environ.get("UDRONE_RUN_LOG_DIR", "") or "/root/log/udrone_runs"
KEEP_RUNS = 300
FLUSH_S = 1.0

BASE_COLUMNS = (
    "t", "sim_t", "mode",
    "x", "y", "z", "vx", "vy", "vz", "u_fwd",
    "roll", "pitch", "yaw",
    "x_des", "y_des", "z_des", "vx_des", "vy_des", "vz_des",
    "err", "herr", "zerr",
    "fx", "fy", "fz", "fn",
    "dir_x", "dir_y", "dir_z",
    "thrust", "pitch_u", "yaw_u", "power",
    "gate", "path_t", "depth_i",
    "pose_src", "gz_miss", "latch", "rotor0_n",
    "boat_x", "boat_y", "boat_z",
)


def _fmt(v):
    if v is None:
        return ""
    if isinstance(v, bool):
        return "1" if v else "0"
    if isinstance(v, float):
        if math.isnan(v):
            return "nan"
        return f"{v:.4f}"
    return str(v)


def _prune(root: str, keep: int):
    try:
        runs = sorted(d for d in os.listdir(root) if os.path.isdir(os.path.join(root, d)))
    except OSError:
        return
    for d in runs[:-keep] if len(runs) > keep else []:
        shutil.rmtree(os.path.join(root, d), ignore_errors=True)


class TrajLog:
    def __init__(self, skill: str, extra_columns=(), meta=None, root: str = ROOT):
        self.enabled = os.environ.get("UDRONE_RUN_LOG", "1") != "0"
        self.skill = skill
        self.columns = tuple(BASE_COLUMNS) + tuple(c for c in extra_columns if c not in BASE_COLUMNS)
        self.dir = None
        self.rows = 0
        self._f = None
        self._ev = None
        self._last_flush = 0.0
        self._t0_wall = time.time()
        self.stats = _Stats()
        if not self.enabled:
            return
        stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime(self._t0_wall))
        self.run_id = f"{stamp}_{skill}_{os.getpid()}"
        self.root = root
        try:
            os.makedirs(root, exist_ok=True)
            _prune(root, KEEP_RUNS - 1)
            self.dir = os.path.join(root, self.run_id)
            os.makedirs(self.dir, exist_ok=True)
            self._f = open(os.path.join(self.dir, "traj.csv"), "w")
            self._f.write(",".join(self.columns) + "\n")
            self._ev = open(os.path.join(self.dir, "events.jsonl"), "w")
        except OSError as exc:
            print(f"TRAJ_LOG off ({exc})", flush=True)
            self.enabled = False
            return
        self.event("start", skill=skill, pid=os.getpid(), **(meta or {}))
        print(f"TRAJ_LOG {self.dir}", flush=True)

    def row(self, values: dict):
        self.stats.add(values)
        if not self.enabled or self._f is None:
            return
        self._f.write(",".join(_fmt(values.get(c)) for c in self.columns) + "\n")
        self.rows += 1
        now = time.time()
        if now - self._last_flush > FLUSH_S:
            self._f.flush()
            self._last_flush = now

    def event(self, kind: str, **fields):
        if not self.enabled or self._ev is None:
            return
        rec = {"wall": round(time.time(), 3), "kind": kind}
        rec.update({k: _jsonable(v) for k, v in fields.items()})
        self._ev.write(json.dumps(rec) + "\n")
        self._ev.flush()

    def close(self, outcome: str = "", **fields):
        summary = {
            "run_id": getattr(self, "run_id", None), "skill": self.skill,
            "outcome": outcome, "wall_start": round(self._t0_wall, 3),
            "wall_s": round(time.time() - self._t0_wall, 2), "rows": self.rows,
        }
        summary.update(self.stats.summary())
        summary.update({k: _jsonable(v) for k, v in fields.items()})
        if not self.enabled:
            return summary
        self.event("end", outcome=outcome)
        for f in (self._f, self._ev):
            try:
                f.close()
            except Exception:
                pass
        self._f = self._ev = None
        try:
            with open(os.path.join(self.dir, "summary.json"), "w") as f:
                json.dump(summary, f, indent=1)
            with open(os.path.join(self.root, "index.jsonl"), "a") as f:
                f.write(json.dumps(summary) + "\n")
        except OSError:
            pass
        self.enabled = False
        return summary


def _jsonable(v):
    if isinstance(v, float):
        return None if math.isnan(v) or math.isinf(v) else round(v, 4)
    if isinstance(v, (list, tuple)):
        return [_jsonable(x) for x in v]
    if isinstance(v, dict):
        return {k: _jsonable(x) for k, x in v.items()}
    return v


class _Stats:
    """Running figures for summary.json, over thrust-mode rows only."""

    def __init__(self):
        self.n = 0
        self.err_sum = 0.0
        self.err_max = 0.0
        self.xte_sum = 0.0
        self.xte_max = 0.0
        self.zerr_max = 0.0
        self.sat = 0
        self.thrust_max = 0.0
        self.nose_down_max = 0.0
        self.nose_up_max = 0.0
        self.speed_max = 0.0
        self.dist = 0.0
        self.t_first = None
        self.t_last = None
        self._prev = None
        self.odom_rows = 0

    def add(self, v: dict):
        if v.get("mode") == "handoff":
            return
        self.n += 1
        t = v.get("t")
        self.t_first = t if self.t_first is None else self.t_first
        self.t_last = t
        err = _num(v.get("err"))
        if err is not None:
            self.err_sum += err
            self.err_max = max(self.err_max, err)
        xte = _num(v.get("xte"))
        if xte is not None:
            self.xte_sum += abs(xte)
            self.xte_max = max(self.xte_max, abs(xte))
        zerr = _num(v.get("zerr"))
        if zerr is not None:
            self.zerr_max = max(self.zerr_max, abs(zerr))
        th = _num(v.get("thrust"))
        pw = _num(v.get("power")) or 1.0
        if th is not None:
            self.thrust_max = max(self.thrust_max, abs(th))
            if abs(th) >= 0.98 * pw:
                self.sat += 1
        pitch = _num(v.get("pitch"))
        if pitch is not None:
            # ENU pitch about the left axis: positive is nose down.
            self.nose_down_max = max(self.nose_down_max, math.degrees(pitch))
            self.nose_up_max = max(self.nose_up_max, -math.degrees(pitch))
        sp = [_num(v.get(k)) for k in ("vx", "vy", "vz")]
        if None not in sp:
            self.speed_max = max(self.speed_max, math.sqrt(sum(c * c for c in sp)))
        p = tuple(_num(v.get(k)) for k in ("x", "y", "z"))
        if None not in p:
            if self._prev is not None:
                self.dist += math.dist(p, self._prev)
            self._prev = p
        if v.get("pose_src") == "odom":
            self.odom_rows += 1

    def summary(self) -> dict:
        n = max(self.n, 1)
        out = {
            "ticks": self.n,
            "thrust_s": None if self.t_first is None else round(self.t_last - self.t_first, 2),
            "err_mean": round(self.err_sum / n, 3), "err_max": round(self.err_max, 3),
            "zerr_max": round(self.zerr_max, 3),
            "thrust_max": round(self.thrust_max, 3), "sat_frac": round(self.sat / n, 3),
            "nose_down_max_deg": round(self.nose_down_max, 1),
            "nose_up_max_deg": round(self.nose_up_max, 1),
            "speed_max": round(self.speed_max, 2), "travel_m": round(self.dist, 2),
            "odom_frac": round(self.odom_rows / n, 3),
        }
        if self.xte_max > 0.0:
            out["xte_mean"] = round(self.xte_sum / n, 3)
            out["xte_max"] = round(self.xte_max, 3)
        return out


def _num(v):
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    return None if math.isnan(f) else f
