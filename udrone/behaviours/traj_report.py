#!/usr/bin/env python3
"""Read the per-run trajectory records written by traj_log.

  traj_report.py                     table of the last 15 runs
  traj_report.py --last 40 --skill swim
  traj_report.py RUN                 one run: summary, events, per-second trace
  traj_report.py RUN --plot out.png  plus top view, depth, speed, thrust, errors

RUN is a run directory, a run id, or "latest" (optionally "latest:swim").
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from traj_log import ROOT  # noqa: E402

TABLE = (
    ("run_id", 32), ("outcome", 7), ("thrust_s", 8), ("err_mean", 8), ("err_max", 7),
    ("xte_max", 7), ("zerr_max", 8), ("sat_frac", 8), ("nose_down_max_deg", 9),
    ("speed_max", 9), ("miss", 6),
)


def _index(root):
    path = os.path.join(root, "index.jsonl")
    if not os.path.isfile(path):
        return []
    out = []
    with open(path) as f:
        for line in f:
            try:
                out.append(json.loads(line))
            except ValueError:
                pass
    return out


def _cell(v, width):
    if v is None:
        s = "-"
    elif isinstance(v, float):
        s = f"{v:.2f}"
    else:
        s = str(v)
    return s[:width].ljust(width)


def table(root, last, skill):
    runs = [r for r in _index(root) if not skill or r.get("skill") == skill][-last:]
    print(" ".join(_cell(name, w) for name, w in TABLE))
    for r in runs:
        print(" ".join(_cell(r.get(name), w) for name, w in TABLE))
    if not runs:
        print(f"(no finished runs under {root})")


def _resolve(root, spec):
    if os.path.isdir(spec):
        return spec
    if spec.startswith("latest"):
        skill = spec.split(":", 1)[1] if ":" in spec else ""
        runs = sorted(d for d in os.listdir(root)
                      if os.path.isdir(os.path.join(root, d)) and (not skill or f"_{skill}_" in d))
        if not runs:
            sys.exit(f"no runs under {root}")
        return os.path.join(root, runs[-1])
    path = os.path.join(root, spec)
    if os.path.isdir(path):
        return path
    sys.exit(f"no run {spec!r} under {root}")


def _rows(run_dir):
    with open(os.path.join(run_dir, "traj.csv")) as f:
        rows = list(csv.DictReader(f))
    for r in rows:
        for k, v in r.items():
            try:
                r[k] = float(v)
            except (TypeError, ValueError):
                pass
    return rows


def _f(r, k):
    v = r.get(k)
    return v if isinstance(v, float) else float("nan")


def detail(run_dir):
    print(f"run {run_dir}")
    path = os.path.join(run_dir, "summary.json")
    if os.path.isfile(path):
        print(json.dumps(json.load(open(path)), indent=1))
    else:
        print("(no summary: the run is still going or was killed)")
    print("events:")
    for line in open(os.path.join(run_dir, "events.jsonl")):
        ev = json.loads(line)
        t = ev.pop("t", None)
        kind = ev.pop("kind")
        ev.pop("wall", None)
        print(f"  t={'-' if t is None else f'{t:6.2f}'} {kind:12s} {json.dumps(ev)}")
    rows = _rows(run_dir)
    if not rows:
        return rows
    has_los = any(isinstance(r.get("xte"), float) for r in rows)
    print("per second:")
    head = "      t mode     x      y      z   u_fwd  err  thrust nose_dn"
    if has_los:
        head += "   xte    vte  u_ref head_err gate"
    print(head)
    next_t = -1.0
    for r in rows:
        t = _f(r, "t")
        if t < next_t:
            continue
        next_t = t + 1.0
        nose = math.degrees(_f(r, "pitch"))
        line = (f"{t:7.1f} {str(r.get('mode'))[:7]:7s} {_f(r, 'x'):6.2f} {_f(r, 'y'):6.2f} {_f(r, 'z'):6.2f} "
                f"{_f(r, 'u_fwd'):+6.2f} {_f(r, 'err'):5.2f} {_f(r, 'thrust'):+6.3f} {nose:6.0f}")
        if has_los:
            line += (f" {_f(r, 'xte'):+6.2f} {_f(r, 'vte'):+6.2f} {_f(r, 'u_ref'):+6.2f} "
                     f"{math.degrees(_f(r, 'head_err')):+7.0f} {_f(r, 'surge_gate'):4.2f}")
        print(line)
    return rows


def plot(run_dir, rows, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fly = [r for r in rows if r.get("mode") != "handoff"]
    t = [_f(r, "t") for r in fly]
    fig, ax = plt.subplots(2, 3, figsize=(15, 8))
    top = ax[0][0]
    top.plot([_f(r, "x") for r in fly], [_f(r, "y") for r in fly], "b-", label="hull")
    top.plot([_f(r, "x_des") for r in fly], [_f(r, "y_des") for r in fly], "m--", lw=1, label="reference / aim")
    for line in open(os.path.join(run_dir, "events.jsonl")):
        ev = json.loads(line)
        if ev.get("kind") == "path" and ev.get("start") and ev.get("end"):
            s, e = ev["start"], ev["end"]
            top.plot([s[0], e[0]], [s[1], e[1]], "k:", lw=1, label="path")
            top.plot([e[0]], [e[1]], "kx")
    if fly:
        top.plot([_f(fly[0], "x")], [_f(fly[0], "y")], "go")
    top.set_aspect("equal", adjustable="datalim")
    top.set_title("top view (ENU m)")
    top.legend(fontsize=7)
    ax[0][1].plot(t, [_f(r, "z") for r in fly], "b-", label="z")
    ax[0][1].plot(t, [_f(r, "z_des") for r in fly], "m--", label="z ref")
    ax[0][1].set_title("depth (m)")
    ax[0][1].legend(fontsize=7)
    ax[0][2].plot(t, [_f(r, "u_fwd") for r in fly], "b-", label="u along nose")
    if any(isinstance(r.get("u_ref"), float) for r in fly):
        ax[0][2].plot(t, [_f(r, "u_ref") for r in fly], "m--", label="u_ref")
    ax[0][2].set_title("speed (m/s)")
    ax[0][2].legend(fontsize=7)
    ax[1][0].plot(t, [_f(r, "thrust") for r in fly], "r-", label="thrust")
    ax[1][0].plot(t, [_f(r, "pitch_u") for r in fly], "g-", lw=1, label="pitch_u")
    ax[1][0].plot(t, [_f(r, "yaw_u") for r in fly], "c-", lw=1, label="yaw_u")
    ax[1][0].set_title("commands (normalized)")
    ax[1][0].legend(fontsize=7)
    ax[1][1].plot(t, [math.degrees(_f(r, "pitch")) for r in fly], "g-", label="nose down (deg)")
    yaw, prev, off = [], None, 0.0
    for r in fly:
        y = _f(r, "yaw")
        if prev is not None and not math.isnan(y):
            if y - prev > math.pi:
                off -= 2.0 * math.pi
            elif y - prev < -math.pi:
                off += 2.0 * math.pi
        if not math.isnan(y):
            prev = y
        yaw.append(math.degrees(y + off))
    ax[1][1].plot(t, yaw, "c-", label="yaw (deg, unwrapped)")
    if any(isinstance(r.get("head_err"), float) for r in fly):
        ax[1][1].plot(t, [math.degrees(_f(r, "head_err")) for r in fly], "k--", lw=1, label="heading error")
    ax[1][1].set_title("attitude")
    ax[1][1].legend(fontsize=7)
    ax[1][2].plot(t, [_f(r, "err") for r in fly], "b-", label="path error")
    if any(isinstance(r.get("xte"), float) for r in fly):
        ax[1][2].plot(t, [_f(r, "xte") for r in fly], "m-", label="cross-track")
        ax[1][2].plot(t, [_f(r, "vte") for r in fly], "g-", label="depth off line")
    ax[1][2].plot(t, [_f(r, "gz_miss") for r in fly], "k:", lw=1, label="odom vs gz")
    ax[1][2].set_title("errors (m)")
    ax[1][2].legend(fontsize=7)
    for a in ax.flat[1:]:
        a.set_xlabel("thrust time (s)")
        a.grid(alpha=0.3)
    top.grid(alpha=0.3)
    fig.suptitle(os.path.basename(run_dir))
    fig.tight_layout()
    fig.savefig(out, dpi=90)
    print(f"plot {out}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run", nargs="?")
    ap.add_argument("--root", default=ROOT)
    ap.add_argument("--last", type=int, default=15)
    ap.add_argument("--skill", default="")
    ap.add_argument("--plot", default="")
    a = ap.parse_args()
    if not a.run:
        table(a.root, a.last, a.skill)
        return 0
    run_dir = _resolve(a.root, a.run)
    rows = detail(run_dir)
    if a.plot and rows:
        plot(run_dir, rows, a.plot)
    return 0


if __name__ == "__main__":
    sys.exit(main())
