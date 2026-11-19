"""A run record gets its own directory, a CSV with the declared columns,
events, a summary, and an index line; old runs are pruned."""
import csv
import json
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import traj_log as tl

root = tempfile.mkdtemp(prefix="trajlog_")
log = tl.TrajLog("swim", extra_columns=("xte", "leg"), meta={"guidance": "los"}, root=root)
assert log.enabled and os.path.isdir(log.dir)
for i in range(5):
    log.row({"t": i * 0.05, "mode": "los", "x": float(i), "y": 0.0, "z": -1.0,
             "vx": 1.0, "vy": 0.0, "vz": 0.0, "err": 0.1 * i, "xte": -0.2 * i, "leg": 0,
             "thrust": 1.0 if i == 4 else 0.2, "power": 1.0, "pitch": 0.1, "pose_src": "odom"})
log.row({"t": 0.3, "mode": "handoff", "x": 99.0, "err": 50.0})
log.event("arrive", miss=0.2)
summary = log.close("pass", miss=0.2)

with open(os.path.join(log.dir, "traj.csv")) as f:
    rows = list(csv.DictReader(f))
assert len(rows) == 6 and "xte" in rows[0] and rows[0]["mode"] == "los"
assert rows[2]["xte"] == "-0.4000"
kinds = [json.loads(line)["kind"] for line in open(os.path.join(log.dir, "events.jsonl"))]
assert kinds == ["start", "arrive", "end"], kinds
s = json.load(open(os.path.join(log.dir, "summary.json")))
assert s["outcome"] == "pass" and s["ticks"] == 5, s
assert abs(s["err_max"] - 0.4) < 1e-9 and abs(s["xte_max"] - 0.8) < 1e-9
assert s["sat_frac"] == 0.2 and s["travel_m"] == 4.0 and s["odom_frac"] == 1.0
idx = [json.loads(line) for line in open(os.path.join(root, "index.jsonl"))]
assert idx[-1]["run_id"] == summary["run_id"]

for i in range(5):
    os.makedirs(os.path.join(root, f"19990101_00000{i}_old_1"))
tl._prune(root, 3)
assert len([d for d in os.listdir(root) if os.path.isdir(os.path.join(root, d))]) == 3

os.environ["UDRONE_RUN_LOG"] = "0"
off = tl.TrajLog("swim", root=root)
off.row({"t": 0.0})
assert not off.enabled and off.close("pass")["rows"] == 0
print("traj log ok")
