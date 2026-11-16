#!/usr/bin/env python3
"""Rebuild the Kiwanis bed as a bowl, 1 m at the shore and 10 m at the deepest point.

The old bed was a cone: every shore vertex at -1 m and one centre vertex at
-2.44 m. This writes a curved bowl (steepest at the bank, rounded at the
bottom) plus a small mid-depth ripple, and retargets the tiled box collision
and the bank walls. Triangle-mesh collision is still not used.

  python3 mk_bathymetry.py [model_dir]
"""
from __future__ import annotations

import json
import math
import re
from pathlib import Path

SHORE_M = 1.0
DEEP_M = 10.0
GRID_M = 8.0
TILE_M = 8.0
UV_TILE_M = 8.0
# Bank walls: above the surface down past the deepest bed.
BANK_TOP = 0.40
BANK_BOTTOM = -10.80
ON_BED = {
    "rock_a": 0.55,
    "rock_b": 0.35,
    "rock_c": 0.35,
    "sunken_crate": 0.35,
    "sunken_tire": 0.40,
    "sunken_pipe": 0.18,
    "bed_boulder": 0.85,
}


def _load_shore(path: Path):
    raw = json.loads(path.read_text(encoding="utf-8"))["enu_m"]
    pts = [(float(x), float(y)) for x, y in raw]
    if len(pts) > 1 and math.hypot(pts[0][0] - pts[-1][0], pts[0][1] - pts[-1][1]) < 0.05:
        pts = pts[:-1]
    return pts


def _inside(x: float, y: float, shore) -> bool:
    n = len(shore)
    hit = False
    j = n - 1
    for i in range(n):
        xi, yi = shore[i]
        xj, yj = shore[j]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi + 0.0) + xi):
            hit = not hit
        j = i
    return hit


def _dist_shore(x: float, y: float, shore) -> float:
    best = 1e18
    n = len(shore)
    for i in range(n):
        ax, ay = shore[i]
        bx, by = shore[(i + 1) % n]
        dx, dy = bx - ax, by - ay
        den = dx * dx + dy * dy
        t = 0.0 if den < 1e-12 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / den))
        best = min(best, math.hypot(x - (ax + t * dx), y - (ay + t * dy)))
    return best


def _incenter(shore):
    xs = [p[0] for p in shore]
    ys = [p[1] for p in shore]
    step = 6.0
    best, bd = (0.0, 0.0), -1.0
    x = min(xs)
    while x <= max(xs):
        y = min(ys)
        while y <= max(ys):
            if _inside(x, y, shore):
                d = _dist_shore(x, y, shore)
                if d > bd:
                    best, bd = (x, y), d
            y += step
        x += step
    # A few tightenings so the 10 m point is the real deepest sample.
    for step in (2.0, 0.5):
        cx, cy = best
        for ix in range(-3, 4):
            for iy in range(-3, 4):
                x, y = cx + ix * step, cy + iy * step
                if _inside(x, y, shore):
                    d = _dist_shore(x, y, shore)
                    if d > bd:
                        best, bd = (x, y), d
    return best, bd


def depth_at(x, y, shore, dmax):
    """Metres below the surface. 1 at the bank, 10 at the incenter, curved."""
    u = max(0.0, min(1.0, _dist_shore(x, y, shore) / dmax))
    bowl = u * (2.0 - u)                 # slope dies out at the bottom
    ripple = 0.8 * math.sin(x / 47.0) * math.sin(y / 33.0) * (4.0 * u * (1.0 - u))
    return min(DEEP_M, max(SHORE_M, SHORE_M + (DEEP_M - SHORE_M) * bowl + ripple))


class _Tri:
    __slots__ = ("a", "b", "c", "cx", "cy", "r2", "alive")

    def __init__(self, pts, a, b, c):
        self.a, self.b, self.c = a, b, c
        self.alive = True
        ax, ay = pts[a]
        bx, by = pts[b]
        cx, cy = pts[c]
        d = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
        if abs(d) < 1e-8:
            self.cx = self.cy = 0.0
            self.r2 = -1.0
            return
        a2, b2, c2 = ax * ax + ay * ay, bx * bx + by * by, cx * cx + cy * cy
        self.cx = (a2 * (by - cy) + b2 * (cy - ay) + c2 * (ay - by)) / d
        self.cy = (a2 * (cx - bx) + b2 * (ax - cx) + c2 * (bx - ax)) / d
        self.r2 = (self.cx - ax) ** 2 + (self.cy - ay) ** 2

    def contains(self, x, y):
        if self.r2 < 0.0:
            return False
        return (x - self.cx) ** 2 + (y - self.cy) ** 2 <= self.r2 * (1.0 + 1e-10)


def _delaunay(pts):
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    span = max(max(xs) - min(xs), max(ys) - min(ys)) * 8.0
    mx, my = (min(xs) + max(xs)) / 2.0, (min(ys) + max(ys)) / 2.0
    work = list(pts) + [(mx - span, my - span), (mx, my + span), (mx + span, my - span)]
    n = len(pts)
    tris = [_Tri(work, n, n + 1, n + 2)]
    for i in range(n):
        x, y = work[i]
        bad = [t for t in tris if t.alive and t.contains(x, y)]
        edges = {}
        for t in bad:
            t.alive = False
            for e in ((t.a, t.b), (t.b, t.c), (t.c, t.a)):
                key = (e[0], e[1]) if e[0] < e[1] else (e[1], e[0])
                edges[key] = edges.get(key, 0) + 1
        for (a, b), count in edges.items():
            if count == 1:
                tris.append(_Tri(work, a, b, i))
    out = []
    for t in tris:
        if t.alive and t.a < n and t.b < n and t.c < n and t.r2 >= 0.0:
            out.append((t.a, t.b, t.c))
    return out


def _build_mesh(shore, center, dmax):
    xs = [p[0] for p in shore]
    ys = [p[1] for p in shore]
    pts = list(shore)
    x = math.floor(min(xs) / GRID_M) * GRID_M
    while x <= max(xs):
        y = math.floor(min(ys) / GRID_M) * GRID_M
        while y <= max(ys):
            if _inside(x, y, shore) and _dist_shore(x, y, shore) > 2.0:
                pts.append((x, y))
            y += GRID_M
        x += GRID_M
    pts.append(center)
    faces = []
    for a, b, c in _delaunay(pts):
        pa, pb, pc = pts[a], pts[b], pts[c]
        mids = (
            ((pa[0] + pb[0]) / 2, (pa[1] + pb[1]) / 2),
            ((pb[0] + pc[0]) / 2, (pb[1] + pc[1]) / 2),
            ((pc[0] + pa[0]) / 2, (pc[1] + pa[1]) / 2),
        )
        if not all(_inside(mx, my, shore) or _dist_shore(mx, my, shore) < 1.0 for mx, my in mids):
            continue
        cross = (pb[0] - pa[0]) * (pc[1] - pa[1]) - (pb[1] - pa[1]) * (pc[0] - pa[0])
        if cross < 0.0:
            a, c = c, a
        faces.append((a, b, c))
    xyz = []
    for x, y in pts:
        xyz.append((x, y, -depth_at(x, y, shore, dmax)))
    # The incenter is the last point and is exactly the deepest sample.
    xyz[-1] = (center[0], center[1], -DEEP_M)
    return xyz, faces


def _normals(xyz, faces):
    acc = [[0.0, 0.0, 0.0] for _ in xyz]
    for a, b, c in faces:
        ax, ay, az = xyz[a]
        bx, by, bz = xyz[b]
        cx, cy, cz = xyz[c]
        nx = (by - ay) * (cz - az) - (bz - az) * (cy - ay)
        ny = (bz - az) * (cx - ax) - (bx - ax) * (cz - az)
        nz = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
        for i in (a, b, c):
            acc[i][0] += nx
            acc[i][1] += ny
            acc[i][2] += nz
    out = []
    for nx, ny, nz in acc:
        n = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
        out.append((nx / n, ny / n, nz / n))
    return out


def _write_obj(path: Path, xyz, faces, normals, with_uv: bool):
    lines = ["# kiwanis bowl bed: shore 1 m, deepest 10 m"]
    lines.append("o kiwanis_bed")
    for x, y, z in xyz:
        lines.append(f"v {x:.5f} {y:.5f} {z:.5f}")
    if with_uv:
        for x, y, _ in xyz:
            lines.append(f"vt {x / UV_TILE_M:.6f} {y / UV_TILE_M:.6f}")
        for nx, ny, nz in normals:
            lines.append(f"vn {nx:.6f} {ny:.6f} {nz:.6f}")
        for a, b, c in faces:
            lines.append(
                f"f {a+1}/{a+1}/{a+1} {b+1}/{b+1}/{b+1} {c+1}/{c+1}/{c+1}"
            )
    else:
        for a, b, c in faces:
            lines.append(f"f {a+1} {b+1} {c+1}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8", newline="\n")


def _tile_xml(i, x, y, top_z):
    cz = top_z - 0.20
    return (
        f'      <collision name="bed_tile_{i}">\n'
        f'        <pose>{x:.3f} {y:.3f} {cz:.4f} 0 0 0</pose>\n'
        f'        <geometry><box><size>{TILE_M:.2f} {TILE_M:.2f} 0.40</size></box></geometry>\n'
        f'        <surface><friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction>\n'
        f'          <contact><ode><max_vel>0.5</max_vel><min_depth>0.001</min_depth></ode></contact></surface>\n'
        f'      </collision>'
    )


def _tiles(shore, dmax):
    xs = [p[0] for p in shore]
    ys = [p[1] for p in shore]
    blocks = []
    x = math.floor(min(xs) / TILE_M) * TILE_M + TILE_M / 2
    while x <= max(xs):
        y = math.floor(min(ys) / TILE_M) * TILE_M + TILE_M / 2
        while y <= max(ys):
            samples = []
            for dx in (-TILE_M / 2, TILE_M / 2):
                for dy in (-TILE_M / 2, TILE_M / 2):
                    px, py = x + dx, y + dy
                    if _inside(px, py, shore):
                        samples.append(depth_at(px, py, shore, dmax))
            if _inside(x, y, shore):
                samples.append(depth_at(x, y, shore, dmax))
            if samples:
                # Top of the box is the shallowest corner, so the hull cannot
                # pass through the bank side of a sloping cell.
                blocks.append((x, y, -min(samples)))
            y += TILE_M
        x += TILE_M
    return blocks


def _stretch_banks(sdf: str) -> str:
    height = BANK_TOP - BANK_BOTTOM
    center = (BANK_TOP + BANK_BOTTOM) / 2.0

    def repl(match):
        block = match.group(0)
        pose = re.search(r"<pose>([^<]+)</pose>", block)
        size = re.search(r"<size>([^<]+)</size>", block)
        pv = [float(v) for v in pose.group(1).split()]
        sv = [float(v) for v in size.group(1).split()]
        pv[2] = center
        sv[2] = height
        block = block.replace(pose.group(0), "<pose>" + " ".join(f"{v:.4f}" for v in pv) + "</pose>", 1)
        block = block.replace(size.group(0), "<size>" + " ".join(f"{v:.4f}" for v in sv) + "</size>", 1)
        return block

    return re.sub(r'<collision name="bank_\d+">.*?</collision>', repl, sdf, flags=re.S)


def _seat_props(sdf: str, shore, dmax) -> str:
    for name, lift in ON_BED.items():
        pat = re.compile(
            rf'(<visual name="{name}">\s*<pose>)([-\d.]+) ([-\d.]+) ([-\d.]+) ([^<]+)(</pose>)'
        )
        m = pat.search(sdf)
        if not m:
            raise SystemExit(f"missing prop {name}")
        x, y = float(m.group(2)), float(m.group(3))
        z = -depth_at(x, y, shore, dmax) + lift
        sdf = pat.sub(rf"\g<1>\2 \3 {z:.2f} \5\6", sdf, count=1)
    return sdf


def _patch_sdf(sdf: str, tiles, shore, dmax) -> str:
    tile_xml = "\n".join(_tile_xml(i, x, y, z) for i, (x, y, z) in enumerate(tiles))
    sdf = re.sub(r'      <collision name="bed_tile_\d+">.*?</collision>\n', "", sdf, flags=re.S)
    sdf = sdf.replace(
        '      <collision name="bank_0">',
        tile_xml + "\n      <collision name=\"bank_0\">",
        1,
    )
    sdf = _stretch_banks(sdf)
    sdf = _seat_props(sdf, shore, dmax)
    sdf = sdf.replace(
        "Depth dish shore 1.0 m → center 2.44 m.",
        "Bowl bed: 1 m at the shore, 10 m at the deepest point.",
    )
    sdf = sdf.replace(
        "Centre sits 0.25 m above the bed dish (~z=-2.08).",
        "Stays in the water column. The bowl under it is much deeper.",
    )
    sdf = sdf.replace(
        "<!-- Bottom: rocks / debris on the depth dish -->",
        "<!-- Bottom: rocks / debris seated on the bowl -->",
    )
    return sdf


def main():
    import sys
    root = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parent
    shore = _load_shore(root / "kiwanis_shore_enu.json")
    center, dmax = _incenter(shore)
    xyz, faces = _build_mesh(shore, center, dmax)
    normals = _normals(xyz, faces)
    mesh = root / "meshes"
    _write_obj(mesh / "bed.obj", xyz, faces, normals, with_uv=True)
    _write_obj(mesh / "bed.obj.pre-uv", xyz, faces, normals, with_uv=False)
    tiles = _tiles(shore, dmax)
    sdf_path = root / "model.sdf"
    sdf_path.write_text(_patch_sdf(sdf_path.read_text(encoding="utf-8"), tiles, shore, dmax), encoding="utf-8", newline="\n")
    zs = [p[2] for p in xyz]
    print(
        f"bowl verts={len(xyz)} faces={len(faces)} tiles={len(tiles)} "
        f"incenter=({center[0]:.1f},{center[1]:.1f}) dmax={dmax:.1f} m "
        f"z=[{min(zs):.2f},{max(zs):.2f}] origin_depth={depth_at(0, 0, shore, dmax):.2f}"
    )


if __name__ == "__main__":
    main()
