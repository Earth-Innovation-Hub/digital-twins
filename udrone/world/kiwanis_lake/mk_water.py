#!/usr/bin/env python3
"""Wavy, see-through Kiwanis water surface.

The cameras look through a classic material (ogre2 PBR drops transparency and
hides the bed and the tags). The chop is in the mesh: a few summed swells,
with normals taken from the slope so the sun glints. A short foam ribbon
marks the bank.

  python3 mk_water.py [model_dir]
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import mk_bathymetry as bath

GRID_M = 3.0
FOAM_M = 4.0
# Height waves (amplitude m, wavelength m, direction rad). Longer than the grid.
SWELLS = (
    (0.140, 16.0, 0.55),
    (0.070, 7.0, 1.90),
)
# Lighting-only chop (slope, wavelength m, direction). Kept out of the
# vertex height so crests stay clear of the hulls. The 3 m grid samples
# these as glitter: neighbouring facets catch the sun differently.
CHOP = (
    (0.38, 3.4, 0.35),
    (0.24, 1.8, 2.40),
)


def swell(x: float, y: float):
    z = 0.01
    dzdx = 0.0
    dzdy = 0.0
    for amp, length, ang in SWELLS:
        k = 2.0 * math.pi / length
        dx, dy = math.cos(ang), math.sin(ang)
        phase = k * (x * dx + y * dy)
        z += amp * math.sin(phase)
        slope = amp * k * math.cos(phase)
        dzdx += slope * dx
        dzdy += slope * dy
    for amp, length, ang in CHOP:
        k = 2.0 * math.pi / length
        dx, dy = math.cos(ang), math.sin(ang)
        phase = k * (x * dx + y * dy)
        slope = amp * math.cos(phase)
        dzdx += slope * dx
        dzdy += slope * dy
    n = math.sqrt(dzdx * dzdx + dzdy * dzdy + 1.0)
    return z, (-dzdx / n, -dzdy / n, 1.0 / n)


def _grid(shore):
    xs = [p[0] for p in shore]
    ys = [p[1] for p in shore]
    pts = list(shore)
    x = math.floor(min(xs) / GRID_M) * GRID_M
    while x <= max(xs):
        y = math.floor(min(ys) / GRID_M) * GRID_M
        while y <= max(ys):
            if bath._inside(x, y, shore) and bath._dist_shore(x, y, shore) > 1.5:
                pts.append((x, y))
            y += GRID_M
        x += GRID_M
    faces = []
    for a, b, c in bath._delaunay(pts):
        tri = (pts[a], pts[b], pts[c])
        mids = (
            ((tri[0][0] + tri[1][0]) / 2, (tri[0][1] + tri[1][1]) / 2),
            ((tri[1][0] + tri[2][0]) / 2, (tri[1][1] + tri[2][1]) / 2),
            ((tri[2][0] + tri[0][0]) / 2, (tri[2][1] + tri[0][1]) / 2),
        )
        if not all(bath._inside(mx, my, shore) or bath._dist_shore(mx, my, shore) < 0.8
                   for mx, my in mids):
            continue
        cross = ((tri[1][0] - tri[0][0]) * (tri[2][1] - tri[0][1])
                 - (tri[1][1] - tri[0][1]) * (tri[2][0] - tri[0][0]))
        if cross < 0.0:
            a, c = c, a
        faces.append((a, b, c))
    return pts, faces


def _write_obj(path: Path, xyz, normals, faces, label: str):
    lines = [f"# {label}", f"o {label}"]
    for x, y, z in xyz:
        lines.append(f"v {x:.5f} {y:.5f} {z:.5f}")
    for x, y, _ in xyz:
        lines.append(f"vt {x / 12.0:.6f} {y / 12.0:.6f}")
    for nx, ny, nz in normals:
        lines.append(f"vn {nx:.5f} {ny:.5f} {nz:.5f}")
    for a, b, c in faces:
        # One winding, normals up. <double_sided> shows it from under the surface
        # without a second coplanar copy (that pair z-fights).
        lines.append(f"f {a+1}/{a+1}/{a+1} {b+1}/{b+1}/{b+1} {c+1}/{c+1}/{c+1}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8", newline="\n")


def _foam(shore):
    xyz, normals, faces = [], [], []
    n = len(shore)

    def add(x, y):
        z, normal = swell(x, y)
        xyz.append((x, y, z + 0.02))
        normals.append(normal)
        return len(xyz) - 1

    for i in range(n):
        ax, ay = shore[i]
        bx, by = shore[(i + 1) % n]
        ex, ey = bx - ax, by - ay
        el = math.hypot(ex, ey)
        if el < 0.6:
            continue
        nx, ny = -ey / el, ex / el
        mx, my = (ax + bx) / 2, (ay + by) / 2
        if not bath._inside(mx + nx * 0.4, my + ny * 0.4, shore):
            nx, ny = -nx, -ny
        if not bath._inside(mx + nx * 0.4, my + ny * 0.4, shore):
            continue
        ia, ib = add(ax, ay), add(bx, by)
        ic, id_ = add(ax + nx * FOAM_M, ay + ny * FOAM_M), add(bx + nx * FOAM_M, by + ny * FOAM_M)
        if not (bath._inside(xyz[ic][0], xyz[ic][1], shore)
                and bath._inside(xyz[id_][0], xyz[id_][1], shore)):
            continue
        faces.append((ia, ib, id_))
        faces.append((ia, id_, ic))
    return xyz, normals, faces


def main():
    root = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parent
    shore = bath._load_shore(root / "kiwanis_shore_enu.json")
    pts, faces = _grid(shore)
    xyz, normals = [], []
    for x, y in pts:
        z, normal = swell(x, y)
        xyz.append((x, y, z))
        normals.append(normal)
    mesh = root / "meshes"
    _write_obj(mesh / "water.obj", xyz, normals, faces, "kiwanis_water")
    # pre-uv keeps the chop if the texture script rebuilds UVs.
    pre = ["# kiwanis water chop", "o kiwanis_water"]
    for x, y, z in xyz:
        pre.append(f"v {x:.5f} {y:.5f} {z:.5f}")
    for nx, ny, nz in normals:
        pre.append(f"vn {nx:.5f} {ny:.5f} {nz:.5f}")
    for a, b, c in faces:
        pre.append(f"f {a+1}//{a+1} {b+1}//{b+1} {c+1}//{c+1}")
    (mesh / "water.obj.pre-uv").write_text("\n".join(pre) + "\n", encoding="utf-8", newline="\n")
    # The AUV looks up at the back of the surface. ogre2 does not draw that
    # side from <double_sided>, so the underside is its own front-facing sheet.
    under = [(x, y, z - 0.008) for x, y, z in xyz]
    under_n = [(-nx, -ny, -nz) for nx, ny, nz in normals]
    under_f = [(a, c, b) for a, b, c in faces]
    _write_obj(mesh / "water_under.obj", under, under_n, under_f, "kiwanis_water_under")
    # The nadir cameras sit on the waterline and look straight down, so the
    # surface itself is behind them. A thin sheet in the water column is what
    # puts the lake colour in those frames. It stays transparent: the bed and
    # the tags are below it.
    veil = [(x, y, -0.55) for x, y, _ in xyz]
    veil_n = [(0.0, 0.0, 1.0) for _ in xyz]
    _write_obj(mesh / "veil.obj", veil, veil_n, faces, "kiwanis_veil")
    foam_xyz, foam_n, foam_f = _foam(shore)
    _write_obj(mesh / "foam.obj", foam_xyz, foam_n, foam_f, "kiwanis_foam")
    zs = [p[2] for p in xyz]
    print(
        f"water verts={len(xyz)} faces={len(faces)} foam={len(foam_f)} "
        f"z=[{min(zs):.3f},{max(zs):.3f}]"
    )


if __name__ == "__main__":
    main()
