"""Map-aware coverage plan for the xudrone. ROS-free.

Lanes run along `heading_deg` (compass: 0 north, 90 east) across a rectangle
centred on `center`, spaced `swath` apart. Each lane keeps only the stretch the
map still needs ("bed": bed depth unknown; "camera": no down-camera footprint
yet) and drops cells the costmap blocks. Lanes are flown boustrophedon. A
transit between lanes is straight when the costmap allows it, otherwise an A*
path over the costmap (inflated cells cost more, unknown water costs
UNKNOWN_COST, blocked cells are never entered).
"""
from __future__ import annotations

import heapq
import math

import numpy as np

import map_geom as mg

SAMPLE_M = 0.5
BLOCK_COST = mg.COST_INSCRIBED      # cells at or above this are not flown
ESCAPE_M = 2.0                      # the start may sit in a blocked cell (docked under the boat)
TARGETS = ("bed", "camera")


def lane_frame(heading_deg):
    """Unit vectors (along lane, across lane to the right) in ENU."""
    h = math.radians(heading_deg)
    return np.array([math.sin(h), math.cos(h)]), np.array([math.cos(h), -math.sin(h)])


def region_mask(grid, center, width, length, heading_deg):
    """[ix, iy] cells inside the plan rectangle."""
    u, v = lane_frame(heading_deg)
    xs, ys = grid.center(np.arange(grid.n)[:, None], np.arange(grid.n)[None, :])
    dx, dy = xs - center[0], ys - center[1]
    a = dx * u[0] + dy * u[1]
    b = dx * v[0] + dy * v[1]
    return (np.abs(a) <= length / 2.0) & (np.abs(b) <= width / 2.0)


def needed(grid, target):
    """[ix, iy] cells the map still wants."""
    if target == "camera":
        return grid.cover == 0
    return ~np.isfinite(grid.bed_z())


def _costs(cost, grid, xy, unknown_cost):
    ix, iy, ok = grid.index(xy[:, 0], xy[:, 1])
    c = np.full(len(xy), 127, np.int32)
    c[ok] = cost[ix[ok], iy[ok]]
    c[ok & (c < 0)] = unknown_cost
    return c


def line_free(cost, grid, a, b, unknown_cost=mg.UNKNOWN_COST, start=None):
    a, b = np.asarray(a, float), np.asarray(b, float)
    n = max(2, int(math.ceil(np.linalg.norm(b - a) / (grid.res / 2.0))) + 1)
    pts = a + np.linspace(0.0, 1.0, n)[:, None] * (b - a)
    c = _costs(cost, grid, pts, unknown_cost)
    if start is not None:
        c[np.hypot(pts[:, 0] - start[0], pts[:, 1] - start[1]) <= ESCAPE_M] = 0
    return bool(np.all(c < BLOCK_COST))


def astar(cost, grid, a, b, unknown_cost=mg.UNKNOWN_COST, margin_m=15.0, start=None):
    """Cell-centre path a -> b over the costmap, or None."""
    ax, ay, oka = grid.index(a[0], a[1])
    bx, by, okb = grid.index(b[0], b[1])
    if not (oka and okb):
        return None
    ax, ay, bx, by = int(ax), int(ay), int(bx), int(by)
    m = int(math.ceil(margin_m / grid.res))
    x0, x1 = max(0, min(ax, bx) - m), min(grid.n - 1, max(ax, bx) + m)
    y0, y1 = max(0, min(ay, by) - m), min(grid.n - 1, max(ay, by) + m)
    sub = cost[x0:x1 + 1, y0:y1 + 1].astype(np.int32)
    sub = np.where(sub < 0, unknown_cost, sub)
    if start is not None:
        sx, sy = grid.center(np.arange(x0, x1 + 1)[:, None], np.arange(y0, y1 + 1)[None, :])
        sub[np.hypot(sx - start[0], sy - start[1]) <= ESCAPE_M] = 0
    blocked = sub >= BLOCK_COST
    s, g = (ax - x0, ay - y0), (bx - x0, by - y0)
    blocked[s] = blocked[g] = False
    steps = [(dx, dy, math.hypot(dx, dy)) for dx in (-1, 0, 1) for dy in (-1, 0, 1) if dx or dy]
    best = {s: 0.0}
    came = {}
    heap = [(0.0, s)]
    while heap:
        _, cur = heapq.heappop(heap)
        if cur == g:
            break
        gc = best[cur]
        for dx, dy, dl in steps:
            nx, ny = cur[0] + dx, cur[1] + dy
            if nx < 0 or ny < 0 or nx >= sub.shape[0] or ny >= sub.shape[1] or blocked[nx, ny]:
                continue
            ng = gc + dl * (1.0 + sub[nx, ny] / 25.0)
            if ng < best.get((nx, ny), math.inf):
                best[(nx, ny)] = ng
                came[(nx, ny)] = cur
                heapq.heappush(heap, (ng + math.hypot(g[0] - nx, g[1] - ny), (nx, ny)))
    if g not in best:
        return None
    cells = [g]
    while cells[-1] != s:
        cells.append(came[cells[-1]])
    cells.reverse()
    return [tuple(grid.center(x0 + cx, y0 + cy)) for cx, cy in cells]


def string_pull(cost, grid, pts, unknown_cost=mg.UNKNOWN_COST, start=None):
    """Drop path points while the straight line stays clear."""
    if len(pts) <= 2:
        return list(pts)
    out = [pts[0]]
    i = 0
    while i < len(pts) - 1:
        j = len(pts) - 1
        while j > i + 1 and not line_free(cost, grid, pts[i], pts[j], unknown_cost, start):
            j -= 1
        out.append(pts[j])
        i = j
    return out


def _lane_segments(grid, cost, need, center, u, v, off, length, swath, unknown_cost):
    """Needed, unblocked stretches of one lane as (start_xy, end_xy, n_needed)."""
    s = np.arange(-length / 2.0, length / 2.0 + 1e-9, SAMPLE_M)
    line = np.asarray(center, float)[None, :] + s[:, None] * u + off * v
    free = _costs(cost, grid, line, unknown_cost) < BLOCK_COST
    want = np.zeros(len(s), bool)
    for side in (-0.5 * swath, 0.0, 0.5 * swath):
        pts = line + side * v
        ix, iy, ok = grid.index(pts[:, 0], pts[:, 1])
        w = np.zeros(len(s), bool)
        w[ok] = need[ix[ok], iy[ok]]
        want |= w
    want &= free
    segs = []
    i = 0
    while i < len(s):
        if not free[i]:
            i += 1
            continue
        j = i
        while j + 1 < len(s) and free[j + 1]:
            j += 1
        idx = np.nonzero(want[i:j + 1])[0]
        if len(idx):
            a, b = i + idx[0], i + idx[-1]
            segs.append((tuple(line[a]), tuple(line[b]), int(len(idx))))
        i = j + 1
    return segs


def plan(grid, cost, center, width, length, heading_deg=0.0, swath=3.0, z=mg.CRUISE_Z,
         start=None, target="bed", unknown_cost=mg.UNKNOWN_COST):
    """Waypoints (x, y, z) covering what the map still needs inside the rectangle."""
    if target not in TARGETS:
        raise ValueError(f"target must be one of {TARGETS}")
    u, v = lane_frame(heading_deg)
    n_lanes = max(1, int(math.ceil(width / swath)))
    offsets = (np.arange(n_lanes) + 0.5) * (width / n_lanes) - width / 2.0
    need = needed(grid, target)
    lanes = []
    for off in offsets:
        segs = _lane_segments(grid, cost, need, center, u, v, off, length, swath, unknown_cost)
        if segs:
            lanes.append(segs)
    if start is not None and lanes:
        first = np.asarray(lanes[0][0][0])
        last = np.asarray(lanes[-1][0][0])
        if np.linalg.norm(last - start[:2]) < np.linalg.norm(first - start[:2]):
            lanes.reverse()
    ordered = []
    for i, segs in enumerate(lanes):
        if i % 2:
            segs = [(b, a, n) for a, b, n in reversed(segs)]
        ordered += segs
    wps, cur = [], (tuple(start[:2]) if start is not None else None)
    skipped = length_m = 0.0
    n_need = 0
    for a, b, n in ordered:
        if cur is not None:
            if line_free(cost, grid, cur, a, unknown_cost, start):
                hop = [cur, a]
            else:
                path = astar(cost, grid, cur, a, unknown_cost, start=start)
                if path is None:
                    skipped += 1
                    continue
                hop = string_pull(cost, grid, [cur] + path[1:-1] + [a], unknown_cost, start)
            for p in hop[1:]:
                length_m += math.dist(wps[-1][:2] if wps else cur, p)
                wps.append((float(p[0]), float(p[1]), float(z)))
        else:
            wps.append((float(a[0]), float(a[1]), float(z)))
        length_m += math.dist(a, b)
        wps.append((float(b[0]), float(b[1]), float(z)))
        cur = b
        n_need += n
    return {
        "waypoints": wps,
        "length_m": round(length_m, 1),
        "lanes": len(lanes),
        "segments": len(ordered) - int(skipped),
        "skipped": int(skipped),
        "need_samples": n_need,
        "target": target,
    }
