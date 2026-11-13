"""Capture DEM and nadir imagery for a look-at bbox.

DEM: Open-Elevation / OpenTopoData public APIs (no lab secrets). Ortho: optional
local JPEG from the pilot client, else a solid placeholder so bake still runs.
"""
from __future__ import annotations

import json
import math
import struct
import urllib.error
import urllib.request
from pathlib import Path

GRID = 129  # Gazebo heightmap odd size


def _get_json(url, timeout=20):
    req = urllib.request.Request(url, headers={"User-Agent": "deepgis-twin-builder/1.0"})
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read().decode("utf-8"))


def sample_dem(bbox, grid=GRID):
    """Return (elevations row-major north->south, min_z, max_z) for the bbox."""
    west, south, east, north = bbox["west"], bbox["south"], bbox["east"], bbox["north"]
    lats = [north - (north - south) * i / (grid - 1) for i in range(grid)]
    lons = [west + (east - west) * j / (grid - 1) for j in range(grid)]
    # Batch Open-Elevation (max ~100 locations per request)
    locations = []
    for lat in lats:
        for lon in lons:
            locations.append(f"{lat:.5f},{lon:.5f}")
    elevs = [0.0] * (grid * grid)
    chunk = 80
    for start in range(0, len(locations), chunk):
        batch = locations[start:start + chunk]
        url = "https://api.open-elevation.com/api/v1/lookup?locations=" + "|".join(batch)
        try:
            data = _get_json(url)
            results = data.get("results") or []
            for i, row in enumerate(results):
                elevs[start + i] = float(row.get("elevation") or 0.0)
        except (urllib.error.URLError, urllib.error.HTTPError, TimeoutError, ValueError, KeyError):
            # Flat fallback at mean of any prior samples, else 0
            known = [e for e in elevs[:start] if e]
            fill = sum(known) / len(known) if known else 0.0
            for i in range(start, min(start + chunk, len(elevs))):
                elevs[i] = fill
    return elevs, min(elevs), max(elevs)


def write_heightmap_png(path, elevs, grid=GRID):
    """16-bit grayscale PNG heightmap (L16) without Pillow — raw zlib PNG."""
    import zlib

    mn, mx = min(elevs), max(elevs)
    span = max(mx - mn, 1.0)
    rows = []
    for i in range(grid):
        row = bytearray([0])  # filter none
        for j in range(grid):
            v = int(round((elevs[i * grid + j] - mn) / span * 65535.0))
            v = max(0, min(65535, v))
            row += struct.pack(">H", v)
        rows.append(bytes(row))
    raw = b"".join(rows)
    compressed = zlib.compress(raw, 9)

    def chunk(tag, data):
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xffffffff)

    ihdr = struct.pack(">IIBBBBB", grid, grid, 16, 0, 0, 0, 0)  # 16-bit grayscale
    png = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", ihdr) + chunk(b"IDAT", compressed) + chunk(b"IEND", b"")
    Path(path).write_bytes(png)
    return {"min_z": mn, "max_z": mx, "size": grid, "path": str(path)}


def write_placeholder_ortho(path, size=512):
    """Solid mid-tone JPEG-less PNG RGB placeholder when no nadir crop is supplied."""
    import zlib

    row = bytearray([0]) + bytes([90, 110, 80]) * size
    raw = bytes(row) * size
    compressed = zlib.compress(raw, 6)

    def chunk(tag, data):
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xffffffff)

    ihdr = struct.pack(">IIBBBBB", size, size, 8, 2, 0, 0, 0)
    png = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", ihdr) + chunk(b"IDAT", compressed) + chunk(b"IEND", b"")
    Path(path).write_bytes(png)
    return str(path)


def capture_site(job_dir, look_at, bbox, nadir_jpeg=None):
    """Write heightmap.png, ortho.png, nadir.jpg (or png), meta.json into job_dir."""
    job_dir = Path(job_dir)
    job_dir.mkdir(parents=True, exist_ok=True)
    elevs, mn, mx = sample_dem(bbox)
    hm = write_heightmap_png(job_dir / "heightmap.png", elevs)
    ortho = write_placeholder_ortho(job_dir / "ortho.png")
    nadir_path = job_dir / "nadir.jpg"
    if nadir_jpeg and Path(nadir_jpeg).is_file():
        nadir_path.write_bytes(Path(nadir_jpeg).read_bytes())
    else:
        # Copy ortho as stand-in for VLM input
        nadir_path = job_dir / "nadir.png"
        nadir_path.write_bytes(Path(ortho).read_bytes())
    west, south, east, north = bbox["west"], bbox["south"], bbox["east"], bbox["north"]
    size_x = max(1.0, (east - west) * 111320.0 * max(0.05, abs(math.cos(math.radians(look_at["lat"])))))
    size_y = max(1.0, (north - south) * 111320.0)
    meta = {
        "look_at": look_at,
        "bbox": bbox,
        "heightmap": hm,
        "ortho": ortho,
        "nadir": str(nadir_path),
        "size_x_m": size_x,
        "size_y_m": size_y,
        "origin_elev_m": look_at.get("elev_m", (mn + mx) / 2.0),
    }
    (job_dir / "capture.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
    return meta
