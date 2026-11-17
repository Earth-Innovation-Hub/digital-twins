"""Rasters the Earth session world can load.

The depth grid is a 16-bit grayscale PNG. Gazebo uses that as the mesh, so
the mesh already has the slope. The satellite photo is an 8-bit RGB PNG
draped on that mesh. The SDF texture element also requires a normal-map
file; an omitted one is filled with ``__default__`` and the world is
rejected. The normal written here is the grid's own surface, so a flat
patch is straight up (128, 128, 255) and a slope leans with the depth.
"""
from __future__ import annotations

from pathlib import Path


def satellite_png(src, dest):
    """Esri export JPEG as an 8-bit RGB PNG. Empty string when it cannot be read."""
    from PIL import Image

    src, dest = Path(src), Path(dest)
    if not src.is_file():
        return ""
    try:
        with Image.open(src) as image:
            rgb = image.convert("RGB")
            dest.parent.mkdir(parents=True, exist_ok=True)
            rgb.save(dest, "PNG")
    except (OSError, ValueError):
        return ""
    return str(dest) if dest.is_file() and dest.stat().st_size > 24 else ""


def write_normal_map(path, heights, grid, width_m, length_m):
    """RGB normal map aligned with the depth grid. Row 0 is north, column 0 is west."""
    from PIL import Image

    grid = int(grid)
    step_e = float(width_m) / (grid - 1)
    step_n = float(length_m) / (grid - 1)
    image = Image.new("RGB", (grid, grid))
    pixels = image.load()

    def height(row, col):
        row = min(grid - 1, max(0, row))
        col = min(grid - 1, max(0, col))
        return float(heights[row * grid + col])

    for row in range(grid):
        for col in range(grid):
            dz_east = (height(row, col + 1) - height(row, col - 1)) / (2.0 * step_e)
            dz_north = (height(row - 1, col) - height(row + 1, col)) / (2.0 * step_n)
            nx, ny, nz = -dz_east, -dz_north, 1.0
            mag = (nx * nx + ny * ny + nz * nz) ** 0.5 or 1.0
            pixels[col, row] = (
                int(round((nx / mag * 0.5 + 0.5) * 255)),
                int(round((ny / mag * 0.5 + 0.5) * 255)),
                int(round((nz / mag * 0.5 + 0.5) * 255)),
            )
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    image.save(path, "PNG")
    return str(path)
