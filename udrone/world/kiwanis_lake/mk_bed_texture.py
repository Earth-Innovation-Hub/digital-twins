#!/usr/bin/env python3
"""Build lake-bed + water albedo textures and planar-UV OBJs for Kiwanis.

Writes into this model folder:
  materials/textures/bed_sand.png
  materials/textures/water_ripples.png
  meshes/bed.obj     (planar ENU UVs, ~8 m per tile)
  meshes/water.obj   (planar ENU UVs, ~12 m per tile)

  python3 mk_bed_texture.py
"""
from __future__ import annotations

import math
import struct
import zlib
from pathlib import Path

ROOT = Path(__file__).resolve().parent
BED_TILE_M = 8.0
WATER_TILE_M = 12.0
TEX = 512


def _hash2(ix: int, iy: int) -> float:
    n = (ix * 374761393 + iy * 668265263) & 0xFFFFFFFF
    n = (n ^ (n >> 13)) * 1274126177 & 0xFFFFFFFF
    return (n & 0xFFFFFF) / 0xFFFFFF


def _smooth(t: float) -> float:
    return t * t * (3.0 - 2.0 * t)


def value_noise(x: float, y: float) -> float:
    x0, y0 = math.floor(x), math.floor(y)
    fx, fy = x - x0, y - y0
    u, v = _smooth(fx), _smooth(fy)
    a = _hash2(x0, y0)
    b = _hash2(x0 + 1, y0)
    c = _hash2(x0, y0 + 1)
    d = _hash2(x0 + 1, y0 + 1)
    return a + (b - a) * u + (c - a) * v + (a - b - c + d) * u * v


def fbm(x: float, y: float, octaves: int = 5) -> float:
    amp, freq, total, norm = 1.0, 1.0, 0.0, 0.0
    for _ in range(octaves):
        total += amp * value_noise(x * freq, y * freq)
        norm += amp
        amp *= 0.5
        freq *= 2.0
    return total / norm


def make_bed_texture(path: Path, n: int = TEX) -> None:
    pixels = bytearray()
    for j in range(n):
        for i in range(n):
            u, v = i / n, j / n
            base = fbm(u * 4.0, v * 4.0, 4)
            streak = fbm(u * 18.0 + 3.1, v * 6.0 + 1.7, 3)
            grain = fbm(u * 64.0, v * 64.0, 2)
            t = 0.55 * base + 0.30 * streak + 0.15 * grain
            r = int(58 + 95 * t)
            g = int(48 + 72 * t)
            b = int(28 + 42 * t)
            if grain > 0.78 and base < 0.45:
                r, g, b = int(r * 0.65), int(g * 0.70), int(b * 0.75)
            pixels.extend((max(0, min(255, r)), max(0, min(255, g)), max(0, min(255, b))))
    _write_png(path, n, n, pixels)
    print(f"wrote {path} ({n}x{n})")


def make_water_texture(path: Path, n: int = TEX) -> None:
    """Subtle blue-green ripple / caustic tint for a semi-transparent surface."""
    pixels = bytearray()
    for j in range(n):
        for i in range(n):
            u, v = i / n, j / n
            swell = fbm(u * 3.0, v * 3.0, 3)
            rip = fbm(u * 14.0 + 1.3, v * 14.0 + 2.1, 4)
            fine = fbm(u * 40.0, v * 40.0, 2)
            wave = 0.55 * swell + 0.35 * rip + 0.10 * fine
            # Darker mid-tones; transparency still shows the bed through.
            r = int(10 + 18 * wave)
            g = int(38 + 32 * wave)
            b = int(52 + 42 * wave)
            pixels.extend((max(0, min(255, r)), max(0, min(255, g)), max(0, min(255, b))))
    _write_png(path, n, n, pixels)
    print(f"wrote {path} ({n}x{n})")


def _write_png(path: Path, w: int, h: int, rgb: bytes) -> None:
    def chunk(tag: bytes, data: bytes) -> bytes:
        return (struct.pack(">I", len(data)) + tag + data
                + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))

    raw = bytearray()
    row = w * 3
    for y in range(h):
        raw.append(0)
        raw.extend(rgb[y * row:(y + 1) * row])
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n")
        f.write(chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)))
        f.write(chunk(b"IDAT", zlib.compress(bytes(raw), 9)))
        f.write(chunk(b"IEND", b""))


def uv_map_obj(src: Path, dst: Path, tile_m: float, label: str) -> None:
    verts, norms, faces = [], [], []
    with open(src, encoding="utf-8") as f:
        for line in f:
            if line.startswith("v "):
                _, x, y, z = line.split()
                verts.append((float(x), float(y), float(z)))
            elif line.startswith("vn "):
                _, x, y, z = line.split()
                norms.append((float(x), float(y), float(z)))
            elif line.startswith("f "):
                face = []
                for p in line.split()[1:]:
                    bits = p.split("/")
                    vi = int(bits[0]) - 1
                    ni = int(bits[2]) - 1 if len(bits) > 2 and bits[2] else vi
                    face.append((vi, ni))
                faces.append(face)

    if not norms:
        norms = [(0.0, 0.0, 1.0)] * len(verts)

    uvs = [(x / tile_m, y / tile_m) for x, y, _ in verts]
    lines = [f"# {label} (planar UVs, tile {tile_m:.1f} m)", f"o {label}"]
    for x, y, z in verts:
        lines.append(f"v {x:.5f} {y:.5f} {z:.5f}")
    for u, v in uvs:
        lines.append(f"vt {u:.6f} {v:.6f}")
    for x, y, z in norms:
        lines.append(f"vn {x:.6f} {y:.6f} {z:.6f}")
    for face in faces:
        bits = [f"{vi + 1}/{vi + 1}/{ni + 1}" for vi, ni in face]
        lines.append("f " + " ".join(bits))
    dst.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"wrote {dst} verts={len(verts)} faces={len(faces)} tile={tile_m}m")


def _ensure_backup(mesh: Path) -> Path:
    bak = mesh.with_suffix(mesh.suffix + ".pre-uv")
    if not bak.exists():
        bak.write_bytes(mesh.read_bytes())
        print(f"backup {bak}")
    return bak


def main():
    import sys
    root = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT
    tex_dir = root / "materials" / "textures"
    make_bed_texture(tex_dir / "bed_sand.png")
    make_water_texture(tex_dir / "water_ripples.png")

    bed = root / "meshes" / "bed.obj"
    water = root / "meshes" / "water.obj"
    uv_map_obj(_ensure_backup(bed), bed, BED_TILE_M, "kiwanis_bed")
    uv_map_obj(_ensure_backup(water), water, WATER_TILE_M, "kiwanis_water")


if __name__ == "__main__":
    main()
