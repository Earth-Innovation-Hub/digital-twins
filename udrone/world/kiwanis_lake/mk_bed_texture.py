#!/usr/bin/env python3
"""Build lake-bed + water albedo textures and planar-UV OBJs for Kiwanis.

The bed tile is a coral reef: sand channels, brain mounds, branching
heads, fans, and plates. Colours stay off the hoop orange gate
(r > 150, r > g+45, r > b+100, g > b+25), so the front camera does not
read the bottom as the ring. The tile repeats every 8 m.

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


def value_noise_wrap(x: float, y: float, period: int) -> float:
    """Value noise that repeats every ``period`` lattice cells."""
    period = max(1, int(period))
    x0, y0 = math.floor(x), math.floor(y)
    fx, fy = x - x0, y - y0
    u, v = _smooth(fx), _smooth(fy)

    def h(ix: int, iy: int) -> float:
        return _hash2(ix % period, iy % period)

    a = h(x0, y0)
    b = h(x0 + 1, y0)
    c = h(x0, y0 + 1)
    d = h(x0 + 1, y0 + 1)
    return a + (b - a) * u + (c - a) * v + (a - b - c + d) * u * v


def fbm_tile(u: float, v: float, octaves: int, base_period: int) -> float:
    """Fractal noise on the unit square, seamless at the edges."""
    amp, freq, total, norm = 1.0, 1.0, 0.0, 0.0
    for _ in range(octaves):
        p = int(base_period * freq)
        total += amp * value_noise_wrap(u * p, v * p, p)
        norm += amp
        amp *= 0.5
        freq *= 2.0
    return total / norm


def _wrap_signed(a: float, b: float) -> float:
    d = a - b
    if d > 0.5:
        d -= 1.0
    elif d < -0.5:
        d += 1.0
    return d


def _hoop_orange(r: int, g: int, b: int) -> bool:
    """Same gate as hoop_geom.orange_ring. The bed must not pass it."""
    return r > 150 and r > g + 45 and r > b + 100 and g > b + 25


# A few heads on an 8 m tile, with open sand between them. The last number
# salts arm counts, angles, and rib spacing so two heads of one kind differ.
# Colours stay off the hoop orange gate.
_COLONIES = (
    (0.22, 0.30, 0.20, "brain", 1.3),
    (0.70, 0.24, 0.15, "branch", 2.7),
    (0.46, 0.68, 0.14, "fan", 4.1),
    (0.84, 0.72, 0.17, "mound", 5.9),
    (0.14, 0.78, 0.13, "plate", 7.2),
    (0.58, 0.90, 0.12, "branch", 8.4),
)


def _mix(a, b, t: float):
    t = max(0.0, min(1.0, t))
    return tuple(int(a[i] + (b[i] - a[i]) * t) for i in range(3))


def _blob_t(du: float, dv: float, radius: float, salt: float):
    """Lumpy disc. Returns 0 at the centre and 1 at the rim, or None outside."""
    wobble = 0.82 + 0.28 * value_noise(du * 14.0 + salt, dv * 14.0 + salt * 0.3)
    dist = math.hypot(du, dv)
    t = dist / (radius * wobble)
    if t >= 1.0:
        return None
    return t


def _unit_hash(a: int, b: int) -> float:
    return _hash2(a, b)


def _branch_mask(du: float, dv: float, radius: float, salt: float) -> float:
    """A few uneven staghorn arms. Not a regular star."""
    n_arms = 3 + int(_unit_hash(int(salt * 10), 1) * 3.0)  # 3..5
    spin = _unit_hash(int(salt * 20), 2) * 2.0 * math.pi
    best = 0.0
    for k in range(n_arms):
        jitter = (_unit_hash(k + 3, int(salt * 30)) - 0.5) * 1.2
        reach = 0.45 + 0.55 * _unit_hash(k + 5, int(salt * 40))
        ang = spin + k * (2.0 * math.pi / n_arms) + jitter
        ax, ay = math.cos(ang), math.sin(ang)
        arm_len = radius * reach
        proj = du * ax + dv * ay
        lat = abs(-du * ay + dv * ax)
        if proj <= 0.0 or proj > arm_len:
            continue
        width = radius * (0.055 + 0.03 * _unit_hash(k, 9)) * (1.0 - 0.4 * proj / arm_len)
        if width > 1e-4 and lat < width:
            best = max(best, 1.0 - lat / width)
        # One fork, on one side, and not on every arm.
        if k % 2 == 0 and proj > arm_len * 0.5:
            sign = -1.0 if _unit_hash(k, int(salt * 7)) < 0.5 else 1.0
            fang = ang + sign * (0.4 + 0.35 * _unit_hash(k, 11))
            fax, fay = math.cos(fang), math.sin(fang)
            ox, oy = ax * arm_len * 0.5, ay * arm_len * 0.5
            pdx, pdy = du - ox, dv - oy
            fp = pdx * fax + pdy * fay
            fl = abs(-pdx * fay + pdy * fax)
            flen = arm_len * (0.25 + 0.2 * _unit_hash(k, 13))
            fw = radius * 0.035
            if 0.0 < fp < flen and fl < fw:
                best = max(best, 1.0 - fl / fw)
    return best


def _fan_mask(du: float, dv: float, radius: float, salt: float) -> float:
    """A rotated oval with a few soft ribs, not a striped disc."""
    rot = _unit_hash(int(salt * 15), 4) * 2.0 * math.pi
    c, s = math.cos(rot), math.sin(rot)
    rx, ry = c * du + s * dv, -s * du + c * dv
    aspect = 0.55 + 0.3 * _unit_hash(int(salt * 8), 6)
    nx = rx / radius
    ny = ry / (radius * aspect)
    r2 = nx * nx + ny * ny
    if r2 >= 1.0:
        return 0.0
    ribs = 5.0 + 4.0 * _unit_hash(int(salt * 12), 8)
    rib = 0.5 + 0.5 * math.cos(ny * ribs)
    return _smooth(1.0 - r2) * (0.72 + 0.28 * rib)


def _colony_paint(kind: str, du: float, dv: float, t: float, cover: float):
    """Colour of one head. ``cover`` is 1 on the coral and 0 at the edge."""
    if kind == "brain":
        field = value_noise(du * 16.0 + 2.0, dv * 16.0)
        ridge = math.exp(-((field - 0.55) ** 2) * 40.0)
        groove = _mix((206, 176, 122), (118, 90, 62), ridge)
        return _mix(groove, (168, 140, 96), 0.15 * t)
    if kind == "branch":
        tip = _smooth(math.hypot(du, dv) / 0.2)
        return _mix((64, 28, 96), (168, 72, 132), tip * cover)
    if kind == "fan":
        return _mix((18, 92, 96), (64, 168, 154), cover)
    if kind == "plate":
        return _mix((40, 96, 58), (78, 140, 72), 0.35 + 0.4 * (1.0 - t))
    return _mix((118, 52, 96), (176, 120, 150), 1.0 - t)


def _cover_at(kind: str, du: float, dv: float, radius: float, salt: float):
    """How much of this colony covers the point, or None."""
    if kind == "branch":
        m = _branch_mask(du, dv, radius, salt)
        if m <= 0.05:
            return None
        return m
    if kind == "fan":
        m = _fan_mask(du, dv, radius, salt)
        if m <= 0.08:
            return None
        return m
    t = _blob_t(du, dv, radius, salt)
    if t is None:
        return None
    return 1.0 - t


def make_bed_texture(path: Path, n: int = TEX) -> None:
    """Seamless reef albedo. Sand between the heads, colonies wrap the tile."""
    pixels = bytearray()
    orange = 0
    for j in range(n):
        v = j / n
        for i in range(n):
            u = i / n
            sand_n = fbm_tile(u, v, 4, 4)
            rip = fbm_tile(u + 0.17, v + 0.11, 3, 8)
            shade = 0.7 * sand_n + 0.3 * rip
            sand = _mix((158, 146, 114), (210, 198, 166), shade)
            rgb = sand
            best = 0.0
            painted = None
            for cu, cv, radius, kind, salt in _COLONIES:
                du = _wrap_signed(u, cu)
                dv = _wrap_signed(v, cv)
                cover = _cover_at(kind, du, dv, radius, salt)
                if cover is None or cover <= best:
                    continue
                best = cover
                dist = math.hypot(du, dv)
                t = min(1.0, dist / max(radius, 1e-3))
                painted = _colony_paint(kind, du, dv, t, cover)
            if painted is not None:
                rgb = _mix(sand, painted, 0.35 + 0.65 * best)
            r, g, b = rgb
            if _hoop_orange(r, g, b):
                # Pull toward the sand so a tip cannot become the hoop tube.
                for _ in range(4):
                    r, g, b = _mix((r, g, b), sand, 0.55)
                    if not _hoop_orange(r, g, b):
                        break
                orange += 1
            pixels.extend((max(0, min(255, r)), max(0, min(255, g)), max(0, min(255, b))))
    _write_png(path, n, n, pixels)
    print(f"wrote {path} ({n}x{n}) orange_pulled={orange}")


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
    make_bed_texture(tex_dir / "bed_sand.png", n=1024)
    make_water_texture(tex_dir / "water_ripples.png")

    bed = root / "meshes" / "bed.obj"
    water = root / "meshes" / "water.obj"
    uv_map_obj(_ensure_backup(bed), bed, BED_TILE_M, "kiwanis_bed")
    uv_map_obj(_ensure_backup(water), water, WATER_TILE_M, "kiwanis_water")


if __name__ == "__main__":
    main()
