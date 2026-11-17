"""Fill the kestrel site_template world with captured DEM/ortho and VLM props."""
from __future__ import annotations

import json
import shutil
from pathlib import Path

from . import capture, rasters, sdf_props, vlm_props

TEMPLATE_DIR = Path(__file__).resolve().parents[1] / "kestrel" / "worlds" / "site_template"


def _fill_template(text, mapping):
    out = text
    for key, val in mapping.items():
        out = out.replace("{{" + key + "}}", str(val))
    return out


def bake_job(job, job_root=None, nadir_jpeg=None, run_vlm=True):
    """
    job: dict with id, callsign, look_at, bbox, radius_m, vehicle_class.
    Returns artifact dict with world_sdf path and props.
    """
    root = Path(job_root or Path.cwd() / "var" / "twin-jobs" / job["id"])
    root.mkdir(parents=True, exist_ok=True)
    look = job["look_at"]
    bbox = job["bbox"]
    meta = capture.capture_site(root / "capture", look, bbox, nadir_jpeg=nadir_jpeg)

    props = []
    if run_vlm:
        props = vlm_props.detect_props(meta.get("nadir"), look, bbox)
    props_xml, props = sdf_props.props_xml_fragment(props, look["lat"], look["lon"])

    world_dir = root / "world"
    if world_dir.exists():
        shutil.rmtree(world_dir)
    shutil.copytree(TEMPLATE_DIR, world_dir)
    # Copy heightmap + ortho into world textures
    tex = world_dir / "textures"
    tex.mkdir(exist_ok=True)
    hm_src = Path(meta["heightmap"]["path"])
    shutil.copy2(hm_src, tex / "heightmap.png")
    shutil.copy2(meta["ortho"], tex / "ortho.png")

    size_x = meta["size_x_m"]
    size_y = meta["size_y_m"]
    mn = meta["heightmap"]["min_z"]
    mx = meta["heightmap"]["max_z"]
    elev = look.get("elev_m", meta["origin_elev_m"])
    mapping = {
        "WORLD_NAME": job["callsign"],
        "LAT": f"{look['lat']:.8f}",
        "LON": f"{look['lon']:.8f}",
        "ELEV": f"{elev:.3f}",
        "SIZE_X": f"{size_x:.1f}",
        "SIZE_Y": f"{size_y:.1f}",
        "HEIGHT_MIN": f"{mn:.3f}",
        "HEIGHT_MAX": f"{mx:.3f}",
        "PROPS": props_xml,
    }
    template = (world_dir / "site.sdf.in").read_text(encoding="utf-8")
    world_sdf = world_dir / "site.sdf"
    world_sdf.write_text(_fill_template(template, mapping), encoding="utf-8")

    artifact = {
        "job_dir": str(root),
        "world_sdf": str(world_sdf),
        "callsign": job["callsign"],
        "look_at": look,
        "bbox": bbox,
        "radius_m": job.get("radius_m"),
        "airframe": "x500_gimbal",
        "props": props,
        "capture": meta,
    }
    (root / "job.json").write_text(json.dumps(artifact, indent=2, default=str), encoding="utf-8")
    return artifact


def bake_client_dem(job, job_root=None, nadir_jpeg=None, body="earth"):
    """Heightmap from the terrain grid the page sampled.

    Heights are metres relative to the look-at, so the center of the patch sits
    near z = 0. The Moon starts the x500 at a fixed 15 m. Earth starts it above
    the highest cell so a ridge does not contain the vehicle. Open-Elevation is
    not consulted: the grid is already the terrain drawn on the globe.
    """
    dem = capture.clean_dem(job.get("dem"))
    root = Path(job_root or Path.cwd() / "var" / "twin-jobs" / job["id"])
    root.mkdir(parents=True, exist_ok=True)
    info = capture.write_heightmap_png(
        root / "heightmap.png", dem["heights"], grid=dem["grid"], min_span=0.05)
    lunar = body == "moon"
    artifact = {
        "job_dir": str(root),
        "world_sdf": "",
        "callsign": job["callsign"],
        "look_at": job["look_at"],
        "bbox": job.get("bbox") or {},
        "body": "moon" if lunar else "earth",
        "width_m": float(job["width_m"]),
        "length_m": float(job["length_m"]),
        "airframe": "x500_gimbal",
        "props": [],
        "heightmap_image": info["path"],
        "heightmap_z_min": info["min_z"],
        "heightmap_span": info["span"],
        "heightmap_mount": "/root/moon_heightmap.png" if lunar else "/root/earth_heightmap.png",
        "spawn_z": 15.0 if lunar else max(3.0, float(info["max_z"]) + 2.0),
        "capture": {"grid": dem["grid"], "min_z": info["min_z"], "max_z": info["max_z"]},
    }
    if nadir_jpeg and Path(nadir_jpeg).is_file():
        artifact["nadir"] = str(nadir_jpeg)
    if not lunar:
        # The photo drape needs this file. The depth PNG is the mesh either way.
        artifact["normal_image"] = rasters.write_normal_map(
            root / "earth_normal.png", dem["heights"], dem["grid"],
            float(job["width_m"]), float(job["length_m"]))
        artifact["normal_mount"] = "/root/earth_normal.png"
    (root / "job.json").write_text(json.dumps(artifact, indent=2, default=str), encoding="utf-8")
    return artifact


def bake_moon(job, job_root=None, nadir_jpeg=None):
    """Heightmap of a lunar look-at. The grid comes from the Moon viewer's terrain.

    OpenStreetMap and Earth imagery are not used.
    """
    return bake_client_dem(job, job_root=job_root, nadir_jpeg=nadir_jpeg, body="moon")


def bake_earth(job, job_root=None, nadir_jpeg=None):
    """Heightmap of an Earth look-at. The grid is Cesium World Terrain, including 3DEP."""
    return bake_client_dem(job, job_root=job_root, nadir_jpeg=nadir_jpeg, body="earth")


# Alias used by deepgis-xr stub import
bake = bake_job
