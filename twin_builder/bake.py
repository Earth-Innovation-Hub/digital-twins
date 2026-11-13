"""Fill the kestrel site_template world with captured DEM/ortho and VLM props."""
from __future__ import annotations

import json
import shutil
from pathlib import Path

from . import capture, sdf_props, vlm_props

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


# Alias used by deepgis-xr stub import
bake = bake_job
