"""Satellite image for the Gazebo ground plane of a Cesium patch.

One public World Imagery export covers the patch. North is up in the file.
Gazebo's plane puts texture v=0 on +Y, which is north, so the file is used
as downloaded. A network failure leaves the stock gray plane.
"""
from __future__ import annotations

import math
import re
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path

from .osm_buildings import bbox_from_job

IMAGERY_URL = "https://services.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/export"
# Inside the session container. The host file is mounted here.
CONTAINER_PATH = "/root/ground.jpg"
IMAGE_PX = 1024
# World Imagery rejects an export finer than about 0.3 m per pixel.
MIN_METRES_PER_PIXEL = 0.6


def fetch_ground(job, dest):
    """Download the patch image. Returns width/length in metres, or None."""
    box = bbox_from_job(job)
    look = (job or {}).get("look_at") or {}
    try:
        lat = float(look.get("lat"))
    except (TypeError, ValueError):
        return None
    if not box:
        return None
    width, length = _size_m(box, lat)
    if width < 5.0 or length < 5.0 or width > 500.0 or length > 500.0:
        return None
    dest = Path(dest)
    px, request_box, frac = _request(box, width, length)
    try:
        raw = _download(request_box, px)
    except (urllib.error.URLError, urllib.error.HTTPError, TimeoutError, ValueError, OSError):
        return None
    if frac < 0.99:
        try:
            raw = _crop_center(raw, frac)
        except (OSError, ValueError):
            return None
    if len(raw) < 1000 or not raw.startswith(b"\xff\xd8"):
        return None
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_bytes(raw)
    return {"path": str(dest), "width_m": width, "length_m": length}


def apply_ground(text, width_m, length_m, image_path=CONTAINER_PATH):
    """Size the ground visual to the patch and use the satellite image.

    The collision plane stays infinite. The model pose moves from the image's
    z=-2 offset up to the origin, which is where the drone and the buildings are.
    """
    if not image_path or width_m <= 0 or length_m <= 0:
        return text
    match = re.search(r'(<model name="ground_plane">)(.*?)(</model>)', text, re.S)
    if not match:
        return text
    body = match.group(2)

    def _visual(vm):
        vis = vm.group(0)
        vis = re.sub(
            r"(<plane>\s*<normal>[^<]*</normal>\s*<size>)[^<]+(</size>)",
            rf"\g<1>{width_m:.2f} {length_m:.2f}\2",
            vis,
            count=1,
            flags=re.S,
        )
        material = (
            "<material>\n"
            "            <ambient>1 1 1 1</ambient>\n"
            "            <diffuse>1 1 1 1</diffuse>\n"
            "            <specular>0 0 0 1</specular>\n"
            "            <pbr><metal>\n"
            f"              <albedo_map>{image_path}</albedo_map>\n"
            "              <roughness>1</roughness>\n"
            "              <metalness>0</metalness>\n"
            "            </metal></pbr>\n"
            "          </material>"
        )
        if "<material>" in vis:
            vis = re.sub(r"<material>.*?</material>", material, vis, count=1, flags=re.S)
        return vis

    body = re.sub(r'<visual name="visual">.*?</visual>', _visual, body, count=1, flags=re.S)
    body = re.sub(
        r"(</link>\s*<pose>)[^<]+(</pose>)",
        r"\g<1>0 0 0 0 0 0\2",
        body,
        count=1,
    )
    return text[:match.start()] + match.group(1) + body + match.group(3) + text[match.end():]


def _size_m(box, lat):
    cos = max(0.05, abs(math.cos(math.radians(lat))))
    width = (box["east"] - box["west"]) * 111320.0 * cos
    length = (box["north"] - box["south"]) * 111320.0
    return width, length


def _request(box, width, length):
    """Pixel count at a scale the imagery service will export, and a center crop."""
    span = max(width, length)
    px = max(64, min(IMAGE_PX, int(round(span / MIN_METRES_PER_PIXEL))))
    covered = px * MIN_METRES_PER_PIXEL
    if covered <= span * 1.02:
        return px, box, 1.0
    need = covered / span
    dlat = (box["north"] - box["south"]) * (need - 1.0) / 2.0
    dlon = (box["east"] - box["west"]) * (need - 1.0) / 2.0
    wider = {
        "west": box["west"] - dlon,
        "south": box["south"] - dlat,
        "east": box["east"] + dlon,
        "north": box["north"] + dlat,
    }
    return px, wider, 1.0 / need


def _crop_center(raw, frac):
    import io
    from PIL import Image
    image = Image.open(io.BytesIO(raw)).convert("RGB")
    width, height = image.size
    crop_w = max(1, int(round(width * frac)))
    crop_h = max(1, int(round(height * frac)))
    left = (width - crop_w) // 2
    top = (height - crop_h) // 2
    image = image.crop((left, top, left + crop_w, top + crop_h))
    if max(image.size) < 512:
        scale = 512 / max(image.size)
        image = image.resize(
            (max(1, int(image.width * scale)), max(1, int(image.height * scale))),
            Image.Resampling.LANCZOS,
        )
    buf = io.BytesIO()
    image.save(buf, format="JPEG", quality=85)
    return buf.getvalue()


def _mercator(lon, lat):
    x = lon * 20037508.34 / 180.0
    lat = max(-85.0, min(85.0, lat))
    y = math.log(math.tan(math.radians(90.0 + lat) / 2.0)) * 20037508.34 / math.pi
    return x, y


def _download(box, pixels):
    west, south = _mercator(box["west"], box["south"])
    east, north = _mercator(box["east"], box["north"])
    query = urllib.parse.urlencode({
        "bbox": f"{west},{south},{east},{north}",
        "bboxSR": "3857",
        "imageSR": "3857",
        "size": f"{pixels},{pixels}",
        "format": "jpg",
        "f": "image",
    })
    req = urllib.request.Request(
        IMAGERY_URL + "?" + query,
        headers={
            "Accept": "image/jpeg",
            "User-Agent": "deepgis-twin-builder/1.0 (DeepGIS digital twin ground)",
        },
    )
    with urllib.request.urlopen(req, timeout=25) as resp:
        data = resp.read(8_000_000)
    return data
