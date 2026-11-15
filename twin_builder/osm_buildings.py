"""OpenStreetMap building footprints for a twin patch.

Queries the public Overpass API for ways tagged building inside the bbox and
turns each footprint into a static box the Gazebo world can splice in. No
shell commands, and a network failure returns an empty list so the rest of
the bake still runs.
"""
from __future__ import annotations

import json
import math
import os
import urllib.error
import urllib.parse
import urllib.request

_DEFAULT_ENDPOINTS = (
    "https://overpass-api.de/api/interpreter",
    "https://overpass.kumi.systems/api/interpreter",
    "https://overpass.openstreetmap.fr/api/interpreter",
)
MAX_BUILDINGS = 40
DEFAULT_HEIGHT_M = 8.0
LEVEL_HEIGHT_M = 3.0


def bbox_from_job(job):
    """Return west/south/east/north. Prefer the page bbox, else the patch size."""
    raw = job.get("bbox") if isinstance(job, dict) else None
    if isinstance(raw, dict):
        try:
            box = {k: float(raw[k]) for k in ("west", "south", "east", "north")}
        except (KeyError, TypeError, ValueError):
            box = None
        else:
            if box["west"] < box["east"] and box["south"] < box["north"]:
                return box
    look = (job or {}).get("look_at") or {}
    try:
        lat = float(look["lat"])
        lon = float(look["lon"])
    except (KeyError, TypeError, ValueError):
        return None
    width = _metres(job.get("width_m"), 150.0)
    length = _metres(job.get("length_m"), 150.0)
    dlat = (length / 2.0) / 111320.0
    cos = max(0.05, abs(math.cos(math.radians(lat))))
    dlon = (width / 2.0) / (111320.0 * cos)
    return {"west": lon - dlon, "south": lat - dlat, "east": lon + dlon, "north": lat + dlat}


def buildings_in_job(job):
    box = bbox_from_job(job)
    if not box:
        return []
    return buildings_in_bbox(box)


def merge_buildings(props, osm_buildings):
    """OSM footprints replace vision-model buildings. Other classes stay."""
    if not osm_buildings:
        return list(props or [])
    others = [p for p in (props or []) if not (isinstance(p, dict) and p.get("class") == "building")]
    return list(osm_buildings) + others


def buildings_in_bbox(box):
    query = (
        "[out:json][timeout:25];"
        f'way["building"]({box["south"]},{box["west"]},{box["north"]},{box["east"]});'
        "out geom;"
    )
    try:
        payload = _post(query)
    except (urllib.error.URLError, urllib.error.HTTPError, TimeoutError, ValueError, OSError):
        return []
    elements = payload.get("elements") if isinstance(payload, dict) else None
    if not isinstance(elements, list):
        return []
    found = []
    for el in elements:
        if not isinstance(el, dict) or el.get("type") != "way":
            continue
        prop = _way_to_prop(el, box)
        if prop:
            found.append(prop)
    found.sort(key=lambda p: p["size_m"][0] * p["size_m"][1], reverse=True)
    return found[:MAX_BUILDINGS]


def _way_to_prop(element, box):
    geom = element.get("geometry")
    if not isinstance(geom, list) or len(geom) < 4:
        return None
    nodes = []
    for pt in geom:
        if not isinstance(pt, dict):
            continue
        try:
            lat, lon = float(pt["lat"]), float(pt["lon"])
        except (KeyError, TypeError, ValueError):
            continue
        nodes.append((lat, lon))
    if len(nodes) >= 2 and nodes[0] == nodes[-1]:
        nodes = nodes[:-1]
    if len(nodes) < 3:
        return None
    fitted = _minimum_box(nodes)
    if not fitted:
        return None
    lat, lon, length, width, yaw_deg = fitted
    if not (box["south"] <= lat <= box["north"] and box["west"] <= lon <= box["east"]):
        return None
    if length < 2.0 or width < 2.0:
        return None
    tags = element.get("tags") if isinstance(element.get("tags"), dict) else {}
    height = _height_m(tags)
    return {
        "class": "building",
        "lat": lat,
        "lon": lon,
        "yaw_deg": yaw_deg,
        "size_m": [length, width, height],
        "source": "osm",
        "osm_id": element.get("id"),
    }


def _minimum_box(nodes):
    """Axis of one footprint edge, the smallest area rectangle, in ENU."""
    lat0 = sum(p[0] for p in nodes) / len(nodes)
    lon0 = sum(p[1] for p in nodes) / len(nodes)
    pts = [_enu(lat0, lon0, lat, lon) for lat, lon in nodes]
    best = None
    for i, (x1, y1) in enumerate(pts):
        x2, y2 = pts[(i + 1) % len(pts)]
        if math.hypot(x2 - x1, y2 - y1) < 0.4:
            continue
        ang = math.atan2(y2 - y1, x2 - x1)
        c, s = math.cos(-ang), math.sin(-ang)
        rot = [(c * x - s * y, s * x + c * y) for x, y in pts]
        xs = [p[0] for p in rot]
        ys = [p[1] for p in rot]
        minx, maxx = min(xs), max(xs)
        miny, maxy = min(ys), max(ys)
        length = maxx - minx
        width = maxy - miny
        area = length * width
        # Center of the rectangle, rotated back into east/north metres.
        cx, cy = (minx + maxx) / 2.0, (miny + maxy) / 2.0
        east = math.cos(ang) * cx - math.sin(ang) * cy
        north = math.sin(ang) * cx + math.cos(ang) * cy
        if best is None or area < best[0]:
            best = (area, length, width, ang, east, north)
    if best is None:
        return None
    _area, length, width, ang, east, north = best
    lat = lat0 + math.degrees(north / 6371000.0)
    lon = lon0 + math.degrees(east / (6371000.0 * math.cos(math.radians(lat0))))
    return lat, lon, length, width, math.degrees(ang) % 360.0


def _enu(lat0, lon0, lat, lon):
    north = math.radians(lat - lat0) * 6371000.0
    east = math.radians(lon - lon0) * 6371000.0 * math.cos(math.radians(lat0))
    return east, north


def _height_m(tags):
    raw = str(tags.get("height") or "").lower().strip()
    feet = raw.endswith("ft") or raw.endswith("feet") or raw.endswith("'")
    raw = raw.replace("feet", "").replace("ft", "").replace("m", "").replace("'", "").strip()
    try:
        val = float(raw)
        if val > 0:
            return val * 0.3048 if feet else val
    except ValueError:
        pass
    try:
        levels = float(tags.get("building:levels"))
        if levels > 0:
            return levels * LEVEL_HEIGHT_M
    except (TypeError, ValueError):
        pass
    return DEFAULT_HEIGHT_M


def _metres(value, default):
    try:
        val = float(value)
    except (TypeError, ValueError):
        return default
    if val <= 0:
        return default
    return val


def _endpoints():
    custom = os.environ.get("DEEPGIS_TWIN_OVERPASS_URL", "").strip()
    if custom:
        return (custom,)
    return _DEFAULT_ENDPOINTS


def _post(query):
    data = urllib.parse.urlencode({"data": query}).encode("utf-8")
    last = None
    for url in _endpoints():
        req = urllib.request.Request(
            url,
            data=data,
            headers={
                "Content-Type": "application/x-www-form-urlencoded",
                "Accept": "application/json",
                "User-Agent": "deepgis-twin-builder/1.0 (DeepGIS digital twin bake)",
            },
            method="POST",
        )
        try:
            with urllib.request.urlopen(req, timeout=30) as resp:
                return json.loads(resp.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            last = exc
            if exc.code not in (406, 429, 502, 503, 504):
                raise
        except (urllib.error.URLError, TimeoutError, OSError, ValueError) as exc:
            last = exc
    if last:
        raise last
    raise OSError("no Overpass endpoint")
