"""Write capped SDF model includes from a validated VLM scene graph (JSON only)."""
from __future__ import annotations

import math
from pathlib import Path

from .catalog import MAX_PROPS, PROP_CLASSES


def _enu_offset_m(origin_lat, origin_lon, lat, lon):
    """Local ENU metres from origin to (lat, lon)."""
    dy = math.radians(lat - origin_lat) * 6371000.0
    dx = math.radians(lon - origin_lon) * 6371000.0 * math.cos(math.radians(origin_lat))
    return dx, dy


def validate_props(raw):
    """Keep only catalog classes with finite lat/lon; cap MAX_PROPS."""
    out = []
    if not isinstance(raw, list):
        return out
    for item in raw:
        if not isinstance(item, dict):
            continue
        cls = item.get("class")
        if cls not in PROP_CLASSES:
            continue
        lat, lon = item.get("lat"), item.get("lon")
        if not (isinstance(lat, (int, float)) and isinstance(lon, (int, float))
                and -90 <= lat <= 90 and -180 <= lon <= 180):
            continue
        yaw = item.get("yaw_deg", 0.0)
        if not isinstance(yaw, (int, float)):
            yaw = 0.0
        size = item.get("size_m")
        defaults = PROP_CLASSES[cls]["default_size_m"]
        if isinstance(size, (list, tuple)) and len(size) >= 3:
            try:
                size_m = [float(size[0]), float(size[1]), float(size[2])]
            except (TypeError, ValueError):
                size_m = list(defaults)
        else:
            size_m = list(defaults)
        # Hard caps so a bad VLM cannot flood the world with kilometre boxes
        size_m = [max(0.2, min(80.0, s)) for s in size_m]
        out.append({
            "class": cls,
            "lat": float(lat),
            "lon": float(lon),
            "yaw_deg": float(yaw) % 360.0,
            "size_m": size_m,
        })
        if len(out) >= MAX_PROPS:
            break
    return out


def prop_model_sdf(name, prop, x, y, z=0.0):
    spec = PROP_CLASSES[prop["class"]]
    sx, sy, sz = prop["size_m"]
    yaw = math.radians(prop["yaw_deg"])
    color = spec["color"]
    if spec["shape"] == "cylinder":
        # size_m = [diameter, diameter, height]
        geom = f"<cylinder><radius>{sx / 2:.3f}</radius><length>{sz:.3f}</length></cylinder>"
        z_off = sz / 2.0
    else:
        geom = f"<box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box>"
        z_off = sz / 2.0
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {z + z_off:.3f} 0 0 {yaw:.4f}</pose>
      <link name="link">
        <collision name="collision"><geometry>{geom}</geometry></collision>
        <visual name="visual">
          <geometry>{geom}</geometry>
          <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
        </visual>
      </link>
    </model>
"""


def write_props_include(path, props, origin_lat, origin_lon, origin_elev=0.0):
    """Write an SDF snippet of static models; returns the list written."""
    clean = validate_props(props)
    chunks = ['<sdf version="1.9"><world name="props">\n']
    for i, p in enumerate(clean):
        x, y = _enu_offset_m(origin_lat, origin_lon, p["lat"], p["lon"])
        chunks.append(prop_model_sdf(f"prop_{p['class']}_{i}", p, x, y, 0.0))
    chunks.append("</world></sdf>\n")
    Path(path).write_text("".join(chunks), encoding="utf-8")
    return clean


def props_xml_fragment(props, origin_lat, origin_lon):
    """SDF model fragments to splice into a world (no outer sdf/world tags)."""
    clean = validate_props(props)
    parts = []
    for i, p in enumerate(clean):
        x, y = _enu_offset_m(origin_lat, origin_lon, p["lat"], p["lon"])
        parts.append(prop_model_sdf(f"prop_{p['class']}_{i}", p, x, y, 0.0))
    return "".join(parts), clean
