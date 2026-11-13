"""Nadir image -> JSON scene graph via dreams_mcp describe_image (schema-only).

Falls back to an empty prop list when MCP is unset; never runs shell from the LLM.
"""
from __future__ import annotations

import base64
import json
import os
import urllib.error
import urllib.request
from pathlib import Path

from .catalog import MAX_PROPS, PROP_CLASSES, SCHEMA_KEYS

VLM_SCHEMA = {
    "type": "object",
    "properties": {
        "objects": {
            "type": "array",
            "maxItems": MAX_PROPS,
            "items": {
                "type": "object",
                "properties": {
                    "class": {"enum": list(PROP_CLASSES)},
                    "lat": {"type": "number"},
                    "lon": {"type": "number"},
                    "yaw_deg": {"type": "number"},
                    "size_m": {
                        "type": "array",
                        "items": {"type": "number"},
                        "minItems": 3,
                        "maxItems": 3,
                    },
                },
                "required": ["class", "lat", "lon"],
            },
        },
    },
    "required": ["objects"],
}

SYSTEM = (
    "You label a nadir (top-down) aerial image for a Gazebo digital twin. "
    "Return JSON only with an objects array. Each object: class one of "
    f"{sorted(PROP_CLASSES)}, lat, lon (WGS84 degrees of the object centre), "
    "optional yaw_deg (degrees clockwise from north), optional size_m [length, width, height] in metres. "
    f"At most {MAX_PROPS} objects. Prefer clear buildings, trees, cars, fences, water. "
    "Do not invent objects you cannot see. No prose."
)


def _mcp_describe(image_b64, look_at, bbox):
    url = os.environ.get("DEEPGIS_TWIN_VLM_MCP_URL") or os.environ.get("DEEPGIS_PILOT_MCP_URL")
    token = os.environ.get("DEEPGIS_TWIN_VLM_MCP_TOKEN") or os.environ.get("DEEPGIS_PILOT_MCP_TOKEN")
    if not url or not token:
        return []
    prompt = (
        f"Nadir patch around lat {look_at['lat']:.5f}, lon {look_at['lon']:.5f}; "
        f"bbox west={bbox['west']:.5f} south={bbox['south']:.5f} "
        f"east={bbox['east']:.5f} north={bbox['north']:.5f}. "
        "List visible ground objects for the twin."
    )
    body = {
        "jsonrpc": "2.0",
        "id": 1,
        "method": "tools/call",
        "params": {
            "name": "describe_image",
            "arguments": {
                "image_b64": image_b64,
                "prompt": prompt,
                "system_prompt": SYSTEM,
                "json_schema": VLM_SCHEMA,
            },
        },
    }
    req = urllib.request.Request(
        url,
        data=json.dumps(body).encode("utf-8"),
        headers={
            "Content-Type": "application/json",
            "Authorization": f"Bearer {token}",
            "User-Agent": "deepgis-twin-builder/1.0",
        },
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=120) as resp:
        payload = json.loads(resp.read().decode("utf-8"))
    # dreams_mcp may nest under result.content or result.json
    result = payload.get("result") if isinstance(payload, dict) else None
    if not isinstance(result, dict):
        return []
    parsed = result.get("json")
    if isinstance(parsed, str):
        try:
            parsed = json.loads(parsed)
        except ValueError:
            parsed = None
    if isinstance(parsed, dict) and isinstance(parsed.get("objects"), list):
        return parsed["objects"]
    return []


def detect_props(nadir_path, look_at, bbox):
    """Return a list of prop dicts (may be empty)."""
    path = Path(nadir_path) if nadir_path else None
    if not path or not path.is_file():
        return []
    try:
        image_b64 = base64.b64encode(path.read_bytes()).decode("ascii")
        raw = _mcp_describe(image_b64, look_at, bbox)
    except (urllib.error.URLError, urllib.error.HTTPError, TimeoutError, ValueError, OSError):
        return []
    # Drop unknown keys early; sdf_props.validate_props does the rest
    out = []
    for item in raw if isinstance(raw, list) else []:
        if not isinstance(item, dict):
            continue
        out.append({k: item[k] for k in SCHEMA_KEYS if k in item})
    return out
