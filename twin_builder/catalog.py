"""Fixed VLM prop catalog: class -> SDF primitive defaults. Cap ~40 props per world."""

PROP_CLASSES = {
    "building": {"shape": "box", "default_size_m": [12.0, 10.0, 8.0], "color": "0.55 0.55 0.5 1"},
    "tree": {"shape": "cylinder", "default_size_m": [1.2, 1.2, 6.0], "color": "0.15 0.45 0.18 1"},
    "car": {"shape": "box", "default_size_m": [4.5, 1.9, 1.5], "color": "0.2 0.2 0.25 1"},
    "fence": {"shape": "box", "default_size_m": [8.0, 0.15, 1.2], "color": "0.45 0.35 0.25 1"},
    "water": {"shape": "box", "default_size_m": [20.0, 20.0, 0.4], "color": "0.15 0.35 0.55 0.7"},
}

MAX_PROPS = 40
SCHEMA_KEYS = ("class", "lat", "lon", "yaw_deg", "size_m")
