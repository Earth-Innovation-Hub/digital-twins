"""Bake a Gazebo site world from DEM + nadir imagery and spawn a session twin.

Used by the dreamslab twin-builder worker (and optionally imported by deepgis-xr
stub jobs). Never mutates protected production containers (locutus, otter, aquatic).
"""

__all__ = ["bake", "capture", "vlm_props", "spawn", "catalog"]
