# twin_builder

Dreamslab worker that turns a Cesium look-at + bbox into a Gazebo x500 twin.

## Flow

1. **capture** — DEM grid → 16-bit heightmap; ortho/nadir image
2. **vlm_props** — nadir → JSON scene graph (catalog classes only, ≤40)
3. **bake** — fill `kestrel/worlds/site_template` SDF
4. **spawn** — `digital-twin-session-<callsign>` (never locutus/otter/aquatic)
5. **enroll** — pose-agent env for DeepGIS chase

## Run (dreamslab)

```bash
export DEEPGIS_TWIN_BUILDER_TOKEN=...   # shared with deepgis-xr
export DEEPGIS_TWIN_JOBS_DIR=/var/twin-jobs
export DEEPGIS_TWIN_SESSIONS_DIR=/var/twin-sessions
export DEEPGIS_TWIN_SPAWN_DRY_RUN=1     # bake+meta only until image is wired
python -m twin_builder.worker
```

DeepGIS posts to `DEEPGIS_TWIN_BUILDER_URL` with the same bearer token. Without
that URL, deepgis-xr runs an in-process stub that still writes a minimal world.

## Safety

- Auth required on every request
- Protected names refused
- One active job per user (enforced in deepgis-xr)
- Idle TTL + disk quota on session dirs
- VLM output is JSON schema → SDF writer only (no shell)
