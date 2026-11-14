# CoRAL behaviour library

Typed command → dock / boat goto / survey / helix / coverage / approach_tag → event.
Runs inside `digital-twin-aquatic` on `/coral/{command,event,status,detections,tracks}`.

| runner | command | role |
|---|---|---|
| (in-process) | `dock`, `goto`, `survey`, `hold`, `abort` | boat + latch |
| `run_helix.py` | `helix` | AUV forward helix |
| `run_coverage.py` | `coverage` | AUV 3D lawnmower (`patterns.volume_lawnmower`) |
| `run_approach_tag.py` | `approach_tag` | AUV PBVS to AprilTag ENU / tracks |
| `run_tags.sh` | (launch) | apriltag_ros + `tag_detector` for otter_k / xudrone |

Helix, coverage, and approach_tag share `UdroneHelixController` (force →
point-hull-axis); only `pos_des(t)` / finish logic changes. See the pack
[README](../README.md#behaviour-library-coral) for schema, screenshots, and CLI.

LLM path (tesseract): `coral_bridge` + `coral_instructions` map operator /
`instruction` text onto these commands (e.g. “short survey until tag, then go to
the tag”).
