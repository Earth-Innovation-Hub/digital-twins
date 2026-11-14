#!/bin/bash
# Launch AprilTag stack on xudrone's nadir camera. Installed as coral_tags_xudrone.sh.
set -e
if [ -x /usr/local/bin/coral_tags.sh ]; then
  exec /usr/local/bin/coral_tags.sh xudrone
fi
ROOT="/root/udrone_boat/behaviours"
if [ ! -f "$ROOT/run_tags.sh" ]; then
  ROOT="$(cd "$(dirname "$0")" && pwd)"
fi
exec "$ROOT/run_tags.sh" xudrone
