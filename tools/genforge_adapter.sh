#!/bin/bash
# genforge adapter entry (repair-topology): runs tools/genforge_adapter.py
# in headless Blender with the extension from this checkout and `retopo`
# ($RETOPO_BIN, this checkout's release build, then PATH). Contract and exit codes are documented there:
#   tools/genforge_adapter.sh repair-topology IN.glb OUT.glb RESULT.json [--class humanoid|quadruped|custom]
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BLENDER="${BLENDER_BIN:-/Applications/Blender.app/Contents/MacOS/Blender}"

if [ ! -x "$BLENDER" ]; then
    echo "genforge_adapter.sh: Blender not found at $BLENDER (set BLENDER_BIN)" >&2
    exit 2
fi

exec "$BLENDER" --background --factory-startup --python-exit-code 2 \
    --python "$ROOT/tools/genforge_adapter.py" -- "$@"
