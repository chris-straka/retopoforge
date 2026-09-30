#!/usr/bin/env bash
# Download benchmark test models (not committed to the repo).
# Source: https://github.com/alecjacobson/common-3d-test-models
#
# License notes (models are gitignored downloads, never shipped with the app):
# - armadillo.obj, xyzrgb_dragon.obj: Stanford 3D Scanning Repository
#   (https://graphics.stanford.edu/data/3Dscanrep/). Terms: free for
#   research use and free redistribution; commercial use / inclusion in a
#   product for sale requires Stanford's permission. Bench evaluation of
#   this open-source project is non-commercial use; models are fetched at
#   test time and never distributed with any build.
# - beast.obj: Autodesk sample (via the mirror above).
# - nefertiti.obj: Berlin Egyptian Museum scan via thing:3974391.
# - fandisk.obj: Pratt & Whitney/Hughes Hoppe CAD part.
set -euo pipefail
mkdir -p "$(dirname "$0")/models"
cd "$(dirname "$0")/models"
BASE="https://raw.githubusercontent.com/alecjacobson/common-3d-test-models/master/data"
for model in armadillo.obj beast.obj nefertiti.obj fandisk.obj xyzrgb_dragon.obj; do
    if [ -f "$model" ]; then
        echo "exists: $model"
        continue
    fi
    echo "fetch: $model"
    curl -sSL -o "$model" "$BASE/$model" || echo "FAILED: $model (skipped)"
done
ls -la
