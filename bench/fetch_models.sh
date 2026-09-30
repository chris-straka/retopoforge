#!/usr/bin/env bash
# Download benchmark test models (not committed to the repo).
# Source: https://github.com/alecjacobson/common-3d-test-models
set -euo pipefail
cd "$(dirname "$0")/models"
BASE="https://raw.githubusercontent.com/alecjacobson/common-3d-test-models/master/data"
for model in armadillo.obj beast.obj nefertiti.obj fandisk.obj; do
    if [ -f "$model" ]; then
        echo "exists: $model"
        continue
    fi
    echo "fetch: $model"
    curl -sSL -o "$model" "$BASE/$model" || echo "FAILED: $model (skipped)"
done
ls -la
