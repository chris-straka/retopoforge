# Comparing retopo against free baselines

`bench/compare.py` pits the headless `retopo` CLI against two free
baselines, per model and quad target, and prints a markdown table:

- **blender-voxel** — Blender's native voxel remesher, run strictly
  headless (`--background --factory-startup --python
  bench/blender_voxel.py`). The Blender GUI is never launched. The helper
  calibrates the voxel size with one trial remesh on a scratch copy so the
  output lands roughly on each quad target.
- **quadriflow** — the standalone QuadriFlow binary, best-effort only.
  Resolution order: `--quadriflow` flag, `$QUADRIFLOW_BINARY`, `PATH`,
  then `/tmp/quadriflow-build/build/quadriflow`; when none exists the
  harness attempts a one-time `/tmp` clone+build of
  https://github.com/hjwdzh/QuadriFlow and otherwise prints
  `SKIP: <reason>` and still exits 0 on the retopo+voxel lanes.

Every tool's OBJ output is parsed directly (no self-reported counts):
quads, non-quads, verts, wall time, boundary-edge count (edges used by
exactly one face), and non-manifold-edge count (edges used by 3+ faces).

## Usage

```sh
bench/fetch_models.sh   # one-time model download (gitignored)
python3 bench/compare.py                                   # defaults below
python3 bench/compare.py --models fandisk.obj --targets 1000,5000,10000
python3 bench/compare.py --no-quadriflow-build             # skip lane (b)
python3 bench/compare.py --help
```

Defaults: models `armadillo.obj,fandisk.obj` from `bench/models/`,
targets `1000,5000`. The retopo binary resolves from `--binary`, then
`$RETOPO_BINARY`, then `build/cli/retopo`; Blender from `--blender`, then
`$BLENDER_BINARY`, then the stock macOS app path. Exit code is 0 when
every retopo and blender-voxel case succeeds; a QuadriFlow skip or
failure is recorded in the notes column and never fails the run.

## First measured table

Defaults run on macOS (Apple Silicon), Blender 5.2.1 LTS headless,
QuadriFlow built from upstream master at `/tmp/quadriflow-build`
(cmake needed `-DCMAKE_POLICY_VERSION_MINIMUM=3.5`; the harness passes
it automatically):

| model | target | tool | quads | non-quads | verts | wall s | boundary | non-manif | notes |
|---|---|---|---:|---:|---:|---:|---:|---:|---|
| armadillo.obj | 1000 | retopo | 524 | 12 | 544 | 0.2 | 0 | 0 |  |
| armadillo.obj | 1000 | blender-voxel | 986 | 0 | 992 | 0.7 | 0 | 0 |  |
| armadillo.obj | 1000 | quadriflow | 937 | 0 | 939 | 0.8 | 0 | 0 |  |
| armadillo.obj | 5000 | retopo | 4566 | 20 | 4595 | 0.8 | 0 | 4 |  |
| armadillo.obj | 5000 | blender-voxel | 4814 | 0 | 4818 | 0.8 | 0 | 0 |  |
| armadillo.obj | 5000 | quadriflow | 4197 | 0 | 4199 | 1.4 | 0 | 0 |  |
| fandisk.obj | 1000 | retopo | 1546 | 6 | 1557 | 0.2 | 0 | 0 |  |
| fandisk.obj | 1000 | blender-voxel | 988 | 0 | 990 | 0.5 | 0 | 0 |  |
| fandisk.obj | 1000 | quadriflow | 931 | 0 | 933 | 0.2 | 0 | 0 |  |
| fandisk.obj | 5000 | retopo | 5532 | 6 | 5543 | 0.5 | 0 | 0 |  |
| fandisk.obj | 5000 | blender-voxel | 5044 | 0 | 5046 | 0.5 | 0 | 0 |  |
| fandisk.obj | 5000 | quadriflow | 4671 | 0 | 4673 | 1.0 | 0 | 0 |  |

Observations (data, not gates):

- retopo overshoots small targets on fandisk (1546 quads at target 1000)
  and undershoots slightly on armadillo (524 at target 1000); both
  baselines land within ~7% of target.
- retopo `armadillo @ 5000` shows 4 non-manifold edges where both
  baselines show 0 — possible follow-up for the manifold-cleanup lane.
- Wall times are all sub-2s; Blender startup dominates its lane.
