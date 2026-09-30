# retopoforge performance profile

Date: 2026-09-30. Sources: `bench/profile.py` (stdlib only),
`core/autoremesher.cpp` (phase report), `cli/main.cpp` (no thread flag).

## How to reproduce

```bash
bench/fetch_models.sh   # one-time download (gitignored)
bench/profile.py                               # dragon, 50k quads, 3 runs
bench/profile.py --repeat 1 --json /tmp/p.json # single run + JSON detail
```

The profiler runs the built `build/cli/retopo` on one mesh, measures
wall time and peak child RSS, and parses the engine's phase report off
stderr. Exit code is 0 on success, 1 when a run fails, 2 on bad usage.

## Setup

- Machine: Apple M4, 10 logical CPUs, 16 GiB RAM, macOS 27.0.1.
- Build: Release, Ninja + Homebrew LLVM toolchain.
- Input: `bench/models/xyzrgb_dragon.obj` — 125,066 verts,
  249,882 triangles, 1 island (Stanford scan, production-size hero mesh).
- Settings: `--target-quads 50000` (the CLI default), everything else
  default. Mesh simplifier SKIPPED (no island above 8x target count).

## Headline numbers (median of 3 runs)

| Metric | Value |
|---|---|
| Wall time | 22.7 s (runs: 23.5 / 22.7 / 22.3 s) |
| Peak RSS | 1,502.5 MiB (max across runs) |
| Output | 47,979 quads + 126 non-quads, 48,146 verts |
| Cores kept busy | 1.00 of 10 (engine's own figure) |

Output counts are bit-stable across runs (identical all three times).

## Phase breakdown (run 1; Total 23,332.7 ms)

Coarse phases first — quad extraction dominates:

| Phase | ms | Share |
|---|---|---|
| Quad extract (accumulated) | 16,074.2 | 69% |
| Parameterize (accumulated) | 5,528.8 | 24% |
| Isotropic remesh (accumulated) | 1,631.0 | 7% |
| Split into islands | 54.9 | <1% |
| Merge islands | 5.5 | <1% |
| Everything else (voxel, contexts, adaptive field) | 31.5 | <1% |

Leaf stages, grouped by phase (accumulated; single island, so these
equal wall time within the phase):

Isotropic remesh:

| Stage | ms |
|---|---|
| Projecting vertices | 821.0 |
| Flipping edges | 179.8 |
| Collapsing short edges | 177.9 |
| Shifting vertices | 120.7 |
| Splitting long edges | 50.6 |
| Building bounding volume tree | 49.8 |
| Computing scaling field | 13.4 |
| Computing vertex normals | 0.9 |

Parameterize:

| Stage | ms |
|---|---|
| Simplifying singularities | 3,548.2 |
| Correcting field curl | 481.9 |
| Rounding cover to integers | 470.5 |
| Solving frame field | 424.1 |
| Eliminating cover constraints | 323.2 |
| Building cover system | 81.4 |
| Computing anisotropy field | 55.2 |
| Computing corner rotations | 49.9 |
| Initializing cover field | 24.5 |
| Building surface topology | 22.5 |
| Smoothing cross field | 21.3 |
| Building cover uvs | 5.2 |

Quad extract:

| Stage | ms |
|---|---|
| Merging shared five edge faces | 5,478.7 |
| Holding singular lines | 3,368.8 |
| Cleaning up triangles | 2,820.6 |
| Collapsing three valence diagonals | 1,750.7 |
| Extracting connections | 364.2 |
| Extracting mesh | 330.8 |
| Extracting edges | 248.5 |
| Switching high valence edges | 212.4 |
| Collapsing three valence edge pairs | 208.4 |
| Merging double shared edge quads | 203.2 |
| Converting triangle and five edge fans | 192.7 |
| Smoothing and projecting | 167.0 |
| Removing non-manifold faces | 160.1 |
| Merging three and five valence triangles | 157.7 |
| Collapsing three valence corners | 151.5 |
| Splitting high valence triangle fans | 126.6 |
| Splitting seven edge faces | 37.0 |
| Splitting six edge faces | 34.8 |
| Fixing holes | 15.8 |
| Collecting singularities | 2.7 |

## Top bottlenecks

1. **Merging shared five edge faces — 5.5 s (24% of total).** The
   single biggest leaf stage, inside quad extraction.
2. **Simplifying singularities — 3.5 s (15%).** Biggest parameterize
   leaf (738 → 261 singularities after 238 pair cancellations).
3. **Holding singular lines — 3.4 s (14%).** Biggest extract-side
   singularity cost.
4. **Cleaning up triangles — 2.8 s (12%).**
5. **Collapsing three valence diagonals — 1.8 s (8%).**

The top five leaves are 69% of the run; all five are serial,
single-island work.

## Threading

The engine exposes **no thread-count knob**: no CLI flag, no TBB
`global_control`, and `TBB_NUM_THREADS=1` leaves the runtime unchanged
(verified: 1.55 s vs 1.66 s on the 5k-quad dragon, within noise).
Islands are the unit of parallelism (`tbb::parallel_for` over islands
in `AutoRemesher::remesh`, plus TBB loops inside the per-island
stages), so a single-island hero mesh keeps ~1 core busy no matter how
many the machine has. Multi-island inputs (scans, kitbash sets) scale
with island count; per-island wall time is bounded by the leaf stages
above.

Practical consequences:

- Batch mode (`--input dir/`) still remeshes files serially — run one
  `retopo` per core (e.g. `xargs -P`) to saturate a workstation.
- Peak RSS (~1.5 GiB for a 250k-triangle input at 50k quads) is the
  per-process budget to plan parallel batches against.
