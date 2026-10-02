# Deformation test (spec; shared gate for retopoforge, rigforge weights, wrapforge)

Static scores (`bench/score.py`) judge a mesh standing still. Game
characters bend, so the main gate for topology and weights work is how a
posed mesh holds up at its joints.

## Procedure (headless Blender, deterministic)

1. Input: a remeshed (or wrapped) mesh plus a rig. For procedural tests,
   generate both: a tube limb with 2 bones (elbow), a biped mannequin
   with rigforge `hll_hero`, a quadruped with `hll_stalker`.
2. Skin with a fixed method (Blender automatic weights for retopoforge
   comparisons, so only topology varies; the method under test when
   testing weights).
3. Apply a fixed pose set: elbow 90 and 135 degrees, knee 90 and 120,
   shoulder raised 90, neck turned 60, spine bent 30, jaw open 20 (when
   the rig has one). Store poses as data files in the repo.
4. Measure per joint region (vertices with weight > 0.1 for the joint's
   two bones):
   - volume loss: signed volume of the region's closed slice, posed vs
     rest (%),
   - stretch: per-face max singular value of the deformation gradient
     (posed vs rest); report p95 and % of faces > 2x,
   - flips: faces whose normal reverses vs rest (must be 0),
   - self-intersection count near the joint (optional, BVH overlap).
5. Output JSON per joint and a summary line; same inputs give
   byte-identical output.

## Use

- `bench/deform.py` runs it over a mesh list, like `bench/noise.py`;
  compare distributions across seeds, not single runs.
- Gate wording for agents: "change X must not worsen p95 stretch or
  volume loss at any joint beyond the seed spread, and must keep flips
  at 0".

## Implementation in retopoforge (v1, `bench/deform.py`)

Stdlib only (no Blender, no numpy, no rigforge), so the gate runs in CI
and same inputs give byte-identical JSON on any platform (floats rounded
to 6 dp, sorted keys, deterministic iteration; verified by re-run `cmp`
and `bench/test_deform.py`).

**Rig (auto-fit).** Each output mesh gets a deterministic 2-bone hinge
rig: the bone axis is the longest bbox axis (ties toward x, then y),
the joint pivot is the bbox centroid, the hinge is the second-longest
bbox axis. For arbitrary corpus meshes this is a synthetic bend probe,
not an anatomical joint; anatomical joints arrive with rigged inputs.

**Procedural tube.** `tube` is a generated limb (capped cylinder along
Y, length 8, radius 0.5, 64 x 24), remeshed like the corpus models. Its
proportions keep the 135-degree bend feasible (bend radius ~1.8x the
tube radius at the smoothstep peak), so clean topology holds 0 flips.

**Skinning (fixed method).** Analytic smoothstep weights across the
joint plane (half-band 0.2 x bone length), deformed with
dual-quaternion skinning (Kavan et al. 2007). DQS, not linear blend:
LBS necks the joint to cos(angle/2) of its radius (candy-wrapper),
which inverts even perfect topology at 135 degrees. Blender automatic
weights remain a possible future comparison lane; DQS matches Blender's
preserve-volume option in spirit.

**Poses.** The fixed set lives in `bench/poses/poses.json` (elbow 90 /
135, knee 90 / 120, shoulder 90, neck 60, spine 30, jaw 20, as above).
A rig runs the poses whose joint it has; the v1 hinge rig has only the
elbow, so corpus runs score elbow90/elbow135. Angles are flexion from
straight about the hinge through the pivot.

**Joint region.** Vertices with both bone weights above 0.1; faces
fully inside that vertex set, fan-split into triangles for measurement.

**Metrics** (per joint region, per pose; quads fan-split):

- stretch: per-triangle max singular value of the rest->posed
  deformation gradient (closed-form 3x3 symmetric eigensolver);
  report p95 (same index rule as `bench/score.py`) and % of tris > 2x.
- volume loss: enclosed volume of the region band plus centroid-fan
  caps over its boundary loops (the spec's "closed slice"), posed vs
  rest (%). True enclosed volume, so rigid motion preserves it
  exactly; sign follows input winding, the ratio does not.
- flips: triangles whose posed normal reverses against the
  bone-transported rest normal (rest normal rotated by the DQS blend
  at the triangle's mean weight). Transport matters: a world-space
  normal test flags clean rigid rotation past 90 degrees, and det(F)
  is the area ratio by construction (always positive), so neither can
  signal inversion at these bend angles.
- self-intersections: intersecting non-adjacent region-triangle pairs
  in the posed mesh (uniform grid + exact-predicate segment tests;
  grid agrees with brute force on the tube). Informational only;
  rest-pose count recorded alongside for context.

**Gate rule.** One uniform rule per case per pose: p95 stretch, volume
loss, and flip count must not exceed the baseline seed-spread max
(+1% relative epsilon on stretch, +0.1 pp on volume). Where the spread
max is 0 (feasible bends: the tube everywhere, plus some corpus
poses), the flip gate is the spec's absolute "must be 0". Folding a
bloblike corpus mesh 135 degrees inverts by construction, so there the
stable spread-gated count is the regression signal. Recorded but never
gating: stretch % > 2x, self-intersections, region sizes.

**Wiring.** `bench/run.py --check` scores deform in-process for every
case (including the procedural `tube`, whose counts are recorded but
ungated) against `deform_<check-basename>.json` next to the `--check`
file (`--deform-baseline` overrides, `--no-deform` skips). Focused
use: `bench/deform.py --check bench/deform_baseline.json` (tube and
corpus, seed 0), `bench/deform.py --seeds 8` for distributions,
`bench/deform.py --mesh out.obj` to score one file (handy for backend
comparisons), `bench/deform.py --write-baseline` to (re)generate.

**Baselines are per-platform**, like `bench/baseline.json` (never
mix). `bench/deform_baseline.json` ships for macOS; Linux CI warns
that deform goes uncompared until its baseline exists:

```bash
bench/deform.py --write-baseline bench/deform_baseline-linux.json
```

**Measured scores** (macOS arm64, 8-seed calibration; seed0 / spread
max; the gate bound is the max column):

| case | pose | p95 seed0 / max | vol% seed0 / max | flips seed0 / max | region tris |
|---|---|---|---|---|---|
| armadillo.obj@1000 | elbow90 | 1.216 / 1.251 | 44.13 / 79.98 | 4 / 20 | 89 |
| armadillo.obj@1000 | elbow135 | 1.317 / 1.394 | 70.68 / 124.66 | 33 / 40 | 89 |
| armadillo.obj@5000 | elbow90 | 1.200 / 1.218 | 47.65 / 47.73 | 79 / 92 | 656 |
| armadillo.obj@5000 | elbow135 | 1.310 / 1.350 | 73.57 / 73.67 | 213 / 235 | 656 |
| beast.obj@1000 | elbow90 | 1.866 / 2.136 | 37.79 / 47.86 | 25 / 47 | 536 |
| beast.obj@1000 | elbow135 | 2.396 / 2.755 | 58.94 / 74.56 | 78 / 107 | 536 |
| beast.obj@5000 | elbow90 | 1.725 / 1.729 | 41.60 / 41.69 | 301 / 301 | 4540 |
| beast.obj@5000 | elbow135 | 2.171 / 2.181 | 64.16 / 64.31 | 717 / 717 | 4540 |
| fandisk.obj@1000 | elbow90 | 2.297 / 2.370 | 33.65 / 35.27 | 191 / 259 | 1093 |
| fandisk.obj@1000 | elbow135 | 3.018 / 3.130 | 52.29 / 54.97 | 263 / 321 | 1093 |
| fandisk.obj@5000 | elbow90 | 2.401 / 2.443 | 33.01 / 33.63 | 899 / 913 | 4372 |
| fandisk.obj@5000 | elbow135 | 3.189 / 3.281 | 51.34 / 52.22 | 1215 / 1220 | 4372 |
| nefertiti.obj@1000 | elbow90 | 1.710 / 1.807 | 5.08 / 5.08 | 0 / 0 | 352 |
| nefertiti.obj@1000 | elbow135 | 2.116 / 2.281 | 9.32 / 9.32 | 45 / 45 | 352 |
| nefertiti.obj@5000 | elbow90 | 1.836 / 1.842 | 1.11 / 1.66 | 2 / 5 | 1914 |
| nefertiti.obj@5000 | elbow135 | 2.329 / 2.340 | 1.99 / 2.79 | 205 / 253 | 1914 |
| tube@1000 | elbow90 | 1.359 / 1.365 | -0.88 / 1.16 | 0 / 0 | 228 |
| tube@1000 | elbow135 | 1.566 / 1.576 | -0.18 / 2.56 | 0 / 0 | 228 |
| tube@5000 | elbow90 | 1.371 / 1.371 | 0.30 / 0.34 | 0 / 0 | 792 |
| tube@5000 | elbow135 | 1.590 / 1.593 | 0.71 / 0.80 | 0 / 0 | 792 |
| xyzrgb_dragon.obj@1000 | elbow90 | 1.630 / 1.892 | -17.12 / -5.31 | 0 / 0 | 120 |
| xyzrgb_dragon.obj@1000 | elbow135 | 2.001 / 2.314 | -23.73 / -4.80 | 0 / 1 | 120 |
| xyzrgb_dragon.obj@5000 | elbow90 | 1.999 / 2.133 | -22.52 / -22.30 | 0 / 1 | 1321 |
| xyzrgb_dragon.obj@5000 | elbow135 | 2.505 / 2.739 | -33.76 / -33.48 | 0 / 1 | 1321 |

Notes: spreads are tight at 5000 (p95 within ~1-2%, volume within
~0.5 pp), so the gate has teeth there. beast@1000 volume swings wide
across seeds (min -103.69 / max 74.56 at elbow135, seed0 58.94): the
known appendage knife-edge moves the slice itself, and the spread
honestly covers it. Negative volume "loss" (dragon, tube@1000) is
slice expansion under the fold, stable across seeds. 96/96
calibration runs scored; no empty regions, no degenerate volumes.

**Future work.** Blender-weights comparison lane (skin the same
remeshes with Blender automatic weights to isolate topology vs
skinning effects); rigforge mannequin rigs (`hll_hero`, `hll_stalker`)
to run the knee/shoulder/neck/spine/jaw poses on limbs instead of the
synthetic whole-mesh fold.
