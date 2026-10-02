# Beast@1000-native knife-edge: stage bisection

Date: 2026-10-01. Question: where, downstream of the native decimator
(`RETOPO_DECIMATOR=native`), does the beast@1000 failure first appear?
Method: `RETOPO_DUMP_STAGES=dir` probe (working mesh, cross field,
singularities, rounded+continuous uvs, extraction output + uvs) dumped
for default vs native on identical input, plus an old-path seed-1 control
and 8-seed spreads for the key counts.

## Result

The left appendage (x < -62, 416 working tris, incl. the worst-fit point
~(-133,147,81)) gets **zero** output quads under native (old emits 103
verts there). The output stays a perfect closed 2-manifold (all 1301
edges used exactly twice; no boundary, no pinches, no degenerate quads):
the appendage's uv region folds **100% inside** the body's uv region (old:
53%), so the extractor traces it once, everything maps to body working
tris, and the appendage silently vanishes. Distortion p90 goes 7 -> 76;
18% of source samples sit >10 units from the mesh (old: 5%).

First localized defect: **stage 3 (uv layout)** — degenerate layout over
the appendage (15 near-zero-area uv tris near the failure point vs 4 in
old; 28% of all degenerate uvs at x < -62 vs 8%; appendage uv footprint
fully contained in the body's). Root driver one stage up: **stage 2
singularity detection lands systematically high** (120-122 singularities
on all 8 native seeds vs 98-117 spread on old seeds — no overlap), while
the cross field itself shows no native-specific shift (17° vs 15.6°
seed-control disagreement: pre-existing chaos, not signal). Stage 1
(working mesh) is clean (<=2.3 units near the failure point). Rounding is
a no-op on this case (rounded uvs bit-identical to continuous), so the
integer layout *is* the folded layout.

Compact chain: native decimation (systematically different but
statically valid tiling; 99.7%+ seed-stable, so 6/8 seeds give
near-identical stage stats) -> chaotic field lands in a fixed basin ->
singularity thresholding keeps 120-122 cones (old jitters over 98-117)
-> cover solve degenerates around the extra cones (93 near-zero-area uv
tris vs <=62; compressed footprints) -> appendage uv region folds fully
inside body uv -> extractor emits nothing for 416 working tris -> closed
mesh missing the part (dist 30 vs 7).

## Untangling verdict

Untangling-as-prototyped (`exp/igm-validity`, Garanzha flip barrier over
the continuous cover with fixed integers) is the **wrong defect class**
for this failure and would not fix it: the energy is a sum of per-face
terms that penalize local inversion/distortion, but the appendage uvs are
locally valid — merely globally overlapped — so the layout already sits
near a local minimum, and fixed integers pin the fold routing anyway. The
barrier would expand degenerate tris in place, not unfold the region out
of the body's footprint. Untangling stays valuable for its own target
(local flips, per `docs/igm-validity-spike.md`) but is orthogonal here.

## Fix directions (not this bisection)

- Singularity control: dipole/spurious-cone cleanup or detection
  thresholds, so the count stops landing systematically high.
- Integer-layout feasibility: keep the appendage uv region from folding
  fully inside the body's (the routing decision that deletes the part).
- Extraction robustness: detect working regions that emitted nothing and
  either emit there or report instead of sealing silently.

## Repro

```sh
RETOPO_DUMP_STAGES=/tmp/s_old retopo --input beast.obj --output o.obj --target-quads 1000
RETOPO_DECIMATOR=native RETOPO_DUMP_STAGES=/tmp/s_new retopo --input beast.obj --output n.obj --target-quads 1000
```

Then compare stage dumps old vs new (`/tmp/stage_bisect.py`,
`/tmp/stage_followup*.py` were the throwaway comparisons; kept in /tmp,
not committed). The repo keeps the `RETOPO_DUMP_STAGES` research probe
for item-7 work.
