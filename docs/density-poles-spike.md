# Density-aware pole placement spike (research)

Date: 2026-09-30. Question: why does strong localized density refinement
saturate (~2x for 4x asks), and what would unlock the full ask?

## Baseline (measured)

Fixture: `tests/fixtures/sphere-pole.obj`, `--target-quads 2000`, density
mask 4.0 on the y>0.5 cap, 1.0 elsewhere (one multiplier per welded input
vertex — 482 lines).

| run      | inside quads | inside mean area | outside mean area |
|----------|-------------:|-----------------:|------------------:|
| no mask  |          390 |         0.007807 |          0.007875 |
| 4x mask  |          760 |         0.004033 |          0.015201 |

Honest linear refinement (inside-vs-inside areas):
sqrt(0.007807/0.004033) = **1.39x for a 4x ask**. (Comparing against the
coarsened outside inflates this to 1.94x; don't.) Budget math caps a 25%
area at ~1.5x linear anyway, so the sphere nearly saturates its cap —
sizing works here and poles are not the binding constraint.

Repro the mask: `python3 -c` over the fixture's `v` lines, `4.0` when
y > 0.5 else `1.0`. Quad areas from the OBJ outputs directly (fan
triangulation); linear refinement = sqrt(outside_area / inside_area).

## Mechanism (measured)

Output valence census (verts with y > 0.5):

| run     | inside valence-3 | total valence-3 | total valence-4 |
|---------|-----------------:|----------------:|----------------:|
| no mask |                4 |               8 |            1570 |
| 4x mask |                4 |               8 |            1366 |

The pole set is IDENTICAL with and without the mask. A quad sphere needs
exactly 8 valence-3 poles (topology), and the pipeline inserts zero
additional dipoles for the dense region: the integer lattice stretches
(quads shrink ~2x by parameterization scaling) but cannot subdivide
further without new poles, so refinement saturates.

SingularitySimplifier is ruled out as the culprit on this fixture: no
"Simplified cross field singularities" line prints (cancelledPairCount
== 0 both runs) — there are no spare pairs to preserve.

Code path: `Parameterizer::parameterize` applies density only to
`faceScalingField` (sizing for the cover solve). Poles come from
`QuadParameterizer::Result::singularVertices`, extracted in
`buildResultUv` (`core/quadparameterizer.cpp`) as verts with nonzero
corner-rotation sum — a pure cross-field-topology property. Sizing
never enters pole placement.

## Armadillo (complex field): sizing vetoed

Same protocol on `bench/models/armadillo.obj` @5000, 4x mask over the
top y-quartile (~30% of quad budget, ~1.45x linear cap):

- with simplification: inside 5.7506 -> 5.6227 = **1.01x** (nothing)
- simplification disabled (scratch build, reverted): **0.97x**

Simplifier cancels 38 pairs both runs; keeping all 206 poles does not
unlock refinement, so density-gated simplification is exonerated too.
Inside-mask irregular verts grow only +13% (3:44->54, 5:28->30).

Refined verdict: on complex fields the MILS integer rounding eats the
~0.7x local sizing factor — the lattice cannot realize sub-2x gradients
without new dipoles, and none are inserted anywhere in the pipeline.

## Options (scoped, not attempted)

1. **Dipole insertion at sizing discontinuities** (the real fix, big):
   introduce +1/-1 corner-rotation pairs where |sizing gradient| is
   large, before the MILS rounding solve, so the lattice can subdivide
   locally. Requires lattice-consistency care; multi-day research bet.
2. **Density-gated singularity simplification** (smaller, partial): on
   real meshes the simplifier DOES cancel pairs; cancelling inside a
   dense region destroys poles the sizing wants. Gate cancellation on
   local density. Needs pair locations exposed from the simplifier
   (currently counts only). Would not help the sphere (nothing
   cancelled) but may recover refinement on characters.
3. **Defer**: keep advising broad 2x masks over hard 4x ones
   (`docs/face-flow-iterate.md` already does).

## Next step

Options 2 exonerated by experiment; option 1 (dipole insertion) is now
the only structural candidate. New option 3 to scope next: sizing-aware
MILS rounding (bias integer rounding toward finer in dense regions) —
smaller than full dipole insertion, may recover part of the ask.

## Follow-up (2026-10-01, lane/density-poles): rounding exonerated, adaptivity fight found

Fixture: subdivided cube (6/subdiv, 218 verts), +X face masked,
`--target-quads 1000`, CLI defaults. Metrics below are inside/opposite
quad counts (x > +0.85 / x < -0.85 slices, test_density protocol);
`totalFrac` is total quads vs the unmasked run (budget health).

### Saturation curves (before; no code change kept)

| ask | hard ratio | hard totalFrac | smooth ratio | smooth totalFrac |
|-----|-----------|----------------|--------------|------------------|
| 1.5 | 1.43 | 0.95 | 1.38 | 1.00 |
| 2.0 | 1.61 | 0.92 | 1.53 | 0.89 |
| 3.0 | 1.99 | 0.79 | 2.10 | 0.76 |
| 4.0 | 2.51 | 0.68 | 3.15 | 0.72 |

(Smooth = mask feathered t^2 across the side faces. Feathering helps the
ratio; the Blender vertex-group export writes raw weights with no
smoothing option — candidate follow-up, not done here.)

Honest-metric warning: the inside/opposite ratio conflates face gain
with opposite-side collapse. At hard-4x the masked face gains only
241/207 = 1.16x quads in absolute terms while the opposite face
collapses to 104/212 = 0.49x and the total to 0.68x. Report
face-vs-plain absolute + total health alongside the ratio. Budget math
also caps a 1/6-area region: 4x face-absolute needs the rest at ~0, so
~2-3x face-absolute with a healthy total is the realistic target, not 4x.

### Option 3 (sizing-aware MILS rounding): scoped, exonerated on cube

- The rounding loop is effectively single-shot nearest-integer: the
  fixing threshold is `(d+0.001).max(1.0) = 1.0`, so iteration 1 solves
  continuous and iteration 2 rounds + fixes ALL integer variables.
- Per-iteration UV gradients (lines/unit, dense face vs opposite):
  hard-4x ITER0 (pre-rounding) 12.80/5.60, ITER1 (rounded) 13.36/5.83.
  Rounding ADDS gradient; the continuous solution already saturates.
- Uniform-energy-weight experiment: dense-face gradient identical to
  3 decimals (13.366 vs 13.365) — LS weights are not the wall either.
- Conclusion: the wall is continuous-infeasibility under the
  sizing-unaware hard-constraint kernel (seam graph + wheel/rotation/
  corner constraints from the frame field). Rounding bias cannot
  recover what the continuous solution forbids. (Cube-measured; the
  armadillo "rounding eats the 0.7x" inference above needs the same
  ITER0-vs-rounded check before it is retired.)

### QPX/FFX premise: rejected

The port's QPX/FFX lists are case-level robustness tiers (which *inputs*
flip under backend noise: non-manifold soup + listed stragglers), not
spatial maps of where within a mesh flips happen — there is nothing in
them to place poles from. Moot in any case given the rounding
exoneration: the wall is not rounding noise.

### Adaptivity fight (default settings): quantified, new

`faceScaling` multiplies the adaptivity field by the density edge
scale. On flat masked regions the two oppose: hard-4x masked-face
interior (x>0.85, |y|,|z|<0.7) gets m = 1.78 vs opposite-face m = 1.62
— an effective ask of 0.9x, the 4x mask entirely cancelled — while the
face rim gets m = 0.69 (2.3x). The gradient piles into the rim band
(uvgrad rim 14.6 vs interior 4.6, plain 10.4/4.3); interior quads move
only 35 -> 42. The rim spike then chokes extraction (starved cones,
boundary loops, hole fixes) and the total collapses to 0.73x.
Singularities are the same 8 corners in all runs (pole identity
re-confirmed). Density-only scaling (adaptivity/anisotropy 0) asks a
clean uniform 2x and delivers a weak-but-uniform 1.17x gradient with a
healthy total — the topology wall binds there instead.

### Density-precedence experiment: tried, not kept

Min/max combination (explicit mask bounds the heuristic: refining
masks cap coarsening, coarsening masks cap refinement; adaptivity keeps
the reinforcing side), budget renorm unchanged. Hard-4x: interior
quads 42 -> 100 (2.9x of plain), rim spike softened, total 0.73x ->
1.26x, opposite collapse fixed. Not kept: (1) it breaks the pinned
uniform-renormalization proof (a uniform 4x field must reproduce the
plain run — min/max reshapes the field); (2) it needs ~28 engine
DENSITY CASEs + the parameterizer density CASEs re-tiered from strict
counts; (3) the topology wall still binds (face-absolute 1.58x, not
4x); (4) it does nothing for the owner's curved thin-feature case
(fingers), where adaptivity already agrees with density. All
experiment code reverted; tree is docs-only vs main.

### Path forward (unchanged, sharpened)

Dipole insertion (option 1) remains the only structural candidate.
Precise mechanism identified: `solve_quad_cover` skips the wheel
constraint rows at vertices with nonzero rotation sum, so a ring of
dipoles along the density boundary would lift the transition-flux
conservation that makes the gradient jump infeasible today. That is a
multi-week research bet (frame field -> rotations -> cover topology +
all oracles), not a lane-sized change. Validate it on finger-like
curved thin fixtures, not flat-cube interiors, where the adaptivity
fight pollutes the measurement.
