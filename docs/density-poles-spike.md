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
