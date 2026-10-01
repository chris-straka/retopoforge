# Integer-grid-map validity + noise floor (research)

Date: 2026-10-01. Tools: `bench/score.py` (quality scorecard),
`bench/noise.py` (K-seed noise floor + best-of-K). Engine experiments
live on branch `exp/igm-validity` (env-gated, not on main). Verdict: the
current back end (least-squares seamless cover -> one-shot rounding ->
heuristic extraction) cannot guarantee a valid quad layout; the defects
start in the continuous solve and some are forced by the integer layout
itself. This motivates evaluating a layout-quantization back end
(QuadWild + Bi-MDF family) before more tuning of this one.

## 1. Noise floor: sub-visible input noise changes the tiling

`bench/noise.py`, 8 seeds (seed 0 = original file; seeds 1-7 jitter every
vertex by +-0.5e-9 of the bbox diagonal), defaults otherwise. Values are
seed 0 [min-max over seeds]:

| case | quads | irr % | mean angle dev | non-quads |
|---|---|---|---|---|
| armadillo @1000 | 522 [513-735] | 9.96 [9.96-13.6] | 11.9 [11.9-15.1] | 12 [8-18] |
| armadillo @5000 | 4483 [4483-4821] | 3.66 [2.99-3.66] | 9.05 [7.89-9.62] | 14 [12-22] |
| beast @1000 | 680 [680-844] | 14.4 [12.9-17.1] | 15.6 [15.6-20.4] | 12 [12-26] |
| beast @5000 | 5423 [5383-5437] | 5.26 [5.2-5.47] | 10.4 [9.97-10.6] | 49 [47-50] |
| nefertiti @1000 | 1104 [961-1150] | 3.23 [2.36-5.38] | 9.5 [7.73-10.7] | 8 [2-12] |
| fandisk @1000 | 1510 [1420-1590] | 2.81 [2.81-6.83] | 10.6 [9.81-12.3] | 10 [10-26] |
| owner AI character @1000 | 1470 [1420-1560] | 11.8 [11.7-14.1] | 12.6 [11.7-13.6] | 4 [4-16] |

- Not a duplicate-vertex artifact (inputs have 0 duplicate positions).
- Decimation (input > 8x target) is the big amplifier: armadillo @20000
  (no decimation) is bit-identical across seeds, as are the finger
  fixtures with masks (dipole gains are robust: single 4x faceAbs
  1.70 -> 2.03 on every seed). Beast @5000 without decimation still
  varies (first divergence: "Hold singular lines").
- Consequence: single-run A/B comparisons on decimated or complex inputs
  are below the noise floor. Gate on `bench/noise.py` distributions.
- Linux-772 is an instance of this, not a separate libm problem.
- Best-of-K by rank sum is mixed (wins on armadillo @5000: angle dev
  9.05 -> 7.89, irr 3.66 -> 2.99; seed 0 already near-best elsewhere).

## 2. Scale dependence (absolute-unit thresholds)

Exact power-of-two rescales (bit-exact in IEEE) change the output, and
below a model diagonal of ~0.01 units quality collapses: owner AI
character x1/1024 gives 152 non-quads (vs 18) and 12.2% irregular (vs
6.1%); fandisk x1/1024 28 non-quads (vs 6). One known site:
`PositionKey` truncates crossing points to 1e-5 absolute units in the
extractor. Cheap fix: normalize input by a power of two on load
(exact, so scale-invariant by construction).

## 3. The cover is not a valid integer-grid map

Census of signed uv-triangle areas at extractor entry (`RETOPO_IGM_DEBUG`
on the branch). A valid IGM has zero inverted triangles:

| case | inverted (rounded) | inverted (continuous, no rounding) |
|---|---|---|
| fandisk @5000 | 54 / 14582 (0.4%) | 52 |
| nefertiti @1000 | 109 / 3736 (2.9%) | 106 |
| armadillo @1000 | 98 / 2384 (4.1%) | 78 |
| armadillo @5000 | 158 / 13786 (1.1%) | 152 |
| beast @1000 | 233 / 2608 (8.9%) | 151 |
| beast @5000 | 681 / 12118 (5.6%) | 512 |

65-97% of the inversions exist BEFORE rounding: the least-squares cover
has no injectivity term. The extractor's repair cascade (five-merge,
hole fixing, triangle cleanup, ...) is downstream compensation.

Rounding note: `solve_iteration`'s threshold is `max(d + 0.001, 1.0)`
with `d <= 0.5`, so it is always 1.0 and every integer is fixed in one
shot (upstream probably meant greedy). A progressive schedule
(`RETOPO_ROUND_SCHEDULE=0.1,0.2,0.3,0.4`) cuts inversions 12-25% (beast
233 -> 175) but the scorecard moves only within the noise floor.

## 4. Untangling with the layout fixed: partially feasible

`RETOPO_UNTANGLE` (branch): Garanzha et al. 2021 barrier energy over the
continuous kernel variables, integer variables held fixed, each face
measured against its own frame/sizing target triangle. Unit tests:
gradient vs finite differences, folded-fan unfold.

| case | inverted before -> after | min det before -> after |
|---|---|---|
| fandisk @5000 | 55 -> 8 | -3.36 -> -0.016 |
| armadillo @1000 | 107 -> 69 | -1.47 -> -0.23 |
| beast @1000 | 294 -> 336 | -8.0 -> -8.0 |

Where the rounded layout admits a valid map, the barrier finds most of
it; on beast it cannot move at all: the integer layout (cone positions +
seam translations) is itself infeasible, so no continuous post-process
can fix it. Not shippable as is (60 outer rounds unconverged, ~10x
slower on fandisk).

## Conclusion

The two defects that drive irregular vertices, non-quads, and chaos
(inversions in the continuous cover; infeasible integer layouts from
greedy rounding) are structural to this family (QuadCover / MIQ,
2007-2010). The post-2015 answer is to make the integer layout feasible
by construction: quantize a patch/T-mesh layout globally (QGP, Campen
et al. 2015; QuadWild, Pietroni et al. 2021; Bi-MDF quantization,
Heistermann et al. 2023). Next experiment: run the QuadWild + Bi-MDF
reference binary (GPL-3, black-box only) through `bench/score.py` and
`bench/noise.py` on the same cases. Decision rule: if it beats the
current engine's seed-0 AND best-of-8 on irregular %, non-quads, and
target yield on the character meshes at 1000-5000 quads, with
comparable surface distance, reimplement that back end in Rust from the
papers (clean-room, no code reuse), keeping our front end (weld,
isotropic remesh, cross field, guides, density, symmetry).
