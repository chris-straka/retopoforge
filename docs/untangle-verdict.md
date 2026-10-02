# Untangling-stage verdict: score-gated NO (item 7)

Date: 2026-10-02. Question: does a post-rounding Garanzha et al. 2021
foldover-free barrier stage (integer kernel variables = quad layout held
fixed, continuous ones re-optimized), prototyped env-gated behind
`RETOPO_UNTANGLE`, earn a place in the pipeline? Gate: `bench/noise.py`
distributions + `bench/score.py` + CLI contract. Answer: **no** — the
stage halves an internal metric nothing downstream sees while measurably
harming quad shape and surface fit. The prototype was removed from main
(history preserves it); this doc records the evidence so nobody reruns it.

## What was tried

- Forward-port of the `exp/igm-validity` prototype: L-BFGS (history 8,
  Armijo backtracking) under the paper's eps continuation, serial and
  bit-deterministic, hooked after greedy rounding in `solve_quad_cover`.
- Inner budget 300 -> 10 per outer round (300 never converged either:
  16174 inner iters / 60 rounds on fandisk, evals/iter 1.04; budget cut
  keeps the trajectory at ~1.5x baseline time vs ~10x).
- Revert-on-worse: keep the result only when flips strictly decrease
  (beast@5000 diverges: 843 -> 872 flips, min_det deepens).

## Convergence: never reaches zero, wastes most rounds

All cases exhaust the 60-round cap. Fandisk@5000 trajectory (55 flips in):

- eps0 = 2*sqrt(1e-3 + min_det^2) = 6.72 (initial min_det -3.36): the
  barrier is nearly flat over flips, rounds 0-30 wander and flips RISE
  55 -> 66.
- Flips fall only once eps < ~0.3 (round 34+), ending at 11 with
  min_det -7.9e-3, still falling when rounds run out. Inner loop always
  runs the full 10 (never converges early); L-BFGS history rebuilds
  from scratch every outer round.
- IGM neg census (extractor entry): fandisk 54 -> 10, armadillo@1000
  105 -> 35, beast@1000 183 -> 93. Never zero anywhere.
- Native beast@1000 stalls immediately (324 -> 298, 310 -> 307):
  the native-decimator IGM is more tangled and the fixed layout leaves
  the barrier nothing to work with.

Free variables outnumber faces (fandisk: 15977 free vs 14582 faces), so
the stall is the fixed-integer layout + eps schedule, not DOF shortage.

## Scores: flips don't propagate, harm does

Fandisk@5000, 8 seeds, off vs on (median; ranges disjoint where noted):

| metric | off | on |
|---|---|---|
| angdev_mean | 6.44 | 9.71 (off max 7.36 < on min 8.40) |
| angdev_p95 | 20.9 | 27.4 (disjoint) |
| dist_mean | 0.0337 | 0.0397 (disjoint) |
| dist_max | 0.71 | 1.22 (disjoint) |
| irr % | 0.74 | 0.68 |
| yield / non-quads | wash | wash |

Beast/armadillo@1000 (8 seeds): irr slightly better (15.4 -> 13.9,
11.7 -> 10.2), dist_mean +10-15% worse, angdev/yield wash-to-worse.
Native beast@1000 single seed: non-quads 12 -> 6 but angdev 17.0 ->
21.0 (+23%), dist wash. Output non-quad share is ~0-0.6% either way
(fandisk even gains 2 non-quads with untangle on).

Mechanism: the barrier minimum (J = identity vs the ideal
frame-aligned reference) fights the least-squares field fit; with the
integer layout fixed the compromise can't be satisfied, so the map
drifts away from the good fit toward det > 0. Quad angles and surface
projection pay for flips the extractor already handles (item 1's
repair passes leave ~0.5% non-quads regardless of IGM neg).

## Verdict + notes for the patch back end

- Post-rounding untangle with fixed integers does not land: no output
  metric improves anywhere, two degrade with disjoint seed ranges.
- Revival sketch (only if a future consumer needs flip-free IGM, which
  the current extractor does not): anchor fidelity to the LS solution
  (barrier + lambda|x - x_LS|^2) so the stage removes flips without
  drifting, cap eps0 (~0.5) to skip the wandering phase, and carry
  L-BFGS history across outer rounds.
- For the patch back end (item 10): flips are baked into the integer
  layout, not repairable downstream of it — quantize a feasible layout
  (QuadWild + Bi-MDF family) rather than untangling after the fact.
  The beast@1000-native appendage fold stays a first-attempt-coverage
  target there (see `docs/coverage-retry.md`).
