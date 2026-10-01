# Dipole-insertion mechanism spike (research)

Date: 2026-10-01. Lane: `lane/dipole-mechanism`. Question: does inserting
dipole (+1/-1 corner-rotation) pairs along the density boundary BEFORE the
MILS rounding solve lift the transition-flux conservation that makes
sizing-gradient jumps infeasible (`docs/density-poles-spike.md` mechanism)?

## Protocol

Fixture: `tests/fixtures/sphere-pole.obj`, `--target-quads 2000`, 4x mask on
y>0.5 cap. Metrics: per-iteration UV gradients (lines/unit, dense vs coarse
faces split at geometric mid of face_scaling; ITER0 = continuous pre-rounding,
ITER1 = rounded) + face-absolute honest refinement sqrt(in_mean_plain/in_mean)
+ total health + output valence census. Sibling `lane/dipole-fixtures` did not
exist at spike time; sphere only (finger fixture TBD below).

## Results (live notes; VERDICT at bottom when done)

Baseline (no dipoles): ITER0 17.403/8.210, ITER1 17.584/8.336; honest 1.391x,
total 0.871, poles identical 8xV3 (reproduces density-poles-spike.md exactly).
