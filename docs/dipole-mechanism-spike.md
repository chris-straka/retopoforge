# Dipole-insertion mechanism spike (research)

Date: 2026-10-01. Lane: `lane/dipole-mechanism`. Verdict: **VALIDATED
(with scope conditions)** — see below.

Question: does inserting dipole (+1/-1 corner-rotation) pairs along the
density boundary BEFORE the MILS rounding solve lift the
transition-flux conservation that makes sizing-gradient jumps
infeasible (`docs/density-poles-spike.md` mechanism: `solve_quad_cover`
skips wheel-constraint rows at vertices with nonzero rotation sum)?

## Prototype

`rust/core/src/quad_parameterizer.rs`, all clearly marked
`EXPERIMENTAL SPIKE (lane/dipole-mechanism)`, all gated on
`RETOPO_DIPOLES` / `RETOPO_DIPOLE_DEBUG` (unset = byte-identical
behavior, proven by bit-identical control rows below):

- `maybe_insert_dipoles` runs AFTER curl correction (field/sizing stay
  identical to baseline) and BEFORE seam + cover solve, so the only
  downstream change is cover topology (wheel skips, seams,
  singularities). The flip op preserves matching antisymmetry mod 4
  and moves exactly +1/-1 onto the two edge endpoints (unit-tested on
  a tetrahedron: `dipole_spike_tests`).
- Boundary keyed off the RAW per-vertex density multipliers (sharp
  mask steps only). An earlier face-scaling-keyed version also fired
  on adaptivity steps (finger base rim) and on unmasked runs; removed
  — density-only now, unmasked runs provably untouched.
- Modes: `radial` (edges crossing the density step, greedy disjoint,
  +1 on the denser-side endpoint) and `loop` (alternating ring walk;
  only works on wide transition bands — broken on knife-sharp steps
  where no two boundary verts are adjacent; radial is the robust
  mode, all verdict numbers are radial).
- Gradient probe: per-iteration mean uv-lines/unit over mask-region
  bands (`RETOPO_DIPOLE_DEBUG=y2.2` / `y2.2x0`). ITER0 solves purely
  continuous (nothing fixed); ITER1 rounds + fixes all integers.
  Masked vs masked+dipoles share the SAME working mesh, so absolute
  lpu deltas are valid there (plain-vs-masked meshes differ —
  invalid).

## Fixtures

Adopted the sibling lane (`lane/dipole-fixtures`, merged in):
`tests/fixtures/finger-{single,split,fused}.obj` + the
`dipole_saturation` harness protocol (1000 quads, engine defaults).
Plus `tests/fixtures/sphere-pole.obj` @2000 with the 4x y>0.5 cap
mask from the earlier spike. (An own procedural finger was built
first, then superseded; its 1-island runs showed a consistent null,
same diagnosis as split-coarse.)

## Results

### finger-single (isolated tip ring) — VALIDATED, strong

Harness 4x (13 flips at y≈2.08–2.20):

| run | faceAbs | linear | totalFrac | totV3/totV5 |
|-----|--------:|-------:|----------:|------------:|
| pinned baseline | 1.18 | 1.08 | 0.92 | 31/7 |
| + dipoles | **1.45** | **1.19** | **0.99** | 33/11 |

3x (12 flips): faceAbs 1.01→1.14, linear 1.00→1.05, total
0.88→0.92. 1.5x/2.0x rows BIT-IDENTICAL to pinned (0 flips below
the ratio threshold — gating control).

Gradients (CLI, same working mesh): ITER0 tip 23.094 → **25.397
(+10.0%)**, shaft 7.632→7.550; ratio 3.03→3.36. ITER1 preserves
it (22.987→25.174, +9.5%) — rounding exonerated, the gain is in
the continuous solution. Dose: 28 flips (wider band) → ITER0
26.373 (**+14.2%**).

### sphere cap (isolated ring, near budget cap) — validated, small

4x mask, radial 30 flips on the y≈0.4–0.47 ring: ITER0 dense
17.403→18.130 (+4.2%); honest linear 1.391→**1.457** (budget cap
is ~1.51 — dipoles recover most of the remaining headroom);
total 0.871→0.919; output poles 8V3 → 17V3+7V5 (2 non-quads).
Dose: 0→15→30 flips gives ITER0 17.40→18.03→18.17, monotonic.

### finger-split (isolated ring, coarse) — null, explained

4x (10 flips): faceAbs 1.25→1.11, linear 1.12→1.07 (worse);
ITER0 tip gradient 20.744→20.810 (+0.3%, flat). Diagnosis: the
continuous tip/shaft ratio is ALREADY 3.50 (ask = 2x) — there is
no wall in continuous space here; the saturation is
extraction-side, so dipoles correctly do nothing. Weak dose
response (22 flips → +2.8%) confirms the direction. Decisive:
split @2000 (masked island = same working mesh as single @1000)
reproduces single's numbers EXACTLY (23.094→25.397, +10.0%) —
the effect is mesh/resolution-dependent, geometry-independent.

### finger-fused (shared septum boundary) — null, explained

4x (15 flips along ring + inter-lobe septum): faceAbs 1.13→1.15,
linear flat 1.10. Gradients: masked +6.4% BUT control +12.7%
(ratio falls 1.29→1.22). Diagnosis: septum vertices are shared
between masked and control faces — freeing them refines BOTH
sides. Adjacent-region masks need offset (interior) rings, not
on-boundary placement. Future work, not attempted.

## Verdict: VALIDATED (scoped)

The mechanism is real and dose-responsive in the continuous
solution: dipole rings along ISOLATED density boundaries open
the gradient (+10% single, +4% sphere) and convert to
face-absolute refinement with healthy-or-better totals
(single 4x: 1.18→1.45 faceAbs, 0.92→0.99 total). Both nulls
have confirmed mechanistic explanations (no continuous wall on
split-coarse; shared-boundary physics on fused), and the
split-q2000 identity proof rules out geometry confounds.

## Production path (follow-up lane, NOT done here)

Auto-placement (ring detection + dose from line-ending count),
offset rings for shared boundaries, sliver-island guards,
dose/threshold auto-tuning, CLI flag replacing the env var,
re-tier of affected oracles. The prototype stays env-gated and
experimental-marked.

## Quirks / flags for sibling lanes

- `finger-single`/`fused` split into 3 orientation-islands each
  (6 for split): the generator's "closed manifold" assertion
  misses orientation consistency. One island's cover solve fails
  silently (no `Failed islands` increment — that counter tracks
  input/output drops, pre-existing semantics). All comparisons
  here are apples-to-apples (identical splits; mask-free islands
  never receive dipoles), but absolute totals lose that island.
- Dipoles on 20-face sliver islands explode the sliver gradient
  (harmless to totals; production placement must skip slivers).
- ITER1 (rounded) is noisier than ITER0 under many singularities;
  ITER0 is the honest mechanism readout.
- CLI vs harness totals differ by ~2 quads on identical inputs
  (892 vs 894): separate code paths, both deterministic; honest
  numbers above are harness (pinned protocol), gradients CLI.
