# Dipole insertion, production (lane/dipole-production)

Date: 2026-10-01. Context: `docs/dipole-mechanism-spike.md` (VALIDATED,
env-gated prototype, radial/on-boundary placement) and
`docs/dipole-fixtures-baseline.md` (fixtures + harness + pinned curves).
This lane productionizes the mechanism: automatic placement, offset
rings, sliver guards, the two ticketed quirks, a CLI flag, and re-tiered
oracles.

## What shipped

`DipoleConfig` (`rust/core/src/quad_parameterizer.rs`), plumbed
`AutoRemesher::set_dipoles` -> `Parameterizer::set_dipoles` ->
`QuadParameterizer::parameterize` (new trailing param), replacing the
`RETOPO_DIPOLES` env spec (removed; `RETOPO_DIPOLE_DEBUG` stays as a
research-only stderr diagnostic). Per-island flip counts surface through
`island_dipole_counts()` alongside the quad counts.

- Auto-placement: ring detection off raw per-vertex density steps
  (per-face mean keys; sharp fans mark boundary vertices; connected
  boundary components are rings, each dosing independently), dose from
  the per-ring line-ending estimate `n_cross * (sqrt(ring_ask) - 1)`
  capped at the disjoint-candidate count, spread by striding.
  Explicit overrides: `every` (dose stride) and `ratio` (sharpness
  gate).
- Auto sharpness gate 1.5 (derived, not tuned): for a vertex step of
  ask A the sharpest adjacent face pair is a 1-dense-vert straddle
  face against a pure coarse face, max pair ratio (A+2)/3 — 4x -> 2.0,
  3x -> 1.67, 2x -> 1.33. The gate fires on asks >= ~3x and skips mild
  (<= 2x) masks, which realize nearly fully without dipoles. Side
  benefit: the differential oracles' 2x density cases stay parity-clean.
- Offset (interior) rings: both poles land dense-side (densest vertex
  of the crossing's denser face + its densest above-mid neighbor), so
  shared steps cannot leak refinement across. Offset-or-nothing: thin
  masks with no interior edge place nothing.
- Production placement rule: **offset everywhere** (no shared-boundary
  detector). Measured on the finger fixtures: offset beats or ties
  on-boundary on 5 of 6 sharp rows and fixes the one harm case (fused
  3x faceAbs 1.02 -> 1.39); `OnBoundary` stays an explicit API override.
- Sliver guards: islands under 64 working-mesh faces skipped (they
  span < 1 UV cell; dipoles there only explode the sliver gradient),
  rings under 4 crossing edges dropped as specks, plus the mild/uniform
  no-op gating. The spike's `loop` mode (broken on knife-sharp steps)
  is removed; radial selection is the only mode.
- Quirks, both closed: (1) the generator's pole fans opposed the side
  quads (each tube split into 3 orientation islands); fan order fixed
  and directed-edge pairing asserted in the generator + harness;
  (2) the "silent cover failure" forensically cleared — instrumented
  runs show parameterize + extract both succeed and the 0-quad island
  is area starvation (tiny island spans < 1 UV cell), and the
  `Failed islands` counter DOES catch it (verified Failed: 1 on the
  old fused fixture in all modes). Genuine silence was
  measurement-side: the harness now prints + asserts per-island counts,
  CLI unit tests pin `dropped_island_count`, and a golden pins the
  starved-sliver 0-count.
- CLI: `--dipoles off|auto` (default **auto**, see below) plus
  `--dipole-every` / `--dipole-ratio` overrides.

## Saturation after (harness protocol, offset placement)

Re-tiered baselines (the orientation fix renumbers everything: closed
tubes extract much coarser plains than the old open-tube-plus-fans, so
masked totals run over budget — pre-existing density behavior, visible
in the off rows, not a dipole effect). Full tables in
`docs/dipole-fixtures-baseline.md`.

4x faceAbs, off -> auto (totals in quads):

| fixture | faceAbs off | faceAbs auto | total off | total auto | flips |
|---------|----------:|-----------:|--------:|---------:|------:|
| single  |        1.70 |       2.04 |     696 |      748 |    14 |
| split   |        1.57 |       1.70 |     758 |      769 |     9 |
| fused   |        1.33 |       1.52 |     720 |      752 |    21 |

All six sharp rows (3x + 4x) improve over off (single 3x 1.57 ->
1.92, split 3x 1.39 -> 1.70, fused 3x 1.15 -> 1.39); dipoles never harm
any row. Valence census moves systematically (poles added on the
masked island only; split control island bit-identical at 83 quads in
every row). 1.5x/2x auto rows are bit-identical to off (0 flips —
gate control), and plain + mild-2x are bitwise off==auto (dedicated
test).

On-boundary vs offset, 4x faceAbs: single 1.83/2.04, split 1.77/1.70,
fused 1.61/1.52; 3x: single 1.83/1.92, split 1.43/1.70, fused
1.02/1.39. Offset wins 4, ties 2 (within noise), fixes the harm case.

## Default call: ON (auto), with evidence

`--dipoles` defaults to auto, and `AutoRemesher::new` enables it. Why
this is safe:

1. Systematic gains, no harm: 6/6 sharp rows improve (above); zero
   lost islands anywhere (asserted); totals +0-5% over off.
2. The gate, not the default, decides: unmasked, mild (<= 2x), and
   smooth painted-like masks (verified: ramp mask bitwise off==auto)
   place nothing — typical painted masks cannot change behavior at
   all. Only sharp procedural steps fire.
3. Bounded worst case: adversarial vertex-checkerboard noise places
   211 flips and still converges sanely (-11% quads, no lost island).
4. Determinism: cross-process byte-identical outputs (asserted by e2e
   and re-verified for masked+dipole runs).
5. Escape hatch + loud failures: `--dipoles off` restores legacy
   behavior exactly; any island drop (none observed) reports via the
   verified `Failed islands` accounting.

Re-tiering the call required (all mechanical, each with a mechanism
note at the site): `auto_remesher_diff` pins off explicitly (34
density cases carry sharp steps; the C++ has no dipoles), the
quad-parameterizer oracles take the new param as off, the e2e help
comparison strips the Rust-only flag lines (C++ frozen), and a golden
pins the shipped default itself. E2E density cases (2x) need no change
(the gate skips them — verified green). Reversal is one line per
default site if the wild disagrees.
