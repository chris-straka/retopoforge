# Corner marks: alignment-only explicit sharps (cage fix)

Date: 2026-10-01. Closes the engine-backlog item "corner
singularities under crossing sharps: full closed cages over-constrain
and distort".

## Symptom

A closed 12-edge `--features` cage on a subdivided box degrades every
metric vs features-off (8-seed noise distributions, target 600):

| case | quads | irr % | angdev | dist mean | dist max |
|---|---|---|---|---|---|
| plain | 561 | 5.3 | 7.0 | 0.21 | 4.3 |
| cage, before fix | 437 [400-455] | 13.7 [7.1-18.3] | 10.0 [8.4-10.7] | 0.34 | 6.9 |
| cage, after fix | 563 [526-603] | 4.0 [2.5-7.2] | 6.2 [4.4-8.4] | 0.22 | 4.3 |

A mark-count sweep (1 / 4-loop / 11-open / 12-closed) shows damage
proportional to marked-edge count, not to closure: partial cages
over-constrain too. Single lines were always fine.

## Cause

Every explicit-sharp mark fed the same full-constraint channel as
automatic dihedral marks, three ways: frame locks (2-edge band),
curl anchors (rotation frozen), and cover integer pins (period 1 +
equality per marked corner). The 12-edge cage pins 912 integer
variables + 456 equalities + ~456 anchors on a ~1200-tri working
mesh — the cover solve has no freedom left. Channel ablation
(scratch worktree, env gates):

- no frame locks: barely moves (444 quads, irr 13.5).
- no corner marks at all: fully recovers (599 quads, irr 4.8) —
  frame locks alone help.
- no integer periods (equalities kept): recovers (470 quads, irr
  6.1, dist max 3.0 vs 7.2) — the periods were the damage.
- no periods + no anchors: matches no-marks everywhere.

Crossings need nothing extra: axis-aligned crossings are
cross-consistent (no charges), and the box cage's 8 triple crossings
recover fully under the fix. The simplifier-freezing idea was
dropped for lack of a measured defect.

## Fix

`rust/core/src/quad_parameterizer.rs`: explicit marks now emit
`AlignU`/`AlignV` (alignment-only). The cover keeps the UV equality
(the crisp line) but skips the integer period (positional locking)
and the curl anchor (rotation freezing). Automatic dihedral marks
keep full U/V. Frame locks unchanged.

Single straight lines are unaffected-or-better on closed meshes
(box single-edge, 8 seeds: quads 564 -> 565, irr 4.2 -> 3.7,
angdev 7.6 -> 7.2, dist max 4.8 -> 3.6, all ranges overlapping).
The grid+line contract golden moves 282 -> 254 quads: the old pins
inflated density along the line (plain grid yields 249); the new
count is the natural density plus alignment, with non-quads halved
(4 -> 2).

## Oracle fallout (all reviewed, sharp-carrying cases only)

- `quadparam_diff`: 5 strict cases (35, 47, 51, 70, 187) demoted to
  QPX — UV/FIELD value only, no ok/progress/ROT/SING divergence.
- `parameterizer_diff`: 32 strict cases demoted to PPX with listed
  identities (180/183/325 precedent) — UV value only.
- `autoremesher_diff`: 45 cases re-pinned via UPDATE_ARDIFF
  (permissive for the non-quad ratchet); verified zero SHARPS=0
  changers, so the default path is byte-identical. Case 12 demoted
  to EPX (value-only iuv drift the count-triggered regen cannot
  re-pin). Coverage pins: case 58 retired (fires no more — its
  "structural" collapse was its own 3 sharps pinning the slab),
  case 99 improves (24 -> 12 initial, 11 -> 0 final).
- CLI contract: the two `--features` goldens re-pinned (both halve
  non-quads), plus a new `box-cage` regression case (603 quads;
  fails pre-fix at 444 — verified by stashing the fix).
- `bench/run.py --check`: green, byte-identical (no bench case uses
  explicit sharps).
