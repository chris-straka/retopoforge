# retopoforge log: what was tried, kept and dropped

Newest round first. Scores come from `bench/creature/` (README there):
the owner's generated creatures and HLH props, plus hand-made rigged
animals as the topology reference. Subjects are named by kind only.

## The keep rule

A change to the engine or its defaults is kept only when, against the
version before it on the same set (`bench/creature/table.py --compare`):

- on the rigged subjects it is better on more subjects than worse for at
  least one of: bend score (weightforge on the creature's own range of
  motion), loops at joints, poles at joints, flipped faces in the bent
  poses;
- it is worse on more subjects than better for none of them;
- its median two-sided surface distance (p95) is at most 10% higher;
- and the before/after pictures (wireframe on the creature, a bent joint)
  agree with the numbers.

## Round 1 (2026-11-13): review, bench, baseline

See `docs/review-2026-11-13.md` for the review.

### Baseline: retopo against plain decimate on the owner's creatures

10,000-triangle budget, each creature rigged once by the creature route,
the same weights carried onto every cut. First four creatures (the fifth
and the hand-made animals were still scoring):

| creature | bend retopo / decimate | worst distance % | open edges | normal dev. deg |
|---|---|---|---|---|
| 1 (furred biped) | 49.7 / 68.1 | 5.33 / 1.17 | 14 / 0 | 32.7 / 21.2 |
| 2 (rooted quadruped) | 64.8 / 75.1 | 9.51 / 0.42 | 78 / 0 | 39.4 / 20.3 |
| 3 (horned biped) | 83.0 / 93.3 | 4.44 / 0.38 | 0 / 0 | 25.3 / 12.8 |
| 4 (spined quadruped) | 68.5 / 85.0 | 4.26 / 0.48 | 0 / 0 | 33.1 / 18.8 |

Retopo loses on every one. The pictures show why: single vertices thrown
far off the body (long spikes), small holes, and spines and fur melted
into blobs. Where the surface is plain (the rooted creature's hind leg),
retopo's rings bend cleaner than decimate's slivers, but the checker sees
the spikes and lost shape first. QuadriFlow (Blender) refused three of the
five creatures even on a closed voxel shell; on the one it cut, it scored
72.2 (between the two). Loops at joints are near zero for every tool
(retopo 0 to 0.4 rings per joint), so nothing on the route makes joint
loops today.

Also found: the horned creature is one connected piece, yet retopo split
it into three islands and dropped two (review finding 11, island split by
face winding); the spined quadruped's six "islands" were 3-vertex specks.
