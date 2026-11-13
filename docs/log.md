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
