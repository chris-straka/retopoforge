# Deformation test (spec; shared gate for retopoforge, rigforge weights, wrapforge)

Static scores (`bench/score.py`) judge a mesh standing still. Game
characters bend, so the main gate for topology and weights work is how a
posed mesh holds up at its joints.

## Procedure (headless Blender, deterministic)

1. Input: a remeshed (or wrapped) mesh plus a rig. For procedural tests,
   generate both: a tube limb with 2 bones (elbow), a biped mannequin
   with rigforge `hll_hero`, a quadruped with `hll_stalker`.
2. Skin with a fixed method (Blender automatic weights for retopoforge
   comparisons, so only topology varies; the method under test when
   testing weights).
3. Apply a fixed pose set: elbow 90 and 135 degrees, knee 90 and 120,
   shoulder raised 90, neck turned 60, spine bent 30, jaw open 20 (when
   the rig has one). Store poses as data files in the repo.
4. Measure per joint region (vertices with weight > 0.1 for the joint's
   two bones):
   - volume loss: signed volume of the region's closed slice, posed vs
     rest (%),
   - stretch: per-face max singular value of the deformation gradient
     (posed vs rest); report p95 and % of faces > 2x,
   - flips: faces whose normal reverses vs rest (must be 0),
   - self-intersection count near the joint (optional, BVH overlap).
5. Output JSON per joint and a summary line; same inputs give
   byte-identical output.

## Use

- `bench/deform.py` (to build) runs it over a mesh list, like
  `bench/noise.py`; compare distributions across seeds, not single runs.
- Gate wording for agents: "change X must not worsen p95 stretch or
  volume loss at any joint beyond the seed spread, and must keep flips
  at 0".
