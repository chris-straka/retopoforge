# Creature and prop bench

Judges a remesh the way the game uses it: rigged, bent at its real
joints. It replaces `bench/deform.py`'s fold through the bounding-box
centre for creature work (that script stays as the old regression gate).

## The set

`WORK/subjects.tsv` (`name <tab> kind <tab> source [<tab> budget]`):

- `tripo`: the owner's generated creatures (Tripo exports, about two
  million triangles). Cleaned once to a 300k-triangle dense source
  (motionforge `tools/tripo_to_glb.py`), and rigged once by skintokens'
  creature route on an 18k cut of that source (`--clean-mesh`, class
  monster, no clips).
- `animal`: hand-made rigged animals (CC0/CC-BY low-poly packs, prepared
  by skintokens' `tools/bench/prep_truth.py`). The artist's rig is the
  reference and the artist's own mesh is scored as tool `artist`; the
  dense source is the artist mesh smoothed two Catmull-Clark levels, and
  every tool cuts at the artist's triangle count.
- `prop`: HLH prop GLBs; no rig, so only shape and census scores.

Copies live outside the repo (the owner's assets are never committed).

## How a candidate is judged

1. Each tool cuts `dense.obj` to the budget (one recalibration if the
   count misses by more than 8%).
2. `transfer.py`: the reference rig moves back into the bench frame
   (exact similarity fit on the shared vertices, else the best quarter
   turn), and the candidate takes its weights from the nearest point on
   the reference surface (interpolated, 4 bones, normalized). Same for
   every tool, so only topology differs.
3. `metrics.py` (faces read from the candidate's OBJ, since a GLB holds
   only triangles):
   - census: triangles at the budget (a quad is two), quads, irregular
     vertices, open edges;
   - shape: two-sided surface distance (mean, p95, max; % of the source's
     diagonal) and normal deviation (mean, p95, degrees);
   - `rest_flips`: faces facing against the source;
   - per joint (a bone with skin whose parent has skin): `loops` (closed
     edge loops that go once round the limb's axis within one limb radius
     of the joint, stay within one radius along it and within 2.5 radii
     of the axis), `poles` (irregular vertices within one radius),
     `flips` (faces whose normal turns past 90 degrees from its rest
     normal carried by its bone, with the bone bent 60 degrees toward each
     world axis across it);
   - summaries: `joint_loops_mean` (loops capped at 3), `joints_ringed_pct`
     (joints with 2+ loops), `joint_poles_mean`, `posed_flips`.
4. `weights check --poses <rom> --no-clips` (weightforge) on the rigged
   candidate: the creature's own range of motion from the route. Its
   score, failing findings and bad vertices are the bend score.
5. `render.py`: clay + wireframe pictures, whole and one bent joint.

## Run (on f)

```sh
source ~/rfwork/env.sh      # BLENDER, RETOPO, WEIGHTS, the route's tools
bench/creature/run.py ~/rfwork/set --stages clean,prep,ref
bench/creature/run.py ~/rfwork/set --tools retopo,quadriflow,decimate,artist --jobs 3
bench/creature/run.py ~/rfwork/set --tag skel --retopo-args "--skeleton ..."   # a variant
bench/creature/table.py ~/rfwork/set > table.md
```

Every stage skips work whose output exists; delete a file to redo it.
