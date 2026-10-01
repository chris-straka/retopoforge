# Direction: what retopoforge is for, and what to build next

Date: 2026-10-01. Owner-approved summary. Read this before proposing
engine work.

## Scope: retopoforge vs wrapforge

- **wrapforge** (`~/SWE/wrapforge`, see its `PLAN.md`) wraps one good
  base mesh onto an AI shape. It handles anything that shares a body
  plan with a base: all humanoids (including humanoid monsters: goblins,
  orcs, skeletons) and creature families once a base exists (one
  quadruped base for every wolf-like enemy).
- **retopoforge** handles everything that fits no base:
  - one-off monsters and bosses (six arms, spiders, tentacles, slimes),
  - clothes, armor, capes, hair (shapes vary too much for a base),
  - **making new bases**: remesh the first creature of a new body plan
    carefully (guides, symmetry, density), clean it by hand, rig it, and
    it becomes the base wrapforge reuses.
- Rule of thumb: 3+ characters with the same body plan -> wrap; a
  one-off or an accessory -> remesh.

Faces and hands on humanoids are therefore wrapforge's job (the base
already has good loops and separate fingers). Do not spend retopoforge
effort on human face/hand topology.

## Engine direction: patch-layout back end

Evidence (`docs/igm-validity-spike.md`): the current back end (least-
squares cover -> one-shot rounding -> heuristic extraction) produces
folded uv maps, integer layouts that cannot be untangled, and tilings
that change under invisible input noise. On the owner's character at
5000 quads the QuadWild + Bi-MDF reference (black-box, GPL, not
vendored) beat it on every score (`bench/quadwild.py`).

Build a Rust patch-layout back end (clean-room from the papers: QuadWild
2021, Bi-MDF quantization 2023), keeping the existing front end (weld,
isotropic remesh, cross field, singularity simplifier, density,
symmetry, guides). Patch boundaries come from three sources:

1. **automatic**: traced from the cross field's singularities;
2. **joint loops**: rings across each bone at elbows, knees, neck, tail
   base, jaw, placed from rigforge's landmarks
   (`~/SWE/rigforge/tools/detect_landmarks.py`, biped and quadruped
   sets), because monsters bend there;
3. **owner strokes**: a few lines drawn in Blender (eye ring, spine
   line, jaw ring) become forced patch boundaries; the tool fills every
   patch with clean quads (sketch-based quad meshing, Takayama et al.
   2013).

Low LOD rungs are not a target for this back end: far rungs are a few
pixels tall, so decimating the hero mesh (Godot import LODs) is fine.
Optimize the hero mesh (roughly 5k-15k triangles per the pipeline doc).

## Measure deformation, not just the static mesh

`bench/score.py` judges a mesh standing still. For monsters the honest
test is a posed mesh: rig it, pose it (knee bent 90 degrees, neck
turned), and measure stretch and volume loss at the joints. This
deformation score becomes the main gate for the patch back end.

## First experiment (cheap, decides how much to build)

Run the deformation test on a few monsters in three versions: current
remesh, remesh with joint-loop guides, and a plain decimated triangle
mesh with a baked normal map. Small or background enemies may be fine
as triangles (Godot renders triangles anyway), which would mean they
skip retopology entirely. The result decides how many monsters need the
full patch back end.
