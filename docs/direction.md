# Direction: what retopoforge is for, and what to build next

## Where retopoforge sits on the creature and prop route (2026-11-13)

Measured on `bench/creature/` (the owner's five new generated
creatures, each rigged once by the creature route with the same weights
carried onto every cut; ten HLH props; 10,000 triangles for creatures,
each prop's own count). Numbers and pictures: `docs/log.md`, round 1.

**Props: never through retopoforge.** HLH's props are built at budget by
the props kit (`~/Games/_tools/blender/props/`), already clean. Remeshing
one at its own triangle count only damages it: surface error (p95) 0.3
to 11.6% of its size, and thin parts lost (the ribcage came back with
1,370 of 4,160 triangles). A generated prop goes through meshkit's clean
copy (closed shell, decimate to budget, bake) like any generated mesh.

**Creatures: not through retopoforge today.** The default is meshkit's
clean copy (closed shell, Blender decimate to the class budget, bake),
then skintokens' creature route for the rig. On all five creatures plain
decimate bends better under the creature's own rig (weightforge score
68-93 against retopo's 50-83) and keeps the shape (surface p95 0.10-0.19%
against 1.1-3.4%). Retopo today throws stray vertices off the body
(spikes up to 10% of its size), leaves small holes, melts spines and fur
into blobs, and splits one connected creature into islands it then drops.
Its one strength, even rings down a plain limb, does not outweigh that.
Blender's QuadriFlow refused three of the five creatures outright.

**When that changes.** Retopoforge goes back on the creature route when,
on this bench, it beats decimate's bend score on most creatures with the
surface error within 10% of decimate's. The work to get there, in order:
spikes and holes (output settled on the input surface), joint loops
(rings cut in and pinned), keeping thin parts (islands, rounding). Each
is tried in `docs/log.md` and kept only on the keep rule there.

**What retopoforge is for meanwhile:** making a new base mesh for
wrapforge, by hand (guides, symmetry, density, then cleaned in Blender),
and one-off hard cases where quads are wanted for hand editing. The
scope below still holds for that.

---

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
  - one-off **pieces** (any separate mesh attached to the body: capes,
    hair, armor, clothes, belts); a piece style reused on 3+ characters
    (a tunic, boots, gloves) can instead get its own piece base, built
    from the body base's faces so it inherits the body's weights, and be
    wrapped,
  - **making new bases**: remesh the first creature of a new body plan
    carefully (guides, symmetry, density), clean it by hand, rig it, and
    it becomes the base wrapforge reuses.
- **Rule of three**: if you will make 3 or more of the same shape (body
  plan or piece style), build a base once and wrap; a one-off -> remesh.

Faces and hands on humanoids are therefore wrapforge's job (the base
already has good loops and separate fingers). Do not spend retopoforge
effort on human face/hand topology.

## Engine direction: patch-layout back end

**Update 2026-10-01 (decision): do NOT build the rewrite yet.** On a real
AI character (owner corpus, 150k-tri input), at matched counts (~13.5k
quads) against QuadWild + Bi-MDF's best edge-flow config: ours 6.3%
irregular vs 7.8%, 10 vs 18 degrees corner error, equal mean surface
error; QuadWild wins worst-case error (2.0% vs 3.2%) and has 0 non-quads
(ours 186). The owner's visual check agreed ours looked cleaner. Do the
targeted fixes in `~/Games/_tools/roadmap.md` section 2 instead;
revisit the patch back end only if a second character reverses this.
The patch back end WILL be built, as an **optional extension** (a
selectable back end, e.g. `--backend patch`, to try and compare), never
a replacement: the current engine stays the
default, and the patch back end serves cases that need all quads or
tighter worst-case accuracy (its measured strengths). Owner decision,
2026-10-01. The plan below describes that extension.

Evidence (`docs/igm-validity-spike.md`): the current back end (least-
squares cover -> one-shot rounding -> heuristic extraction) produces
folded uv maps, integer layouts that cannot be untangled, and tilings
that change under invisible input noise. Against the QuadWild + Bi-MDF
reference (black-box, GPL, not vendored; `bench/quadwild.py`) at
**equal quad counts** (`bench/matched.py`, 10 cases incl. the owner's
character) the result is mixed: QuadWild wins surface accuracy
(especially worst-case) and is always all-quads; ours wins irregular
vertices (8/10) and corner angles (8/10). (An earlier comparison at
unequal counts overstated QuadWild.) The patch back end should aim for
both: QuadWild's all-quad layout and fidelity with our field quality.

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
