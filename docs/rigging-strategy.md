# Rigging strategy (HLL character pipeline)

Date: 2026-09-30, revised 2026-10-01. Goal: AI mesh -> clean,
rigged, textured character -> HLL's `glb/` folders. Commercial use,
sold worldwide including the EU, UK, and South Korea (owner is in
Canada and may relocate to the EU/UK). Mac-first hardware (M4 mini
16GB) plus **rented NVIDIA GPUs on demand** (allowed as of
2026-10-01). Owner's bar is good quality output, not bit-identical
numbers. The game-side companion doc is HLL's
`tools/asset-pipeline.md`.

## Route split (revised 2026-10-01)

Field-guided remeshing follows surface curvature, not deformation, so
faces and hands never get animation-grade loops from it. Humanoids
therefore stop going through the remesher for their body:

- **Humanoids: base-mesh wrap.** One base body is built once (face
  loops, separate fingers, mouth interior, UVs, rig, weights). Each
  character is that base fitted onto the AI mesh, which serves as a
  shape target only: rig-driven proportion fit, Shrinkwrap +
  Corrective Smooth, then vertex-moving sculpt (no Dyntopo / voxel
  remesh). Same topology everywhere means one rig, identical weights,
  and shared animation. Base candidates: MPFB2 (code GPL, generated
  output and assets CC0, Blender extension), a hand-made stylized
  base, or one retopoforge hero promoted to base after hand cleanup.
- **Clothes, armor, hair, capes:** split from the AI mesh as separate
  objects, remeshed by retopoforge per object, weights copied from the
  body via Blender's Data Transfer modifier (nearest face
  interpolated) and touched up. A region-remesh mode in the engine is
  not needed while per-object remeshing covers this.
- **Creatures, bosses, unique non-humanoids:** the original route.
  retopoforge (guides, symmetry, density) -> UniRig or rigforge.
- **Static props:** decimate with UVs preserved, or retopoforge LODs
  when the asset will be edited. No rig.

## Two rig tracks + comparison

- `rigforge` (`~/SWE/rigforge`): Rigify 0.6.10 fork, module renamed
  to coexist with bundled Rigify, GPL-2.0-or-later. Headless smoke
  green (159-bone metarig -> 706-bone rig). Role after the route
  split: rigs the humanoid base **once** (the `hll_hero` preset); all
  wrapped humanoids inherit it. Also the fallback for creatures UniRig
  mangles.
- `unirig-mac` (`~/SWE/unirig-mac`): UniRig (SIGGRAPH 2025) port to
  Apple Silicon CPU. MIT code + MIT weights — commercial OK,
  self-hostable, no Tripo account needed. Parity done on the M4 mini
  (all four examples through skeleton + skin; the giraffe takes about
  1.5 min), so the hardware is sufficient for inference. Role: creature
  and non-humanoid auto-placement.
- Comparison: the bake-off stays (one hero + one creature), with
  Mixamo and AccuRig added as free baselines. Judge joint placement,
  weight quality, tweak time, Godot import cleanliness. For humanoids
  the bake-off now decides how the base gets rigged, not how every
  character gets rigged.

## Don't merge the repos

Keep rigforge and unirig-mac separate: rigforge must be GPL (Blender
addon) and runs in Blender's Python; unirig-mac is MIT and needs a
PyTorch environment Blender doesn't ship. End-to-end "mesh -> rigged"
comes later as an orchestrator that calls each as a command-line tool,
the same subprocess pattern the retopoforge addon uses with `retopo`.

## GPU rental (allowed as of 2026-10-01)

Rent per batch (spin up, process a queue of assets, shut down); never
leave instances idle. What it unlocks:

- UniRig fine-tune / LoRA on stylized characters (skin training needs
  roughly 60GB on one GPU per upstream README; parked on the mini).
- Texture-only passes on remeshed or wrapped meshes (see licenses:
  most open texturers are currently blocked).
- Open-weight image-to-3D as shape targets.

## License findings (verified unless noted)

- UniRig: MIT code + MIT weights. Caveat: these are the published
  research weights; Tripo's live product may run newer internal
  models — self-hosting gives you the paper's models, not their
  latest.
- Hunyuan3D (2.1 license, verified 2026-10-01): **out for shipped
  assets.** The agreement excludes the EU, UK, and South Korea and
  forbids using, distributing, or displaying the Output outside its
  territory; there is also a 1M-MAU licensing threshold. Incompatible
  with selling there or relocating there.
- TRELLIS.2: MIT code, has a shape-conditioned texturing mode (24GB+
  NVIDIA GPU), but depends on nvdiffrast / nvdiffrec. nvdiffrast is the
  NVIDIA Source Code License (verified 2026-10-01): research or
  evaluation only, no direct or indirect monetary gain. Blocked for
  shipped assets until the dependency is swapped or licensed.
- Tripo (verified via third-party summaries 2026-10-01): free plan is
  public CC BY and non-commercial; paid plans (Pro about $20/month,
  3000 credits; roughly 40 credits for an HD-textured image-to-3D)
  grant commercial rights for assets generated while subscribed.
- MPFB2: code GPL-3.0-or-later, assets and generated characters CC0.
- SMPL-based motion models (most open video/text-to-motion research):
  SMPL is non-commercial without a paid license. Check before using
  any output in a shipped animation.
- RigAnything (direct UniRig rival): Adobe non-commercial. Out.
- AniGen (same lab, image straight to rigged character): MIT source
  but bundles non-commercial NVIDIA code (stated in its README).
  Watch, don't fork.
- Make-It-Animatable: license contradicts itself between pages, and
  humanoid-only. Out until resolved.
- Anymate: reported Apache-2.0 by two sources, LICENSE file not
  yet verified. Possible third contender.

## Rust note

Owner prefers Rust. Greenfield Rust (headless rig CLI, model
inference) is favored over porting proven engine math; the C++
engine stays until/unless a quality-gated rewrite is justified.
Correction: faer has real sparse support now (formats + sparse
Cholesky/LU) — the earlier "thin at sparse" claim was stale. The
remaining Eigen gap is breadth + battle-testing, and any engine
port is a quality re-verification job regardless of language.
