# Rigging strategy (HLL character pipeline)

Updated 2026-10-01 (supersedes the 2026-09-30 two-track plan). Rigging
lives outside this repo; this note records how it fits the pipeline.

## Order in the pipeline

Topology first, rig second:

- Humanoids: rig the **base** once with rigforge; every character
  wrapped onto it (`~/SWE/wrapforge`) inherits the rig and weights,
  bones follow the wrap.
- One-off monsters: retopoforge remesh -> rigforge (`hll_stalker` or a
  new preset) -> weights.
- Pieces (capes, hair, armor, clothes): weights copied from the body (Data
  Transfer), see `~/SWE/games/tools/asset-pipeline.md`.

## Tools

- **rigforge** (`~/SWE/rigforge`, GPL-2.0-or-later): Rigify 0.6.10 fork
  with HLL presets (`hll_hero`, `hll_stalker`), a deform-bones-only GLB
  export for Godot, and a deterministic landmark detector + metarig
  fitter. The production rigger.
- **unirig-mac** (`~/SWE/blender/unirig-mac`): UniRig (SIGGRAPH 2025) ported to
  Apple Silicon CPU. Works on the M4 mini (hero ~1.5 min skeleton +
  ~45 s skin). Optional joint-hint source only; no code merge.

## Bake-off verdict (2026-09-30, `~/SWE/rigforge/docs/bakeoff/final.md`)

rigforge won hero and creature quality: 160 deform bones with face,
fingers and twist bones vs UniRig's 28 body-only joints on the hero and
12 weak joints on the creature. UniRig won only zero-touch time (~7 min
vs ~30 min scripted); rigforge's landmark fitter targets that gap
(re-run of the bake-off with the fitter is an open rigforge TODO).
UniRig output is also seed-dependent, which clashes with the
deterministic pipeline. The verdict is sound for this game: fingers and
a face rig are required, and UniRig cannot produce them.

## Open points

- **Mobile bone budget**: 160 deform bones is heavy for mobile skinning.
  Add a game-export profile that drops or merges face/twist bones for
  mobile LODs (one rig, two export profiles), and measure skinning cost
  in Godot on a target phone.
- **Weights are the real time sink** (rigforge TODO calls weight
  painting the highest-value item). Humanoids get weights from the base
  via wrapforge. For one-off monsters, a better binding method than
  Blender's heat weighting (which fails on messy meshes) is the tool
  worth building: geodesic voxel binding (Dionne & de Lasa 2013) for
  creatures and robust weight transfer with inpainting (Abdrashitov et
  al. 2023) for pieces.
- **License check**: the earlier version of this note called UniRig
  MIT (code + weights); the pipeline doc calls it effectively
  GPL-3.0-or-later because of its shape encoder. Verify before shipping
  anything that depends on its output beyond hints.
