# Rigging strategy (HLL character pipeline)

Updated 2026-10-01 (supersedes the 2026-09-30 two-track plan). Rigging
lives outside this repo; this note records how it fits the pipeline.

## Order in the pipeline

Topology first, rig second:

- Humanoids: rig the **base** once with rigforge; every character
  wrapped onto it (`~/SWE/blender/wrapforge`) inherits the rig and weights,
  bones follow the wrap.
- One-off monsters: retopoforge remesh -> rigforge (`hll_stalker` or a
  new preset) -> weights.
- Pieces (capes, hair, armor, clothes): weights copied from the body (Data
  Transfer), see `~/SWE/games/tools/asset-pipeline.md`.

## Tools

- **rigforge** (`~/SWE/blender/rigforge`, GPL-2.0-or-later): Rigify 0.6.10 fork
  with HLL presets (`hll_hero`, `hll_stalker`), a deform-bones-only GLB
  export for the game (Bevy), and a deterministic landmark detector + metarig
  fitter. The production rigger.
- **skintokens** (`~/SWE/blender/skintokens`, local repo): SkinTokens /
  TokenRig (VAST-AI, MIT code and weights), UniRig's successor, on the
  M4's GPU (~35-55 s per character, skeleton and weights in one pass).
  The ML rigger since 2026-10-05: weightforge's ML candidate
  (`weights fix --skintokens`) and rigforge's joint-hint source
  (`skintokens joints`). On the genforge rehearsal Andras its weights
  score 47.9/100 raw and 61.6 after weightforge's fix, against UniRig's
  10.1 / 33.6 on the same mesh (`skintokens/docs/evaluation.md`); none
  pass the gate yet (shoulder and hip stretch at the extreme ROM poses).
- **unirig-mac** (`~/SWE/blender/unirig-mac`): retired 2026-10-05 in
  favor of skintokens; kept for reference, not deleted.

## Bake-off verdict (2026-09-30, `~/SWE/blender/rigforge/docs/bakeoff/final.md`)

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
  in the game (Bevy) on a target phone.
- **Weights are the real time sink** (rigforge TODO calls weight
  painting the highest-value item). Humanoids get weights from the base
  via wrapforge. For one-off monsters, a better binding method than
  Blender's heat weighting (which fails on messy meshes) is the tool
  worth building: geodesic voxel binding (Dionne & de Lasa 2013) for
  creatures and robust weight transfer with inpainting (Abdrashitov et
  al. 2023) for pieces.
- **License check**: UniRig and SkinTokens are MIT (code + weights), but
  both run a shape encoder derived from Michelangelo (GPL-3.0). Running
  them locally and shipping only the rigs is fine; distributing the tools
  would need GPL compliance (`skintokens/PROVENANCE.md`).
