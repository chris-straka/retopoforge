# Rigging strategy (HLL character pipeline)

Date: 2026-09-30. Goal: AI mesh -> clean quads (retopoforge) ->
rigged character -> HLL's `glb/` folders. Commercial use, Mac-only
hardware (M4 mini 16GB, no GPU rental). Owner's bar is good quality
output, not bit-identical numbers.

## Two tracks + comparison

- `rigforge` (`~/SWE/rigforge`): Rigify 0.6.10 fork, module renamed
  to coexist with bundled Rigify, GPL-2.0-or-later. Headless smoke
  green (159-bone metarig -> 706-bone rig). The quality/production
  path: runs everywhere Blender runs, human + animal templates.
- `unirig-mac` (`~/SWE/unirig-mac`): UniRig (SIGGRAPH 2025) port to
  Apple Silicon CPU. MIT code (LICENSE file) + MIT weights
  (HuggingFace page) — commercial OK, self-hostable, no Tripo
  account needed. Checkpoints: skeleton ~1-2GB, skin ~4.4GB, run
  sequentially, fit 16GB. No custom CUDA kernels; four library
  swaps, of which only the sparse-geometry one is real work.
  Parity gate: reproduce `examples/` outputs on the Mac.
- Comparison: same HLL hero + creature through both tracks (plus
  the Mixamo free service as baseline); judge joint placement,
  weight quality, tweak time, Godot import cleanliness. Winner
  decides the base. Possible endgame: auto-placement feeding
  hand-grade output, beating both parents.

## License findings (verified unless noted)

- UniRig: MIT code + MIT weights. Caveat: these are the published
  research weights; Tripo's live product may run newer internal
  models — self-hosting gives you the paper's models, not their
  latest.
- RigAnything (direct UniRig rival): Adobe non-commercial. Out.
- AniGen (same lab, image straight to rigged character): MIT source
  but bundles non-commercial NVIDIA code (stated in its README).
  Watch, don't fork.
- Make-It-Animatable: license contradicts itself between pages, and
  humanoid-only. Out until resolved.
- Anymate: reported Apache-2.0 by two sources, LICENSE file not
  yet verified. Possible third contender.

## Extension plan

Separate Blender addons during the comparison: merging now would
prejudge the contest and bloat the working remesh tool. End-to-end
"mesh -> rigged" comes later as orchestration over the winner.

## Rust note

Owner prefers Rust. Greenfield Rust (headless rig CLI, model
inference) is favored over porting proven engine math; the C++
engine stays until/unless a quality-gated rewrite is justified.
Correction: faer has real sparse support now (formats + sparse
Cholesky/LU) — the earlier "thin at sparse" claim was stale. The
remaining Eigen gap is breadth + battle-testing, and any engine
port is a quality re-verification job regardless of language.
