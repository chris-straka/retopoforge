# Face-flow iterate loop (animation-grade topology workflow)

Date: 2026-09-30. Honest scope, repeated from TODO: no automatic
remesher emits animator-grade face topology. The engine gets you ~80%
of the way; this loop plus fast manual cleanup covers the rest. The
goal is clean eye/mouth loops that deform well, not a one-click face.

## The loop (all in the Blender panel)

1. **Preset + budget.** Organic preset, target quads for the head's
   share of the asset budget (a 2k-quad head inside a 10k body is a
   sane start — see `docs/lod-strategy.md`). Remesh, inspect the loops.
2. **Sharp angle.** Lower `--sharp-edge` toward 30° if the lips/nose
   bridge wash out; raise it if noise turns into false creases.
3. **Adaptivity.** Raise toward 1.0 to pull quads into high-curvature
   zones (nostrils, eyelids); lower toward 0 if density bunches up.
4. **Head-only passes.** Select the head mesh (or split it), iterate
   there at low cost, then run the full body once with the winning
   settings. Per-object settings recall makes do-overs one click.
5. **Flow guides.** Select flow-stroke edges in Edit Mode (rings around
   the eyes/mouth, radial lines on the cheeks), enable Flow Guides,
   remesh. Quad edges follow the strokes; sharp-marked creases win ties.
6. **Density mask.** Weight-paint the face/hands, export as the density
   mask (min 0.25 / max 4.0 defaults). Mild masks realize nearly fully;
   note strong localized refinement saturates (~2.3x for 4x asks), so
   prefer a broad 2x face mask over a hard 4x one.
7. **Symmetry.** Characters are symmetric: enable it (auto-detect) so
   both eye loops match exactly. Asymmetric sculpts fall back to
   unconstrained output with a stderr note.

## Manual cleanup (RetopoFlow or vanilla Blender)

- Fix the 2–3 worst poles (typically nose tip, chin, ear backs) by hand:
  dissolve/spin edges until loops run clean around the mouth and eyes.
- Check deformation early: a quick jaw-open + blink shape key (or two
  bones) shows looping problems faster than staring at wireframes.
- Re-run Bake Assist after cleanup to refresh the low-poly textures.

## What not to do

- Don't chase exact quad counts run to run (parallel scheduling drifts
  a few faces); verify emitted files, not targets.
- Don't stack hand LOD chains with Godot import auto-LOD on the same
  asset (see the Godot section in `docs/lod-strategy.md`).
- Don't mark full closed cages as sharp features on coarse parts —
  crossing sharps at corners over-constrain and distort; use disjoint
  strokes plus small corner-mark radii.
