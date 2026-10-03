# PLAN: review findings and next steps

Written 2026-10-03 at the end of a review session (Claude Code). Read
`AGENTS.md` and `docs/direction.md` first; this file only records what
the review found and what is left.

## Done (on main via PR #2, or on branch `claude/wonderful-meitner-7wce5s`)

Blender extension (`blender/retopoforge/__init__.py`), all with
headless-test coverage (221 checks, green on Blender 5.0.1 + 4.5.4 LTS):

- 4.2-4.5 LTS could not remesh (`apply_transform` is a 5.0+ exporter
  keyword); now passed only when present.
- Remesh + Bake All could swap HIGH's mesh in place (recall overwrote
  forced keep_original); now a hidden `force_keep_original` operator
  property; the chained bake ignores a stale cage (`ignore_cage`).
- Apply Modifiers left the stack live on the result (Mirror applied
  twice); cleared now.
- Project HIGH UVs shrank UVs ~4x (one HIGH face per LOW face); now
  per-corner nearest point, kept inside the anchor's UV island.
- Bake: AO 1 sample -> 64, data maps Non-Color, all touched render
  settings restored, temp dir instead of `/tmp`, images/nodes reused,
  maps wired into a bake-created LOW material.
- Settings recall now fires when an object becomes active (depsgraph
  handler), never on Remesh, so panel edits stick for a do-over. Our
  own operators pause recall. Remesh leaves its results selected.

Engine (`rust/core`), all output-identical:

- `CoverageGrid::build` could expand one spike triangle's whole cell
  box (~1e12 cells) before the cap check: hang/OOM on folded-uv spike
  outputs. Now a per-tri span check bails to the exact scan first.
  Regression test: `spike_tri_bails_to_scan_before_expanding`.
- `coverage_gaps` (working-side coverage, every island and attempt) was
  an O(working verts x output tris) scan: ~25% of a 15k-quad armadillo
  remesh. It now walks the coverage grid with an exact nearest-distance
  query (exits with a full cell of slack, falls back to the scan past 6
  rings), so gaps are bitwise the scan's. Armadillo@15k 11.8s -> 9.6s,
  dragon@15k 22.5s -> 19.5s, OBJ outputs byte-identical. Test:
  `coverage_gaps_grid_matches_scan_bitwise`.
- The isotropic kernel printed one "Found repeated halfedge" line per
  duplicate on every run (dozens on the dragon, even with `--quiet`).
  Now counted, and summarized once per island only under `--verbose`.

Review verdicts (no action needed): the `auto_remesher.rs` /
`quad_extractor.rs` splits are pure moves; the clippy sweep, residual
route cleanup, power-of-two normalization, corner-mark `AlignU/AlignV`,
coverage retry chain and native decimator all read correct.

## Verify first next session

- The engine commit above was checked on Linux: full suite has the same
  11 platform-only failures with identical messages (see `AGENTS.md`),
  zero warnings, and byte-identical outputs on armadillo + dragon @15k.
  Confirm macOS CI (`development` workflow, `cargo test`) is green and
  `bench/run.py --check bench/baseline.json` passes on the Mac.

## Next steps (ordered)

1. Open a PR for the branch commits after PR #2 (recall-on-switch,
   engine coverage changes, this file) once macOS CI is green.
2. Blender, small:
   - Multiple LOW material slots: only slot 0 gets the bake image node,
     so Cycles errors "no active image" on multi-material LOWs. Add the
     map nodes to every slot's material (or merge slots for the bake).
   - Transfer HIGH Colors makes a new `.001` color layer on every run;
     consider reusing a same-named layer.
   - Project UVs reads HIGH's base mesh (modifiers ignored). Document it
     or project from the evaluated mesh, using its own polygon indices.
3. Engine, small:
   - `snap_decimator_input` snaps on a world-origin grid in f32, so it
     does nothing for meshes far from the origin (the f32 ulp exceeds
     the grid). Snapping relative to the bbox minimum would fix it, but
     it changes outputs, so goldens and baselines need a re-pin and a
     noise.py A/B.
   - `bench/rings.py` ignores negative (relative) OBJ indices.
4. Linux test goldens (low priority, owner is on macOS): ~11 binaries
   differ by last-ulp libm / NaN-sign. Either Linux goldens or
   ulp-tolerant oracles; skip unless Linux dev matters.
5. Engine direction work stays per `docs/direction.md` and
   `~/Games/tools/roadmap.md` section 2 (targeted fixes; the patch back
   end later as an optional `--backend patch`).
