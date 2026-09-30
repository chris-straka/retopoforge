# LOD rung strategy (desktop + mobile game)

Date: 2026-09-30. Source: `retopo --lods` chain measured on
`bench/models/armadillo.obj` (49,990 verts, 99,976 triangles).

## The chain

One CLI run emits every rung (input is loaded once, each rung remeshes
independently, outputs keep the `--output` extension — `.glb` chains
stay `.glb`, triangulated):

```bash
./build/cli/retopo --input bench/models/armadillo.obj \
    --output build/hero.obj --lods 20000,8000,3000,1000
# writes build/hero_lod0.obj ... build/hero_lod3.obj (~4 s total)
```

Measured result (render-tris = quads×2 + pentagon×3 + tri×1, counted
from the emitted faces; non-quads are almost all pentagons):

| Rung | Target quads | Quads | Non-quads | Verts | Render tris |
|---|---|---|---|---|---|
| lod0 | 20,000 | 18,393 | 18 | 18,418 | 36,832 |
| lod1 | 8,000 | 8,182 | 20 | 8,212 | 16,420 |
| lod2 | 3,000 | 2,370 | 14 | 2,388 | 4,772 |
| lod3 | 1,000 | 524 | 12 | 544 | 1,084 |

## Which rung serves which platform

| Platform | Close-up | Mid distance | Far / crowd |
|---|---|---|---|
| Desktop | lod0 (~37k tris) | lod1 (~16k tris) | lod2 (~5k tris) |
| Mobile | lod1 (~16k tris) | lod2 (~5k tris) | lod3 (~1k tris) |

Rationale: mobile GPUs budget roughly a quarter of the desktop
triangle spend per hero asset, so the mobile set is the desktop set
shifted one rung down — one chain serves both, no second bake. Desktop
never needs lod3 (sub-2k tris at hero scale shows silhouette breakup);
mobile never needs lod0 (37k tris buys nothing on a small screen).

Suggested switch distances (screen-height fraction of the asset's
bounding sphere, tune per scene): desktop lod0 → lod1 at 25%,
lod1 → lod2 at 10%; mobile lod1 → lod2 at 25%, lod2 → lod3 at 10%.

## Budget notes

- **Target quads are a target, not a promise.** Yield on this chain
  ran 52–102% of target and degrades at low counts (lod3 asked for
  1,000 quads, got 524). Pad low-rung targets ~2x, then verify the
  emitted counts — never ship the target numbers as budgets.
- **Count triangles, not quads.** Engines rasterize triangles; quads
  triangulate ×2 and the pentagon stragglers ×3. The render-tris
  column above is the budget column.
- **Regenerate, don't hand-tune.** If a rung misses its budget,
  adjust the `--lods` list and rerun the one command; the chain takes
  seconds on character-size meshes. Rung counts also drift by a few
  faces run to run (parallel scheduling — a rerun gave lod2 as
  2,376q+12nq vs 2,370q+14nq), so always verify the emitted files, not
  this table.

## Godot import-time auto-LOD vs hand chains

Godot 4 generates meshopt-simplified LOD variants per imported mesh when
`Meshes > Generate LODs` (`meshes/generate_lods`, default true) is on —
this is automatic decimation of whatever you import, independent of these
hand-authored chains. The two compose as follows:

- Import each chain rung as its own mesh and pick ONE LOD mechanism per
  asset: either drive rung switching yourself (load lod0/lod1/lod2 and
  swap by distance) with `Generate LODs` OFF, or import one rung and let
  Godot generate the rest. Don't stack both — auto-LOD on top of an
  already-low rung double-simplifies and wastes import time.
- Recommended split: characters use hand chains (auto-LOD is known to
  break skinned meshes — turn it off per-mesh for anything rigged);
  static environment props can use a single imported mesh with
  `Generate LODs` ON and skip hand chains entirely.
- OBJ imports as a bare mesh resource, not a scene: LOD generation
  needs Import As → Scene + Reimport. Prefer importing the `.glb`
  chain rungs (the `--lods` outputs keep the `--output` extension).
