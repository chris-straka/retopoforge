# retopoforge architecture

Date: 2026-10-01 (Rust-only tree; item-6 flip deleted `build.rs` +
`thirdparty/`). Sources: `rust/Cargo.toml`, `rust/core/src/lib.rs`,
`rust/cli/src/main.rs`, `blender/retopoforge/__init__.py`, `bench/run.py`.

## The three pieces

| Piece | Directory | Target | License |
|---|---|---|---|
| Engine | `rust/core/` + `rust/solvers/` | `retopo_core`, `retopo_solvers` crates | MIT |
| CLI | `rust/cli/` | `retopo` binary | MIT |
| Blender extension | `blender/retopoforge/` | extension v0.3.0, Blender 4.2+ | GPL-3.0-or-later |

(There is no desktop app: the upstream Qt shell was removed and Blender
is the UI. The original C++ engine was the differential oracle for the
1:1 port and has since been removed.)

Dependency rules:

- The engine depends on `faer` (linear algebra) only. Pre-decimation
  is the native Rust port (`retopo_core::decimator`); the vendored
  meshoptimizer, its FFI, and the `build.rs` cc step were deleted with
  the item-6 flip.
- The CLI depends on `retopo_core`; OBJ and GLB IO are std-only.
- The Blender extension links against nothing: it drives the `retopo`
  binary as a subprocess over OBJ files. That subprocess boundary is
  also the license boundary — the GPL extension never links or imports
  the MIT engine.
- `bench/*.py` and the CLI contract test (`rust/cli/tests/cli_contract.rs`)
  treat the built CLI as a black box (binary path in, mesh out, exit
  code + report parsed).

Data flow:

```text
Blender mesh --wm.obj_export--> in_N.obj --retopo--> out_N.obj --wm.obj_import--> Blender mesh
bench/models/*.obj --retopo--> results JSON --check--> baseline verdict
```

## Engine pipeline (`retopo_core`)

`auto_remesher` orchestrates, per input mesh:

1. Load + weld (`obj_reader` / `glb`, CLI side), split into islands
   (`mesh_separator`), optional native decimation (`decimator`) when an
   island exceeds 8x the target triangle count.
2. Isotropic remesh (`isotropic_remesher`, `iso_remesh_kernel`) to a
   sizing-adaptive working mesh.
3. Parameterize (`parameterizer`): cross field (`frame_field`, with
   `guides` / sharp features as constraints, optional `symmetry`),
   singularity cancellation (`singularity_simplifier`), density masks
   (`density`), then the seamless quad cover (`quad_parameterizer`,
   solved by `retopo_solvers::{constrained, mixed_integer}`; optional
   dipole insertion along density steps).
4. Quad extraction (`quad_extractor`, `position_key`) plus its topology
   cleanup passes, then island merge and optional UV atlas.

Shared helpers: `vector2`, `vector3`, `double_utils`, `surface_mesh`
(half-edge container), `progress` (thread-safe progress callbacks), and
`par` (deterministic scoped-thread data parallelism; islands also run
in parallel). Every stage is bit-deterministic run-to-run.

## The Blender temp-OBJ subprocess round-trip

`blender/retopoforge/__init__.py` (extension v0.1.0, Blender 4.2+) runs
this loop per selected mesh object, with temp files under a per-run
`tempfile.mkdtemp(prefix="retopoforge_")` directory that is removed
afterwards:

1. Export: the object is put under the identity transform (so local ==
   world and the round-trip cannot move, rotate, or scale anything)
   and `wm.obj_export` writes `in_<N>.obj` with local coordinates, no
   UVs, no normals, no materials, and optional modifier evaluation
   (the panel's *Apply Modifiers* toggle, default on). The original
   world matrix is saved first and restored on every path, including
   failures.
2. Remesh: the `retopo` binary runs as a subprocess with the seven
   panel parameters mapped 1:1 to CLI flags (`--target-quads`,
   `--edge-scaling`, `--sharp-edge`, `--smooth-normal`,
   `--adaptivity`, `--anisotropy`, `--model-type`).
3. Import: `wm.obj_import` reads `out_<N>.obj`; the imported mesh is
   renamed `<object>_remeshed` and swapped onto the original object in
   a single undo step (or, with *Keep Original*, onto a spawned
   `<object>_retopo` copy while the source is hidden). The temporary
   import objects are deleted.
4. Report: the CLI's `=== retopoforge Report ===` block is parsed for
   quads / non-quads / verts / seconds and appended to the scene's
   `retopoforge_last_report` string, which the panel displays.

The operator has two paths. `execute()` is synchronous — used headless
and by tests — with `capture_output=True`. `invoke()`/`modal()` keeps
the UI alive interactively: a progress bar, a `VIEW_3D` header line
showing current-object counts, and ESC to cancel (which kills the
child and restores every matrix). The modal path deliberately uses a
log file instead of pipes: the CLI prints megabytes of progress and
the modal poll loop never drains, so pipes would fill and wedge the
child in a classic deadlock; the transcript is parsed from the log at
completion.

Binary resolution (`find_retopo_binary`): the explicit *Retopo CLI*
path in the add-on preferences first, then `PATH`, then
`rust/target/{release,debug}/retopo` relative to a retopoforge
checkout. The sidebar
panel (`VIEW_3D` / `UI` region, *RetopoForge* tab) shows a
found/missing status box, the **Remesh Selected** button, the ten
parameters (seven CLI flags plus *Apply Modifiers*, *Keep Original*,
and the LOD rung field), the **Generate LODs** button, the Bake Assist
box, the last report, a UV/vertex-color-loss notice, and a
**Reload Scripts** button (the stock `script.reload` operator) so
extension updates apply without restarting Blender.

Two more operators share the temp-OBJ subprocess pattern.
`retopoforge.generate_lods` is synchronous (no modal loop): per
selected object it exports the identity OBJ, runs the CLI once with
`--lods` from the comma-separated rung field, and adopts each rung as
a `<object>_lod<N>` sibling in the source's collections with the
source's world matrix. Per-rung quad counts are parsed from the CLI's
`LOD N: ... quads=K non-quads=M` lines into the last-report string.
`retopoforge.bake_textures` (the Bake Assist box: size, extrusion,
margin, normal-map toggle) bakes high-to-low textures with the active
object as HIGH and the selected object as LOW.

Settings recall is a per-object JSON blob in the scene's
`retopoforge_recall` string, keyed by object name. Every successful
remesh or LOD run snapshots the panel parameters; the next run on the
same object restores them first (multi-object remesh recalls the
active object's entry), with an INFO report confirming the recall.
