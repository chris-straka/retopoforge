# retopoforge architecture

Date: 2026-09-30. Sources: `CMakeLists.txt`, `cli/CMakeLists.txt`,
`app/CMakeLists.txt`, `tests/CMakeLists.txt`, `cli/main.cpp`,
`core/*.cppm`, `app/*.cppm`, `blender/retopoforge/__init__.py`,
`blender/retopoforge/blender_manifest.toml`, `bench/run.py`,
`bench/profile.py`.

## The four pieces

| Piece | Directory | Target | Qt? | License |
|---|---|---|---|---|
| Engine | `core/` | `retopo_core` static library | no | MIT |
| CLI | `cli/` | `retopo` binary | no | MIT |
| Desktop app | `app/` | `retopoforge` Qt6 app (macOS bundle) | yes (Qt6) | MIT |
| Blender extension | `blender/retopoforge/` | extension v0.1.0, Blender 4.2+ | n/a (Python) | GPL-3.0-or-later |

Dependency rules:

- The engine depends on nothing else in the repo — only vendored
  `thirdparty/` code (Eigen, isotropicremesher, meshoptimizer), the
  system TBB install, and system libraries (Accelerate, zlib).
- The CLI and the desktop app link `retopo_core`. Neither the engine
  nor the CLI may touch Qt (`otool -L build/cli/retopo | grep -i qt`
  must print nothing).
- The Blender extension links against nothing: it drives the `retopo`
  binary as a subprocess over OBJ files. That subprocess boundary is
  also the license boundary — the GPL extension never links or imports
  the MIT engine.
- `bench/run.py`, `bench/profile.py`, and the five `tests/test_cli_*`
  suites treat the built CLI as a black box (binary path in, mesh out,
  exit code + report parsed).

Data flow:

```text
Blender mesh --wm.obj_export--> in_N.obj --retopo--> out_N.obj --wm.obj_import--> Blender mesh
bench/models/*.obj --retopo--> results JSON --check--> baseline verdict
```

## The `retopo.core.*` module graph (17 modules)

One named module per converted component. The interface lives in
`core/<name>.cppm`: copyright header, then `module;` plus third-party
and not-yet-converted includes (global fragment), then
`export module retopo.core.<name>;`, then the `export`ed declarations.
The implementation stays in `core/<name>.cpp`, starting with `module;`
+ includes and then `module <name>;`. Importers `import` exactly what
they use — there is no transitive reliance. CMake lists each `.cppm` in
the target's `FILE_SET CXX_MODULES`, and `CXX_SCAN_FOR_MODULES` is on
for every target with importers (plain `.cpp` files would otherwise
compile unscanned: no BMI flags, no build ordering).

Leaf-first, with interface-level imports:

- `retopo.core.double_utils` (`double.cppm`) — leaf: floating-point
  helpers.
- `retopo.core.progress` (`progress.cppm`) — leaf: the
  `ProgressHandler` callback type (`fraction` 0..1 plus step name;
  handlers must be thread-safe, islands remesh on TBB workers).
- `retopo.core.mesh_separator` (`meshseparator.cppm`) — no module
  imports in its interface: connected-component splitting.
- `retopo.core.constrained_least_squares`
  (`constrainedleastsquares.cppm`) — no module imports in its
  interface: constrained least-squares solver.
- `retopo.core.vector2` (`vector2.cppm`) — imports `double_utils`.
- `retopo.core.vector3` (`vector3.cppm`) — imports `double_utils`,
  `vector2`. The most-imported module in the tree (engine, CLI, app).
- `retopo.core.position_key` (`positionkey.cppm`) — imports `vector3`:
  spatial hashing of vertex positions.
- `retopo.core.surface_mesh` (`surfacemesh.cppm`) — imports `vector2`,
  `vector3`: the half-edge-style mesh container.
- `retopo.core.mixed_integer_least_squares`
  (`mixedintegerleastsquares.cppm`) — imports
  `constrained_least_squares`.
- `retopo.core.frame_field` (`framefield.cppm`) — imports
  `surface_mesh`, `vector3`.
- `retopo.core.singularity_simplifier` (`singularitysimplifier.cppm`) —
  imports `surface_mesh`, `vector3`.
- `retopo.core.symmetry` (`symmetry.cppm`) — imports `vector3`:
  mirror-plane detection/scoring plus frame-field and vertex
  symmetrization. Imported by the `parameterizer` interface and by the
  (unconverted) `autoremesher.h` orchestrator header.
- `retopo.core.guides` (`guides.cppm`) — imports `vector3`:
  guide-polyline proximity queries (influence radius, tangent lookup).
  Imported by the `frame_field` implementation unit, which locks
  near-guide faces to guide tangents as hard constraints in the
  sharp-edge solve.
- `retopo.core.parameterizer` (`parameterizer.cppm`) — imports
  `progress`, `symmetry`, `vector2`, `vector3`.
- `retopo.core.quad_parameterizer` (`quadparameterizer.cppm`) —
  imports `progress`, `vector2`, `vector3`.
- `retopo.core.quad_extractor` (`quadextractor.cppm`) — imports
  `progress`, `vector2`, `vector3`.
- `retopo.core.isotropic_remesher` (`isotropicremesher.cppm`) —
  imports `progress`, `vector3`.

Not yet converted (plain headers + implementation units, reached via
the `<AutoRemesher/...>` forwarders in `core/include/`):

- `core/autoremesher.h` / `core/autoremesher.cpp` — the pipeline
  orchestrator (`AutoRemesher::AutoRemesher`: target counts, model
  type, adaptivity/anisotropy, sharp/smooth angles, symmetry, `remesh()`).
  The header imports `progress`, `symmetry`, `vector2`, and `vector3`;
  the implementation unit imports `isotropic_remesher`,
  `mesh_separator`, `parameterizer`, `quad_extractor`, and `symmetry`.
- `core/objreader.h` / `core/objreader.cpp` — the OBJ loader
  (`loadObjPositionsAndTriangles`, with ear-clip triangulation of
  polygonal faces).

## The `retopo.app.*` module graph (10 modules)

Same one-module-per-component pattern for the converted plain (non-Qt)
app components in `app/*.cppm`:

- Leaves: `retopo.app.model_shader_vertex`
  (`modelshadervertex.cppm`), `retopo.app.monochrome_opengl_vertex`
  (`monochromeopenglvertex.cppm`), `retopo.app.opengl_buffer_util`
  (`openglbufferutil.cppm`), `retopo.app.model_shader_program`
  (`modelshaderprogram.cppm`), `retopo.app.monochrome_opengl_program`
  (`monochromeopenglprogram.cppm`), `retopo.app.theme` (`theme.cppm`),
  `retopo.app.util` (`util.cppm`).
- `retopo.app.model_shader_mesh` (`modelshadermesh.cppm`) — imports
  `model_shader_vertex` and `retopo.core.vector3`.
- `retopo.app.monochrome_opengl_object`
  (`monochromeopenglobject.cppm`) — imports
  `monochrome_opengl_vertex`.
- `retopo.app.model_shader_mesh_binder`
  (`modelshadermeshbinder.cppm`) — imports `model_shader_mesh`,
  `model_shader_program`, `monochrome_opengl_object`, and
  `monochrome_opengl_program`.

## Why `Q_OBJECT` widgets stay in headers

moc generates member-function definitions plus Qt includes that cannot
coexist inside module purview — a conversion pilot proved this, so the
rule in `app/CMakeLists.txt` is: `Q_OBJECT` widgets stay in headers,
only plain components become modules. The 14 widget/generator headers
(`mainwindow.h`, `graphicswidget.h`, `modelshaderwidget.h`,
`preferences.h`, the `*numberwidget.h` pair, log browser, spinners,
and the mesh generators) are therefore still Automoc-processed
headers, while everything listed above is a module.

One build-system consequence lives in the top-level `CMakeLists.txt`:
Automoc's unity file (`mocs_compilation.cpp`) includes moc outputs for
app headers that themselves import `retopo.core` / `retopo.app`
modules, but CMake compiles autogen sources with the unscanned rule
(no dyndep BMI flags). So the build hands that unity file explicit
`-fmodule-file=<module>=<path>.pcm` flags plus `OBJECT_DEPENDS` edges
on the finished `libretopo_core.a` and the app modules' own object
files. The edges deliberately never name a `.pcm` path (dyndep
produces PCMS with no static ninja rule, which breaks fresh-build
planning); when new app modules land, both lists must be extended.

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
`build/cli/retopo` relative to a retopoforge checkout. The sidebar
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
