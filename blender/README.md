# RetopoForge Blender extension

Quad remeshing inside Blender, driven by the `retopo` CLI from this repo.
Select mesh objects, open the *RetopoForge* tab in the 3D Viewport sidebar,
tune the parameters, hit **Remesh Selected**.

How it works: each object is exported to a temp OBJ under the identity
transform (so the round-trip cannot move, rotate, or scale anything), the
CLI remeshes it, and the result is imported back onto the original object
in a single undo step. Temp files are removed afterwards. New topology
cannot carry UVs or vertex colors — the panel says so.

## Panel reference

- **Remesh Selected** runs the CLI on every selected mesh object
  (`target-quads`, `model-type`, `sharp-edge`, `smooth-normal`,
  `edge-scaling`, `adaptivity`, `anisotropy`, plus the toggles below).
  `Apply Modifiers` remeshes the evaluated mesh; `Keep Original` spawns
  a remeshed copy and hides the source instead of replacing its mesh.
- **Symmetry** passes `--symmetry` to the CLI (off by default). The plane
  selector picks `Auto` (detect the dominant plane) or pins `X`/`Y`/`Z`.
- **Generate LODs** remeshes each selected object once per comma-separated
  `LOD Targets` value (e.g. `10000,5000,2000`) and adopts the rungs as
  visible sibling objects named `<name>_lod0`, `<name>_lod1`, … — the
  source mesh and transform are never touched. One CLI call emits the
  whole chain (`--lods`).
- **Settings recall** remembers the exact panel values used per object
  (including LOD targets, symmetry, guides, sharp features, and density
  settings). The
  next remesh of the same object restores them first, so a do-over is
  one click; the blob lives on the scene and survives save/reload.

## Flow guides

Steer quad edge flow with strokes painted as an **edge selection**: in
Edit Mode, select the edges the flow should follow (eye/mouth loops,
panel lines), then enable **Flow Guides** and remesh — each target's
selection is traced into polylines and passed as the CLI's `--guides`
file. Points are the selection's local coordinates, so guides line up
with the identity-transform export by construction.

- **Export Guide Strokes** writes the active object's selection to a
  `--guides` file (one `x y z` point per line, blank lines separate
  polylines, `#` starts a comment) for inspection or CLI-side reuse.
- Guides on but nothing selected is a skip, not an error: that target
  remeshes unconstrained (multi-object runs with partial selections
  still finish).
- A remesh replaces the mesh, so the selection is gone afterwards:
  re-select edges before a guided do-over.

## Sharp features

Keep hard-surface edges crisp with **Edge > Mark Sharp**: in Edit Mode,
mark the edges that must survive (panel lines, chamfer borders, CAD
edges), then enable **Sharp Features** and remesh — each target's marked
edges are traced into polylines and passed as the CLI's `--features`
file (same format as `--guides`). Points are the marked edges' local
coordinates, so features line up with the identity-transform export by
construction.

- **Export Sharp Features** writes the active object's marked edges to
  a `--features` file (one `x y z` point per line, blank lines separate
  polylines, `#` starts a comment) for inspection or CLI-side reuse.
- Sharp features on but nothing marked is a skip, not an error: that
  target remeshes unconstrained (multi-object runs with partially
  marked selections still finish).
- A remesh replaces the mesh, so the marks are gone afterwards:
  re-mark edges before a featured do-over.

## Density masks

Drive local density from **weight paint**: paint a vertex group (1.0 =
dense, 0.0 = sparse), pick it in the **Density** panel, and remesh with
**Density Mask** on — weights map linearly onto the `Density Min` ..
`Density Max` multipliers (defaults `0.25` .. `4.0`; the CLI clamps
outside `0.25`–`4.0`) and are passed as the CLI's `--density` file, one
multiplier per vertex in mesh order.

- Vertices with no group assignment stay neutral at `1.0`, so painting
  a small dense region never sparsifies the unpainted rest.
- **Export Density Mask** writes the active object's group to a
  `--density` file (one multiplier per line, `#` starts a comment).
- A topology-changing modifier (e.g. Subdivision) under **Apply
  Modifiers** changes the exported vertex count, so the mask would no
  longer line up: the remesh cancels with an error naming both counts
  instead of sending a misaligned mask. Turn off Apply Modifiers or
  apply the modifier first.
- A remesh replaces the mesh, so the painted weights are gone
  afterwards: repaint before a masked do-over (a recalled group name
  that no longer exists remeshes unconstrained with a note, never an
  error).

## Bake assist

After remeshing, project the original detail back: select the LOW-poly
target, make the HIGH-poly source the **active** object, and hit **Bake
High to Low**. The low gets a Smart UV project, both bake under Cycles
on the CPU (selected-to-active, with the panel's size/extrusion/margin),
and `<low>_diffuse.png` plus `<low>_normal.png` land next to the blend
file (`/tmp` when the blend is unsaved). The scene's render engine and
sample count are restored afterwards.

## Install

1. Build the CLI: `cmake -S . -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/macos-llvm.cmake -DCMAKE_BUILD_TYPE=Release && cmake --build build`
2. Zip the `blender/retopoforge/` directory (the folder containing
   `blender_manifest.toml`).
3. In Blender: *Edit → Preferences → Extensions → Install from Disk*,
   pick the zip, enable *RetopoForge*.
4. If the `retopo` binary is not on `PATH`, set its location in the
   add-on preferences (the panel shows whether it was found).

Requires Blender 4.2+ (uses the `wm.obj_export` / `wm.obj_import`
operators and the extensions platform).

## Test headless

Uses the Blender app binary directly — the `~/.local/bin/blender` shim
has a broken Python environment and cannot run scripts:

```sh
/Applications/Blender.app/Contents/MacOS/Blender --background \
    --factory-startup --python blender/tests/test_headless.py
```

The test registers the extension, remeshes a transformed subdivided
cube, and asserts the mesh was replaced, is mostly quads, keeps the
object transform bit-exact, records a report, and leaves no temp
objects. It then covers settings recall, the symmetry toggle, guide
export plus guided remesh, density export plus masked remesh (with the
missing-group, empty-selection, and modifier-mismatch cancels), LOD
generation, and the high-to-low bake. It skips (exit 0) when no
`retopo` binary is available.

## Licensing note

This extension is GPL-3.0-or-later, as Blender requires of anything
using its Python API. The C++ engine stays MIT: the extension talks to
it only as a subprocess over OBJ files — never linked, never imported.
