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
  `Apply Modifiers` remeshes the evaluated mesh and clears the stack on
  the result (it is already baked in; a live Mirror would mirror the
  remesh again); `Keep Original` spawns a remeshed copy and hides the
  source instead of replacing its mesh.
- **Symmetry** passes `--symmetry` to the CLI (off by default). The plane
  selector picks `Auto` (detect the dominant plane) or pins `X`/`Y`/`Z`.
- **Generate LODs** remeshes each selected object once per comma-separated
  `LOD Targets` value (e.g. `10000,5000,2000`) and adopts the rungs as
  visible sibling objects named `<name>_lod0`, `<name>_lod1`, … — the
  source mesh and transform are never touched. One CLI call emits the
  whole chain (`--lods`).
- **Settings recall** remembers the exact panel values used per object
  (including LOD targets, symmetry, guides, sharp features, and density
  settings). Making a remeshed object active again restores them into
  the panel; objects never remeshed leave the panel as it is. Recall
  never fires on Remesh itself, so values you edit with the object
  still active are what the do-over uses. Remesh leaves its results
  selected and active, so a tweak-and-redo is one click. The blob lives
  on the scene and survives save/reload.

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
and the PBR maps land next to the blend file (the system temp dir when
the blend is unsaved): `<low>_diffuse.png` always, plus `<low>_normal.png`,
`<low>_roughness.png`, `<low>_metallic.png`, `<low>_ao.png`, and
`<low>_emission.png` per the panel toggles. Roughness/metallic/emission
bake only when the HIGH source actually uses those sockets (texture
link or off-default value) and note the skip otherwise; metallic
bakes through a temporary Metallic→Emission rewire of a HIGH
duplicate (Blender has no metallic bake type; the duplicate is
deleted afterwards). Set **Bake Cage** to a mesh to cast rays from a
cage instead of extrusion — it must match LOW's face count
(duplicate LOW and inflate it slightly). The scene's render engine,
device, sample count, and bake settings are restored afterwards.

Data maps (normal, roughness, metallic, AO) are baked as Non-Color, so
the PNGs hold raw values; AO renders with 64 samples (the other passes
are deterministic and take one). When LOW had no material, the bake
creates one and wires the maps into its Principled BSDF (normal via a
Normal Map node; AO stays an unlinked node), so LOW renders textured
right away; a material LOW already had only gains the image nodes.
Re-baking reuses the same images, nodes, and files.

**LOW UVs** picks how the bake target gets its UVs: **Smart UV**
(the default, as before) or **Unwrap + Pack** — a real unwrap
(angle-based or conformal), uniform texel density across islands,
packed with the panel margin. The report always notes the measured
density (px/unit at the bake size); set **Texel Density** to enforce
a target, applied as a uniform post-pack scale when it still fits
the 0-1 tile (bakes clip outside it), otherwise the pack fit stands
and the report says what the tile fits.

**Remesh + Bake All** chains the whole pipeline in one action: make
the HIGH-poly source the active object and hit it — the add-on
remeshes (keeping the original regardless of the panel toggle or
recalled settings), then Smart-UVs and bakes every PBR map to the
`_retopo` result (without a cage: none can match a mesh that did not
exist yet).

**Project HIGH UVs** skips the re-bake instead: it copies the HIGH
source's UVs onto the LOW mesh by nearest-point projection: every
LOW corner samples its own nearest HIGH point, kept inside the HIGH
UV island under the face's center so no LOW face straddles a seam
and the original seams survive. Only
faces within **Projection Range** (fraction of the HIGH bbox
diagonal, default 1%) take UVs; the rest keep theirs, and the
report counts both.

**Transfer HIGH Colors** does the same nearest-point projection for
the HIGH active vertex-color layer — the path for non-textured AI
outputs — writing a same-named face-corner color layer on LOW
(beyond-range faces keep the fill color).

## Install

1. Build the CLI: `cargo build --locked --release -p retopo` (binary at `rust/target/release/retopo`)
2. Zip the `blender/retopoforge/` directory (the folder containing
   `blender_manifest.toml`).
3. In Blender: *Edit → Preferences → Extensions → Install from Disk*,
   pick the zip, enable *RetopoForge*.
4. If the `retopo` binary is not on `PATH`, set its location in the
   add-on preferences (the panel shows whether it was found).

Requires Blender 4.2+ (uses the `wm.obj_export` / `wm.obj_import`
operators and the extensions platform). The headless test passes on
4.5 LTS and 5.0.

## Test headless

Uses the Blender app binary directly — the `~/.local/bin/blender` shim
has a broken Python environment and cannot run scripts:

```sh
/Applications/Blender.app/Contents/MacOS/Blender --background \
    --factory-startup --python blender/tests/test_headless.py
```

Without a Blender install, the `bpy` wheel (Blender as a Python module)
runs the same script: `python3.11 -m venv bpyenv &&
bpyenv/bin/pip install bpy==4.5.4` (or `bpy==5.0.1`), then
`bpyenv/bin/python blender/tests/test_headless.py`.

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
using its Python API. The Rust engine stays MIT: the extension talks to
it only as a subprocess over OBJ files — never linked, never imported.
