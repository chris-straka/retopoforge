# RetopoForge Blender extension

Quad remeshing inside Blender, driven by the `retopo` CLI from this repo.
Select mesh objects, open the *RetopoForge* tab in the 3D Viewport sidebar,
tune the parameters, hit **Remesh Selected**.

How it works: each object is exported to a temp OBJ under the identity
transform (so the round-trip cannot move, rotate, or scale anything), the
CLI remeshes it, and the result is imported back onto the original object
in a single undo step. Temp files are removed afterwards. New topology
cannot carry UVs or vertex colors — the panel says so.

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
objects. It skips (exit 0) when no `retopo` binary is available.

## Licensing note

This extension is GPL-3.0-or-later, as Blender requires of anything
using its Python API. The C++ engine stays MIT: the extension talks to
it only as a subprocess over OBJ files — never linked, never imported.
