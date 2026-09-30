# Headless end-to-end test for the RetopoForge Blender extension.
#
# Run with the Blender app binary (NOT the ~/.local/bin/blender shim, whose
# Python environment is broken):
#
#   /Applications/Blender.app/Contents/MacOS/Blender --background \
#       --factory-startup --python blender/tests/test_headless.py
#
# Needs a built `retopo` CLI (build/cli/retopo in the repo, or on PATH).
# Exits 0 on pass or SKIP (binary missing), 1 on failure.

import os
import sys

import bpy

REPO = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                     "..", ".."))
sys.path.insert(0, os.path.join(REPO, "blender"))

import retopoforge  # noqa: E402  (the extension package under test)


def check(condition, message):
    print(("PASS" if condition else "FAIL") + ": " + message)
    if not condition:
        raise SystemExit(1)


def main():
    bpy.ops.preferences.addon_enable(module="retopoforge")
    try:
        check(hasattr(bpy.ops.retopoforge, "remesh"), "operator registered")
        check(hasattr(bpy.types, "RETOPOFORGE_PT_panel"), "panel registered")

        binary = retopoforge.find_retopo_binary(
            bpy.context.preferences.addons["retopoforge"].preferences.retopo_binary)
        if not binary:
            print("SKIP: no retopo binary (build the repo first)")
            return
        print("retopo binary:", binary)

        # Test mesh: subdivided cube with a transform applied, so the
        # round-trip must preserve the object matrix exactly.
        bpy.ops.mesh.primitive_cube_add(size=2.0)
        obj = bpy.context.active_object
        for _ in range(2):
            bpy.ops.object.mode_set(mode="EDIT")
            bpy.ops.mesh.subdivide(number_cuts=3)
            bpy.ops.object.mode_set(mode="OBJECT")
        obj.location = (1.0, 2.0, 3.0)
        obj.rotation_euler = (0.4, 0.2, 0.1)
        obj.scale = (1.0, 2.0, 0.5)
        bpy.context.view_layer.update()
        matrix_before = obj.matrix_world.copy()
        polys_before = len(obj.data.polygons)
        mesh_before = obj.data
        print(f"before: {polys_before} polys")

        bpy.context.scene.retopoforge_params.target_quads = 200
        result = bpy.ops.retopoforge.remesh()
        check("FINISHED" in result, f"operator finished (got {result})")

        check(obj.data is not mesh_before, "mesh data replaced")
        polys_after = len(obj.data.polygons)
        print(f"after: {polys_after} polys")
        check(polys_after > 0, "result has polygons")
        check(polys_after < polys_before, "remesh simplified the mesh")
        quads = sum(1 for p in obj.data.polygons if len(p.vertices) == 4)
        check(quads / polys_after > 0.9,
              f"mostly quads ({quads}/{polys_after})")
        diff = sum(abs(a - b) for crow, brow in
                   zip(matrix_before, obj.matrix_world) for a, b in zip(crow, brow))
        check(diff < 1e-6, f"object transform preserved (drift {diff:.2e})")

        report = bpy.context.scene.retopoforge_last_report
        check("quads" in report.lower(), "report recorded")
        print("report:", report.strip().replace("\n", " | "))

        leftovers = [o for o in bpy.data.objects if o.name.startswith("in_")]
        check(not leftovers, "no temp objects left behind")

        print("ALL HEADLESS TESTS PASSED")
    finally:
        bpy.ops.preferences.addon_disable(module="retopoforge")


main()
