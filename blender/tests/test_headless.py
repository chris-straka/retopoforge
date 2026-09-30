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

import json
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
        check(hasattr(bpy.ops.retopoforge, "generate_lods"), "lods operator registered")
        check(hasattr(bpy.ops.retopoforge, "bake_textures"), "bake operator registered")
        check(hasattr(bpy.ops.retopoforge, "reload_scripts"), "reload operator registered")
        check(hasattr(bpy.types, "RETOPOFORGE_PT_panel"), "panel registered")
        check(hasattr(bpy.context.scene, "retopoforge_recall"),
              "recall blob property registered")
        check(bpy.context.scene.retopoforge_params.lod_targets == "10000,5000,2000",
              "lod targets default")

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

        # --- per-object settings recall: fresh object (no blob entry yet),
        # so the distinctive values below survive to the CLI call and get
        # saved; a second run with changed scene params must restore them.
        bpy.ops.object.select_all(action="DESELECT")
        bpy.ops.mesh.primitive_cube_add(size=2.0)
        recall_obj = bpy.context.active_object
        for _ in range(2):
            bpy.ops.object.mode_set(mode="EDIT")
            bpy.ops.mesh.subdivide(number_cuts=3)
            bpy.ops.object.mode_set(mode="OBJECT")
        params = bpy.context.scene.retopoforge_params
        params.target_quads = 321
        params.model_type = "HARDSURFACE"
        params.sharp_edge = 45.0
        params.adaptivity = 0.25
        params.apply_modifiers = False
        params.lod_targets = "400,200,80"
        result = bpy.ops.retopoforge.remesh()
        check("FINISHED" in result, f"recall remesh finished (got {result})")
        blob = json.loads(bpy.context.scene.retopoforge_recall or "{}")
        entry = blob.get(recall_obj.name)
        check(isinstance(entry, dict), "recall blob has object entry")
        check(entry.get("target_quads") == 321, "recall blob target_quads")
        check(entry.get("model_type") == "HARDSURFACE", "recall blob model_type")
        check(abs(entry.get("sharp_edge", -1) - 45.0) < 1e-6,
              "recall blob sharp_edge")
        check(abs(entry.get("adaptivity", -1) - 0.25) < 1e-6,
              "recall blob adaptivity")
        check(entry.get("apply_modifiers") is False,
              "recall blob apply_modifiers")
        check(entry.get("lod_targets") == "400,200,80",
              "recall blob lod_targets")
        print("recall entry:", json.dumps(entry, sort_keys=True))

        params.target_quads = 50
        params.model_type = "ORGANIC"
        params.sharp_edge = 120.0
        params.adaptivity = 0.9
        params.apply_modifiers = True
        params.lod_targets = "1,2,3"
        # The remesh above left nothing selected (the importer selects its
        # temps, which are then removed), so re-select like a user would.
        recall_obj.select_set(True)
        bpy.context.view_layer.objects.active = recall_obj
        result = bpy.ops.retopoforge.remesh()
        check("FINISHED" in result, f"do-over remesh finished (got {result})")
        check(params.target_quads == 321, "recall restored target_quads")
        check(params.model_type == "HARDSURFACE", "recall restored model_type")
        check(abs(params.sharp_edge - 45.0) < 1e-6, "recall restored sharp_edge")
        check(abs(params.adaptivity - 0.25) < 1e-6, "recall restored adaptivity")
        check(params.apply_modifiers is False,
              "recall restored apply_modifiers")
        check(params.lod_targets == "400,200,80",
              "recall restored lod_targets")

        # --- Symmetry toggle: defaults off, cli_args maps toggle+plane to
        # the CLI --symmetry value, and a remesh with symmetry on finishes
        # (operator FINISHED means the subprocess exited 0 with the flag).
        check(params.symmetry_enabled is False, "symmetry defaults off")
        check(params.symmetry_plane == "AUTO", "symmetry plane defaults auto")
        off_args = params.cli_args("retopo", "in.obj", "out.obj")
        check("--symmetry" in off_args
              and off_args[off_args.index("--symmetry") + 1] == "off",
              "cli_args passes --symmetry off by default")
        bpy.ops.object.select_all(action="DESELECT")
        bpy.ops.mesh.primitive_cube_add(size=2.0)
        sym_obj = bpy.context.active_object
        for _ in range(2):
            bpy.ops.object.mode_set(mode="EDIT")
            bpy.ops.mesh.subdivide(number_cuts=3)
            bpy.ops.object.mode_set(mode="OBJECT")
        params.target_quads = 200
        params.symmetry_enabled = True
        params.symmetry_plane = "X"
        on_args = params.cli_args("retopo", "in.obj", "out.obj")
        check("--symmetry" in on_args
              and on_args[on_args.index("--symmetry") + 1] == "x",
              "cli_args passes --symmetry x when enabled")
        result = bpy.ops.retopoforge.remesh()
        check("FINISHED" in result, f"symmetry remesh finished (got {result})")
        sym_polys = len(sym_obj.data.polygons)
        check(sym_polys > 0, "symmetry result has polygons")
        blob = json.loads(bpy.context.scene.retopoforge_recall or "{}")
        sym_entry = blob.get(sym_obj.name)
        check(isinstance(sym_entry, dict), "recall blob has symmetry entry")
        check(sym_entry.get("symmetry_enabled") is True,
              "recall blob symmetry_enabled")
        check(sym_entry.get("symmetry_plane") == "X",
              "recall blob symmetry_plane")
        print("symmetry entry:", json.dumps(sym_entry, sort_keys=True))
        params.symmetry_enabled = False
        params.symmetry_plane = "AUTO"
        sym_obj.select_set(True)
        bpy.context.view_layer.objects.active = sym_obj
        result = bpy.ops.retopoforge.remesh()
        check("FINISHED" in result,
              f"symmetry do-over remesh finished (got {result})")
        check(params.symmetry_enabled is True,
              "recall restored symmetry_enabled")
        check(params.symmetry_plane == "X", "recall restored symmetry_plane")
        # Leave symmetry off so the LOD/bake sections below run unconstrained,
        # exactly as before this toggle existed.
        params.symmetry_enabled = False
        params.symmetry_plane = "AUTO"

        # --- Generate LODs: fresh object so rung counts are unaffected by
        # the remeshes above; only it may be selected (the operator runs on
        # every selected mesh object).
        bpy.ops.object.select_all(action="DESELECT")
        bpy.ops.mesh.primitive_cube_add(size=2.0)
        lod_obj = bpy.context.active_object
        for _ in range(2):
            bpy.ops.object.mode_set(mode="EDIT")
            bpy.ops.mesh.subdivide(number_cuts=3)
            bpy.ops.object.mode_set(mode="OBJECT")
        lod_obj.location = (4.0, 5.0, 6.0)
        lod_obj.rotation_euler = (0.1, 0.3, 0.2)
        lod_obj.scale = (2.0, 1.0, 1.5)
        bpy.context.view_layer.update()
        lod_base = lod_obj.name
        lod_matrix = lod_obj.matrix_world.copy()
        lod_mesh = lod_obj.data
        lod_colls = set(lod_obj.users_collection)
        print(f"lod source: {len(lod_mesh.polygons)} polys")

        params.lod_targets = "400,150,60"
        result = bpy.ops.retopoforge.generate_lods()
        check("FINISHED" in result, f"generate_lods finished (got {result})")
        rungs = [bpy.data.objects.get(f"{lod_base}_lod{i}") for i in range(3)]
        check(all(r is not None for r in rungs),
              f"3 rung objects ({[r.name if r else None for r in rungs]})")
        counts = [len(r.data.polygons) for r in rungs]
        print(f"rung counts: {counts}")
        check(all(c > 0 for c in counts), "every rung has polygons")
        check(counts[0] > counts[1] > counts[2],
              f"rung counts decrease ({counts})")
        check(lod_obj.data is lod_mesh, "lod source mesh untouched")
        for r in rungs:
            diff = sum(abs(a - b) for crow, brow in
                       zip(lod_matrix, r.matrix_world) for a, b in zip(crow, brow))
            check(diff < 1e-6, f"{r.name} transform matches source")
            check(set(r.users_collection) == lod_colls,
                  f"{r.name} is a collection sibling")
            check(not r.hide_viewport, f"{r.name} visible")
        check(lod_obj.select_get() and
              bpy.context.view_layer.objects.active == lod_obj,
              "source re-selected after lods")
        lod_report = bpy.context.scene.retopoforge_last_report
        check(all(f"{lod_base}_lod{i}" in lod_report for i in range(3)),
              "report lists all rungs")
        print("lod report:", lod_report.strip().replace("\n", " | "))

        # --- Bake assist: two-tone subdivided-cube high, remeshed low via
        # the existing remesh op; the diffuse bake must finish and land
        # non-uniform pixels in a PNG next to the (unsaved -> /tmp) blend.
        bpy.ops.object.select_all(action="DESELECT")
        bpy.ops.mesh.primitive_cube_add(size=2.0)
        high = bpy.context.active_object
        for _ in range(2):
            bpy.ops.object.mode_set(mode="EDIT")
            bpy.ops.mesh.subdivide(number_cuts=2)
            bpy.ops.object.mode_set(mode="OBJECT")
        mat_red = bpy.data.materials.new("BakeHighRed")
        mat_red.use_nodes = True
        (mat_red.node_tree.nodes["Principled BSDF"].inputs["Base Color"]
         .default_value) = (1.0, 0.0, 0.0, 1.0)
        mat_green = bpy.data.materials.new("BakeHighGreen")
        mat_green.use_nodes = True
        (mat_green.node_tree.nodes["Principled BSDF"].inputs["Base Color"]
         .default_value) = (0.0, 1.0, 0.0, 1.0)
        high.data.materials.append(mat_red)
        high.data.materials.append(mat_green)
        for poly in high.data.polygons:
            poly.material_index = 0 if poly.center.x < 0.0 else 1
        print(f"bake high: {len(high.data.polygons)} polys, "
              f"{len(high.data.materials)} mats")

        bpy.ops.object.select_all(action="DESELECT")
        bpy.ops.mesh.primitive_cube_add(size=2.0)
        low_src = bpy.context.active_object
        bpy.ops.object.mode_set(mode="EDIT")
        bpy.ops.mesh.subdivide(number_cuts=1)
        bpy.ops.object.mode_set(mode="OBJECT")
        params.target_quads = 100
        params.keep_original = False
        result = bpy.ops.retopoforge.remesh()
        check("FINISHED" in result, f"bake-low remesh finished (got {result})")
        low = bpy.data.objects[low_src.name]
        print(f"bake low: {len(low.data.polygons)} polys")

        low.select_set(True)
        high.select_set(True)
        bpy.context.view_layer.objects.active = high  # ACTIVE is HIGH
        params.bake_size = 256
        params.bake_normal = True
        result = bpy.ops.retopoforge.bake_textures()
        check("FINISHED" in result, f"bake finished (got {result})")
        check(len(low.data.uv_layers) > 0, "low got Smart-UV layers")
        diff_path = os.path.join("/tmp", f"{low.name}_diffuse.png")
        norm_path = os.path.join("/tmp", f"{low.name}_normal.png")
        check(os.path.isfile(diff_path), f"diffuse png saved ({diff_path})")
        check(os.path.isfile(norm_path), f"normal png saved ({norm_path})")
        check(diff_path in bpy.context.scene.retopoforge_last_report,
              "report names the diffuse path")
        check(bpy.context.view_layer.objects.active == high,
              "active=HIGH restored after bake")
        probe = bpy.data.images.load(diff_path)
        try:
            px = list(probe.pixels)
            check(len(px) > 0, "diffuse image has pixels")
            spread = max(px) - min(px)
            check(spread > 0.05, f"diffuse pixels non-uniform (spread {spread:.3f})")
        finally:
            bpy.data.images.remove(probe)
        os.remove(diff_path)
        os.remove(norm_path)

        leftovers = [o for o in bpy.data.objects if o.name.startswith("in_")]
        check(not leftovers, "no temp objects left behind")

        print("ALL HEADLESS TESTS PASSED")
    finally:
        bpy.ops.preferences.addon_disable(module="retopoforge")


main()
