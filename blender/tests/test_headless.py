# Headless end-to-end test for the RetopoForge Blender extension.
#
# Run with the Blender app binary (NOT the ~/.local/bin/blender shim, whose
# Python environment is broken):
#
#   /Applications/Blender.app/Contents/MacOS/Blender --background \
#       --factory-startup --python blender/tests/test_headless.py
#
# Needs a built `retopo` CLI (rust/target/release/retopo in the repo, or on PATH).
# Exits 0 on pass or SKIP (binary missing), 1 on failure.

import json
import math
import os
import re
import sys
import tempfile

import bpy
from mathutils import Matrix

# Where the add-on writes bake PNGs for an unsaved blend.
TMP = tempfile.gettempdir()

REPO = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                     "..", ".."))
sys.path.insert(0, os.path.join(REPO, "blender"))

import retopoforge  # noqa: E402  (the extension package under test)


def check(condition, message):
    print(("PASS" if condition else "FAIL") + ": " + message)
    if not condition:
        raise SystemExit(1)


def check_cancel(label, func):
    """Negative-path operator call: passes on a cancel, whether Blender
    hands back {'CANCELLED'} or raises RuntimeError (an ERROR-reporting
    cancel raises through bpy.ops on Blender 5.x)."""
    try:
        result = func()
    except RuntimeError as exc:
        print(f"PASS: {label} cancels (raised: {exc})")
        return
    check("CANCELLED" in result, f"{label} cancels (got {result})")


def main():
    bpy.ops.preferences.addon_enable(module="retopoforge")
    try:
        check(hasattr(bpy.ops.retopoforge, "remesh"), "operator registered")
        check(hasattr(bpy.ops.retopoforge, "generate_lods"), "lods operator registered")
        check(hasattr(bpy.ops.retopoforge, "bake_textures"), "bake operator registered")
        check(hasattr(bpy.ops.retopoforge, "export_guides"),
              "guides export operator registered")
        check(hasattr(bpy.ops.retopoforge, "export_features"),
              "features export operator registered")
        check(hasattr(bpy.ops.retopoforge, "export_density"),
              "density export operator registered")
        check(hasattr(bpy.ops.retopoforge, "reload_scripts"), "reload operator registered")
        check(hasattr(bpy.types, "RETOPOFORGE_PT_panel"), "panel registered")
        check(hasattr(bpy.context.scene, "retopoforge_recall"),
              "recall blob property registered")
        check(bpy.context.scene.retopoforge_params.lod_targets == "10000,5000,2000",
              "lod targets default")
        check(bpy.context.scene.retopoforge_params.guides_enabled is False,
              "guides default off")
        check(bpy.context.scene.retopoforge_params.features_enabled is False,
              "features default off")
        check(bpy.context.scene.retopoforge_params.density_enabled is False,
              "density default off")
        check(abs(bpy.context.scene.retopoforge_params.density_min - 0.25) < 1e-9,
              "density min default")
        check(abs(bpy.context.scene.retopoforge_params.density_max - 4.0) < 1e-9,
              "density max default")

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

        # --- Orientation: a rotated directional mesh must keep its
        # world-space orientation (regression: OBJ axis conversion tipped
        # characters onto their backs while the object matrix stayed put).
        bpy.ops.object.select_all(action="DESELECT")
        bpy.ops.mesh.primitive_cone_add(radius1=0.3, depth=2.0)
        tall = bpy.context.active_object
        # Character-like fixture: mesh data lies along local +Y with its
        # base at the local origin (feet at origin), while the object
        # rotation stands it upright in world — the exact setup whose
        # remesh tipped over (bug 1) and then flipped upside down (bug 2).
        # Direct mesh transform (rotate, then translate), not edit-mode
        # transform operators: those need a 3D-view context and silently
        # no-op without one (bpy-as-a-module, some --background setups).
        tall.data.transform(Matrix.Translation((0.0, 1.0, 0.0))
                            @ Matrix.Rotation(math.pi / 2, 4, "X"))
        tall.rotation_euler = (math.pi / 2, 0, 0)
        bpy.context.view_layer.update()

        def world_extents(o):
            ws = [o.matrix_world @ v.co for v in o.data.vertices]
            xs = [c.x for c in ws]
            ys = [c.y for c in ws]
            zs = [c.z for c in ws]
            return (max(xs) - min(xs), max(ys) - min(ys),
                    max(zs) - min(zs))

        def world_centroid(o):
            ws = [o.matrix_world @ v.co for v in o.data.vertices]
            n = len(ws)
            return (sum(c.x for c in ws) / n, sum(c.y for c in ws) / n,
                    sum(c.z for c in ws) / n)

        before = world_extents(tall)
        cbx, cby, cbz = world_centroid(tall)
        check(before[2] > before[0] * 2 and before[2] > before[1] * 2
              and cbz > 0.5,
              f"fixture standing before remesh (ext {before[0]:.2f}, "
              f"{before[1]:.2f}, {before[2]:.2f} centroid z {cbz:.2f})")
        bpy.context.scene.retopoforge_params.target_quads = 200
        result = bpy.ops.retopoforge.remesh()
        check("FINISHED" in result, f"orient remesh finished (got {result})")
        after = world_extents(tall)
        check(after[2] > after[0] * 1.5 and after[2] > after[1] * 1.5,
              f"mesh still standing after remesh ({after[0]:.2f}, "
              f"{after[1]:.2f}, {after[2]:.2f})")
        # Extents alone cannot tell standing from upside-down (both are
        # Z-tall); the centroid sign catches the 180-degree flip.
        cx, cy, cz = world_centroid(tall)
        check(cz > 0.3 and abs(cx) < 0.5 and abs(cy) < 0.5,
              f"mesh head-up after remesh ({cx:.2f}, {cy:.2f}, {cz:.2f})")

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

        # --- Guide-stroke export: an edge ring on a fresh cube is traced
        # into polylines by the export operator; a remesh with guides on
        # passes --guides through (the CLI rejects malformed guides
        # files, so FINISHED proves both the flag and the format).
        bpy.ops.object.select_all(action="DESELECT")
        bpy.ops.mesh.primitive_cube_add(size=2.0)
        guide_obj = bpy.context.active_object
        bpy.ops.object.mode_set(mode="EDIT")
        bpy.ops.mesh.subdivide(number_cuts=2)
        bpy.ops.object.mode_set(mode="OBJECT")
        guide_mesh = guide_obj.data
        top_z = max(v.co.z for v in guide_mesh.vertices)
        for e in guide_mesh.edges:
            e.select = all(guide_mesh.vertices[i].co.z > top_z - 1e-6
                           for i in e.vertices)
        n_sel = sum(1 for e in guide_mesh.edges if e.select)
        check(n_sel >= 4, f"guide ring selection ({n_sel} edges)")
        guides_path = os.path.join(TMP, "retopoforge_test_guides.txt")
        if os.path.exists(guides_path):
            os.remove(guides_path)
        result = bpy.ops.retopoforge.export_guides(filepath=guides_path)
        check("FINISHED" in result, f"export_guides finished (got {result})")
        check(os.path.isfile(guides_path),
              f"guides file written ({guides_path})")
        polylines = [[]]
        bad_lines = []
        with open(guides_path, encoding="utf-8") as f:
            for line in f:
                stripped = line.split("#", 1)[0].strip()
                if not stripped:
                    if polylines[-1]:
                        polylines.append([])
                    continue
                parts = stripped.split()
                try:
                    point = [float(p) for p in parts]
                except ValueError:
                    point = []
                if len(point) != 3:
                    bad_lines.append(line.rstrip())
                else:
                    polylines[-1].append(point)
        polylines = [p for p in polylines if p]
        check(not bad_lines,
              f"every guide line parses as 'x y z' ({bad_lines[:2]})")
        check(len(polylines) >= 1,
              f"guides hold polylines ({len(polylines)})")
        check(all(len(p) >= 2 for p in polylines),
              "every guide polyline has 2+ points")
        n_points = sum(len(p) for p in polylines)
        check(n_points >= n_sel, "guide points cover the selection")
        vert_coords = {(v.co.x, v.co.y, v.co.z)
                       for v in guide_mesh.vertices}
        check(all(tuple(p) in vert_coords
                  for poly in polylines for p in poly),
              "guide points sit on mesh verts")
        print(f"guides: {len(polylines)} polylines, {n_points} points")

        params.target_quads = 200
        params.guides_enabled = True
        result = bpy.ops.retopoforge.remesh()
        check("FINISHED" in result, f"guides remesh finished (got {result})")
        check(len(guide_obj.data.polygons) > 0, "guides result has polygons")
        blob = json.loads(bpy.context.scene.retopoforge_recall or "{}")
        guide_entry = blob.get(guide_obj.name)
        check(isinstance(guide_entry, dict), "recall blob has guides entry")
        check(guide_entry.get("guides_enabled") is True,
              "recall blob guides_enabled")
        params.guides_enabled = False
        guide_obj.select_set(True)
        bpy.context.view_layer.objects.active = guide_obj
        result = bpy.ops.retopoforge.remesh()
        check("FINISHED" in result,
              f"guides do-over finished (got {result})")
        check(params.guides_enabled is True,
              "recall restored guides_enabled")
        params.guides_enabled = False
        os.remove(guides_path)

        # Exporting with no edge selection cancels cleanly (the do-over
        # above replaced the mesh, so this object now has no selection).
        bpy.ops.object.select_all(action="DESELECT")
        guide_obj.select_set(True)
        bpy.context.view_layer.objects.active = guide_obj
        empty_path = os.path.join(TMP, "retopoforge_test_guides_empty.txt")
        if os.path.exists(empty_path):
            os.remove(empty_path)
        check_cancel("guides export without selection",
                     lambda: bpy.ops.retopoforge.export_guides(
                         filepath=empty_path))
        check(not os.path.exists(empty_path), "no guides file on cancel")

        # --- Sharp-feature export: a sharp-marked edge ring on a fresh
        # cube is traced into polylines by the export operator; a remesh
        # with sharp features on passes --features through (the CLI
        # rejects malformed features files, so FINISHED proves both the
        # flag and the format).
        bpy.ops.object.select_all(action="DESELECT")
        bpy.ops.mesh.primitive_cube_add(size=2.0)
        sharp_obj = bpy.context.active_object
        bpy.ops.object.mode_set(mode="EDIT")
        bpy.ops.mesh.subdivide(number_cuts=2)
        bpy.ops.object.mode_set(mode="OBJECT")
        sharp_mesh = sharp_obj.data
        top_z = max(v.co.z for v in sharp_mesh.vertices)
        for e in sharp_mesh.edges:
            e.use_edge_sharp = all(sharp_mesh.vertices[i].co.z > top_z - 1e-6
                                   for i in e.vertices)
        n_sharp = sum(1 for e in sharp_mesh.edges if e.use_edge_sharp)
        check(n_sharp >= 4, f"sharp ring marking ({n_sharp} edges)")
        features_path = os.path.join(TMP, "retopoforge_test_features.txt")
        if os.path.exists(features_path):
            os.remove(features_path)
        result = bpy.ops.retopoforge.export_features(filepath=features_path)
        check("FINISHED" in result,
              f"export_features finished (got {result})")
        check(os.path.isfile(features_path),
              f"features file written ({features_path})")
        # Deterministic chain tracing: a second export of the same flags
        # is byte-identical.
        features_again = os.path.join(TMP, "retopoforge_test_features_again.txt")
        result = bpy.ops.retopoforge.export_features(filepath=features_again)
        check("FINISHED" in result, "second export_features finished")
        with open(features_path, "rb") as f, open(features_again, "rb") as g:
            check(f.read() == g.read(), "sharp export is deterministic")
        os.remove(features_again)
        polylines = [[]]
        bad_lines = []
        with open(features_path, encoding="utf-8") as f:
            for line in f:
                stripped = line.split("#", 1)[0].strip()
                if not stripped:
                    if polylines[-1]:
                        polylines.append([])
                    continue
                parts = stripped.split()
                try:
                    point = [float(p) for p in parts]
                except ValueError:
                    point = []
                if len(point) != 3:
                    bad_lines.append(line.rstrip())
                else:
                    polylines[-1].append(point)
        polylines = [p for p in polylines if p]
        check(not bad_lines,
              f"every features line parses as 'x y z' ({bad_lines[:2]})")
        check(len(polylines) >= 1,
              f"features hold polylines ({len(polylines)})")
        check(all(len(p) >= 2 for p in polylines),
              "every features polyline has 2+ points")
        n_points = sum(len(p) for p in polylines)
        check(n_points >= n_sharp, "features points cover the marking")
        vert_coords = {(v.co.x, v.co.y, v.co.z)
                       for v in sharp_mesh.vertices}
        check(all(tuple(p) in vert_coords
                  for poly in polylines for p in poly),
              "features points sit on mesh verts")
        print(f"features: {len(polylines)} polylines, {n_points} points")

        params.target_quads = 200
        params.features_enabled = True
        result = bpy.ops.retopoforge.remesh()
        check("FINISHED" in result,
              f"features remesh finished (got {result})")
        check(len(sharp_obj.data.polygons) > 0,
              "features result has polygons")
        blob = json.loads(bpy.context.scene.retopoforge_recall or "{}")
        feat_entry = blob.get(sharp_obj.name)
        check(isinstance(feat_entry, dict), "recall blob has features entry")
        check(feat_entry.get("features_enabled") is True,
              "recall blob features_enabled")
        params.features_enabled = False
        sharp_obj.select_set(True)
        bpy.context.view_layer.objects.active = sharp_obj
        result = bpy.ops.retopoforge.remesh()
        check("FINISHED" in result,
              f"features do-over finished (got {result})")
        check(params.features_enabled is True,
              "recall restored features_enabled")
        params.features_enabled = False
        os.remove(features_path)

        # Exporting with nothing marked cancels cleanly (the do-over
        # above replaced the mesh, so this object now has no sharp flags).
        bpy.ops.object.select_all(action="DESELECT")
        sharp_obj.select_set(True)
        bpy.context.view_layer.objects.active = sharp_obj
        empty_feat = os.path.join(TMP, "retopoforge_test_features_empty.txt")
        if os.path.exists(empty_feat):
            os.remove(empty_feat)
        check_cancel("features export without marking",
                     lambda: bpy.ops.retopoforge.export_features(
                         filepath=empty_feat))
        check(not os.path.exists(empty_feat), "no features file on cancel")

        # --- Density export: painted halves map onto min..max, one
        # unassigned vertex stays neutral, and the remesh passes
        # --density through (the CLI rejects count mismatches, so
        # FINISHED proves the mask lines up with the export).
        bpy.ops.object.select_all(action="DESELECT")
        bpy.ops.mesh.primitive_cube_add(size=2.0)
        dens_obj = bpy.context.active_object
        bpy.ops.object.mode_set(mode="EDIT")
        bpy.ops.mesh.subdivide(number_cuts=1)
        bpy.ops.object.mode_set(mode="OBJECT")
        dens_mesh = dens_obj.data
        n_verts = len(dens_mesh.vertices)
        dens_vg = dens_obj.vertex_groups.new(name="DensityTest")
        for v in dens_mesh.vertices:
            if v.index == 0:
                continue  # unassigned: must stay neutral 1.0
            dens_vg.add([v.index], 1.0 if v.co.z > 0.0 else 0.0, "REPLACE")
        params.density_enabled = True
        params.density_vertex_group = "DensityTest"
        params.density_min = 0.25
        params.density_max = 4.0
        dens_path = os.path.join(TMP, "retopoforge_test_density.txt")
        if os.path.exists(dens_path):
            os.remove(dens_path)
        result = bpy.ops.retopoforge.export_density(filepath=dens_path)
        check("FINISHED" in result, f"export_density finished (got {result})")
        check(os.path.isfile(dens_path),
              f"density file written ({dens_path})")
        values = []
        bad_values = []
        with open(dens_path, encoding="utf-8") as f:
            for line in f:
                stripped = line.split("#", 1)[0].strip()
                if stripped:
                    try:
                        values.append(float(stripped))
                    except ValueError:
                        bad_values.append(line.rstrip())
        check(not bad_values,
              f"every density line parses as a number ({bad_values[:2]})")
        check(len(values) == n_verts,
              f"density count matches verts ({len(values)}/{n_verts})")
        check(abs(values[0] - 1.0) < 1e-9, "unassigned vert stays neutral")
        check(all(abs(val - (4.0 if dens_mesh.vertices[i].co.z > 0.0
                             else 0.25)) < 1e-9
                  for i, val in enumerate(values) if i),
              "weights linear-map onto 0.25..4.0")

        params.target_quads = 200
        result = bpy.ops.retopoforge.remesh()
        check("FINISHED" in result, f"density remesh finished (got {result})")
        blob = json.loads(bpy.context.scene.retopoforge_recall or "{}")
        dens_entry = blob.get(dens_obj.name)
        check(isinstance(dens_entry, dict), "recall blob has density entry")
        check(dens_entry.get("density_enabled") is True,
              "recall blob density_enabled")
        check(dens_entry.get("density_vertex_group") == "DensityTest",
              "recall blob density group")
        check(abs(dens_entry.get("density_min", -1) - 0.25) < 1e-9,
              "recall blob density_min")
        check(abs(dens_entry.get("density_max", -1) - 4.0) < 1e-9,
              "recall blob density_max")
        params.density_enabled = False
        params.density_vertex_group = ""
        # The mesh swap drops vertex groups (weights lived on the old
        # topology); remove any survivor so the do-over deterministically
        # takes the missing-group skip path on every Blender version.
        stale = dens_obj.vertex_groups.get("DensityTest")
        if stale is not None:
            dens_obj.vertex_groups.remove(stale)
        dens_obj.select_set(True)
        bpy.context.view_layer.objects.active = dens_obj
        result = bpy.ops.retopoforge.remesh()
        check("FINISHED" in result,
              f"density do-over finished (got {result})")
        check(params.density_enabled is True,
              "recall restored density_enabled")
        check(params.density_vertex_group == "DensityTest",
              "recall restored density group")
        params.density_enabled = False
        params.density_vertex_group = ""
        os.remove(dens_path)

        # --- Authoring negatives: a missing group cancels the export; a
        # topology-changing modifier under Apply Modifiers cancels the
        # remesh instead of sending a misaligned mask.
        bpy.ops.object.select_all(action="DESELECT")
        dens_obj.select_set(True)
        bpy.context.view_layer.objects.active = dens_obj
        params.density_vertex_group = "NoSuchGroup"
        missing_path = os.path.join(TMP, "retopoforge_test_density_missing.txt")
        if os.path.exists(missing_path):
            os.remove(missing_path)
        check_cancel("density export with missing group",
                     lambda: bpy.ops.retopoforge.export_density(
                         filepath=missing_path))
        check(not os.path.exists(missing_path), "no density file on cancel")
        params.density_vertex_group = ""

        bpy.ops.object.select_all(action="DESELECT")
        bpy.ops.mesh.primitive_cube_add(size=2.0)
        mod_obj = bpy.context.active_object
        bpy.ops.object.mode_set(mode="EDIT")
        bpy.ops.mesh.subdivide(number_cuts=1)
        bpy.ops.object.mode_set(mode="OBJECT")
        mod_vg = mod_obj.vertex_groups.new(name="DensityMod")
        for v in mod_obj.data.vertices:
            mod_vg.add([v.index], 0.5, "REPLACE")
        mod_obj.modifiers.new("SubsurfGuard", "SUBSURF")
        mod_mesh_before = mod_obj.data
        params.target_quads = 200
        params.apply_modifiers = True
        params.density_enabled = True
        params.density_vertex_group = "DensityMod"
        check_cancel("density+subsurf remesh",
                     lambda: bpy.ops.retopoforge.remesh())
        check(mod_obj.data is mod_mesh_before, "cancelled remesh keeps mesh")
        params.apply_modifiers = False
        params.density_enabled = False
        params.density_vertex_group = ""

        # Apply Modifiers bakes the stack into the exported mesh, so the
        # result must not keep it live (a Mirror would mirror the mirrored
        # remesh again). Keep Original leaves the source's stack alone.
        bpy.ops.object.select_all(action="DESELECT")
        bpy.ops.mesh.primitive_cube_add(size=1.0)
        mirror_obj = bpy.context.active_object
        # Off-center mesh data, so the mirror adds a separate twin cube.
        mirror_obj.data.transform(Matrix.Translation((1.5, 0.0, 0.0)))
        mirror_obj.modifiers.new("MirrorX", "MIRROR")
        params.apply_modifiers = True
        params.keep_original = True
        params.target_quads = 100
        result = bpy.ops.retopoforge.remesh()
        check("FINISHED" in result, f"mirror remesh finished (got {result})")
        mirror_copy = bpy.data.objects.get(f"{mirror_obj.name}_retopo")
        check(mirror_copy is not None and len(mirror_copy.modifiers) == 0,
              "applied modifiers cleared on the remeshed copy")
        check(len(mirror_obj.modifiers) == 1,
              "keep-original source keeps its modifier stack")
        # Forget the run above (recall would restore keep_original=True).
        blob = json.loads(bpy.context.scene.retopoforge_recall)
        blob.pop(mirror_obj.name)
        bpy.context.scene.retopoforge_recall = json.dumps(blob)
        params.keep_original = False
        bpy.ops.object.select_all(action="DESELECT")
        mirror_obj.hide_viewport = False
        mirror_obj.select_set(True)
        bpy.context.view_layer.objects.active = mirror_obj
        result = bpy.ops.retopoforge.remesh()
        check("FINISHED" in result, f"mirror swap finished (got {result})")
        check(len(mirror_obj.modifiers) == 0,
              "applied modifiers cleared after an in-place remesh")
        params.apply_modifiers = False

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
        # non-uniform pixels in a PNG next to the (unsaved -> temp dir) blend.
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
        # Drop stale bake PNGs from a previously crashed run: factory
        # startup reuses object names, so litter would poison the
        # phase-A skip asserts (which check map files do NOT exist).
        for stale in ("diffuse", "normal", "roughness", "metallic", "ao",
                      "emission"):
            stale_path = os.path.join(TMP, f"{low.name}_{stale}.png")
            if os.path.isfile(stale_path):
                os.remove(stale_path)

        low.select_set(True)
        high.select_set(True)
        bpy.context.view_layer.objects.active = high  # ACTIVE is HIGH
        params.bake_size = 256
        params.bake_normal = True
        scene = bpy.context.scene
        engine_before = scene.render.engine
        samples_before = scene.cycles.samples
        extrusion_before = scene.render.bake.cage_extrusion
        result = bpy.ops.retopoforge.bake_textures()
        check("FINISHED" in result, f"bake finished (got {result})")
        check(scene.render.engine == engine_before
              and scene.cycles.samples == samples_before
              and scene.render.bake.cage_extrusion == extrusion_before,
              "bake restored engine, samples and extrusion")
        check(bpy.data.images[f"{low.name}_normal"]
              .colorspace_settings.name == "Non-Color",
              "normal map baked as Non-Color data")
        check(bpy.data.images[f"{low.name}_diffuse"]
              .colorspace_settings.name == "sRGB",
              "diffuse map baked as sRGB color")
        low_mat = low.data.materials[0]
        low_bsdf = next(n for n in low_mat.node_tree.nodes
                        if n.type == "BSDF_PRINCIPLED")
        base_links = low_bsdf.inputs["Base Color"].links
        check(len(base_links) == 1
              and base_links[0].from_node.image
              == bpy.data.images[f"{low.name}_diffuse"],
              "diffuse map wired into LOW's Base Color")
        normal_links = low_bsdf.inputs["Normal"].links
        check(len(normal_links) == 1
              and normal_links[0].from_node.type == "NORMAL_MAP",
              "normal map wired through a Normal Map node")
        check(len(low.data.uv_layers) > 0, "low got Smart-UV layers")
        diff_path = os.path.join(TMP, f"{low.name}_diffuse.png")
        norm_path = os.path.join(TMP, f"{low.name}_normal.png")
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
        # Full PBR: AO always bakes; roughness/metallic/emission skip
        # with a note when the HIGH source doesn't use those sockets.
        ao_path = os.path.join(TMP, f"{low.name}_ao.png")
        check(os.path.isfile(ao_path), f"ao png saved ({ao_path})")
        for skipped in ("roughness", "metallic", "emission"):
            skip_path = os.path.join(TMP, f"{low.name}_{skipped}.png")
            check(not os.path.isfile(skip_path),
                  f"{skipped} skipped without source maps ({skip_path})")
            check(f"{low.name}: {skipped} skipped" in
                  bpy.context.scene.retopoforge_last_report,
                  f"report notes the {skipped} skip")
        os.remove(diff_path)
        os.remove(norm_path)
        os.remove(ao_path)

        # Phase B: give the two HIGH mats distinct roughness / metallic /
        # emission so every PBR map bakes non-uniform pixels.
        red_bsdf = mat_red.node_tree.nodes["Principled BSDF"]
        green_bsdf = mat_green.node_tree.nodes["Principled BSDF"]
        red_bsdf.inputs["Roughness"].default_value = 0.2
        green_bsdf.inputs["Roughness"].default_value = 0.9
        red_bsdf.inputs["Metallic"].default_value = 0.0
        green_bsdf.inputs["Metallic"].default_value = 1.0
        red_bsdf.inputs["Emission Color"].default_value = (1.0, 0.0, 0.0, 1.0)
        red_bsdf.inputs["Emission Strength"].default_value = 2.0
        mats_before = len(bpy.data.materials)
        bpy.ops.object.select_all(action="DESELECT")
        low.select_set(True)
        high.select_set(True)
        bpy.context.view_layer.objects.active = high
        nodes_before = len(low.data.materials[0].node_tree.nodes)
        result = bpy.ops.retopoforge.bake_textures()
        check("FINISHED" in result, f"pbr bake finished (got {result})")
        check(not any(i.name.startswith(f"{low.name}_diffuse.")
                      for i in bpy.data.images),
              "re-bake reuses the map images (no .001 copies)")
        check(len(low.data.materials[0].node_tree.nodes)
              == nodes_before + 3,
              "re-bake adds only the three newly baked map nodes")
        check(bpy.data.images[f"{low.name}_metallic"]
              .colorspace_settings.name == "Non-Color",
              "metallic map baked as Non-Color data")
        for name in ("diffuse", "normal", "roughness", "metallic", "ao",
                     "emission"):
            pbr_path = os.path.join(TMP, f"{low.name}_{name}.png")
            check(os.path.isfile(pbr_path), f"{name} png saved ({pbr_path})")
        for name in ("roughness", "metallic", "emission"):
            pbr_path = os.path.join(TMP, f"{low.name}_{name}.png")
            probe = bpy.data.images.load(pbr_path)
            try:
                px = list(probe.pixels)
                spread = max(px) - min(px)
                check(spread > 0.05,
                      f"{name} pixels non-uniform (spread {spread:.3f})")
            finally:
                bpy.data.images.remove(probe)
            os.remove(pbr_path)
        for name in ("diffuse", "normal", "ao"):
            os.remove(os.path.join(TMP, f"{low.name}_{name}.png"))
        check(not any("_metallic_src" in o.name for o in bpy.data.objects),
              "metallic rewire duplicate deleted after bake")
        check(len(bpy.data.materials) == mats_before,
              "metallic rewire materials deleted after bake")

        # Phase C: cage bake — an inflated duplicate of LOW (Blender
        # requires the cage to match the active object's face count).
        # A wrong-topology cage must cancel with a clear error first.
        bpy.ops.object.select_all(action="DESELECT")
        bpy.ops.mesh.primitive_cube_add(size=2.4)
        bad_cage = bpy.context.active_object
        params.bake_cage = bad_cage
        bpy.ops.object.select_all(action="DESELECT")
        low.select_set(True)
        high.select_set(True)
        bpy.context.view_layer.objects.active = high
        check_cancel("wrong-topology cage", bpy.ops.retopoforge.bake_textures)
        cage = low.copy()
        cage.data = low.data.copy()
        cage.scale = (1.05, 1.05, 1.05)
        bpy.context.collection.objects.link(cage)
        params.bake_cage = cage
        bpy.ops.object.select_all(action="DESELECT")
        low.select_set(True)
        high.select_set(True)
        bpy.context.view_layer.objects.active = high
        result = bpy.ops.retopoforge.bake_textures()
        check("FINISHED" in result, f"cage bake finished (got {result})")
        check(os.path.isfile(diff_path), "cage bake saved diffuse png")
        check(not bpy.context.scene.render.bake.use_cage,
              "use_cage restored after bake")
        params.bake_cage = None
        for name in ("diffuse", "normal", "roughness", "metallic", "ao",
                     "emission"):
            p = os.path.join(TMP, f"{low.name}_{name}.png")
            if os.path.isfile(p):
                os.remove(p)

        # Phase D: Unwrap + Pack LOW UVs with texel-density control.
        params.bake_uv_mode = "UNWRAP"
        params.bake_texel_density = 0.0
        bpy.ops.object.select_all(action="DESELECT")
        low.select_set(True)
        high.select_set(True)
        bpy.context.view_layer.objects.active = high
        result = bpy.ops.retopoforge.bake_textures()
        check("FINISHED" in result, f"unwrap bake finished (got {result})")
        check(f"{low.name}: uv: unwrap+pack @" in
              bpy.context.scene.retopoforge_last_report,
              "report notes the unwrap+pack density")
        check(all(-1e-6 <= c <= 1.0 + 1e-6
                  for item in low.data.uv_layers.active.uv
                  for c in item.vector),
              "unwrapped UVs fit the 0-1 tile")
        check(os.path.isfile(diff_path), "unwrap bake saved diffuse png")
        # A feasible density target is enforced exactly; an absurd one
        # keeps the pack fit and says so.
        params.bake_texel_density = 8.0
        result = bpy.ops.retopoforge.bake_textures()
        check("FINISHED" in result, "density-target bake finished")
        check("target met" in bpy.context.scene.retopoforge_last_report,
              "report notes the met density target")
        uv_area, world_area = retopoforge._uv_world_areas(low)
        measured = params.bake_size * (uv_area / world_area) ** 0.5
        check(abs(measured - 8.0) < 0.5,
              f"enforced density ~8px/unit (got {measured:.2f})")
        params.bake_texel_density = 100000.0
        result = bpy.ops.retopoforge.bake_textures()
        check("FINISHED" in result, "absurd-target bake finished")
        check("exceeds the tile fit" in
              bpy.context.scene.retopoforge_last_report,
              "report notes the unmet density target")
        check(all(-1e-6 <= c <= 1.0 + 1e-6
                  for item in low.data.uv_layers.active.uv
                  for c in item.vector),
              "unmet-target UVs still fit the tile")
        params.bake_uv_mode = "SMART"
        params.bake_texel_density = 0.0
        for name in ("diffuse", "normal", "roughness", "metallic", "ao",
                     "emission"):
            p = os.path.join(TMP, f"{low.name}_{name}.png")
            if os.path.isfile(p):
                os.remove(p)

        # --- One-click end-to-end: fresh textured high, single action
        # remeshes it, then Smart-UVs + bakes every PBR map to the
        # result (keep_original forced for the chain, then restored).
        check(hasattr(bpy.ops.retopoforge, "remesh_and_bake"),
              "one-click operator registered")
        bpy.ops.object.select_all(action="DESELECT")
        bpy.ops.mesh.primitive_cube_add(size=2.0)
        high2 = bpy.context.active_object
        for _ in range(2):
            bpy.ops.object.mode_set(mode="EDIT")
            bpy.ops.mesh.subdivide(number_cuts=2)
            bpy.ops.object.mode_set(mode="OBJECT")
        mat_a = bpy.data.materials.new("OneClickA")
        mat_a.use_nodes = True
        bsdf_a = mat_a.node_tree.nodes["Principled BSDF"]
        bsdf_a.inputs["Base Color"].default_value = (1.0, 0.0, 0.0, 1.0)
        bsdf_a.inputs["Roughness"].default_value = 0.2
        bsdf_a.inputs["Emission Color"].default_value = (1.0, 0.0, 0.0, 1.0)
        bsdf_a.inputs["Emission Strength"].default_value = 2.0
        mat_b = bpy.data.materials.new("OneClickB")
        mat_b.use_nodes = True
        bsdf_b = mat_b.node_tree.nodes["Principled BSDF"]
        bsdf_b.inputs["Base Color"].default_value = (0.0, 1.0, 0.0, 1.0)
        bsdf_b.inputs["Roughness"].default_value = 0.9
        bsdf_b.inputs["Metallic"].default_value = 1.0
        high2.data.materials.append(mat_a)
        high2.data.materials.append(mat_b)
        for poly in high2.data.polygons:
            poly.material_index = 0 if poly.center.x < 0.0 else 1
        params.target_quads = 100
        params.keep_original = False
        params.bake_size = 256
        # A saved entry with keep_original off: the remesh leg's recall
        # must not turn the chain into an in-place swap of HIGH. A cage
        # left over from another bake cannot match the fresh LOW, so the
        # chain must bake without it.
        retopoforge.save_recall_entry(bpy.context, high2.name, params)
        high2_polys = len(high2.data.polygons)
        params.bake_cage = bad_cage
        bpy.ops.object.select_all(action="DESELECT")
        high2.select_set(True)
        bpy.context.view_layer.objects.active = high2
        result = bpy.ops.retopoforge.remesh_and_bake()
        check("FINISHED" in result, f"one-click finished (got {result})")
        check(len(high2.data.polygons) == high2_polys,
              "one-click left HIGH's mesh intact despite recall")
        recalled = json.loads(bpy.context.scene.retopoforge_recall)
        check(recalled[high2.name]["keep_original"] is False,
              "one-click did not save its forced keep_original")
        check(params.bake_cage == bad_cage, "one-click left the cage setting")
        params.bake_cage = None
        retopo = bpy.data.objects.get(f"{high2.name}_retopo")
        check(retopo is not None, "one-click produced a _retopo object")
        check(len(retopo.data.polygons) < len(high2.data.polygons),
              "one-click low is lighter than high")
        check(params.keep_original is False,
              "one-click restored keep_original")
        check(bpy.context.view_layer.objects.active == high2,
              "active=HIGH restored after one-click")
        for name in ("diffuse", "normal", "roughness", "metallic", "ao",
                     "emission"):
            oc_path = os.path.join(TMP, f"{retopo.name}_{name}.png")
            check(os.path.isfile(oc_path), f"one-click {name} png saved")
        probe = bpy.data.images.load(
            os.path.join(TMP, f"{retopo.name}_diffuse.png"))
        try:
            spread = max(probe.pixels) - min(probe.pixels)
            check(spread > 0.05,
                  f"one-click diffuse saw HIGH (spread {spread:.3f})")
        finally:
            bpy.data.images.remove(probe)
        report = bpy.context.scene.retopoforge_last_report
        check("quads" in report and "diffuse ->" in report,
              "one-click report has remesh + bake lines")
        for name in ("diffuse", "normal", "roughness", "metallic", "ao",
                     "emission"):
            os.remove(os.path.join(TMP, f"{retopo.name}_{name}.png"))

        # --- Direct UV projection: HIGH UVs copied onto the hugging
        # LOW per-face (seams survive); out-of-range faces keep UVs.
        check(hasattr(bpy.ops.retopoforge, "project_uvs"),
              "project-uvs operator registered")
        bpy.ops.object.select_all(action="DESELECT")
        high.select_set(True)
        bpy.context.view_layer.objects.active = high
        bpy.ops.object.mode_set(mode="EDIT")
        bpy.ops.mesh.select_all(action="SELECT")
        bpy.ops.uv.cube_project()
        bpy.ops.object.mode_set(mode="OBJECT")
        before = [tuple(v.vector) for v in low.data.uv_layers.active.uv]
        params.project_uv_max_dist = 0.05
        bpy.ops.object.select_all(action="DESELECT")
        low.select_set(True)
        high.select_set(True)
        bpy.context.view_layer.objects.active = high
        result = bpy.ops.retopoforge.project_uvs()
        check("FINISHED" in result, f"project uvs finished (got {result})")
        after = [tuple(v.vector) for v in low.data.uv_layers.active.uv]
        check(before != after, "projection rewrote LOW UVs")

        def uv_world_ratios(o):
            uv = o.data.uv_layers.active.uv
            ratios = []
            for poly in o.data.polygons:
                lv = [uv[li].vector for li in poly.loop_indices]
                area = sum(abs((b.x - lv[0].x) * (c.y - lv[0].y)
                               - (c.x - lv[0].x) * (b.y - lv[0].y)) / 2.0
                           for b, c in zip(lv[1:], lv[2:]))
                ratios.append(area / poly.area)
            return sorted(ratios)

        high_ratio = uv_world_ratios(high)[len(high.data.polygons) // 2]
        low_ratios = uv_world_ratios(low)
        low_ratio = low_ratios[len(low_ratios) // 2]
        # Each LOW corner samples its own nearest HIGH point; sampling one
        # HIGH face per LOW face clamps the corners and shrinks the UVs.
        check(abs(low_ratio / high_ratio - 1.0) < 0.2,
              f"projected UVs keep HIGH's texel scale "
              f"(median uv/world {low_ratio:.4f} vs {high_ratio:.4f})")
        check(all(-0.1 <= c <= 1.1 for uv in after for c in uv),
              "projected UVs stay near the tile")
        nfaces = len(low.data.polygons)
        m = re.search(r"projected UVs on (\d+)/(\d+) faces \((\d+) beyond",
                      bpy.context.scene.retopoforge_last_report)
        check(m is not None and int(m.group(2)) == nfaces,
              "report counts projected faces")
        projected_n = int(m.group(1)) if m is not None else -1
        check(projected_n / nfaces > 0.8,
              f"most faces projected ({projected_n}/{nfaces})")
        # Out of range: shove LOW far away, nothing projects, UVs kept.
        saved_matrix = low.matrix_world.copy()
        low.matrix_world.translation.x += 10.0
        bpy.context.view_layer.update()
        result = bpy.ops.retopoforge.project_uvs()
        check("FINISHED" in result, "out-of-range projection finished")
        check(f"projected UVs on 0/{nfaces} faces ({nfaces} beyond range)"
              in bpy.context.scene.retopoforge_last_report,
              "report counts all faces skipped")
        kept = [tuple(v.vector) for v in low.data.uv_layers.active.uv]
        check(kept == after, "out-of-range faces keep their UVs")
        low.matrix_world = saved_matrix
        # HIGH without UVs cancels cleanly (primitives ship default
        # UVs now, so strip them explicitly).
        bpy.ops.object.select_all(action="DESELECT")
        bpy.ops.mesh.primitive_cube_add(size=1.0)
        nouvs = bpy.context.active_object
        for layer in list(nouvs.data.uv_layers):
            nouvs.data.uv_layers.remove(layer)
        low.select_set(True)
        nouvs.select_set(True)
        bpy.context.view_layer.objects.active = nouvs
        check_cancel("uv-less HIGH", bpy.ops.retopoforge.project_uvs)

        # --- Vertex-color transfer: HIGH's active color layer lands on
        # the hugging LOW by nearest-point projection.
        check(hasattr(bpy.ops.retopoforge, "transfer_colors"),
              "transfer-colors operator registered")
        col = high.data.color_attributes.new("TestCol", "FLOAT_COLOR",
                                             "CORNER")
        for poly in high.data.polygons:
            c = (1.0, 0.0, 0.0, 1.0) if poly.center.x < 0.0 else \
                (0.0, 1.0, 0.0, 1.0)
            for li in poly.loop_indices:
                col.data[li].color = c
        high.data.color_attributes.active_color = col
        params.transfer_max_dist = 0.05
        bpy.ops.object.select_all(action="DESELECT")
        low.select_set(True)
        high.select_set(True)
        bpy.context.view_layer.objects.active = high
        result = bpy.ops.retopoforge.transfer_colors()
        check("FINISHED" in result, f"transfer finished (got {result})")
        low_col = low.data.color_attributes.get("TestCol")
        check(low_col is not None, "LOW gained the transferred layer")
        reds = [low_col.data[li].color[0]
                for poly in low.data.polygons for li in poly.loop_indices]
        check(max(reds) - min(reds) > 0.5,
              f"transferred colors non-uniform "
              f"(red spread {max(reds) - min(reds):.3f})")
        m = re.search(r"transferred colors on (\d+)/(\d+) faces",
                      bpy.context.scene.retopoforge_last_report)
        check(m is not None and int(m.group(2)) == nfaces,
              "report counts transferred faces")
        check(m is not None and int(m.group(1)) / nfaces > 0.8,
              "most faces transferred colors")
        # Far away: a fresh layer keeps the fill color everywhere.
        low.matrix_world.translation.x += 10.0
        bpy.context.view_layer.update()
        result = bpy.ops.retopoforge.transfer_colors()
        check("FINISHED" in result, "far transfer finished")
        check(f"transferred colors on 0/{nfaces} faces "
              f"({nfaces} beyond range)"
              in bpy.context.scene.retopoforge_last_report,
              "report counts far transfer all skipped")
        far_col = low.data.color_attributes.active_color
        reds = [far_col.data[li].color[0]
                for poly in low.data.polygons for li in poly.loop_indices]
        check(max(reds) - min(reds) == 0.0, "far transfer kept fill color")
        low.matrix_world = saved_matrix
        # HIGH without colors cancels cleanly.
        bpy.ops.object.select_all(action="DESELECT")
        bpy.ops.mesh.primitive_cube_add(size=1.0)
        nocol = bpy.context.active_object
        low.select_set(True)
        nocol.select_set(True)
        bpy.context.view_layer.objects.active = nocol
        check_cancel("color-less HIGH",
                     bpy.ops.retopoforge.transfer_colors)

        leftovers = [o for o in bpy.data.objects if o.name.startswith("in_")]
        check(not leftovers, "no temp objects left behind")

        print("ALL HEADLESS TESTS PASSED")
    finally:
        bpy.ops.preferences.addon_disable(module="retopoforge")


main()
