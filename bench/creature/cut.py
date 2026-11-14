"""Blender: the bench's rival cuts.

    blender -b --factory-startup -P bench/creature/cut.py -- \
        quadriflow|decimate|artist IN OUT.obj TRIS

quadriflow  Blender's QuadriFlow at TRIS/2 quads; the input is first
            decimated to max(6 x TRIS, 150k) triangles (as meshkit's
            clean_copy does), and quadriflow's mesh is laid back onto the
            source surface (nearest point), so its rounded shape does not
            count against it. A surface QuadriFlow refuses (open,
            non-manifold) gets Blender's voxel remesh first, its voxel
            sized so the shell has about that many faces.
decimate    Blender's collapse decimate to TRIS triangles
artist      IN is the artist's GLB: its mesh, welded, its quads recovered
            (consecutive triangle pairs; TRIS ignored)
"""
import sys

import bmesh
import bpy
from mathutils.bvhtree import BVHTree


def main():
    a = sys.argv[sys.argv.index("--") + 1:]
    how, src, out, tris = a[0], a[1], a[2], int(a[3])
    bpy.ops.wm.read_factory_settings(use_empty=True)
    if src.lower().endswith(".obj"):
        bpy.ops.wm.obj_import(filepath=src)
    else:
        bpy.ops.import_scene.gltf(filepath=src, merge_vertices=True)
    meshes = [o for o in bpy.context.scene.objects if o.type == "MESH"]
    bpy.ops.object.select_all(action="DESELECT")
    for o in meshes:
        o.select_set(True)
    bpy.context.view_layer.objects.active = meshes[0]
    if len(meshes) > 1:
        bpy.ops.object.join()
    ob = bpy.context.view_layer.objects.active
    bpy.context.view_layer.update()
    world = ob.matrix_world.copy()  # a node above (a turn, a scale) counts
    ob.parent = None
    ob.matrix_world = world
    ob.modifiers.clear()
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    n = len(ob.data.polygons)
    if how == "decimate":
        m = ob.modifiers.new("d", "DECIMATE")
        m.ratio = min(1.0, tris / max(1, sum(len(p.vertices) - 2 for p in ob.data.polygons)))
        bpy.ops.object.modifier_apply(modifier=m.name)
    elif how == "quadriflow":
        src_copy = ob.copy()
        src_copy.data = ob.data.copy()
        bpy.context.scene.collection.objects.link(src_copy)
        pre = max(6 * tris, 150000)
        if n > pre:
            m = ob.modifiers.new("d", "DECIMATE")
            m.ratio = pre / n
            bpy.ops.object.modifier_apply(modifier=m.name)
        bpy.ops.object.select_all(action="DESELECT")
        ob.select_set(True)
        bpy.context.view_layer.objects.active = ob
        # Blender refuses a mesh with any edge shorter than 1e-4 units
        # ("needs to be manifold"), which a dense one-unit creature always
        # has: work at 100 units across, then scale back.
        up = 100.0 / max(ob.dimensions)
        for v in ob.data.vertices:
            v.co *= up
        bm = bmesh.new()
        bm.from_mesh(ob.data)
        bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=2e-4)
        bm.to_mesh(ob.data)
        bm.free()
        ob.data.update()

        def qf():
            try:
                return "FINISHED" in bpy.ops.object.quadriflow_remesh(
                    mode="FACES", target_faces=tris // 2, use_mesh_symmetry=False, use_preserve_sharp=False,
                    use_preserve_boundary=False, smooth_normals=False, seed=0)
            except RuntimeError:
                return False
        if not qf():
            # QuadriFlow wants a closed, consistently wound surface: give it
            # Blender's voxel remesh of the source (0.25% of the largest side)
            # sized from the area so it lands near `pre` faces with no
            # decimation (a collapse would break the closed surface again)
            area = sum(p.area for p in ob.data.polygons)
            ob.data.remesh_voxel_size = max((area / pre) ** 0.5, 1e-4 * max(ob.dimensions))
            bpy.ops.object.voxel_remesh()
            print("CUT quadriflow: voxel shell first")
            if not qf():
                raise SystemExit("cut: quadriflow failed")
        for v in ob.data.vertices:
            v.co /= up
        sv = [v.co.copy() for v in src_copy.data.vertices]
        bvh = BVHTree.FromPolygons(sv, [tuple(p.vertices) for p in src_copy.data.polygons])
        for v in ob.data.vertices:
            hit = bvh.find_nearest(v.co)
            if hit[0] is not None:
                v.co = hit[0]
        bpy.data.objects.remove(src_copy, do_unlink=True)
    elif how == "artist":
        # one vertex a position (the GLB splits at UV seams and hard edges)
        bm = bmesh.new()
        bm.from_mesh(ob.data)
        bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-6 * max(ob.dimensions))
        # An exporter writes each quad as two triangles one after the other,
        # so the artist's quads are recovered exactly by pairing neighbours
        # in file order that share an edge (and are not folded more than 60
        # degrees); triangles the artist made stay triangles.
        import math
        bm.faces.ensure_lookup_table()
        join, i, n_faces = [], 0, len(bm.faces)
        while i + 1 < n_faces:
            f, g = bm.faces[i], bm.faces[i + 1]
            shared = [e for e in f.edges if e in g.edges]
            if len(f.verts) == 3 and len(g.verts) == 3 and len(shared) == 1 \
                    and f.normal.angle(g.normal, math.pi) < math.radians(60):
                join.append(shared[0])
                i += 2
            else:
                i += 1
        if join:
            bmesh.ops.dissolve_edges(bm, edges=join, use_verts=False)
        bm.to_mesh(ob.data)
        bm.free()
    else:
        raise SystemExit(f"cut: unknown {how}")
    for o in list(bpy.context.scene.objects):
        o.select_set(o is ob)
    bpy.ops.wm.obj_export(filepath=out, export_selected_objects=True, export_uv=False,
                          export_normals=False, export_materials=False)
    print("CUT", how, len(ob.data.polygons), "faces")


main()
