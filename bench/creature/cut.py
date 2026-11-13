"""Blender: the bench's rival cuts.

    blender -b --factory-startup -P bench/creature/cut.py -- \
        quadriflow|decimate|artist IN OUT.obj TRIS

quadriflow  Blender's QuadriFlow at TRIS/2 quads; the input is first
            decimated to max(6 x TRIS, 150k) triangles (as meshkit's
            clean_copy does), and quadriflow's mesh is laid back onto the
            source surface (nearest point), so its rounded shape does not
            count against it. A surface QuadriFlow refuses (open,
            non-manifold) gets Blender's voxel remesh first.
decimate    Blender's collapse decimate to TRIS triangles
artist      IN is the artist's GLB: its mesh, welded, triangles paired back
            into quads (TRIS ignored)
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
    ob.parent = None
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
            ob.data.remesh_voxel_size = 0.0025 * max(ob.dimensions)
            bpy.ops.object.voxel_remesh()
            if len(ob.data.polygons) > pre:
                m = ob.modifiers.new("d", "DECIMATE")
                m.ratio = pre / len(ob.data.polygons)
                bpy.ops.object.modifier_apply(modifier=m.name)
            print("CUT quadriflow: voxel shell first")
            if not qf():
                raise SystemExit("cut: quadriflow failed")
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
        # an exported artist mesh is triangulated: pair its triangles back
        # into the quads they were (Blender's Tris to Quads, 40 deg limits)
        import math
        bmesh.ops.join_triangles(bm, faces=bm.faces, cmp_seam=False, cmp_sharp=False, cmp_uvs=False,
                                 cmp_vcols=False, cmp_materials=False,
                                 angle_face_threshold=math.radians(40), angle_shape_threshold=math.radians(40))
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
