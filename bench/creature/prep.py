"""Blender: one clean source mesh -> the bench's inputs for a subject.

    blender -b --factory-startup -P bench/creature/prep.py -- CLEAN.glb OUT_DIR
        [--ref-tris 18000] [--subdivide N] [--fuse F]

CLEAN.glb is a single mesh already facing forward and standing on the
ground (motionforge `tools/tripo_to_glb.py` output, or an artist animal's
`bare.glb`). Writes:

  OUT_DIR/dense.obj    the source, welded by position (GLB splits
                       vertices at UV seams), triangulated: every tool's
                       input and the surface the scores measure against
  OUT_DIR/ref_cut.glb  the source decimated to --ref-tris: the mesh the
                       reference rig is made on (only when --ref-tris > 0)

--subdivide N welds a low-poly artist mesh and smooths it N Catmull-Clark
levels first. --fuse F then voxel-remeshes it at F x its largest side
(parts fused, triangulated, at most 300k triangles): a pro mesh turned
into what a generator gives.

Files keep glTF's frame (+Y up) so the OBJ, the GLBs and every tool's
output share one frame: Blender's default OBJ/GLB importers and exporters
are used throughout the bench.
"""
import os
import sys

import bpy
import bmesh


def args():
    a = sys.argv[sys.argv.index("--") + 1:]
    o = {"ref-tris": "18000", "subdivide": "0", "fuse": "0"}
    pos = []
    i = 0
    while i < len(a):
        if a[i].startswith("--"):
            o[a[i][2:]] = a[i + 1]
            i += 2
        else:
            pos.append(a[i])
            i += 1
    if len(pos) != 2:
        raise SystemExit(__doc__)
    return pos[0], pos[1], int(o["ref-tris"]), int(o["subdivide"]), float(o["fuse"])


def only_mesh():
    meshes = [o for o in bpy.context.scene.objects if o.type == "MESH"]
    if not meshes:
        raise SystemExit("prep: no mesh in the input")
    bpy.ops.object.select_all(action="DESELECT")
    for o in meshes:
        o.select_set(True)
    bpy.context.view_layer.objects.active = meshes[0]
    if len(meshes) > 1:
        bpy.ops.object.join()
    ob = bpy.context.view_layer.objects.active
    # world transform baked in, armature (if any) dropped
    ob.parent = None
    ob.modifiers.clear()
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    for o in list(bpy.context.scene.objects):
        if o is not ob:
            bpy.data.objects.remove(o, do_unlink=True)
    return ob


def weld(ob):
    bm = bmesh.new()
    bm.from_mesh(ob.data)
    diag = max(ob.dimensions)
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-6 * diag)
    bmesh.ops.triangulate(bm, faces=bm.faces)
    bm.to_mesh(ob.data)
    bm.free()
    ob.vertex_groups.clear()


def main():
    src, out, ref_tris, subdivide, fuse = args()
    os.makedirs(out, exist_ok=True)
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.import_scene.gltf(filepath=src)
    ob = only_mesh()
    if subdivide:
        # a low-poly artist mesh as a dense, smooth source: weld, then
        # Catmull-Clark (the artist's own cut is scored apart, as `artist`)
        bm = bmesh.new()
        bm.from_mesh(ob.data)
        bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-6 * max(ob.dimensions))
        bm.to_mesh(ob.data)
        bm.free()
        m = ob.modifiers.new("sub", "SUBSURF")
        m.levels = subdivide
        bpy.ops.object.modifier_apply(modifier=m.name)
    if fuse > 0:
        # what a generator gives: one voxel surface, touching parts melted
        # together, thin gaps closed, the artist's edge flow gone
        bpy.ops.object.select_all(action="DESELECT")
        ob.select_set(True)
        bpy.context.view_layer.objects.active = ob
        ob.data.remesh_voxel_size = fuse * max(ob.dimensions)
        bpy.ops.object.voxel_remesh()
        weld(ob)
        if len(ob.data.polygons) > 300000:
            m = ob.modifiers.new("cap", "DECIMATE")
            m.ratio = 300000 / len(ob.data.polygons)
            bpy.ops.object.modifier_apply(modifier=m.name)
    weld(ob)
    bpy.ops.object.select_all(action="DESELECT")
    ob.select_set(True)
    bpy.ops.wm.obj_export(filepath=os.path.join(out, "dense.obj"), export_selected_objects=True,
                          export_uv=False, export_normals=False, export_materials=False,
                          export_triangulated_mesh=True)
    print("PREP dense", len(ob.data.vertices), "verts", len(ob.data.polygons), "tris")
    if ref_tris > 0:
        tris = len(ob.data.polygons)
        if tris > ref_tris:
            m = ob.modifiers.new("cut", "DECIMATE")
            m.ratio = ref_tris / tris
            bpy.ops.object.modifier_apply(modifier=m.name)
        bpy.ops.export_scene.gltf(filepath=os.path.join(out, "ref_cut.glb"), use_selection=True,
                                  export_format="GLB", export_materials="NONE")
        print("PREP ref_cut", len(ob.data.polygons), "tris")


main()
