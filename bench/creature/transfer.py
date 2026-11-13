"""Blender: put a reference rig on a candidate mesh, the same way for every
candidate, so only the topology differs between them.

    blender -b --factory-startup -P bench/creature/transfer.py -- \
        REF_RIGGED.glb REF_CUT.glb CANDIDATE.(obj|glb) OUT.glb

REF_RIGGED.glb is the rig the route made on REF_CUT.glb (same vertices,
maybe turned or scaled by the route); it is moved back into REF_CUT's
frame (the bench frame) first. The candidate's skin weights are the
reference mesh's, interpolated from the nearest point on its surface
(Blender Data Transfer, nearest face interpolated), limited to 4 bones
and normalized. OUT.glb holds the candidate, the armature and its rest
pose and nothing else, in the bench frame.
"""
import sys

import bpy
import numpy as np
from mathutils import Matrix
from mathutils.bvhtree import BVHTree


def args():
    a = sys.argv[sys.argv.index("--") + 1:]
    if len(a) != 4:
        raise SystemExit(__doc__)
    return a


def imported(fn):
    before = set(bpy.data.objects)
    fn()
    return [o for o in bpy.data.objects if o not in before]


def world_verts(ob):
    m = np.array(ob.matrix_world)
    v = np.empty(len(ob.data.vertices) * 3)
    ob.data.vertices.foreach_get("co", v)
    v = v.reshape(-1, 3)
    return v @ m[:3, :3].T + m[:3, 3]


def similarity(src, dst):
    """Umeyama: s, R, t with dst ~ s R src + t."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    a, b = src - mu_s, dst - mu_d
    u, sig, vt = np.linalg.svd(b.T @ a / len(src))
    d = np.ones(3)
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        d[2] = -1
    r = u @ np.diag(d) @ vt
    s = (sig * d).sum() / (a ** 2).sum() * len(src)
    return s, r, mu_d - s * r @ mu_s


def bvh_of(ob):
    v = world_verts(ob)
    polys = [tuple(p.vertices) for p in ob.data.polygons]
    return BVHTree.FromPolygons([tuple(x) for x in v], polys)


def mean_gap(points, bvh):
    d = [bvh.find_nearest(tuple(p))[3] for p in points[:: max(1, len(points) // 2000)]]
    return float(np.mean([x for x in d if x is not None]))


def main():
    ref_rigged, ref_cut, cand, out = args()
    bpy.ops.wm.read_factory_settings(use_empty=True)
    rig_objs = imported(lambda: bpy.ops.import_scene.gltf(filepath=ref_rigged))
    arm = next(o for o in rig_objs if o.type == "ARMATURE")
    # the rest pose only: no clip may pose the reference while weights move
    for o in rig_objs:
        o.animation_data_clear()
    for pb in arm.pose.bones:
        pb.matrix_basis = Matrix.Identity(4)
    arm.data.pose_position = "REST"
    skin = [o for o in rig_objs if o.type == "MESH" and o.vertex_groups]
    if not skin:
        raise SystemExit("transfer: reference has no skinned mesh")
    rmesh = max(skin, key=lambda o: len(o.data.vertices))
    cut_objs = imported(lambda: bpy.ops.import_scene.gltf(filepath=ref_cut))
    cut = next(o for o in cut_objs if o.type == "MESH")
    bpy.context.view_layer.update()

    # move the rig back into the bench frame
    a, b = world_verts(rmesh), world_verts(cut)
    if len(a) == len(b):
        s, r, t = similarity(a, b)
        res = np.linalg.norm((s * a @ r.T + t) - b, axis=1).max()
    else:
        res = None
    if res is None or res > 1e-3 * np.ptp(b, 0).max():
        # vertex order lost: best of the four quarter turns about up (+Z here)
        bvh = bvh_of(cut)
        best = None
        for k in range(4):
            th = k * np.pi / 2
            r = np.array([[np.cos(th), -np.sin(th), 0], [np.sin(th), np.cos(th), 0], [0, 0, 1]])
            ac = a @ r.T
            s = np.ptp(b, 0).max() / np.ptp(ac, 0).max()
            t = (b.min(0) + b.max(0)) / 2 - s * (ac.min(0) + ac.max(0)) / 2
            g = mean_gap(s * ac + t, bvh)
            if best is None or g < best[0]:
                best = (g, s, r, t)
        _, s, r, t = best
    m = Matrix.Identity(4)
    for i in range(3):
        for j in range(3):
            m[i][j] = s * r[i, j]
        m[i][3] = t[i]
    roots = [o for o in rig_objs if o.parent is None]
    for o in roots:
        o.matrix_world = m @ o.matrix_world
    bpy.context.view_layer.update()
    for o in cut_objs:
        bpy.data.objects.remove(o, do_unlink=True)

    # the candidate
    if cand.lower().endswith(".obj"):
        c_objs = imported(lambda: bpy.ops.wm.obj_import(filepath=cand))
    else:
        c_objs = imported(lambda: bpy.ops.import_scene.gltf(filepath=cand))
    meshes = [o for o in c_objs if o.type == "MESH"]
    for o in c_objs:
        if o.type != "MESH":
            bpy.data.objects.remove(o, do_unlink=True)
    bpy.ops.object.select_all(action="DESELECT")
    for o in meshes:
        o.select_set(True)
    bpy.context.view_layer.objects.active = meshes[0]
    if len(meshes) > 1:
        bpy.ops.object.join()
    c = bpy.context.view_layer.objects.active
    c.name = "candidate"
    c.parent = None
    c.modifiers.clear()
    c.vertex_groups.clear()
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    # one vertex a position in the GLB: smooth, no UVs, no custom normals
    # (a flat or seamed face splits its vertices on export)
    while c.data.uv_layers:
        c.data.uv_layers.remove(c.data.uv_layers[0])
    if c.data.has_custom_normals:
        bpy.ops.mesh.customdata_custom_splitnormals_clear()
    c.data.shade_smooth()
    c.data.materials.clear()

    # weights: nearest surface point of the reference mesh, interpolated
    for vg in rmesh.vertex_groups:
        c.vertex_groups.new(name=vg.name)
    dt = c.modifiers.new("dt", "DATA_TRANSFER")
    dt.object = rmesh
    dt.use_vert_data = True
    dt.data_types_verts = {"VGROUP_WEIGHTS"}
    dt.vert_mapping = "POLYINTERP_NEAREST"
    dt.layers_vgroup_select_src = "ALL"
    dt.layers_vgroup_select_dst = "NAME"
    bpy.ops.object.select_all(action="DESELECT")
    c.select_set(True)
    bpy.context.view_layer.objects.active = c
    bpy.ops.object.modifier_apply(modifier=dt.name)
    bpy.ops.object.vertex_group_limit_total(group_select_mode="ALL", limit=4)
    bpy.ops.object.vertex_group_normalize_all(group_select_mode="ALL", lock_active=False)
    bpy.ops.object.vertex_group_clean(group_select_mode="ALL", limit=1e-4)
    bpy.ops.object.vertex_group_normalize_all(group_select_mode="ALL", lock_active=False)

    # candidate under the armature, nothing else
    for o in rig_objs:
        if o.type == "MESH":
            bpy.data.objects.remove(o, do_unlink=True)
    amw = arm.matrix_world.copy()
    arm.parent = None
    arm.matrix_world = amw
    arm.data.pose_position = "POSE"
    mw = c.matrix_world.copy()
    c.parent = arm
    c.matrix_world = mw
    mod = c.modifiers.new("Armature", "ARMATURE")
    mod.object = arm
    bpy.ops.object.select_all(action="DESELECT")
    c.select_set(True)
    arm.select_set(True)
    for o in bpy.data.objects:
        if o not in (c, arm) and o.type != "ARMATURE":
            o.select_set(False)
    bpy.ops.export_scene.gltf(filepath=out, use_selection=True, export_format="GLB",
                              export_materials="NONE", export_animations=False,
                              export_skins=True, export_all_influences=False)
    print("TRANSFER ok", len(c.data.vertices), "verts", len(c.data.polygons), "faces",
          "frame residual", None if res is None else float(res))


main()
