"""Blender: pictures of one rigged candidate, for before/after sheets.

    blender -b --factory-startup -P bench/creature/render.py -- \
        CANDIDATE_RIGGED.glb CANDIDATE.obj OUT_PREFIX [--joint BONE] [--bend 60]
        [--size 900] [--frame MESH.obj] [--no-wire]

--frame aims the whole view at another mesh's extent (the source), so a
before/after pair shares one camera; the extent ignores the outer 1% of
vertices each way (stray hair strands). --no-wire draws clay only (the
dense source's wireframe would be solid black). RIGGED may be `-`.

Writes OUT_PREFIX.whole.png (clay with the wireframe, three-quarter front
view) and, when the mesh has a rig, OUT_PREFIX.joint.png: a close-up of
one joint bent --bend degrees (the joint named by --joint, else the first
knee-like joint: a leg bone under a leg bone, else the first bone with a
skinned parent), with the wireframe, so loops and folds at the bend show.
OUT_PREFIX.joint.txt names the joint and the bend. Workbench, so it runs
headless without a GPU and the same input gives the same picture.
"""
import math
import os
import sys

import bpy
import numpy as np
from mathutils import Matrix, Vector

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def args():
    a = sys.argv[sys.argv.index("--") + 1:]
    o = {"joint": None, "bend": 60.0, "size": 900, "frame": None, "wire": True}
    pos = []
    i = 0
    while i < len(a):
        if a[i] == "--no-wire":
            o["wire"] = False
            i += 1
        elif a[i].startswith("--"):
            o[a[i][2:]] = a[i + 1]
            i += 2
        else:
            pos.append(a[i])
            i += 1
    return pos, o


def imported(fn):
    before = set(bpy.data.objects)
    fn()
    return [o for o in bpy.data.objects if o not in before]


def material(name, rgb):
    m = bpy.data.materials.new(name)
    m.diffuse_color = (*rgb, 1.0)
    return m


def setup_scene(size):
    sc = bpy.context.scene
    sc.render.engine = "BLENDER_WORKBENCH"
    sc.display.shading.light = "STUDIO"
    sc.display.shading.color_type = "MATERIAL"
    sc.display.shading.show_cavity = False
    sc.render.resolution_x = sc.render.resolution_y = size
    sc.render.film_transparent = False
    sc.world = bpy.data.worlds.new("w")
    sc.world.color = (0.93, 0.93, 0.92)
    sc.view_settings.view_transform = "Standard"
    cam = bpy.data.objects.new("cam", bpy.data.cameras.new("cam"))
    sc.collection.objects.link(cam)
    sc.camera = cam
    return cam


def aim(cam, target, direction, dist, lens=50):
    cam.data.lens = lens
    loc = Vector(target) + Vector(direction).normalized() * dist
    cam.location = loc
    cam.rotation_euler = (Vector(target) - loc).to_track_quat("-Z", "Y").to_euler()
    cam.data.clip_start = dist * 0.01
    cam.data.clip_end = dist * 10


def render(path):
    bpy.context.scene.render.filepath = path
    bpy.ops.render.render(write_still=True)


def main():
    pos, o = args()
    rigged, obj, prefix = pos
    size, bend = int(o["size"]), float(o["bend"])
    bpy.ops.wm.read_factory_settings(use_empty=True)
    g_objs = imported(lambda: bpy.ops.import_scene.gltf(filepath=rigged, merge_vertices=True)) \
        if rigged != "-" else []
    arm = next((x for x in g_objs if x.type == "ARMATURE"), None)
    for x in g_objs:
        x.animation_data_clear()
    if arm:
        for pb in arm.pose.bones:
            pb.matrix_basis = Matrix.Identity(4)
    bpy.context.view_layer.update()
    glb_mesh = max((x for x in g_objs if x.type == "MESH"), key=lambda x: len(x.data.vertices), default=None)
    import metrics_lib as ml
    if glb_mesh is not None:
        c = ml.with_faces_of(glb_mesh, arm, obj)
    else:
        c = next(x for x in imported(lambda: bpy.ops.wm.obj_import(filepath=obj)) if x.type == "MESH")
    for x in list(bpy.data.objects):
        if x.type == "MESH" and x is not c:
            bpy.data.objects.remove(x, do_unlink=True)
    c.data.materials.clear()
    c.data.materials.append(material("clay", (0.72, 0.70, 0.66)))
    for p in c.data.polygons:
        p.use_smooth = False
    edges = np.array([(c.data.vertices[e.vertices[0]].co - c.data.vertices[e.vertices[1]].co).length
                      for e in c.data.edges])
    wire = c.copy()
    wire.data = c.data.copy()
    if o["wire"]:
        bpy.context.scene.collection.objects.link(wire)
    wire.data.materials.clear()
    wire.data.materials.append(material("wire", (0.08, 0.09, 0.12)))
    wm = wire.modifiers.new("wire", "WIREFRAME")
    wm.thickness = float(np.median(edges)) * 0.06
    wm.use_replace = True
    wm.use_even_offset = True
    cam = setup_scene(size)
    bpy.context.view_layer.update()

    v = ml.coords(c)
    if o["frame"]:
        f_objs = imported(lambda: bpy.ops.wm.obj_import(filepath=o["frame"]))
        fm = next(x for x in f_objs if x.type == "MESH")
        v = ml.coords(fm)
        for x in f_objs:
            bpy.data.objects.remove(x, do_unlink=True)
    lo, hi = np.percentile(v, 1, axis=0), np.percentile(v, 99, axis=0)
    ctr, ext = (lo + hi) / 2, float(np.linalg.norm(hi - lo))
    aim(cam, ctr, (-0.55, -1.0, 0.35), ext * 1.35)
    render(prefix + ".whole.png")

    if arm is None:
        return
    joints = ml.find_joints(c, arm)
    if not joints:
        return
    pick = None
    if o["joint"]:
        pick = next((j for j in joints if j["bone"] == o["joint"]), None)
    if pick is None:
        legs = [j for j in joints if "leg" in j["bone"].lower() and "leg" in j["parent"].lower()]
        pick = (legs or joints)[0]
    # bend toward the world direction most across the bone (prefer forward, -Y)
    best = None
    for target in ([0, -1.0, 0], [0, 1.0, 0], [0, 0, 1.0], [1.0, 0, 0], [-1.0, 0, 0], [0, 0, -1.0]):
        across = abs(np.dot(target, pick["db"]))
        if best is None or across < best[0] - 0.2:
            best = (across, np.array(target))
    target = best[1]
    axis = np.cross(pick["db"], target)
    axis /= np.linalg.norm(axis)
    rw = np.array(Matrix.Rotation(math.radians(bend), 3, Vector(axis)))
    mw = np.eye(4)
    mw[:3, :3] = rw
    mw[:3, 3] = pick["head"] - rw @ pick["head"]
    amat = np.array(arm.matrix_world)
    pb = arm.pose.bones[pick["bone"]]
    pb.matrix = Matrix((np.linalg.inv(amat) @ mw @ amat).tolist()) @ pb.bone.matrix_local
    for x in (c, wire) if o["wire"] else (c,):
        if not any(m.type == "ARMATURE" for m in x.modifiers):
            am = x.modifiers.new("Armature", "ARMATURE")
            am.object = arm
    # armature before wireframe on the wire copy
    while o["wire"] and wire.modifiers[0].type != "ARMATURE":
        bpy.context.view_layer.objects.active = wire
        bpy.ops.object.modifier_move_up(modifier=[m for m in wire.modifiers if m.type == "ARMATURE"][0].name)
    bpy.context.view_layer.update()
    side = axis if abs(axis[2]) < 0.9 else np.array([1.0, 0, 0])
    aim(cam, pick["head"], side + np.array([0, 0, 0.25]), pick["r"] * 9.0)
    render(prefix + ".joint.png")
    with open(prefix + ".joint.txt", "w") as f:
        f.write(f"{pick['bone']} bent {bend:g} deg\n")


if __name__ == "__main__":
    main()
