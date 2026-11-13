"""Blender: frames of a rigged candidate moving, for a short looping clip.

    blender -b --factory-startup -P bench/creature/clip.py -- \
        CANDIDATE_RIGGED.glb CANDIDATE.obj OUT_DIR [--frame MESH.obj]
        [--frames 36] [--size 480] [--swing 30]

Every limb (a bone named leg, arm, tail, neck or wing whose parent is not
named the same way, so each chain swings from its root, its children
following at half the swing) turns about its own local X by a sine wave,
left and right sides half a cycle apart, front and back a quarter apart:
a rough walk that bends every joint the rig has. Clay with the wireframe,
three-quarter view framed on --frame (the source) so two candidates line
up. Writes OUT_DIR/0000.png ... ; ffmpeg turns them into the clip.
"""
import math
import os
import sys

import bpy
import numpy as np
from mathutils import Matrix, Quaternion

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import metrics_lib as ml  # noqa: E402
import render as rd  # noqa: E402  (scene helpers; its main() is guarded below)


def main():
    a = sys.argv[sys.argv.index("--") + 1:]
    o = {"frame": None, "frames": "36", "size": "480", "swing": "30"}
    pos = []
    i = 0
    while i < len(a):
        if a[i].startswith("--"):
            o[a[i][2:]] = a[i + 1]
            i += 2
        else:
            pos.append(a[i])
            i += 1
    rigged, obj, out = pos
    os.makedirs(out, exist_ok=True)
    frames, size, swing = int(o["frames"]), int(o["size"]), math.radians(float(o["swing"]))
    bpy.ops.wm.read_factory_settings(use_empty=True)
    objs = ml.imported(lambda: bpy.ops.import_scene.gltf(filepath=rigged, merge_vertices=True))
    arm = next(x for x in objs if x.type == "ARMATURE")
    for x in objs:
        x.animation_data_clear()
    for pb in arm.pose.bones:
        pb.matrix_basis = Matrix.Identity(4)
    bpy.context.view_layer.update()
    glb = max((x for x in objs if x.type == "MESH"), key=lambda x: len(x.data.vertices))
    c = ml.with_faces_of(glb, arm, obj)
    c.data.materials.clear()
    c.data.materials.append(rd.material("clay", (0.72, 0.70, 0.66)))
    for p in c.data.polygons:
        p.use_smooth = False
    edges = np.array([(c.data.vertices[e.vertices[0]].co - c.data.vertices[e.vertices[1]].co).length
                      for e in c.data.edges])
    wire = c.copy()
    wire.data = c.data.copy()
    bpy.context.scene.collection.objects.link(wire)
    wire.data.materials.clear()
    wire.data.materials.append(rd.material("wire", (0.08, 0.09, 0.12)))
    wm = wire.modifiers.new("wire", "WIREFRAME")
    wm.thickness = float(np.median(edges)) * 0.06
    wm.use_replace = True
    cam = rd.setup_scene(size)
    v = ml.coords(c)
    if o["frame"]:
        f_objs = ml.imported(lambda: bpy.ops.wm.obj_import(filepath=o["frame"]))
        v = ml.coords(next(x for x in f_objs if x.type == "MESH"))
        for x in f_objs:
            bpy.data.objects.remove(x, do_unlink=True)
    lo, hi = np.percentile(v, 1, axis=0), np.percentile(v, 99, axis=0)
    rd.aim(cam, (lo + hi) / 2, (-0.55, -1.0, 0.35), float(np.linalg.norm(hi - lo)) * 1.35)

    words = ("leg", "arm", "tail", "neck", "wing")

    def kind(name):
        n = name.lower()
        return next((w for w in words if w in n), None)

    moving = []
    for pb in arm.pose.bones:
        k = kind(pb.name)
        if not k:
            continue
        root = pb.parent is None or kind(pb.parent.name) != k
        side = 0.5 if pb.name.endswith((".R", "_R", "Right", ".r")) else 0.0
        head = (arm.matrix_world @ pb.bone.head_local)
        front = 0.25 if head.y < 0 else 0.0  # glTF forward is Blender -Y
        moving.append((pb, 1.0 if root else 0.5, side + front))
        pb.rotation_mode = "QUATERNION"
    for f in range(frames):
        t = 2 * math.pi * f / frames
        for pb, scale, phase in moving:
            pb.rotation_quaternion = Quaternion((1, 0, 0), swing * scale * math.sin(t + 2 * math.pi * phase))
        bpy.context.view_layer.update()
        rd.render(os.path.join(out, f"{f:04d}.png"))
    print("CLIP", len(moving), "bones moving,", frames, "frames")


main()
