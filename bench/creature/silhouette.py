"""Blender: silhouette error of a cut against its source.

    blender -b --factory-startup -P bench/creature/silhouette.py -- \
        DENSE.obj CANDIDATE.obj OUT.json [--size 384] [--cache DIR]

Both meshes are drawn as flat black shapes from eight directions (front,
back, both sides, top, and three three-quarter views), same camera for
both (orthographic, framed on the source). Per view the error is the
area where the two shapes differ as a share of the source's shape;
OUT.json gets `sil_err` (mean over the views, %), `sil_worst` (the worst
view) and the per-view list. 0 is the same outline from every side; a
lost horn or a fattened limb shows here. The source's shapes are kept in
--cache (one .npy per view) so each candidate renders only itself.
"""
import json
import os
import sys
import tempfile

import bpy
import numpy as np
from mathutils import Vector

VIEWS = [(0, -1, 0), (0, 1, 0), (1, 0, 0), (-1, 0, 0), (0, 0, 1), (-0.6, -1, 0.4), (0.6, -1, 0.4), (0.6, 1, 0.4)]


def load(path):
    before = set(bpy.data.objects)
    bpy.ops.wm.obj_import(filepath=path)
    return next(o for o in bpy.data.objects if o not in before and o.type == "MESH")


def mask(path):
    img = bpy.data.images.load(path)
    px = np.array(img.pixels[:]).reshape(img.size[1], img.size[0], 4)
    bpy.data.images.remove(img)
    return px[:, :, 0] < 0.5


def main():
    a = sys.argv[sys.argv.index("--") + 1:]
    o = {"size": "384", "cache": None}
    pos = []
    i = 0
    while i < len(a):
        if a[i].startswith("--"):
            o[a[i][2:]] = a[i + 1]
            i += 2
        else:
            pos.append(a[i])
            i += 1
    dense, cand, out = pos
    size = int(o["size"])
    cache = o["cache"]
    bpy.ops.wm.read_factory_settings(use_empty=True)
    sc = bpy.context.scene
    sc.render.engine = "BLENDER_WORKBENCH"
    sc.display.shading.light = "FLAT"
    sc.display.shading.color_type = "SINGLE"
    sc.display.shading.single_color = (0, 0, 0)
    sc.display.render_aa = "OFF"
    sc.render.resolution_x = sc.render.resolution_y = size
    sc.render.film_transparent = False
    sc.world = bpy.data.worlds.new("w")
    sc.world.color = (1, 1, 1)
    sc.view_settings.view_transform = "Standard"
    cam = bpy.data.objects.new("cam", bpy.data.cameras.new("cam"))
    cam.data.type = "ORTHO"
    sc.collection.objects.link(cam)
    sc.camera = cam

    need_src = not cache or not all(os.path.exists(os.path.join(cache, f"src_{k}_{size}.npy")) for k in range(len(VIEWS)))
    frame_file = os.path.join(cache, "frame.json") if cache else None
    src = None
    if need_src or not (frame_file and os.path.exists(frame_file)):
        src = load(dense)
        v = np.array([x.co[:] for x in src.data.vertices])
        lo, hi = v.min(0), v.max(0)
        frame = {"centre": ((lo + hi) / 2).tolist(), "diag": float(np.linalg.norm(hi - lo))}
        if cache:
            os.makedirs(cache, exist_ok=True)
            json.dump(frame, open(frame_file, "w"))
    else:
        frame = json.load(open(frame_file))
    centre, diag = Vector(frame["centre"]), frame["diag"]
    cam.data.ortho_scale = diag * 1.05
    cam.data.clip_start, cam.data.clip_end = 0.01 * diag, 10 * diag
    tmp = tempfile.mkdtemp()

    def shots(ob, tag):
        res = []
        for k, d in enumerate(VIEWS):
            loc = centre + Vector(d).normalized() * 3 * diag
            cam.location = loc
            cam.rotation_euler = (centre - loc).to_track_quat("-Z", "Y").to_euler()
            sc.render.filepath = os.path.join(tmp, f"{tag}_{k}.png")
            bpy.ops.render.render(write_still=True)
            res.append(mask(sc.render.filepath))
        return res

    if need_src:
        s_masks = shots(src, "s")
        if cache:
            for k, m in enumerate(s_masks):
                np.save(os.path.join(cache, f"src_{k}_{size}.npy"), m)
    else:
        s_masks = [np.load(os.path.join(cache, f"src_{k}_{size}.npy")) for k in range(len(VIEWS))]
    if src is not None:
        bpy.data.objects.remove(src, do_unlink=True)
    c = load(cand)
    c_masks = shots(c, "c")
    errs = [round(100.0 * float(np.logical_xor(s, m).sum()) / max(1, int(s.sum())), 3) for s, m in zip(s_masks, c_masks)]
    res = {"sil_err": round(float(np.mean(errs)), 3), "sil_worst": max(errs), "views": errs}
    json.dump(res, open(out, "w"))
    print("SILHOUETTE", json.dumps(res))


main()
