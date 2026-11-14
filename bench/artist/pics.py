"""Blender: wireframe pictures of one case, every tool from the same camera.

    blender -b --factory-startup -P bench/artist/pics.py -- CASE_DIR RUNS_DIR OUT.png \
        --tools pro,retopo,quadwild,quadriflow --focus fx,fy,fz,size [--view x,y,z] [--px 520]

Each mesh (the artist's `pro.obj`, then each tool's `RUNS_DIR/<tool>/<case>.obj`)
is drawn as clay with its wireframe, the camera aimed at the focus point
(fractions of the source's bounding box, 0-1 on each axis) with the view
`size` (a fraction of the box's diagonal) across, looking along --view
(default front: the camera on the -Y side looking +Y). The pictures are laid side by side, labelled with
the tool, its quad count and the share of the artist's rings it kept.
"""
import json
import os
import sys

import bpy
import numpy as np
from mathutils import Vector


def args():
    a = sys.argv[sys.argv.index("--") + 1:]
    o = {"tools": "pro,retopo,quadwild,quadriflow", "focus": "0.5,0.5,0.5,1.0", "view": "0.3,1,-0.12", "px": "520"}
    pos = []
    i = 0
    while i < len(a):
        if a[i].startswith("--"):
            o[a[i][2:]] = a[i + 1]
            i += 2
        else:
            pos.append(a[i])
            i += 1
    return pos, o


def load(path):
    before = set(bpy.data.objects)
    bpy.ops.wm.obj_import(filepath=path, forward_axis="Y", up_axis="Z")
    return next(o for o in bpy.data.objects if o not in before and o.type == "MESH")


def mat(name, rgb):
    m = bpy.data.materials.new(name)
    m.diffuse_color = (*rgb, 1)
    return m


def main():
    (case, runs, out), o = args()
    name = os.path.basename(case.rstrip("/"))
    tools = o["tools"].split(",")
    px = int(o["px"])
    fx, fy, fz, size = (float(x) for x in o["focus"].split(","))
    view = Vector(tuple(float(x) for x in o["view"].split(","))).normalized()
    scores = {}
    for t in tools:
        js = os.path.join(runs, t + ".json")
        if os.path.exists(js):
            for r in json.load(open(js))["rows"]:
                if r["name"] == name:
                    scores[t] = r
    tmp = os.path.join(os.path.dirname(out) or ".", f".{name}_pics")
    os.makedirs(tmp, exist_ok=True)
    shots = []
    for t in tools:
        path = os.path.join(case, "pro.obj") if t == "pro" else os.path.join(runs, t, name + ".obj")
        if not os.path.exists(path):
            continue
        bpy.ops.wm.read_factory_settings(use_empty=True)
        sc = bpy.context.scene
        sc.render.engine = "BLENDER_WORKBENCH"
        sc.display.shading.light = "STUDIO"
        sc.display.shading.color_type = "MATERIAL"
        sc.render.resolution_x = sc.render.resolution_y = px
        sc.world = bpy.data.worlds.new("w")
        sc.world.color = (0.93, 0.93, 0.92)
        sc.view_settings.view_transform = "Standard"
        src = load(os.path.join(case, "dense.obj"))
        v = np.array([x.co[:] for x in src.data.vertices])
        lo, hi = v.min(0), v.max(0)
        bpy.data.objects.remove(src, do_unlink=True)
        diag = float(np.linalg.norm(hi - lo))
        target = Vector(tuple(lo + (hi - lo) * np.array([fx, fy, fz])))
        ob = load(path)
        ob.data.materials.append(mat("clay", (0.72, 0.70, 0.66)))
        for p in ob.data.polygons:
            p.use_smooth = False
        wire = ob.copy()
        wire.data = ob.data.copy()
        sc.collection.objects.link(wire)
        wire.data.materials.clear()
        wire.data.materials.append(mat("wire", (0.08, 0.09, 0.12)))
        lens = [(ob.data.vertices[e.vertices[0]].co - ob.data.vertices[e.vertices[1]].co).length for e in ob.data.edges]
        wm = wire.modifiers.new("w", "WIREFRAME")
        wm.thickness = float(np.median(lens)) * 0.07
        wm.use_replace = True
        cam = bpy.data.objects.new("cam", bpy.data.cameras.new("cam"))
        cam.data.type = "ORTHO"
        cam.data.ortho_scale = size * diag
        sc.collection.objects.link(cam)
        sc.camera = cam
        cam.location = target - view * 3 * diag
        cam.rotation_euler = view.to_track_quat("-Z", "Y").to_euler()
        cam.data.clip_start, cam.data.clip_end = 0.01 * diag, 10 * diag
        sc.render.filepath = os.path.join(tmp, t + ".png")
        bpy.ops.render.render(write_still=True)
        quads = sum(1 for p in ob.data.polygons if len(p.vertices) == 4)
        odd = len(ob.data.polygons) - quads
        r = scores.get(t, {})
        label = "artist" if t == "pro" else t
        info = f"{quads} quads" + (f", {odd} odd" if odd else "")
        if t != "pro" and r.get("rings_pct") is not None:
            info += f", rings {r['rings_pct']:.0f}%"
        shots.append((sc.render.filepath, label, info))
    # side by side with labels: drawn into the images with Blender's image API
    imgs = [bpy.data.images.load(p) for p, _, _ in shots]
    w, h = px * len(imgs), px
    canvas = np.ones((h, w, 4), dtype=np.float32)
    for k, im in enumerate(imgs):
        a = np.array(im.pixels[:], dtype=np.float32).reshape(px, px, 4)
        canvas[:, k * px:(k + 1) * px] = a
    outimg = bpy.data.images.new("sheet", w, h)
    outimg.pixels = canvas.ravel()
    outimg.filepath_raw = out
    outimg.file_format = "PNG"
    outimg.save()
    json.dump([{"tool": t, "label": lab, "info": inf} for (_, lab, inf), t in zip(shots, tools)],
              open(out[:-4] + ".json", "w"))
    print("PICS", out, [s[1] for s in shots])


main()
