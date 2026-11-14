"""Blender: one phone-sized comparison sheet, every mesh from the same camera.

    blender -b --factory-startup -P bench/artist/sheet.py -- OUT.png SOURCE.obj \
        "label=path.obj" ["label=path.obj" ...] [--title TEXT] [--focus fx,fy,fz,size]
        [--view x,y,z] [--cols 2] [--w 540] [--h 960] [--up z|y]

Each mesh is drawn as clay with its wireframe over it (workbench), the
orthographic camera aimed at the focus point (fractions of SOURCE's
bounding box, 0-1 on each axis) with `size` (a fraction of the box's
diagonal) across the panel's longer side, looking along --view (default
front: from -Y). Panels are laid out in a grid, `--cols` across, each
labelled at the top with its label and quad count; the sheet is tall
for a phone at the defaults (2x2 of 540x960). `--up y` reads Y-up OBJs
(the character bench's) instead of the artist cases' Z-up ones.
"""
import os
import sys

import bpy
import numpy as np
from mathutils import Vector


def args():
    a = sys.argv[sys.argv.index("--") + 1:]
    o = {"focus": "0.5,0.5,0.5,1.0", "view": "0.25,1,-0.08", "cols": "2", "w": "540", "h": "960", "title": "",
         "up": "z"}
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


UP = "Z"


def load(path):
    before = set(bpy.data.objects)
    if UP == "Y":
        bpy.ops.wm.obj_import(filepath=path, forward_axis="NEGATIVE_Z", up_axis="Y")
    else:
        bpy.ops.wm.obj_import(filepath=path, forward_axis="Y", up_axis="Z")
    return next(o for o in bpy.data.objects if o not in before and o.type == "MESH")


def mat(name, rgb):
    m = bpy.data.materials.new(name)
    m.diffuse_color = (*rgb, 1)
    return m


def label(sc, cam, text, w, h, scale, depth):
    """A dark text line pinned to the top-left of the orthographic view."""
    cu = bpy.data.curves.new("t", "FONT")
    cu.body = text
    cu.size = scale * 0.04
    ob = bpy.data.objects.new("t", cu)
    ob.data.materials.append(mat("ink", (0.05, 0.05, 0.07)))
    sc.collection.objects.link(ob)
    ob.parent = cam
    aspect_w = scale * (w / max(w, h))
    aspect_h = scale * (h / max(w, h))
    # just past the camera's near clip plane, in front of everything else
    ob.location = (-aspect_w / 2 + scale * 0.03, aspect_h / 2 - scale * 0.07, -depth)
    return ob


def main():
    global UP
    pos, o = args()
    UP = o["up"].upper()
    out, source, items = pos[0], pos[1], pos[2:]
    w, h, cols = int(o["w"]), int(o["h"]), int(o["cols"])
    fx, fy, fz, size = (float(x) for x in o["focus"].split(","))
    view = Vector(tuple(float(x) for x in o["view"].split(","))).normalized()
    tmp = os.path.join(os.path.dirname(out) or ".", "." + os.path.basename(out)[:-4] + "_panels")
    os.makedirs(tmp, exist_ok=True)
    panels = []
    for k, item in enumerate(items):
        name, path = item.split("=", 1)
        bpy.ops.wm.read_factory_settings(use_empty=True)
        sc = bpy.context.scene
        sc.render.engine = "BLENDER_WORKBENCH"
        sc.display.shading.light = "STUDIO"
        sc.display.shading.color_type = "MATERIAL"
        sc.render.resolution_x, sc.render.resolution_y = w, h
        sc.world = bpy.data.worlds.new("w")
        sc.world.color = (0.93, 0.93, 0.92)
        sc.view_settings.view_transform = "Standard"
        src = load(source)
        mw = src.matrix_world
        v = np.array([(mw @ x.co)[:] for x in src.data.vertices])
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
        lens = [(ob.data.vertices[e.vertices[0]].co - ob.data.vertices[e.vertices[1]].co).length
                for e in ob.data.edges]
        lens = [x * max(ob.matrix_world.to_scale()) for x in lens]
        wm = wire.modifiers.new("w", "WIREFRAME")
        wm.thickness = float(np.median(lens)) * 0.07
        wm.use_replace = True
        wm.use_even_offset = False  # even offset spikes at near-degenerate corners
        cam = bpy.data.objects.new("cam", bpy.data.cameras.new("cam"))
        cam.data.type = "ORTHO"
        scale = size * diag
        cam.data.ortho_scale = scale
        sc.collection.objects.link(cam)
        sc.camera = cam
        cam.location = target - view * 3 * diag
        cam.rotation_euler = view.to_track_quat("-Z", "Y").to_euler()
        cam.data.clip_start, cam.data.clip_end = 0.01 * diag, 10 * diag
        quads = sum(1 for p in ob.data.polygons if len(p.vertices) == 4)
        odd = len(ob.data.polygons) - quads
        text = f"{name}  {quads} quads" + (f", {odd} other" if odd else "")
        bpy.context.view_layer.update()
        label(sc, cam, text, w, h, scale, 2.0 * cam.data.clip_start)
        sc.render.filepath = os.path.join(tmp, f"{k}.png")
        bpy.ops.render.render(write_still=True)
        panels.append(sc.render.filepath)
    rows = (len(panels) + cols - 1) // cols
    gap = 6
    W, H = cols * w + (cols - 1) * gap, rows * h + (rows - 1) * gap
    canvas = np.ones((H, W, 4), dtype=np.float32)
    for k, p in enumerate(panels):
        im = bpy.data.images.load(p)
        a = np.array(im.pixels[:], dtype=np.float32).reshape(h, w, 4)
        r, c = k // cols, k % cols
        y0 = H - (r + 1) * h - r * gap  # Blender images run bottom-up
        x0 = c * (w + gap)
        canvas[y0:y0 + h, x0:x0 + w] = a
    img = bpy.data.images.new("sheet", W, H)
    img.pixels = canvas.ravel()
    img.filepath_raw = out
    img.file_format = "PNG"
    img.save()
    print("SHEET", out, len(panels))


main()
