"""Blender side of bench/humans.py (headless; never opens the GUI).

    blender -b --factory-startup --python bench/humans_blender.py -- prep IN.glb CLEAN.obj SCAN.obj
    blender -b --factory-startup --python bench/humans_blender.py -- quadriflow SCAN.obj OUT.obj FACES
    blender -b --factory-startup --python bench/humans_blender.py -- render OUT.png MESH.obj [MESH.obj ...]

prep: joins a corpus character's meshes, fuses them into one outer skin
(voxel remesh), writes it as CLEAN.obj (the shape to match) and writes
SCAN.obj, a photogrammetry/AI-style soup of it: voxel
remeshed at 1/200 of its height (one closed, dense, triangulated skin with
the clothes fused on, ~300-400k tris) plus 0.1% of height of surface noise.
quadriflow: Blender's built-in QuadriFlow remesher (the free baseline every
Blender user has), timed by the caller.
render: three-quarter Workbench views of each mesh with its wireframe,
side by side (one tile per mesh), for the contact sheet.
"""

import math
import os
import random
import sys

import bpy

argv = sys.argv[sys.argv.index("--") + 1:]
mode = argv[0]


def clear():
    for o in list(bpy.data.objects):
        bpy.data.objects.remove(o)


def export_obj(o, path):
    bpy.ops.object.select_all(action='DESELECT')
    o.select_set(True)
    bpy.context.view_layer.objects.active = o
    bpy.ops.wm.obj_export(filepath=path, export_selected_objects=True, export_materials=False,
                          export_uv=False, export_normals=False, apply_modifiers=True,
                          forward_axis='NEGATIVE_Z', up_axis='Y')


def import_obj(path):
    bpy.ops.wm.obj_import(filepath=path, forward_axis='NEGATIVE_Z', up_axis='Y')
    return bpy.context.selected_objects[0]


def keep_outer_shell(o):
    """Drop inner shells the thickened layers leave inside: keep the
    largest connected component (the outer skin)."""
    import bmesh
    bm = bmesh.new()
    bm.from_mesh(o.data)
    seen, comps = set(), []
    for v in bm.verts:
        if v in seen:
            continue
        comp, stack = [], [v]
        seen.add(v)
        while stack:
            x = stack.pop()
            comp.append(x)
            for e in x.link_edges:
                y = e.other_vert(x)
                if y not in seen:
                    seen.add(y)
                    stack.append(y)
        comps.append(comp)
    comps.sort(key=len, reverse=True)
    bmesh.ops.delete(bm, geom=[v for c in comps[1:] for v in c], context='VERTS')
    bm.to_mesh(o.data)
    bm.free()
    print(f"SHELLS {len(comps)} kept {len(comps[0])} verts")


def prep(src, clean_path, scan_path):
    clear()
    bpy.ops.import_scene.gltf(filepath=src)
    meshes = [o for o in bpy.data.objects if o.type == 'MESH' and o.visible_get()]
    for o in meshes:
        for m in list(o.modifiers):
            o.modifiers.remove(m)
        o.parent = None
    bpy.ops.object.select_all(action='DESELECT')
    for o in meshes:
        o.select_set(True)
    bpy.context.view_layer.objects.active = meshes[0]
    bpy.ops.object.join()
    body = bpy.context.active_object
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    h = body.dimensions.z
    # Clothes and proxies are open single-layer shells; give them thickness
    # (inward, the outer surface stays) so the voxel pass sees volumes.
    sol = body.modifiers.new("solid", 'SOLIDIFY')
    sol.thickness = h / 120
    sol.offset = -1.0
    bpy.ops.object.modifier_apply(modifier=sol.name)
    m = body.modifiers.new("voxel", 'REMESH')
    m.mode = 'VOXEL'
    m.voxel_size = h / 200
    m.use_smooth_shade = False
    bpy.ops.object.modifier_apply(modifier=m.name)
    tri = body.modifiers.new("tri", 'TRIANGULATE')
    bpy.ops.object.modifier_apply(modifier=tri.name)
    keep_outer_shell(body)
    # The outer skin without noise: what a remesher should reproduce (the
    # clean character still carries the body hidden under its clothes).
    export_obj(body, clean_path)
    rnd = random.Random(7)
    amp = 0.001 * h
    for v in body.data.vertices:
        v.co.x += rnd.uniform(-amp, amp)
        v.co.y += rnd.uniform(-amp, amp)
        v.co.z += rnd.uniform(-amp, amp)
    export_obj(body, scan_path)
    print(f"PREP tris {len(body.data.polygons)}")


def quadriflow(scan, out, faces):
    clear()
    o = import_obj(scan)
    bpy.context.view_layer.objects.active = o
    r = bpy.ops.object.quadriflow_remesh(mode='FACES', target_faces=int(faces), use_mesh_symmetry=False,
                                         use_preserve_sharp=False, use_preserve_boundary=False, seed=0)
    print("QUADRIFLOW", r, len(o.data.polygons))
    export_obj(o, out)


def render(out, paths):
    clear()
    scn = bpy.context.scene
    scn.render.engine = 'BLENDER_WORKBENCH'
    scn.display.shading.light = 'STUDIO'
    scn.display.shading.color_type = 'SINGLE'
    scn.display.shading.single_color = (0.8, 0.8, 0.8)
    scn.render.resolution_x, scn.render.resolution_y = 360, 480
    tiles = []
    for i, p in enumerate(paths):
        clear()
        o = import_obj(p)
        # wire overlay: a thin wireframe copy in dark grey
        w = o.copy()
        w.data = o.data.copy()
        scn.collection.objects.link(w)
        mod = w.modifiers.new("wf", 'WIREFRAME')
        mod.thickness = o.dimensions.z * 0.0012
        mod.use_replace = True
        mat = bpy.data.materials.new("dark")
        mat.diffuse_color = (0.05, 0.05, 0.05, 1)
        w.data.materials.append(mat)
        scn.display.shading.color_type = 'MATERIAL'
        o.data.materials.append(bpy.data.materials.get("light") or bpy.data.materials.new("light"))
        bpy.data.materials["light"].diffuse_color = (0.82, 0.82, 0.82, 1)
        P = [o.matrix_world @ v.co for v in o.data.vertices]
        lo, hi = min(p.z for p in P), max(p.z for p in P)
        h = hi - lo
        cx = (min(p.x for p in P) + max(p.x for p in P)) / 2
        cy = (min(p.y for p in P) + max(p.y for p in P)) / 2
        cam = bpy.data.objects.new("cam", bpy.data.cameras.new("cam"))
        scn.collection.objects.link(cam)
        cam.data.type = 'ORTHO'
        cam.data.ortho_scale = h * 0.55
        cam.data.clip_end = h * 10
        # upper body, three-quarter view from the front-left
        a = math.radians(30)
        cam.location = (cx + 3 * h * math.sin(a), cy - 3 * h * math.cos(a), hi - h * 0.27)
        cam.rotation_euler = (math.radians(90), 0, a)
        scn.camera = cam
        t = out + f".tile{i}.png"
        scn.render.filepath = t
        bpy.ops.render.render(write_still=True)
        tiles.append(t)
    print("TILES", " ".join(tiles))


if mode == "prep":
    prep(*argv[1:4])
elif mode == "quadriflow":
    quadriflow(*argv[1:4])
elif mode == "render":
    render(argv[1], argv[2:])
