"""Blender: build the artist bench's cases from the reference library.

    blender -b --disable-autoexec -P bench/artist/make_cases.py -- CASES.json OUT_DIR [--only a,b]

CASES.json (bench/artist/cases.json) lists hand-made quad meshes in
`~/Games/_refs` (CC0 or CC-BY, benchmark use only; see each source's
SOURCE.md): `name`, `file` (under ~/Games/_refs), `object` (a mesh name
in a .blend; omitted for an FBX, whose meshes are joined), `kind`
(`organic` or `hard`), `set` (`fast` or `full`).

Per case, OUT_DIR/<name>/ gets:

  pro.obj     the artist's mesh: its Mirror modifier applied, subdivision
              and sculpt modifiers dropped, welded, scaled so its largest
              side is 1. The truth for loops, flow and poles.
  dense.obj   the shape only, what a remesher is given: the artist's mesh
              subdivided (Catmull-Clark for organic, flat for hard-surface),
              triangulated, then at least half its triangles collapsed
              away, down to 12 per artist quad (30k to 150k), so no trace
              of the artist's edges is left to follow. Same scale and place as pro.obj.
  case.json   name, kind, set, quads (the artist's quad count: the target)

Nothing from the library is copied into the repo; OUT_DIR is a work
folder.
"""
import json
import os
import sys

import bmesh
import bpy


def main():
    a = sys.argv[sys.argv.index("--") + 1:]
    only = None
    if "--only" in a:
        i = a.index("--only")
        only = set(a[i + 1].split(","))
        del a[i:i + 2]
    cases_path, out = a
    refs = os.path.expanduser("~/Games/_refs")
    cases = json.load(open(cases_path))["cases"]
    by_file = {}
    for c in cases:
        if only and c["name"] not in only:
            continue
        by_file.setdefault(c["file"], []).append(c)
    for file, group in by_file.items():
        path = os.path.join(refs, file)
        for c in group:
            if path.endswith(".blend"):
                bpy.ops.wm.open_mainfile(filepath=path, use_scripts=False)
                src = bpy.data.objects[c["object"]]
                parts = [src]
            else:
                bpy.ops.wm.read_factory_settings(use_empty=True)
                bpy.ops.import_scene.fbx(filepath=path)
                parts = [o for o in bpy.data.objects if o.type == "MESH"]
            # one fresh scene object holding the artist's faces in world space
            bm = bmesh.new()
            dg = bpy.context.evaluated_depsgraph_get()
            for o in parts:
                for m in list(o.modifiers):
                    if m.type != "MIRROR":
                        o.modifiers.remove(m)
                dg.update()
                ev = o.evaluated_get(dg)
                me = ev.to_mesh()
                me.transform(o.matrix_world)
                bm.from_mesh(me)
                ev.to_mesh_clear()
            size = max(max(v.co[k] for v in bm.verts) - min(v.co[k] for v in bm.verts) for k in range(3))
            lo = [min(v.co[k] for v in bm.verts) for k in range(3)]
            for v in bm.verts:
                for k in range(3):
                    v.co[k] = (v.co[k] - lo[k]) / size
            bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-6)
            bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
            quads = sum(1 for f in bm.faces if len(f.verts) == 4)
            d = os.path.join(out, c["name"])
            os.makedirs(d, exist_ok=True)
            write_obj(bm, os.path.join(d, "pro.obj"))
            # the shape only
            pm = bpy.data.meshes.new("dense")
            bm.to_mesh(pm)
            bm.free()
            ob = bpy.data.objects.new("dense", pm)
            bpy.context.scene.collection.objects.link(ob)
            bpy.ops.object.select_all(action="DESELECT")
            ob.select_set(True)
            bpy.context.view_layer.objects.active = ob
            want = min(150000, max(30000, 12 * quads))
            levels = 1
            while len(pm.polygons) * 2 * 4 ** levels < 2 * want and levels < 6:
                levels += 1
            sub = ob.modifiers.new("sub", "SUBSURF")
            sub.subdivision_type = "CATMULL_CLARK" if c["kind"] == "organic" else "SIMPLE"
            sub.levels = levels
            bpy.ops.object.modifier_apply(modifier=sub.name)
            tri = ob.modifiers.new("tri", "TRIANGULATE")
            bpy.ops.object.modifier_apply(modifier=tri.name)
            dec = ob.modifiers.new("dec", "DECIMATE")
            dec.ratio = min(0.5, want / max(1, len(ob.data.polygons)))
            bpy.ops.object.modifier_apply(modifier=dec.name)
            bm2 = bmesh.new()
            bm2.from_mesh(ob.data)
            write_obj(bm2, os.path.join(d, "dense.obj"))
            tris = len(bm2.faces)
            bm2.free()
            json.dump({"name": c["name"], "kind": c["kind"], "set": c.get("set", "fast"), "quads": quads,
                       "dense_tris": tris}, open(os.path.join(d, "case.json"), "w"))
            print("CASE", c["name"], c["kind"], "quads", quads, "dense", tris, flush=True)


def write_obj(bm, path):
    bm.verts.index_update()
    with open(path, "w") as f:
        for v in bm.verts:
            f.write(f"v {v.co.x:.7f} {v.co.y:.7f} {v.co.z:.7f}\n")
        for fc in bm.faces:
            f.write("f " + " ".join(str(v.index + 1) for v in fc.verts) + "\n")


main()
