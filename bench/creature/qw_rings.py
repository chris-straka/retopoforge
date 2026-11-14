"""Blender: a dense mesh with the joint rings cut in, plus a QuadWild
`.sharp` file that marks the ring edges as features to keep.

    blender -b --factory-startup -P bench/creature/qw_rings.py -- \
        DENSE.obj JOINTS.json OUT.obj

JOINTS.json is bench/creature/rig_guides.py's (OBJ frame: head, axis,
limb radius). Each joint's plane (through the head, across the axis) is
cut into the faces within two limb radii of the head; the new edges on
the plane within 2.5 radii of the axis are the ring. OUT.obj is
triangulated and written in the input's own coordinates (no axis
conversion); OUT.sharp lists the ring edges as `type face edge` lines
after their count, the format QuadWild reads for features.
"""
import json
import sys

import bmesh
import bpy
from mathutils import Vector


def read_obj(path):
    v, f = [], []
    for line in open(path):
        if line.startswith("v "):
            v.append(tuple(float(x) for x in line.split()[1:4]))
        elif line.startswith("f "):
            f.append([int(x.split("/")[0]) - 1 for x in line.split()[1:]])
    return v, f


def main():
    dense, joints_path, out = sys.argv[sys.argv.index("--") + 1:][:3]
    v, f = read_obj(dense)
    me = bpy.data.meshes.new("m")
    me.from_pydata(v, [], f)
    bm = bmesh.new()
    bm.from_mesh(me)
    ring_edges = set()
    for j in json.load(open(joints_path)):
        h, a, r = Vector(j["head"]), Vector(j["axis"]).normalized(), j["r"]
        faces = [fc for fc in bm.faces if (fc.calc_center_median() - h).length < 2 * r]
        if not faces:
            continue
        geom = list({e for fc in faces for e in fc.edges}) + faces + list({x for fc in faces for x in fc.verts})
        res = bmesh.ops.bisect_plane(bm, geom=geom, plane_co=h, plane_no=a, dist=1e-9 * r)
        for e in res["geom_cut"]:
            if not isinstance(e, bmesh.types.BMEdge):
                continue
            ok = True
            for x in e.verts:
                rel = x.co - h
                radial = (rel - a * rel.dot(a)).length
                if abs(rel.dot(a)) > 1e-6 * r or radial > 2.5 * r:
                    ok = False
            if ok:
                ring_edges.add(e)
    bmesh.ops.triangulate(bm, faces=bm.faces)
    bm.verts.index_update()
    bm.faces.index_update()
    lines = []
    for e in ring_edges:
        if not e.is_valid or not e.link_faces:
            continue
        fc = e.link_faces[0]
        vs = [x for x in fc.verts]
        for k in range(3):
            if {vs[k], vs[(k + 1) % 3]} == set(e.verts):
                lines.append(f"0 {fc.index} {k}")
                break
    with open(out, "w") as fo:
        for x in bm.verts:
            fo.write(f"v {x.co.x:.7f} {x.co.y:.7f} {x.co.z:.7f}\n")
        for fc in bm.faces:
            fo.write("f " + " ".join(str(x.index + 1) for x in fc.verts) + "\n")
    with open(out[:-4] + ".sharp", "w") as fs:
        fs.write(f"{len(lines)}\n" + "\n".join(lines) + "\n")
    print("QW_RINGS", len(lines), "ring edges,", len(bm.faces), "faces")


main()
