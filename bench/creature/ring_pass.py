"""Blender: a joint-ring pass for any mesh (quads or triangles).

    blender -b --factory-startup -P bench/creature/ring_pass.py -- \
        CUT.obj JOINTS.json OUT.obj [--rings 3] [--span 0.5] [--reach 1.8] [--trunk 1.6]

Professional game meshes carry 1.5 to 4 times more edge rings at a knee
or elbow than along the limb (game-tools `docs/reference-lessons.md`).
This pass spends triangles there, placed from the skeleton: at every
joint in JOINTS.json (bench/creature/rig_guides.py: head, axis and limb
radius in the OBJ's own frame) it cuts `--rings` planes across the limb,
spread over `--span` limb radii either side of the joint (3 rings at
span 0.5: -0.5 r, 0, +0.5 r). Only faces of that limb are cut: those
within `--reach` radii of the limb's axis and within span + 1 radii of
the joint along it. Joints thicker than `--trunk` times the median
joint radius (the spine and chest) are left alone: a ring around the
trunk costs the most triangles and bends the least. A plane that passes within a sixth of the local edge
length of a vertex uses that vertex (no slivers). New many-sided faces
are split back into triangles and quads. Positions of existing vertices
do not move; every new vertex lies on an old edge, so the shape and the
silhouette stay as they were.

Prints `RING_PASS <joints cut> <faces before> <faces after>`.
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
    a = sys.argv[sys.argv.index("--") + 1:]
    o = {"rings": "3", "span": "0.5", "reach": "1.8", "trunk": "1.6"}
    pos = []
    i = 0
    while i < len(a):
        if a[i].startswith("--"):
            o[a[i][2:]] = a[i + 1]
            i += 2
        else:
            pos.append(a[i])
            i += 1
    cut, joints_path, out = pos
    rings, span, reach, trunk = int(o["rings"]), float(o["span"]), float(o["reach"]), float(o["trunk"])
    v, f = read_obj(cut)
    me = bpy.data.meshes.new("m")
    me.from_pydata(v, [], f)
    bm = bmesh.new()
    bm.from_mesh(me)
    before = len(bm.faces)
    done = 0
    joints = json.load(open(joints_path))
    radii = sorted(j["r"] for j in joints)
    median_r = radii[len(radii) // 2] if radii else 0.0
    for j in joints:
        h, ax, r = Vector(j["head"]), Vector(j["axis"]).normalized(), j["r"]
        if r > trunk * median_r:
            continue  # a trunk joint: a ring there costs the most and bends the least
        offsets = [0.0] if rings == 1 else [(-1 + 2 * k / (rings - 1)) * span * r for k in range(rings)]
        touched = False
        for off in offsets:
            p = h + ax * off
            faces = []
            for fc in bm.faces:
                c = fc.calc_center_median() - h
                t = c.dot(ax)
                if abs(t) <= (span + 1.0) * r and (c - ax * t).length <= reach * r:
                    faces.append(fc)
            if not faces:
                continue
            edges = list({e for fc in faces for e in fc.edges})
            verts = list({x for fc in faces for x in fc.verts})
            local = sorted(e.calc_length() for e in edges)
            snap = local[len(local) // 2] / 6.0
            bmesh.ops.bisect_plane(bm, geom=verts + edges + faces, plane_co=p, plane_no=ax, dist=snap)
            touched = True
        done += touched
    # many-sided faces from the cuts back to triangles and quads
    big = [fc for fc in bm.faces if len(fc.verts) > 4]
    if big:
        res = bmesh.ops.triangulate(bm, faces=big)
        bmesh.ops.join_triangles(bm, faces=res["faces"], cmp_seam=False, cmp_sharp=False, cmp_uvs=False,
                                 cmp_vcols=False, cmp_materials=False, angle_face_threshold=0.7,
                                 angle_shape_threshold=0.7)
    bm.verts.index_update()
    with open(out, "w") as fo:
        for x in bm.verts:
            fo.write(f"v {x.co.x:.7f} {x.co.y:.7f} {x.co.z:.7f}\n")
        for fc in bm.faces:
            fo.write("f " + " ".join(str(x.index + 1) for x in fc.verts) + "\n")
    print("RING_PASS", done, before, len(bm.faces))


main()
