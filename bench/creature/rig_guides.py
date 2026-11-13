"""Blender: retopo inputs made from the reference rig, in dense.obj's frame.

    blender -b --factory-startup -P bench/creature/rig_guides.py -- \
        REF_ALIGNED.glb DENSE.obj OUT_DIR [--bend 90] [--offsets 0]

REF_ALIGNED.glb is the reference rig moved into the bench frame (run.py's
`align` stage: transfer.py with the reference's own cut as the
candidate). Writes, in the OBJ file's own coordinates (+Y up, what retopo
reads):

  OUT_DIR/skeleton.txt  `--skeleton` file: every bone, parents first,
                        `hx hy hz tx ty tz parent bend` (bend for every
                        bone with a parent)
  OUT_DIR/rings.txt     `--guides` file: for each joint with skin on both
                        sides, the source surface cut by the plane through
                        the joint across the limb (the loop that goes
                        round the limb's axis, nearest the joint); with
                        --offsets a,b,... one more ring at each offset
                        along the axis, in limb radii
  OUT_DIR/joints.json   the joints (name, head, axis, limb radius, rings)
"""
import json
import math
import os
import sys

import bpy
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import metrics_lib as ml  # noqa: E402


def to_file(p):
    """Blender's default OBJ import maps file (x, y, z) to (x, -z, y)."""
    return np.array([p[0], p[2], -p[1]])


def load_obj(path):
    v, f = [], []
    with open(path) as fh:
        for line in fh:
            if line.startswith("v "):
                v.append([float(x) for x in line.split()[1:4]])
            elif line.startswith("f "):
                f.append([int(x.split("/")[0]) - 1 for x in line.split()[1:]])
    return np.array(v), np.array([x[:3] for x in f])


def cut(v, f, point, normal, near):
    """Plane-cut segments of the triangles within `near` of point."""
    c = v[f].mean(1)
    keep = np.linalg.norm(c - point, axis=1) < near
    tri = v[f[keep]]
    s = (tri - point) @ normal
    segs = []
    for t, d in zip(tri, s):
        pts = []
        for i in range(3):
            j = (i + 1) % 3
            if (d[i] < 0) != (d[j] < 0):
                a = d[i] / (d[i] - d[j])
                pts.append(t[i] + a * (t[j] - t[i]))
        if len(pts) == 2:
            segs.append((pts[0], pts[1]))
    return segs


def chain(segs, cell):
    key = lambda p: tuple(np.round(p / cell).astype(int))
    adj = {}
    for sid, (a, b) in enumerate(segs):
        ka, kb = key(a), key(b)
        if ka == kb:
            continue
        adj.setdefault(ka, []).append((kb, sid, b))
        adj.setdefault(kb, []).append((ka, sid, a))
    used, loops = set(), []
    for start in sorted(adj):
        for nb, sid, pt in adj[start]:
            if sid in used:
                continue
            loop, node, cur = [], start, (nb, sid, pt)
            ok = False
            for _ in range(len(segs) + 1):
                used.add(cur[1])
                loop.append(cur[2])
                node = cur[0]
                if node == start:
                    ok = True
                    break
                nxt = [x for x in adj.get(node, []) if x[1] not in used]
                if not nxt:
                    break
                cur = nxt[0]
            if ok and len(loop) >= 6:
                loops.append(np.array(loop))
    return loops


def winds(loop, point, axis):
    u = np.cross(axis, [1.0, 0, 0])
    if np.linalg.norm(u) < 0.3:
        u = np.cross(axis, [0, 1.0, 0])
    u /= np.linalg.norm(u)
    w = np.cross(axis, u)
    q = loop - point
    th = np.arctan2(q @ w, q @ u)
    d = np.diff(np.append(th, th[0]))
    d = (d + np.pi) % (2 * np.pi) - np.pi
    return abs(d.sum()) > 1.5 * np.pi


def main():
    a = sys.argv[sys.argv.index("--") + 1:]
    opt = {"bend": "90", "offsets": "0"}
    pos = []
    i = 0
    while i < len(a):
        if a[i].startswith("--"):
            opt[a[i][2:]] = a[i + 1]
            i += 2
        else:
            pos.append(a[i])
            i += 1
    ref, dense, out = pos
    offsets = [float(x) for x in opt["offsets"].split(",")]
    os.makedirs(out, exist_ok=True)
    bpy.ops.wm.read_factory_settings(use_empty=True)
    objs = ml.imported(lambda: bpy.ops.import_scene.gltf(filepath=ref, merge_vertices=True))
    arm = next(o for o in objs if o.type == "ARMATURE")
    for o in objs:
        o.animation_data_clear()
    bpy.context.view_layer.update()
    cm = max((o for o in objs if o.type == "MESH"), key=lambda o: len(o.data.vertices))
    joints = ml.find_joints(cm, arm)

    # skeleton, parents first
    am = arm.matrix_world
    bones = sorted(arm.data.bones, key=lambda b: len(b.parent_recursive))
    index = {b.name: k for k, b in enumerate(bones)}
    lines = ["# hx hy hz tx ty tz parent bend_degrees (OBJ frame), from the reference rig"]
    for b in bones:
        h, t = to_file(np.array(am @ b.head_local)), to_file(np.array(am @ b.tail_local))
        par = index[b.parent.name] if b.parent else -1
        bend = float(opt["bend"]) if b.parent else 0.0
        lines.append(" ".join(f"{x:.6f}" for x in (*h, *t)) + f" {par} {bend:g}")
    open(os.path.join(out, "skeleton.txt"), "w").write("\n".join(lines) + "\n")

    v, f = load_obj(dense)
    diag = float(np.linalg.norm(np.ptp(v, 0)))
    rings, info = [], []
    for j in joints:
        h, d, r = to_file(j["head"]), to_file(j["d"]), j["r"]
        got = []
        for off in offsets:
            p = h + off * r * d
            loops = chain(cut(v, f, p, d, 4 * r), 1e-5 * diag)
            good = [lp for lp in loops if winds(lp, p, d)
                    and np.linalg.norm(lp - p, axis=1).max() < 3 * r]
            if not good:
                continue
            best = min(good, key=lambda lp: np.linalg.norm(lp.mean(0) - p))
            rings.append(best)
            got.append(off)
        info.append({"bone": j["bone"], "parent": j["parent"], "head": h.tolist(), "axis": d.tolist(),
                     "r": r, "rings_at": got})
    with open(os.path.join(out, "rings.txt"), "w") as fh:
        fh.write("# joint rings from the reference rig (OBJ frame)\n")
        for lp in rings:
            for p in np.vstack([lp, lp[:1]]):
                fh.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")
            fh.write("\n")
    json.dump(info, open(os.path.join(out, "joints.json"), "w"), indent=1)
    print("RIG_GUIDES", len(bones), "bones", len(joints), "joints", len(rings), "rings")


main()
