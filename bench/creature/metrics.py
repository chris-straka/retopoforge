"""Blender: score one rigged candidate against its source surface and rig.

    blender -b --factory-startup -P bench/creature/metrics.py -- \
        CANDIDATE_RIGGED.glb DENSE.obj OUT.json [CANDIDATE.obj] [--bend 60] [--no-poses]

CANDIDATE_RIGGED.glb comes from bench/creature/transfer.py (candidate mesh,
the reference armature, weights carried over); DENSE.obj is the source.
A GLB holds triangles only, so the faces (quads, loops, poles) are read
from CANDIDATE.obj when given, its weights matched by vertex position
from the GLB. A prop has no rig: pass its OBJ as the first argument.
Both in the bench frame. Every length is a share of the source's
bounding-box diagonal (%), every joint size a share of that joint's limb
radius. Writes one JSON object:

  tris, quads, tris_raw, ngons, verts, irr_pct, open_edges
      the census; `tris` counts a quad as two triangles (the budget)
  dist_mean, dist_p95, dist_max
      two-sided surface distance (candidate vertices and face centres to
      the source, a fixed sample of source vertices to the candidate)
  ndev_mean, ndev_p95
      angle between each candidate face's normal and the source's normal
      at the nearest point, degrees
  rest_flips
      candidate faces whose normal points against the source's (> 90 deg)
  joints: per joint (a bone with skin whose head is its parent's tail)
      loops   closed edge loops that ring the limb within one limb radius
              of the joint (they go once round the bone's axis, stay
              within one radius along it, and stay near the limb)
      poles   irregular vertices within one limb radius of the joint
      flips   faces whose posed normal turns more than 90 deg away from
              its rest normal carried by its bone, summed over the joint's
              poses (the bone bent --bend degrees toward each of the
              world's up/down/front/back/left/right that is across it)
  joint_loops_mean, joints_ringed_pct (>= 2 loops), joint_poles_mean,
  posed_flips (all poses), posed_flip_poses_pct (poses with any flip)
"""
import json
import math
import sys

import bmesh
import bpy
import os

import numpy as np
from mathutils import Matrix, Vector
from mathutils.bvhtree import BVHTree

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import metrics_lib as ml  # noqa: E402
from metrics_lib import coords, face_lists, imported, newell, with_faces_of  # noqa: E402


def args():
    a = sys.argv[sys.argv.index("--") + 1:]
    o = {"bend": 60.0, "poses": True}
    pos = []
    i = 0
    while i < len(a):
        if a[i] == "--bend":
            o["bend"] = float(a[i + 1])
            i += 2
        elif a[i] == "--no-poses":
            o["poses"] = False
            i += 1
        else:
            pos.append(a[i])
            i += 1
    if len(pos) not in (3, 4):
        raise SystemExit(__doc__)
    if len(pos) == 3:
        pos.append(None)
    return pos, o




def pct(x, q):
    return float(np.percentile(x, q)) if len(x) else 0.0


def main():
    (cand_path, dense_path, out_path, faces_path), opt = args()
    bpy.ops.wm.read_factory_settings(use_empty=True)
    src_objs = imported(lambda: bpy.ops.wm.obj_import(filepath=dense_path))
    src = next(o for o in src_objs if o.type == "MESH")
    sv = coords(src)
    sf = face_lists(src.data)
    diag = float(np.linalg.norm(np.ptp(sv, 0)))
    sbvh = BVHTree.FromPolygons([tuple(x) for x in sv], sf)

    if cand_path.lower().endswith(".obj"):
        c_objs = imported(lambda: bpy.ops.wm.obj_import(filepath=cand_path))
    else:
        c_objs = imported(lambda: bpy.ops.import_scene.gltf(filepath=cand_path, merge_vertices=True))
    arm = next((o for o in c_objs if o.type == "ARMATURE"), None)
    cm = max((o for o in c_objs if o.type == "MESH"), key=lambda o: len(o.data.vertices))
    for o in c_objs:
        o.animation_data_clear()
    if arm:
        for pb in arm.pose.bones:
            pb.matrix_basis = Matrix.Identity(4)
    bpy.context.view_layer.update()
    if faces_path:
        cm = with_faces_of(cm, arm, faces_path)
    me = cm.data
    cf = face_lists(me)
    cv = coords(cm)
    out = {}

    # ---- census
    sizes = np.array([len(f) for f in cf])
    out["quads"] = int((sizes == 4).sum())
    out["tris_raw"] = int((sizes == 3).sum())
    out["ngons"] = int((sizes > 4).sum())
    out["tris"] = int((sizes - 2).sum())
    out["verts"] = len(me.vertices)
    bm = bmesh.new()
    bm.from_mesh(me)
    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    boundary = np.zeros(len(bm.verts), bool)
    open_edges = 0
    for e in bm.edges:
        if len(e.link_faces) == 1:
            open_edges += 1
            boundary[e.verts[0].index] = boundary[e.verts[1].index] = True
    val = np.array([len(v.link_edges) for v in bm.verts])
    irregular = np.where(boundary, val != 3, val != 4)
    out["irr_pct"] = round(100.0 * irregular.mean(), 3)
    out["open_edges"] = open_edges

    # ---- distance and normals
    cn, cc, ca = newell(cv, cf)
    probe = np.vstack([cv, cc])
    d1 = np.array([sbvh.find_nearest(tuple(p))[3] for p in probe])
    rng = np.random.default_rng(0)
    pick = rng.choice(len(sv), size=min(30000, len(sv)), replace=False)
    cbvh = BVHTree.FromPolygons([tuple(x) for x in cv], cf)
    d2 = np.array([cbvh.find_nearest(tuple(sv[i]))[3] for i in pick])
    both = np.concatenate([d1, d2]) / diag * 100
    out["dist_mean"] = round(float(both.mean()), 4)
    out["dist_p95"] = round(pct(both, 95), 4)
    out["dist_max"] = round(float(both.max()), 4)
    ang = []
    flips = 0
    for k in range(len(cf)):
        hit = sbvh.find_nearest(tuple(cc[k]))
        if hit[0] is None or ca[k] == 0:
            continue
        dot = float(np.dot(cn[k], np.array(hit[1])))
        ang.append(math.degrees(math.acos(max(-1.0, min(1.0, dot)))))
        if dot < 0:
            flips += 1
    ang = np.array(ang)
    out["ndev_mean"] = round(float(ang.mean()), 3)
    out["ndev_p95"] = round(pct(ang, 95), 3)
    out["rest_flips"] = flips

    if arm is None:
        out["joints"] = {}
        json.dump(out, open(out_path, "w"), indent=1, sort_keys=True)
        return

    # ---- weights: dominant bone per vertex and per face
    names, w = ml.weights_of(cm)
    dom_v = np.where(w.max(1) > 0, w.argmax(1), -1)
    dom_f = np.array([int(w[list(f)].sum(0).argmax()) if w[list(f)].sum() > 0 else -1 for f in cf])
    skinned = set(np.nonzero((w > 0.1).any(0))[0])

    am = arm.matrix_world
    gi = {n: i for i, n in enumerate(names)}
    joints = ml.find_joints(cm, arm, w, names)

    # ---- closed edge loops
    loops = []
    seen = set()
    for e0 in bm.edges:
        if e0.index in seen or len(e0.link_faces) != 2:
            continue
        chain = [e0]
        seen.add(e0.index)
        e, v = e0, e0.verts[1]
        closed = False
        for _ in range(len(bm.edges)):
            if boundary[v.index] or len(v.link_edges) != 4 or len(v.link_faces) != 4 \
                    or any(len(f.verts) != 4 for f in v.link_faces):
                break
            fs = set(e.link_faces)
            nxt = [x for x in v.link_edges if x is not e and not (set(x.link_faces) & fs)]
            if len(nxt) != 1:
                break
            e = nxt[0]
            if e is e0:
                closed = True
                break
            if e.index in seen:
                break
            seen.add(e.index)
            chain.append(e)
            v = e.other_vert(v)
        if closed and len(chain) >= 3:
            vs = []
            a = chain[0].verts[0]
            for ed in chain:
                vs.append(a.index)
                a = ed.other_vert(a)
            loops.append(np.array(vs))
    loop_pts = [cv[ix] for ix in loops]
    loop_ctr = np.array([p.mean(0) for p in loop_pts]) if loops else np.zeros((0, 3))

    res = {}
    for j in joints:
        h, d, r = j["head"], j["d"], j["r"]
        u = np.cross(d, [1.0, 0, 0])
        if np.linalg.norm(u) < 0.3:
            u = np.cross(d, [0, 1.0, 0])
        u /= np.linalg.norm(u)
        wv = np.cross(d, u)
        n_loops = 0
        if len(loops):
            near = np.nonzero(np.linalg.norm(loop_ctr - h, axis=1) < 3 * r)[0]
            for k in near:
                p = loop_pts[k] - h
                t = p @ d
                if abs(t.mean()) > r or np.ptp(t) > r:
                    continue
                q = p - np.outer(t, d)
                if np.linalg.norm(q, axis=1).max() > 2.5 * r:
                    continue
                th = np.arctan2(q @ wv, q @ u)
                dth = np.diff(np.append(th, th[0]))
                dth = (dth + np.pi) % (2 * np.pi) - np.pi
                if abs(dth.sum()) > 1.5 * np.pi:
                    n_loops += 1
        rel = cv - h
        ax = rel @ d
        rad = np.linalg.norm(rel - np.outer(ax, d), axis=1)
        band = (np.abs(ax) <= r) & (rad <= 1.5 * r) & np.isin(dom_v, [gi[j["bone"]], gi[j["parent"]]])
        res[j["bone"]] = {"loops": n_loops, "poles": int((band & irregular).sum()), "r_pct": round(r / diag * 100, 3)}

    # ---- posed flips
    n_poses = n_flip_poses = total_flips = 0
    if opt["poses"]:
        amat = np.array(am)
        ainv = np.linalg.inv(amat)
        rest_n, _, _ = newell(cv, cf)
        dg = bpy.context.evaluated_depsgraph_get()
        theta = math.radians(opt["bend"])
        for j in joints:
            pb = arm.pose.bones[j["bone"]]
            f_in = np.isin(dom_f, list(j["sub"]))
            jf = 0
            for target in np.vstack([np.eye(3), -np.eye(3)]):
                if abs(np.dot(target, j["db"])) > 0.5:
                    continue
                axis = np.cross(j["db"], target)
                axis /= np.linalg.norm(axis)
                rw = np.array(Matrix.Rotation(theta, 3, Vector(axis)))
                # world rotation about the joint, expressed in armature space
                mw = np.eye(4)
                mw[:3, :3] = rw
                mw[:3, 3] = j["head"] - rw @ j["head"]
                marm = ainv @ mw @ amat
                pb.matrix = Matrix(marm.tolist()) @ pb.bone.matrix_local
                bpy.context.view_layer.update()
                ev = cm.evaluated_get(dg)
                tm = ev.to_mesh()
                pv = coords(cm, tm)
                ev.to_mesh_clear()
                pn, _, _ = newell(pv, cf)
                carried = np.where(f_in[:, None], rest_n @ rw.T, rest_n)
                fl = int(((pn * carried).sum(1) < 0).sum())
                pb.matrix_basis = Matrix.Identity(4)
                n_poses += 1
                jf += fl
                total_flips += fl
                n_flip_poses += fl > 0
            res[j["bone"]]["flips"] = jf
        bpy.context.view_layer.update()

    out["joints"] = res
    nj = len(res)
    out["n_joints"] = nj
    out["joint_loops_mean"] = round(float(np.mean([min(x["loops"], 3) for x in res.values()])) if nj else 0.0, 3)
    out["joints_ringed_pct"] = round(100.0 * np.mean([x["loops"] >= 2 for x in res.values()]) if nj else 0.0, 2)
    out["joint_poles_mean"] = round(float(np.mean([x["poles"] for x in res.values()])) if nj else 0.0, 3)
    out["posed_flips"] = total_flips
    out["posed_poses"] = n_poses
    out["posed_flip_poses_pct"] = round(100.0 * n_flip_poses / n_poses, 2) if n_poses else 0.0
    out["n_loops"] = len(loops)
    json.dump(out, open(out_path, "w"), indent=1, sort_keys=True)
    print("METRICS", json.dumps({k: v for k, v in out.items() if k != "joints"}))


main()
