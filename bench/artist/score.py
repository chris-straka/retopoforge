"""Blender (for its BVH and KD trees only): score a tool's cuts against
the artists' meshes.

    blender -b -P bench/artist/score.py -- CASES_DIR CUTS_DIR OUT.json [--set fast|full|all]

CASES_DIR/<name>/{pro.obj,dense.obj,case.json} come from make_cases.py;
CUTS_DIR/<name>.obj is the tool's remesh of dense.obj. Per case:

  rings_pct   the headline: of the artist's closed edge loops (rings round
              an eye, a mouth, a finger, a limb, a barrel), the share the
              cut also has. A ring counts when one closed loop of the cut
              runs along it: within 0.75 of the artist's local ring spacing
              for at least 80% of the ring, and between half and 1.6 times
              its length. A cut whose rings run the same way at the same or
              higher density gets them all; flow that runs another way gets
              none.
  flow_deg    mean angle between the cut's quad edges and the nearer edge
              direction of the artist's quad under them, folded to 0-45
              degrees (0: the artist's flow; 22.5: unrelated).
  poles_x     the cut's irregular vertices as a multiple of the artist's.
  poles_near  share of the cut's irregular vertices within two artist edge
              lengths of one of the artist's (the artist's quiet places).
  odd         faces that are not quads.
  count_x     quads as a multiple of the artist's quad count.
  d95, dmax   two-sided distance to the dense shape, % of its diagonal.
  dens_err    does the cut put its small quads where the artist did? Each
              cut face's area against the artist's face under it, as log2
              of the ratio; the median ratio (the count difference) is
              taken off, and the area-weighted mean of what is left is the
              score (0: the artist's density everywhere; 1: off by 2x in
              area on average; a uniform cut of a face-heavy head scores
              high).

OUT.json holds every case and the medians and means over them.
"""
import json
import math
import os
import sys
from collections import defaultdict

import numpy as np
from mathutils import Vector
from mathutils.bvhtree import BVHTree
from mathutils.kdtree import KDTree


def read_obj(path):
    v, f = [], []
    for line in open(path):
        if line.startswith("v "):
            v.append([float(x) for x in line.split()[1:4]])
        elif line.startswith("f "):
            f.append([int(x.split("/")[0]) - 1 for x in line.split()[1:]])
    return np.array(v), f


class Topo:
    def __init__(self, v, faces):
        self.v, self.faces = v, faces
        self.edge_faces = defaultdict(list)
        self.vert_edges = defaultdict(set)
        self.vert_faces = defaultdict(list)
        for k, f in enumerate(faces):
            for i in range(len(f)):
                a, b = f[i], f[(i + 1) % len(f)]
                e = (a, b) if a < b else (b, a)
                self.edge_faces[e].append(k)
                self.vert_edges[a].add(e)
                self.vert_edges[b].add(e)
            for a in f:
                self.vert_faces[a].append(k)
        self.boundary = set()
        for e, fs in self.edge_faces.items():
            if len(fs) != 2:
                self.boundary.update(e)
        n = len(v)
        self.regular = np.zeros(n, bool)
        self.irregular = np.zeros(n, bool)
        for a in range(n):
            if a not in self.vert_edges:
                continue
            val = len(self.vert_edges[a])
            if a in self.boundary:
                continue
            if val == 4 and all(len(faces[k]) == 4 for k in self.vert_faces[a]):
                self.regular[a] = True
            elif val != 4:
                self.irregular[a] = True

    def closed_loops(self, min_edges=6):
        seen, loops = set(), []
        for e0 in self.edge_faces:
            if e0 in seen or len(self.edge_faces[e0]) != 2:
                continue
            seen.add(e0)
            chain, e, at = [e0], e0, e0[1]
            closed = False
            for _ in range(len(self.edge_faces)):
                if not self.regular[at]:
                    break
                fs = set(self.edge_faces[e])
                nxt = [x for x in self.vert_edges[at] if x != e and not (set(self.edge_faces[x]) & fs)]
                if len(nxt) != 1:
                    break
                e = nxt[0]
                if e == e0:
                    closed = True
                    break
                if e in seen:
                    break
                seen.add(e)
                chain.append(e)
                at = e[0] if e[1] == at else e[1]
            if closed and len(chain) >= min_edges:
                verts, a = [], chain[0][0]
                for ed in chain:
                    verts.append(a)
                    a = ed[0] if ed[1] == a else ed[1]
                loops.append((verts, set(chain)))
        return loops


def loop_points(v, verts):
    p = v[verts]
    return np.vstack([p, (p + np.roll(p, -1, axis=0)) / 2])


def loop_length(v, verts):
    p = v[verts]
    return float(np.linalg.norm(p - np.roll(p, -1, axis=0), axis=1).sum())


def rings_recall(pro, cut):
    """Share of pro's closed loops that cut also has (see module docs)."""
    la, lc = pro.closed_loops(), cut.closed_loops()
    if not la:
        return None, 0, len(lc)
    if not lc:
        return 0.0, len(la), 0
    pts, ids = [], []
    for k, (verts, _) in enumerate(lc):
        p = loop_points(cut.v, verts)
        pts.append(p)
        ids += [k] * len(p)
    pts = np.vstack(pts)
    kd = KDTree(len(pts))
    for i, p in enumerate(pts):
        kd.insert(p, i)
    kd.balance()
    len_c = [loop_length(cut.v, verts) for verts, _ in lc]
    got = 0
    for verts, edges in la:
        cross = []
        for a in verts:
            for e in pro.vert_edges[a]:
                if e not in edges:
                    cross.append(np.linalg.norm(pro.v[e[0]] - pro.v[e[1]]))
        if not cross:
            continue
        tol = 0.75 * float(np.mean(cross))
        samples = loop_points(pro.v, verts)
        counts = defaultdict(int)
        for p in samples:
            for k in {ids[i] for _, i, _ in kd.find_range(p, tol)}:
                counts[k] += 1
        length = loop_length(pro.v, verts)
        if any(c >= 0.8 * len(samples) and 0.5 * length <= len_c[k] <= 1.6 * length for k, c in counts.items()):
            got += 1
    return 100.0 * got / len(la), len(la), len(lc)


def face_normals(v, faces):
    n = np.zeros((len(faces), 3))
    c = np.zeros((len(faces), 3))
    for k, f in enumerate(faces):
        p = v[f]
        q = np.roll(p, -1, axis=0)
        n[k] = [((p[:, 1] - q[:, 1]) * (p[:, 2] + q[:, 2])).sum(), ((p[:, 2] - q[:, 2]) * (p[:, 0] + q[:, 0])).sum(),
                ((p[:, 0] - q[:, 0]) * (p[:, 1] + q[:, 1])).sum()]
        ln = np.linalg.norm(n[k])
        if ln > 0:
            n[k] /= ln
        c[k] = p.mean(0)
    return n, c


def flow(pro, cut):
    quads = [f for f in pro.faces if len(f) == 4]
    if not quads:
        return None
    bvh = BVHTree.FromPolygons([tuple(x) for x in pro.v], quads)
    qn, _ = face_normals(pro.v, quads)
    d0 = np.array([pro.v[f[1]] - pro.v[f[0]] for f in quads])
    d1 = np.array([pro.v[f[2]] - pro.v[f[1]] for f in quads])
    angs = []
    for f in cut.faces:
        if len(f) != 4:
            continue
        for k in range(4):
            a, b = cut.v[f[k]], cut.v[f[(k + 1) % 4]]
            hit = bvh.find_nearest(tuple((a + b) / 2))
            if hit[0] is None:
                continue
            q = hit[2]
            n = qn[q]
            e = (b - a) - n * np.dot(b - a, n)
            best = 90.0
            for dd in (d0[q], d1[q]):
                dd = dd - n * np.dot(dd, n)
                den = np.linalg.norm(e) * np.linalg.norm(dd)
                if den > 0:
                    best = min(best, math.degrees(math.acos(min(1.0, abs(np.dot(e, dd)) / den))))
            if best < 90.0:
                angs.append(min(best, 90.0 - best))
    return float(np.mean(angs)) if angs else None


def poles(pro, cut):
    pa, pc = np.nonzero(pro.irregular)[0], np.nonzero(cut.irregular)[0]
    ratio = len(pc) / max(1, len(pa))
    if not len(pa) or not len(pc):
        return ratio, None, len(pa), len(pc)
    lens = [np.linalg.norm(pro.v[a] - pro.v[b]) for a, b in pro.edge_faces]
    reach = 2.0 * float(np.mean(lens))
    kd = KDTree(len(pa))
    for i, a in enumerate(pa):
        kd.insert(pro.v[a], i)
    kd.balance()
    near = sum(1 for c in pc if kd.find(cut.v[c])[2] <= reach)
    return ratio, 100.0 * near / len(pc), len(pa), len(pc)


def face_areas(v, faces):
    a = np.zeros(len(faces))
    for k, f in enumerate(faces):
        p = v[f]
        for i in range(1, len(f) - 1):
            a[k] += 0.5 * np.linalg.norm(np.cross(p[i] - p[0], p[i + 1] - p[0]))
    return a


def density_match(pro, cut):
    if not pro.faces or not cut.faces:
        return None
    bvh = BVHTree.FromPolygons([tuple(x) for x in pro.v], pro.faces)
    pa, ca = face_areas(pro.v, pro.faces), face_areas(cut.v, cut.faces)
    _, cc = face_normals(cut.v, cut.faces)
    logs, w = [], []
    for k, c in enumerate(cc):
        hit = bvh.find_nearest(tuple(c))
        if hit[0] is None or pa[hit[2]] <= 0 or ca[k] <= 0:
            continue
        logs.append(math.log2(ca[k] / pa[hit[2]]))
        w.append(ca[k])
    if not logs:
        return None
    logs, w = np.array(logs), np.array(w)
    return float(np.sum(np.abs(logs - np.median(logs)) * w) / np.sum(w))


def distance(dense_v, dense_f, cut):
    diag = float(np.linalg.norm(np.ptp(dense_v, 0)))
    dbvh = BVHTree.FromPolygons([tuple(x) for x in dense_v], dense_f)
    _, cc = face_normals(cut.v, cut.faces)
    d1 = np.array([dbvh.find_nearest(tuple(p))[3] for p in np.vstack([cut.v, cc])])
    cbvh = BVHTree.FromPolygons([tuple(x) for x in cut.v], cut.faces)
    rng = np.random.default_rng(0)
    pick = rng.choice(len(dense_v), size=min(15000, len(dense_v)), replace=False)
    d2 = np.array([cbvh.find_nearest(tuple(dense_v[i]))[3] for i in pick])
    both = np.concatenate([d1, d2]) / diag * 100
    return float(np.percentile(both, 95)), float(both.max())


def score_case(d, cut_path):
    case = json.load(open(os.path.join(d, "case.json")))
    pv, pf = read_obj(os.path.join(d, "pro.obj"))
    cv, cf = read_obj(cut_path)
    if not len(cv) or not cf:
        return {"name": case["name"], "kind": case["kind"], "error": "empty cut"}
    dv, df = read_obj(os.path.join(d, "dense.obj"))
    pro, cut = Topo(pv, pf), Topo(cv, cf)
    rings, n_pro, n_cut = rings_recall(pro, cut)
    ratio, near, n_pa, n_pc = poles(pro, cut)
    d95, dmax = distance(dv, df, cut)
    quads = sum(1 for f in cf if len(f) == 4)
    fl = flow(pro, cut)
    de = density_match(pro, cut)
    r = lambda x, k=2: None if x is None else round(x, k)
    return {"name": case["name"], "kind": case["kind"], "rings_pct": r(rings, 1), "rings_pro": n_pro, "rings_cut": n_cut,
            "flow_deg": r(fl), "poles_x": r(ratio), "poles_near": r(near, 1), "poles_pro": n_pa, "poles": n_pc,
            "odd": len(cf) - quads, "quads": quads, "count_x": r(quads / max(1, case["quads"])),
            "d95": r(d95, 3), "dmax": r(dmax, 3), "dens_err": r(de, 3)}


KEYS = ["rings_pct", "flow_deg", "poles_x", "poles_near", "odd", "count_x", "d95", "dmax", "dens_err"]


def main():
    a = sys.argv[sys.argv.index("--") + 1:]
    which = "fast"
    if "--set" in a:
        i = a.index("--set")
        which = a[i + 1]
        del a[i:i + 2]
    cases_dir, cuts, out = a
    rows = []
    for name in sorted(os.listdir(cases_dir)):
        d = os.path.join(cases_dir, name)
        cj = os.path.join(d, "case.json")
        if not os.path.exists(cj):
            continue
        if which != "all" and json.load(open(cj)).get("set", "fast") != which:
            continue
        cut = os.path.join(cuts, name + ".obj")
        if not os.path.exists(cut):
            rows.append({"name": name, "kind": json.load(open(cj))["kind"], "error": "no cut"})
            continue
        rows.append(score_case(d, cut))
    summary = {"cases": len(rows), "failed": sum(1 for r in rows if "error" in r)}
    for k in KEYS:
        vals = [r[k] for r in rows if r.get(k) is not None]
        if vals:
            summary[k + "_median"] = round(float(np.median(vals)), 3)
            summary[k + "_mean"] = round(float(np.mean(vals)), 3)
    json.dump({"summary": summary, "rows": rows}, open(out, "w"), indent=1)
    print("ARTIST", json.dumps(summary))


main()
