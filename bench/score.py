#!/usr/bin/env python3
"""retopoforge quad-quality scorecard (stdlib only).

Scores a remeshed OBJ against its source mesh:

  quads / non_quads / verts   face census of the output
  yield                       quads / target (when --target is given)
  irr_pct                     irregular vertices, % of all output verts
                              (interior valence != 4, boundary valence != 3)
  angdev_mean / angdev_p95    |corner angle - 90 deg| over all quad corners
  dist_mean / dist_max        two-sided surface distance (output verts ->
                              source surface and sampled source verts ->
                              output surface), % of source bbox diagonal

Usage:
  bench/score.py --source bench/models/armadillo.obj --output out.obj \
      [--target 5000] [--json]

Library use: `score(source_path, output_path, target=None)` returns the
metric dict; `load_obj` / `rank_sum_pick` are reused by bench/noise.py.
"""

import argparse
import json
import math
import sys

MAX_SOURCE_SAMPLES = 20000


def load_obj(path):
    """Return (verts, faces): verts as (x, y, z) tuples, faces as 0-based
    index lists. Negative (relative) indices are resolved."""
    verts = []
    faces = []
    with open(path, "r", errors="replace") as f:
        for line in f:
            if line.startswith("v "):
                p = line.split()
                verts.append((float(p[1]), float(p[2]), float(p[3])))
            elif line.startswith("f "):
                idx = []
                for tok in line.split()[1:]:
                    i = int(tok.split("/")[0])
                    idx.append(i - 1 if i > 0 else len(verts) + i)
                faces.append(idx)
    return verts, faces


def bbox_diag(verts):
    lo = [min(v[k] for v in verts) for k in range(3)]
    hi = [max(v[k] for v in verts) for k in range(3)]
    return math.dist(lo, hi)


def triangulate(faces):
    tris = []
    for f in faces:
        for k in range(1, len(f) - 1):
            tris.append((f[0], f[k], f[k + 1]))
    return tris


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def point_triangle_dist2(p, a, b, c):
    """Squared distance from p to triangle abc (Ericson, RTCD 5.1.5)."""
    ab = _sub(b, a)
    ac = _sub(c, a)
    ap = _sub(p, a)
    d1 = _dot(ab, ap)
    d2 = _dot(ac, ap)
    if d1 <= 0 and d2 <= 0:
        q = a
    else:
        bp = _sub(p, b)
        d3 = _dot(ab, bp)
        d4 = _dot(ac, bp)
        if d3 >= 0 and d4 <= d3:
            q = b
        else:
            vc = d1 * d4 - d3 * d2
            if vc <= 0 and d1 >= 0 and d3 <= 0:
                t = d1 / (d1 - d3) if d1 != d3 else 0.0
                q = (a[0] + t * ab[0], a[1] + t * ab[1], a[2] + t * ab[2])
            else:
                cp = _sub(p, c)
                d5 = _dot(ab, cp)
                d6 = _dot(ac, cp)
                if d6 >= 0 and d5 <= d6:
                    q = c
                else:
                    vb = d5 * d2 - d1 * d6
                    if vb <= 0 and d2 >= 0 and d6 <= 0:
                        t = d2 / (d2 - d6) if d2 != d6 else 0.0
                        q = (a[0] + t * ac[0], a[1] + t * ac[1],
                             a[2] + t * ac[2])
                    else:
                        va = d3 * d6 - d5 * d4
                        if va <= 0 and (d4 - d3) >= 0 and (d5 - d6) >= 0:
                            den = (d4 - d3) + (d5 - d6)
                            t = (d4 - d3) / den if den else 0.0
                            q = (b[0] + t * (c[0] - b[0]),
                                 b[1] + t * (c[1] - b[1]),
                                 b[2] + t * (c[2] - b[2]))
                        else:
                            if va + vb + vc == 0:
                                # Degenerate (collinear) triangle: the
                                # nearest corner bounds the distance.
                                return min(_dot(_sub(p, x), _sub(p, x))
                                           for x in (a, b, c))
                            denom = 1.0 / (va + vb + vc)
                            v = vb * denom
                            w = vc * denom
                            q = (a[0] + ab[0] * v + ac[0] * w,
                                 a[1] + ab[1] * v + ac[1] * w,
                                 a[2] + ab[2] * v + ac[2] * w)
    d = _sub(p, q)
    return _dot(d, d)


class TriangleGrid:
    """Uniform hash grid over a triangle set for closest-point queries."""

    def __init__(self, verts, tris):
        self.verts = verts
        self.tris = tris
        total = 0.0
        for a, b, c in tris:
            total += math.dist(verts[a], verts[b])
        mean_edge = total / max(len(tris), 1)
        self.h = max(2.0 * mean_edge, 1e-12)
        self.cells = {}
        h = self.h
        for t, (a, b, c) in enumerate(tris):
            pa, pb, pc = verts[a], verts[b], verts[c]
            lo = [math.floor(min(pa[k], pb[k], pc[k]) / h) for k in range(3)]
            hi = [math.floor(max(pa[k], pb[k], pc[k]) / h) for k in range(3)]
            for i in range(lo[0], hi[0] + 1):
                for j in range(lo[1], hi[1] + 1):
                    for k in range(lo[2], hi[2] + 1):
                        self.cells.setdefault((i, j, k), []).append(t)
        keys = self.cells.keys()
        self.max_ring = max(
            max(k[d] for k in keys) - min(k[d] for k in keys) for d in range(3)
        ) + 1

    def dist(self, p):
        h = self.h
        ci = [math.floor(p[k] / h) for k in range(3)]
        best = math.inf
        seen = set()
        for r in range(self.max_ring + 1):
            for i in range(ci[0] - r, ci[0] + r + 1):
                for j in range(ci[1] - r, ci[1] + r + 1):
                    for k in range(ci[2] - r, ci[2] + r + 1):
                        if max(abs(i - ci[0]), abs(j - ci[1]),
                               abs(k - ci[2])) != r:
                            continue
                        for t in self.cells.get((i, j, k), ()):
                            if t in seen:
                                continue
                            seen.add(t)
                            a, b, c = self.tris[t]
                            d2 = point_triangle_dist2(
                                p, self.verts[a], self.verts[b], self.verts[c])
                            if d2 < best:
                                best = d2
            # Every unvisited cell is at least r*h away from p.
            if best <= (r * h) ** 2:
                break
        return math.sqrt(best)


def topology_metrics(verts, faces):
    edge_faces = {}
    for f in faces:
        n = len(f)
        for i in range(n):
            a, b = f[i], f[(i + 1) % n]
            e = (a, b) if a < b else (b, a)
            edge_faces[e] = edge_faces.get(e, 0) + 1
    valence = {}
    boundary = set()
    for (a, b), n in edge_faces.items():
        valence[a] = valence.get(a, 0) + 1
        valence[b] = valence.get(b, 0) + 1
        if n == 1:
            boundary.add(a)
            boundary.add(b)
    irr = 0
    for v, n in valence.items():
        if n != (3 if v in boundary else 4):
            irr += 1
    devs = []
    for f in faces:
        if len(f) != 4:
            continue
        for i in range(4):
            p0, p1, p2 = verts[f[i - 1]], verts[f[i]], verts[f[(i + 1) % 4]]
            u = _sub(p0, p1)
            w = _sub(p2, p1)
            nu = math.sqrt(_dot(u, u))
            nw = math.sqrt(_dot(w, w))
            if nu == 0 or nw == 0:
                devs.append(90.0)
                continue
            c = max(-1.0, min(1.0, _dot(u, w) / (nu * nw)))
            devs.append(abs(math.degrees(math.acos(c)) - 90.0))
    devs.sort()
    quads = sum(1 for f in faces if len(f) == 4)
    return {
        "quads": quads,
        "non_quads": len(faces) - quads,
        "verts": len(valence),
        "irr_pct": 100.0 * irr / max(len(valence), 1),
        "angdev_mean": sum(devs) / max(len(devs), 1),
        "angdev_p95": devs[int(0.95 * (len(devs) - 1))] if devs else 0.0,
    }


def distance_metrics(src_verts, src_faces, out_verts, out_faces):
    diag = bbox_diag(src_verts)
    src_grid = TriangleGrid(src_verts, triangulate(src_faces))
    out_grid = TriangleGrid(out_verts, triangulate(out_faces))
    used = sorted({i for f in out_faces for i in f})
    d_out = [src_grid.dist(out_verts[i]) for i in used]
    stride = max(1, len(src_verts) // MAX_SOURCE_SAMPLES)
    d_in = [out_grid.dist(src_verts[i])
            for i in range(0, len(src_verts), stride)]
    both = d_out + d_in
    return {
        "dist_mean": 100.0 * (sum(both) / len(both)) / diag,
        "dist_max": 100.0 * max(both) / diag,
    }


def score(source_path, output_path, target=None, source=None):
    """Score output_path against source_path. `source` may carry a
    pre-loaded (verts, faces) pair to skip re-parsing the source."""
    src_verts, src_faces = source or load_obj(source_path)
    out_verts, out_faces = load_obj(output_path)
    if not out_faces:
        return {"quads": 0, "non_quads": 0, "verts": 0, "empty": True}
    m = topology_metrics(out_verts, out_faces)
    m.update(distance_metrics(src_verts, src_faces, out_verts, out_faces))
    if target:
        m["yield"] = m["quads"] / target
    return m


# Lower is better for every selection metric; yield scores by |yield - 1|.
SELECTION_METRICS = ("irr_pct", "angdev_mean", "dist_mean", "non_quads",
                     "yield_miss")


def rank_sum_pick(candidates):
    """Pick the best candidate by rank sum over SELECTION_METRICS.

    Scale-free (no tuned weights): each metric ranks the candidates
    independently, ties share the lower rank, lowest total wins, and
    remaining ties go to the lowest index (deterministic). Returns the
    winning index."""
    rows = []
    for m in candidates:
        r = dict(m)
        r["yield_miss"] = abs(m.get("yield", 1.0) - 1.0)
        rows.append(r)
    totals = [0] * len(rows)
    for key in SELECTION_METRICS:
        vals = [r.get(key, math.inf) for r in rows]
        for i, v in enumerate(vals):
            totals[i] += sum(1 for w in vals if w < v)
    return min(range(len(rows)), key=lambda i: (totals[i], i))


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--source", required=True)
    ap.add_argument("--output", required=True)
    ap.add_argument("--target", type=int)
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args(argv)
    m = score(args.source, args.output, args.target)
    if args.json:
        print(json.dumps(m, indent=2))
    else:
        for k, v in m.items():
            print(f"{k:12s} {v:.4g}" if isinstance(v, float) else
                  f"{k:12s} {v}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
