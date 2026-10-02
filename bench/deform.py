#!/usr/bin/env python3
"""Deformation gate: rig + pose + joint distortion (stdlib only).

Implements docs/deformation-test.md over remeshed outputs: fit a
deterministic 2-bone hinge rig to each output mesh, skin it with the fixed
analytic method, apply the fixed pose set (bench/poses/poses.json), and
measure per-joint distortion: stretch (p95 + % of faces > 2x), volume
loss, flips, and self-intersections near the joint.

v1 fixed method (rationale in docs/deformation-test.md): the rig is
auto-fit (hinge across the bbox centroid on the longest axis), weights are
analytic smoothstep blends across the joint plane, deformation is
dual-quaternion skinning. No Blender, no numpy, no rigforge: same inputs
give byte-identical JSON on any platform, so this gates in CI.

Usage:
  bench/deform.py [--models armadillo.obj,beast.obj,tube] [--targets 1000,5000]
                  [--seeds 1] [--binary rust/target/release/retopo]
                  [--json out.json] [-- extra retopo args...]
  bench/deform.py --mesh out.obj [--json out.json]
                  # score one mesh file (rig + poses + metrics as JSON)
  bench/deform.py --check bench/deform_baseline.json [...]
                  # remesh once per case (seed 0), fail on regression
  bench/deform.py --write-baseline bench/deform_baseline.json [--seeds 8] [...]
                  # (re)generate the baseline from a calibration run

`tube` is a procedural limb (capped cylinder + exact 2-bone elbow rig),
remeshed like the corpus models. Gate rule: at any joint, p95 stretch,
volume loss, or flip count beyond the baseline seed spread fails. Feasible
bends (the tube) hold 0 flips across the spread, so there the flip gate is
the spec's absolute "must be 0"; folding a bloblike corpus mesh 135
degrees inverts by construction, so there the stable spread-gated count is
the regression signal. Recorded but never gating: stretch %>2x,
self-intersections, region sizes.
"""

import argparse
import json
import math
import os
import statistics
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import score  # noqa: E402
import noise  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_POSES = os.path.join(ROOT, "bench", "poses", "poses.json")
DEFAULT_MODELS = ["armadillo.obj", "beast.obj", "nefertiti.obj", "fandisk.obj",
                  "xyzrgb_dragon.obj", "tube"]
DEFAULT_TARGETS = ["1000", "5000"]

# Fixed method constants. These define the gate: changing them changes
# every score, so they take no CLI flags and any change must regenerate
# the baselines and be reviewed as a gate change.
BLEND = 0.2           # skinning half-band as a fraction of bone length
WEIGHT_CUT = 0.1      # joint region: verts with both weights above this
STRETCH_GATE_REL = 0.01  # fail when p95 stretch exceeds spread max by this
VOLUME_GATE_ABS = 0.1    # fail when volume loss exceeds spread max by this (pp)

TUBE_LENGTH = 8.0
TUBE_RADIUS = 0.5
TUBE_RINGS = 64
TUBE_SIDES = 24

BASELINE_METRICS = ("stretch_p95", "volume_loss_pct", "stretch_over2_pct",
                    "flips", "selfint_posed", "selfint_rest",
                    "region_tris", "region_verts")
SUMMARY_METRICS = ("stretch_p95", "volume_loss_pct", "stretch_over2_pct",
                   "flips", "selfint_posed", "region_tris")
PICK_METRICS = ("stretch_p95", "volume_loss_pct", "stretch_over2_pct",
                "flips")


def r6(x):
    """Round a float for stable, byte-identical JSON output."""
    return round(float(x), 6)


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def generate_tube():
    """Procedural limb: capped cylinder along Y, deterministic.

    Returns (verts, faces) with 0-based faces, in score.load_obj format.
    """
    verts, faces = [], []
    length, radius, rings, sides = (TUBE_LENGTH, TUBE_RADIUS,
                                   TUBE_RINGS, TUBE_SIDES)
    for r in range(rings + 1):
        y = -length / 2 + length * r / rings
        for s in range(sides):
            a = 2 * math.pi * s / sides
            verts.append((radius * math.cos(a), y, radius * math.sin(a)))
    for r in range(rings):
        for s in range(sides):
            a = r * sides + s
            b = r * sides + (s + 1) % sides
            c = (r + 1) * sides + (s + 1) % sides
            d = (r + 1) * sides + s
            faces.append([a, b, c, d])
    bot = len(verts)
    verts.append((0.0, -length / 2, 0.0))
    top = len(verts)
    verts.append((0.0, length / 2, 0.0))
    for s in range(sides):
        faces.append([bot, (s + 1) % sides, s])
        base = rings * sides
        faces.append([top, base + s, base + (s + 1) % sides])
    return verts, faces


def write_obj(verts, faces, path):
    with open(path, "w") as f:
        for v in verts:
            f.write("v %.17g %.17g %.17g\n" % v)
        for face in faces:
            f.write("f " + " ".join(str(i + 1) for i in face) + "\n")


def fit_rig(verts):
    """Fit the deterministic 2-bone hinge rig: longest bbox axis is the
    bone axis, the joint pivot is the bbox centroid, the hinge is the
    second-longest bbox axis. Ties resolve toward x, then y, then z."""
    lo = [min(v[k] for v in verts) for k in range(3)]
    hi = [max(v[k] for v in verts) for k in range(3)]
    ext = [hi[k] - lo[k] for k in range(3)]
    axis = max(range(3), key=lambda k: (ext[k], -k))
    rest = [k for k in range(3) if k != axis]
    hinge = max(rest, key=lambda k: (ext[k], -k))
    pivot = tuple((lo[k] + hi[k]) / 2 for k in range(3))
    return {"axis": axis, "hinge": hinge, "pivot": pivot,
            "bone_len": ext[axis], "joints": ["elbow"]}


def skin_weights(verts, rig):
    """Fixed analytic skinning: smoothstep blend across the joint plane.
    Returns [(w0, w1)] per vert; w1 covers the +axis (distal) side."""
    ax = rig["axis"]
    half = BLEND * rig["bone_len"]
    out = []
    for v in verts:
        s = 0.0 if half == 0 else (v[ax] - rig["pivot"][ax]) / half
        s = max(-1.0, min(1.0, s))
        t = (s + 1.0) / 2.0
        w1 = t * t * (3.0 - 2.0 * t)
        out.append((1.0 - w1, w1))
    return out


def load_poses(path):
    with open(path) as f:
        return json.load(f)["poses"]


def poses_for_rig(poses, rig):
    """The fixed pose set filtered to joints the rig has."""
    return [p for p in poses if p["joint"] in rig["joints"]]


def rotate_about_axis(p, pivot, hinge, theta):
    """Right-hand rotation of p about the hinge axis through pivot."""
    x, y, z = p
    cx, cy, cz = pivot
    c, s = math.cos(theta), math.sin(theta)
    if hinge == 0:
        return (x,
                cy + (y - cy) * c - (z - cz) * s,
                cz + (y - cy) * s + (z - cz) * c)
    if hinge == 1:
        return (cx + (x - cx) * c + (z - cz) * s,
                y,
                cz - (x - cx) * s + (z - cz) * c)
    return (cx + (x - cx) * c - (y - cy) * s,
            cy + (x - cx) * s + (y - cy) * c,
            z)


def _quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + bw * ax + ay * bz - az * by,
            aw * by + bw * ay + az * bx - ax * bz,
            aw * bz + bw * az + ax * by - ay * bx,
            aw * bw - ax * bx - ay * by - az * bz)


def distal_quat(rig, angle_deg):
    """Unit quaternion of the distal bone rotation about the hinge."""
    theta = math.radians(angle_deg)
    half = theta / 2.0
    s, c = math.sin(half), math.cos(half)
    ax = [0.0, 0.0, 0.0]
    ax[rig["hinge"]] = 1.0
    q1 = (ax[0] * s, ax[1] * s, ax[2] * s, c)
    if c < 0.0:
        # Antipodal quats would cancel instead of blend; our poses stay
        # below 180 degrees, so this never fires, but stay general.
        q1 = tuple(-x for x in q1)
    return q1


def blend_quat(w1, q1):
    """DQS rotation blend between identity (w1=0) and q1 (w1=1)."""
    w0 = 1.0 - w1
    q = (w1 * q1[0], w1 * q1[1], w1 * q1[2], w0 + w1 * q1[3])
    n = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    return tuple(x / n for x in q)


def quat_rotate_vector(q, p):
    """Rotate vector p by unit quaternion q."""
    qv = (q[0], q[1], q[2])
    inner = _cross(qv, p)
    inner = (inner[0] + q[3] * p[0],
             inner[1] + q[3] * p[1],
             inner[2] + q[3] * p[2])
    rot = _cross(qv, inner)
    return (p[0] + 2 * rot[0], p[1] + 2 * rot[1], p[2] + 2 * rot[2])


def pose_verts(verts, weights, rig, angle_deg):
    """Dual-quaternion skinning (Kavan et al. 2007): proximal bone fixed,
    distal rotated, transforms blended as dual quaternions. DQS, not
    linear blend: LBS necks the joint to cos(angle/2) of its radius
    (candy-wrapper), which inverts even perfect topology at 135 degrees;
    DQS keeps cross-sections rigid so feasible bends stay flip-free."""
    theta = math.radians(angle_deg)
    q1 = distal_quat(rig, angle_deg)
    # Distal rigid motion as rotation about the origin plus translation:
    # rotation about the hinge through the pivot.
    rp = rotate_about_axis(rig["pivot"], (0.0, 0.0, 0.0), rig["hinge"], theta)
    t = _sub(rig["pivot"], rp)
    tq = (t[0], t[1], t[2], 0.0)
    dq = _quat_mul(tq, q1)
    # d1 pairs with q1 as one dual quat; distal_quat's antipodal negation
    # already applies to both halves, so no second negation here.
    d1 = (0.5 * dq[0], 0.5 * dq[1], 0.5 * dq[2], 0.5 * dq[3])
    out = []
    for v, (w0, w1) in zip(verts, weights):
        if w1 == 0.0:
            out.append(v)
            continue
        q = blend_quat(w1, q1)
        d = (w1 * d1[0], w1 * d1[1], w1 * d1[2], w1 * d1[3])
        # blend_quat already normalized; rescale d by the same factor the
        # blend used (recompute it: |w0*I + w1*q1|).
        qb = (w1 * q1[0], w1 * q1[1], w1 * q1[2], w0 + w1 * q1[3])
        nb = math.sqrt(qb[0] * qb[0] + qb[1] * qb[1] +
                       qb[2] * qb[2] + qb[3] * qb[3])
        d = tuple(x / nb for x in d)
        p1 = quat_rotate_vector(q, v)
        qv = (q[0], q[1], q[2])
        dv = (d[0], d[1], d[2])
        cr = _cross(qv, dv)
        tr = (q[3] * dv[0] - d[3] * qv[0] + cr[0],
              q[3] * dv[1] - d[3] * qv[1] + cr[1],
              q[3] * dv[2] - d[3] * qv[2] + cr[2])
        out.append((p1[0] + 2 * tr[0], p1[1] + 2 * tr[1], p1[2] + 2 * tr[2]))
    return out


def tri_flipped(a, b, c, ap, bp, cp, w1bar, q1):
    """True when the triangle's posed normal reverses against its
    bone-transported rest normal: the rest unit normal rotated by the
    DQS blend at the triangle's mean distal weight. Rigid motion never
    flips (a world-space normal test would flag every face past 90
    degrees of bend); only faces the deformation turns inside out do."""
    n = _cross(_sub(b, a), _sub(c, a))
    nn = _dot(n, n)
    if nn == 0.0:
        return False
    npp = _cross(_sub(bp, ap), _sub(cp, ap))
    pp = _dot(npp, npp)
    if pp == 0.0:
        return False
    inv = 1.0 / math.sqrt(nn)
    n = (n[0] * inv, n[1] * inv, n[2] * inv)
    expected = quat_rotate_vector(blend_quat(w1bar, q1), n)
    return _dot(npp, expected) < 0.0


def joint_region(faces, weights):
    """Joint region: verts with both bone weights above WEIGHT_CUT, and
    the faces fully inside that vertex set. Returns (inside, region)."""
    inside = [w0 > WEIGHT_CUT and w1 > WEIGHT_CUT for w0, w1 in weights]
    region = [i for i, f in enumerate(faces)
              if all(inside[v] for v in f)]
    return inside, region


def _det3(m):
    return (m[0] * (m[4] * m[8] - m[5] * m[7])
            - m[1] * (m[3] * m[8] - m[5] * m[6])
            + m[2] * (m[3] * m[7] - m[4] * m[6]))


def _max_eig_sym3(m):
    """Largest eigenvalue of a symmetric 3x3 (row-major), closed form
    (Smith 1961). Deterministic; no iteration, no convergence test."""
    a, d, e = m[0], m[1], m[2]
    b, f = m[4], m[5]
    c = m[8]
    p1 = d * d + e * e + f * f
    if p1 == 0.0:
        return max(a, b, c)
    q = (a + b + c) / 3.0
    p2 = (a - q) ** 2 + (b - q) ** 2 + (c - q) ** 2 + 2 * p1
    p = math.sqrt(p2 / 6.0)
    if p == 0.0:
        return q
    invp = 1.0 / p
    b00, b11, b22 = (a - q) * invp, (b - q) * invp, (c - q) * invp
    b01, b02, b12 = d * invp, e * invp, f * invp
    r = (b00 * (b11 * b22 - b12 * b12)
         - b01 * (b01 * b22 - b12 * b02)
         + b02 * (b01 * b12 - b11 * b02)) / 2.0
    r = max(-1.0, min(1.0, r))
    phi = math.acos(r) / 3.0
    e1 = q + 2 * p * math.cos(phi)
    e3 = q + 2 * p * math.cos(phi + 2 * math.pi / 3)
    e2 = 3 * q - e1 - e3
    return max(e1, e2, e3)


def _tri_frame(a, b, c):
    """Rest/posed frame [e1 e2 n_hat] as row-major 3x3, or None when the
    triangle is degenerate."""
    e1, e2 = _sub(b, a), _sub(c, a)
    n = _cross(e1, e2)
    nn = _dot(n, n)
    if nn == 0.0:
        return None
    inv = 1.0 / math.sqrt(nn)
    n = (n[0] * inv, n[1] * inv, n[2] * inv)
    return (e1[0], e2[0], n[0], e1[1], e2[1], n[1], e1[2], e2[2], n[2])


def _tri_has_area(a, b, c):
    e1, e2 = _sub(b, a), _sub(c, a)
    return _dot(_cross(e1, e2), _cross(e1, e2)) != 0.0


def tri_F(a, b, c, ap, bp, cp):
    """Rest->posed deformation gradient for one triangle. F maps rest
    edges and unit normal to posed. None when rest or posed is
    degenerate. Note det(F) is the posed/rest area ratio by construction,
    so it cannot signal inversion; flips use tri_flipped instead."""
    e = _tri_frame(a, b, c)
    if e is None:
        return None
    det = _det3(e)
    if det == 0.0:
        return None
    ep = _tri_frame(ap, bp, cp)
    if ep is None:
        return None
    invd = 1.0 / det
    ie = ((e[4] * e[8] - e[5] * e[7]) * invd,
          (e[2] * e[7] - e[1] * e[8]) * invd,
          (e[1] * e[5] - e[2] * e[4]) * invd,
          (e[5] * e[6] - e[3] * e[8]) * invd,
          (e[0] * e[8] - e[2] * e[6]) * invd,
          (e[2] * e[3] - e[0] * e[5]) * invd,
          (e[3] * e[7] - e[4] * e[6]) * invd,
          (e[1] * e[6] - e[0] * e[7]) * invd,
          (e[0] * e[4] - e[1] * e[3]) * invd)
    return [sum(ep[r * 3 + k] * ie[k * 3 + c] for k in range(3))
            for r in range(3) for c in range(3)]


def tri_smax(a, b, c, ap, bp, cp):
    """Max singular value of the rest->posed deformation gradient for one
    triangle: sqrt(max eig(F'F)). None for degenerate rest triangles;
    posed-collapsed triangles report stretch via edges."""
    f = tri_F(a, b, c, ap, bp, cp)
    if f is None:
        if not _tri_has_area(a, b, c):
            return None
        e1, e2 = _sub(b, a), _sub(c, a)
        e1p, e2p = _sub(bp, ap), _sub(cp, ap)
        return math.sqrt(max(_dot(e1p, e1p) / _dot(e1, e1),
                             _dot(e2p, e2p) / _dot(e2, e2)))
    m = [sum(f[k * 3 + r] * f[k * 3 + c] for k in range(3))
         for r in range(3) for c in range(3)]
    return math.sqrt(max(0.0, _max_eig_sym3(m)))


def _seg_tri(p, q, a, b, c):
    """True when segment pq pierces triangle abc (Moller-Trumbore with
    exact-zero predicates: deterministic, no epsilon tuning)."""
    d = _sub(q, p)
    e1 = _sub(b, a)
    e2 = _sub(c, a)
    h = _cross(d, e2)
    det = _dot(e1, h)
    if det == 0.0:
        return False
    invd = 1.0 / det
    s = _sub(p, a)
    u = _dot(s, h) * invd
    if u < 0.0 or u > 1.0:
        return False
    t = _cross(s, e1)
    v = _dot(d, t) * invd
    if v < 0.0 or u + v > 1.0:
        return False
    k = _dot(e2, t) * invd
    return 0.0 <= k <= 1.0


def _seg_seg_2d(p, q, r, s):
    """True when 2D segments pq and rs touch or cross (exact predicates).
    Collinear overlap needs a 1D range check: shared orientation zeros
    alone would report disjoint collinear segments as crossing."""
    def side(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

    def on(a, b, c):
        return min(a[0], b[0]) <= c[0] <= max(a[0], b[0]) and \
            min(a[1], b[1]) <= c[1] <= max(a[1], b[1])

    d1, d2 = side(r, s, p), side(r, s, q)
    d3, d4 = side(p, q, r), side(p, q, s)
    if ((d1 < 0) != (d2 < 0)) and ((d3 < 0) != (d4 < 0)):
        return True
    return (d1 == 0 and on(r, s, p)) or (d2 == 0 and on(r, s, q)) or \
        (d3 == 0 and on(p, q, r)) or (d4 == 0 and on(p, q, s))


def _tris_intersect(p1, q1, r1, p2, q2, r2):
    """Triangle-triangle intersection. Non-coplanar contact always pierces
    an edge through the other triangle, so six segment tests suffice; the
    coplanar case falls back to 2D edge crossings on the dominant plane
    (pure containment without edge contact is not counted)."""
    n1 = _cross(_sub(q1, p1), _sub(r1, p1))
    d1 = -_dot(n1, p1)
    du = [_dot(n1, p) + d1 for p in (p2, q2, r2)]
    n2 = _cross(_sub(q2, p2), _sub(r2, p2))
    d2 = -_dot(n2, p2)
    dv = [_dot(n2, p) + d2 for p in (p1, q1, r1)]
    if all(x == 0.0 for x in du) and all(x == 0.0 for x in dv):
        ax = max(range(3), key=lambda k: abs(n1[k]))
        px = [k for k in range(3) if k != ax]
        t1 = [(p[px[0]], p[px[1]]) for p in (p1, q1, r1)]
        t2 = [(p[px[0]], p[px[1]]) for p in (p2, q2, r2)]
        for i in range(3):
            for j in range(3):
                if _seg_seg_2d(t1[i], t1[(i + 1) % 3],
                               t2[j], t2[(j + 1) % 3]):
                    return True
        return False
    for p, q in ((p1, q1), (q1, r1), (r1, p1)):
        if _seg_tri(p, q, p2, q2, r2):
            return True
    for p, q in ((p2, q2), (q2, r2), (r2, p2)):
        if _seg_tri(p, q, p1, q1, r1):
            return True
    return False


def selfint_count(tris, verts):
    """Count intersecting index-triangle pairs (no shared verts) via a
    uniform grid. Pairs are canonical (i < j), so the count is
    order-independent and deterministic."""
    if len(tris) < 2:
        return 0
    total = 0.0
    for a, b, c in tris:
        total += math.dist(verts[a], verts[b])
    h = max(2.0 * total / len(tris), 1e-12)
    cells = {}
    for t, (a, b, c) in enumerate(tris):
        pa, pb, pc = verts[a], verts[b], verts[c]
        lo = [math.floor(min(pa[k], pb[k], pc[k]) / h) for k in range(3)]
        hi = [math.floor(max(pa[k], pb[k], pc[k]) / h) for k in range(3)]
        for i in range(lo[0], hi[0] + 1):
            for j in range(lo[1], hi[1] + 1):
                for k in range(lo[2], hi[2] + 1):
                    cells.setdefault((i, j, k), []).append(t)
    vert_sets = [frozenset(t) for t in tris]
    count = 0
    done = set()
    for key in sorted(cells):
        members = cells[key]
        for x in range(len(members)):
            for y in range(x + 1, len(members)):
                i, j = members[x], members[y]
                pair = (i, j) if i < j else (j, i)
                if pair in done:
                    continue
                done.add(pair)
                if vert_sets[i] & vert_sets[j]:
                    continue
                a, b, c = tris[i]
                d, e, g = tris[j]
                if _tris_intersect(verts[a], verts[b], verts[c],
                                   verts[d], verts[e], verts[g]):
                    count += 1
    return count


def boundary_loops(faces, region):
    """Chain the region's boundary edges (used once within the region)
    into loops. Deterministic: smallest-first starts and neighbors, loops
    sorted by first vertex. Returns [(loop_verts, directed_edges)]."""
    uses = {}
    directed = {}
    for fi in region:
        f = faces[fi]
        for i in range(len(f)):
            a, b = f[i], f[(i + 1) % len(f)]
            key = (a, b) if a < b else (b, a)
            uses[key] = uses.get(key, 0) + 1
            directed[key] = (a, b)
    adj = {}
    for (a, b), n in uses.items():
        if n == 1:
            adj.setdefault(a, []).append(b)
            adj.setdefault(b, []).append(a)
    for v in adj:
        adj[v].sort()
    unused = {key for key, n in uses.items() if n == 1}
    loops = []
    while unused:
        start = min(v for e in unused for v in e)
        loop, edges = [start], []
        cur = start
        while True:
            nxt = None
            for cand in adj[cur]:
                key = (cur, cand) if cur < cand else (cand, cur)
                if key in unused:
                    nxt = cand
                    break
            if nxt is None:
                break
            edges.append((cur, nxt))
            unused.discard((cur, nxt) if cur < nxt else (nxt, cur))
            cur = nxt
            if cur == start:
                break
            loop.append(cur)
        loops.append((loop, edges))
    loops.sort(key=lambda lp: lp[0][0] if lp[0] else -1)
    return loops, directed


def cap_tris(faces, region):
    """Centroid-fan caps closing the region band: one fan per boundary
    loop, wound coherently with the region faces. Returns (loops, caps);
    each cap is (loop_id, b, c), fanning centroid loop_id over edge bc."""
    loops, directed = boundary_loops(faces, region)
    caps = []
    for lid, (loop, edges) in enumerate(loops):
        if len(loop) < 3:
            continue
        for a, b in edges:
            key = (a, b) if a < b else (b, a)
            # The cap edge must oppose the region face's direction.
            if directed[key] == (a, b):
                caps.append((lid, b, a))
            else:
                caps.append((lid, a, b))
    return loops, caps


def closed_volume(tris, loops, caps, verts):
    """True enclosed volume of the region band plus its caps (the spec's
    "closed slice"), via origin tetrahedra. Rigid-invariant: rigid motion
    of the slice preserves it exactly."""
    centroids = []
    for loop, _ in loops:
        n = len(loop)
        centroids.append((sum(verts[i][0] for i in loop) / n,
                          sum(verts[i][1] for i in loop) / n,
                          sum(verts[i][2] for i in loop) / n))
    tot = 0.0
    for a, b, c in tris:
        tot += _dot(verts[a], _cross(verts[b], verts[c])) / 6.0
    for lid, b, c in caps:
        tot += _dot(centroids[lid], _cross(verts[b], verts[c])) / 6.0
    return tot


def score_pose(rest_verts, faces, region, weights, rig, pose):
    """Score one pose over the joint region. Quad faces are fan-split; the
    stretch/flip/selfint stats pool the region triangles, and the volume
    is the enclosed volume of the region band plus its caps (the spec's
    "closed slice"). Flips use tri_flipped (bone-transported normals)."""
    posed = pose_verts(rest_verts, weights, rig, pose["angle_deg"])
    q1 = distal_quat(rig, pose["angle_deg"])
    rtris = []
    for fi in region:
        f = faces[fi]
        for k in range(1, len(f) - 1):
            rtris.append((f[0], f[k], f[k + 1]))
    region_verts = sum(1 for w in weights
                       if w[0] > WEIGHT_CUT and w[1] > WEIGHT_CUT)
    base = {"region_tris": len(rtris), "region_verts": region_verts}
    if not rtris:
        base.update({"empty_region": True, "stretch_p95": 0.0,
                     "stretch_over2_pct": 0.0, "volume_loss_pct": 0.0,
                     "flips": 0, "selfint_posed": 0, "selfint_rest": 0})
        return base
    smax_vals = []
    flips = 0
    for a, b, c in rtris:
        v = tri_smax(rest_verts[a], rest_verts[b], rest_verts[c],
                     posed[a], posed[b], posed[c])
        if v is not None:
            smax_vals.append(v)
        w1bar = (weights[a][1] + weights[b][1] + weights[c][1]) / 3.0
        if tri_flipped(rest_verts[a], rest_verts[b], rest_verts[c],
                       posed[a], posed[b], posed[c], w1bar, q1):
            flips += 1
    smax_vals.sort()
    if smax_vals:
        p95 = smax_vals[int(0.95 * (len(smax_vals) - 1))]
        over2 = 100.0 * sum(1 for v in smax_vals if v > 2.0) / len(smax_vals)
    else:
        p95, over2 = 0.0, 0.0
    loops, caps = cap_tris(faces, region)
    v_rest = closed_volume(rtris, loops, caps, rest_verts)
    v_posed = closed_volume(rtris, loops, caps, posed)
    if abs(v_rest) < 1e-12 * rig["bone_len"] ** 3:
        vol_loss, degen = 0.0, True
    else:
        vol_loss, degen = 100.0 * (v_rest - v_posed) / v_rest, False
    base.update({
        "stretch_p95": r6(p95),
        "stretch_over2_pct": r6(over2),
        "volume_loss_pct": r6(vol_loss),
        "flips": flips,
        "selfint_posed": selfint_count(rtris, posed),
        "selfint_rest": selfint_count(rtris, rest_verts),
    })
    if degen:
        base["degenerate_volume"] = True
    return base


def score_output(verts, faces, poses):
    """Fit the rig, skin, and score every applicable pose. Returns
    {"rig": ..., "poses": {name: metrics}} with rounded floats."""
    rig = fit_rig(verts)
    weights = skin_weights(verts, rig)
    _, region = joint_region(faces, weights)
    out = {}
    for pose in poses_for_rig(poses, rig):
        out[pose["name"]] = score_pose(verts, faces, region, weights,
                                       rig, pose)
    axes = "xyz"
    return {
        "rig": {"axis": axes[rig["axis"]], "hinge": axes[rig["hinge"]],
                "pivot": [r6(x) for x in rig["pivot"]],
                "bone_len": r6(rig["bone_len"]),
                "region_faces": len(region)},
        "poses": out,
    }


def score_file(path, poses):
    """Score one OBJ file. Used by bench/run.py and --mesh mode."""
    verts, faces = score.load_obj(path)
    if not faces:
        return {"rig": {}, "poses": {}, "empty": True}
    return score_output(verts, faces, poses)


def remesh(binary, input_path, target, output_path, extra):
    proc = subprocess.run(
        [binary, "--input", input_path, "--output", output_path,
         "--target-quads", str(target), "--quiet"] + extra,
        capture_output=True, text=True)
    return proc.returncode == 0 and os.path.exists(output_path)


def resolve_input(model, tmp):
    """Return (input_path, source) for a model name. `tube` is generated
    procedurally; anything else loads from bench/models/."""
    if model == "tube":
        source = generate_tube()
        path = os.path.join(tmp, "tube.obj")
        write_obj(*source, path)
        return path, source
    path = model if os.path.isabs(model) else os.path.join(
        ROOT, "bench", "models", model)
    return path, score.load_obj(path)


def run_case(binary, model, target, seeds, extra, poses, tmp):
    input_path, source = resolve_input(model, tmp)
    verts, faces = source
    diag = score.bbox_diag(verts)
    rows = []
    for seed in range(seeds):
        if seed == 0:
            inp = input_path
        else:
            inp = os.path.join(tmp, "in.obj")
            noise.write_jittered(verts, faces, inp, seed, diag)
        out = os.path.join(tmp, "out.obj")
        if os.path.exists(out):
            os.remove(out)
        if not remesh(binary, inp, target, out, extra):
            rows.append({"seed": seed, "failed": True})
            continue
        m = score_file(out, poses)
        m["seed"] = seed
        if m.get("empty"):
            m["failed"] = True
        rows.append(m)
        if os.path.exists(out):
            os.remove(out)
    return rows


def _rank_sum(rows, keys):
    """Best row by rank sum over keys (lower is better); ties share the
    lower rank, remaining ties go to the lowest index (deterministic)."""
    totals = [0] * len(rows)
    for key in keys:
        vals = [r[key] for r in rows]
        for i, v in enumerate(vals):
            totals[i] += sum(1 for w in vals if w < v)
    return min(range(len(rows)), key=lambda i: (totals[i], i))


def fmt(v):
    return f"{v:.3g}" if isinstance(v, float) else str(v)


def summarize(name, target, rows):
    ok = [r for r in rows if not r.get("failed") and not r.get("empty")]
    print(f"\n### {name} @ {target}  ({len(ok)}/{len(rows)} runs ok)")
    if not ok or not ok[0].get("poses"):
        if ok:
            print("(no applicable poses scored)")
        return {}
    agg = []
    for r in ok:
        pm = {}
        for pname, m in r["poses"].items():
            for key in PICK_METRICS:
                pm.setdefault(key, []).append(m[key])
        agg.append({k: sum(v) / len(v) for k, v in pm.items()})
    pick = ok[_rank_sum(agg, PICK_METRICS)]
    base = next((r for r in ok if r["seed"] == 0), None)
    out = {"pick_seed": pick["seed"], "poses": {}}
    for pname in ok[0]["poses"]:
        print(f"--- {pname} ---")
        print("| metric | seed0 | median | min | max | "
              f"best-of-K (seed {pick['seed']}) |")
        print("|---|---:|---:|---:|---:|---:|")
        pout = {}
        for key in SUMMARY_METRICS:
            vals = [r["poses"][pname][key] for r in ok]
            b = base["poses"][pname][key] if base else None
            med = statistics.median(vals)
            print(f"| {key} | {fmt(b) if b is not None else '-'} | "
                  f"{fmt(med)} | {fmt(min(vals))} | {fmt(max(vals))} | "
                  f"{fmt(pick['poses'][pname][key])} |")
            pout[key] = {"seed0": b, "median": med,
                         "min": min(vals), "max": max(vals),
                         "pick": pick["poses"][pname][key]}
        out["poses"][pname] = pout
    return out


def build_baseline(cases_rows):
    """Build the check baseline from multi-seed rows: per case per pose
    per metric, {seed0, min, max} over the seed spread."""
    cases = {}
    for case, rows in cases_rows.items():
        ok = [r for r in rows if not r.get("failed") and not r.get("empty")]
        if not ok:
            continue
        base0 = next((r for r in ok if r["seed"] == 0), ok[0])
        poses = {}
        for pname in ok[0]["poses"]:
            vals = {}
            for key in BASELINE_METRICS:
                vs = [r["poses"][pname][key] for r in ok]
                vals[key] = {"seed0": base0["poses"][pname][key],
                             "min": min(vs), "max": max(vs)}
            poses[pname] = vals
        cases[case] = {"poses": poses}
    return cases


def check_case(label, current, base_poses):
    """Gate one case's current {pose: metrics} against baseline poses (or
    None for the absolute-only check). One uniform rule: flips must not
    exceed the baseline spread max, which is the spec's absolute "must be
    0" wherever the spread max is 0 (feasible bends). Returns failures."""
    failures = []
    if not current:
        return [f"{label}: no poses scored"]
    for pname, cur in current.items():
        if cur.get("empty_region"):
            failures.append(f"{label} {pname}: empty joint region")
            continue
        if base_poses is None or pname not in base_poses:
            if cur["flips"] != 0:
                failures.append(
                    f"{label} {pname}: flips={cur['flips']} (must be 0)")
            continue
        base = base_poses[pname]
        spread = base["flips"]["max"]
        if cur["flips"] > spread:
            failures.append(
                f"{label} {pname}: flips={cur['flips']} beyond seed "
                f"spread (max {spread})")
        spread = base["stretch_p95"]["max"]
        if cur["stretch_p95"] > spread * (1 + STRETCH_GATE_REL) + 1e-9:
            failures.append(
                f"{label} {pname}: stretch_p95 {cur['stretch_p95']} "
                f"beyond seed spread (max {spread})")
        spread = base["volume_loss_pct"]["max"]
        if cur["volume_loss_pct"] > spread + VOLUME_GATE_ABS:
            failures.append(
                f"{label} {pname}: volume_loss_pct "
                f"{cur['volume_loss_pct']} beyond seed spread "
                f"(max {spread})")
    return failures


def write_json(path, obj):
    with open(path, "w") as f:
        json.dump(obj, f, indent=2, sort_keys=True)
        f.write("\n")


def main(argv):
    extra = []
    if "--" in argv:
        i = argv.index("--")
        argv, extra = argv[:i], argv[i + 1:]
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--models", default=",".join(DEFAULT_MODELS))
    ap.add_argument("--targets", default=",".join(DEFAULT_TARGETS))
    ap.add_argument("--seeds", type=int, default=None)
    ap.add_argument("--binary",
                    default=os.path.join(ROOT, "rust/target/release/retopo"))
    ap.add_argument("--poses", default=DEFAULT_POSES)
    ap.add_argument("--mesh", default=None,
                    help="score one OBJ file instead of remeshing")
    ap.add_argument("--json", default=None)
    ap.add_argument("--check", default=None, metavar="BASELINE",
                    help="fail when seed-0 scores regress vs BASELINE")
    ap.add_argument("--write-baseline", default=None, metavar="BASELINE",
                    help="write a multi-seed baseline to BASELINE")
    args = ap.parse_args(argv)
    poses = load_poses(args.poses)

    if args.mesh and (args.check or args.write_baseline):
        print("--mesh cannot be combined with --check/--write-baseline",
              file=sys.stderr)
        return 2
    if args.mesh:
        m = score_file(args.mesh, poses)
        if args.json:
            write_json(args.json, m)
        else:
            print(json.dumps(m, indent=2, sort_keys=True))
        for pname, pm in m.get("poses", {}).items():
            print(f"deform {args.mesh} {pname}: "
                  f"stretch_p95={pm['stretch_p95']} "
                  f"vol={pm['volume_loss_pct']}% flips={pm['flips']}")
        return 0

    if args.check and args.write_baseline:
        print("--check and --write-baseline are mutually exclusive",
              file=sys.stderr)
        return 2
    if args.check:
        seeds = 1
    elif args.seeds is not None:
        seeds = args.seeds
    elif args.write_baseline:
        seeds = 8
    else:
        seeds = 1
    if not os.path.exists(args.binary):
        print(f"binary not found: {args.binary} "
              f"(build with cargo first: cargo build --locked --release -p retopo)",
              file=sys.stderr)
        return 2

    baseline = None
    if args.check:
        if not os.path.exists(args.check):
            print(f"deform baseline not found: {args.check}",
                  file=sys.stderr)
            return 2
        with open(args.check) as f:
            baseline = json.load(f)

    report = {"binary": args.binary, "seeds": seeds, "jitter": noise.JITTER,
              "extra": extra, "cases": {}}
    cases_rows = {}
    failures = []
    with tempfile.TemporaryDirectory(prefix="retopo_deform_") as tmp:
        for model in args.models.split(","):
            for t in args.targets.split(","):
                target = int(t)
                case = f"{os.path.basename(model)}@{target}"
                print(f"run {case}...", flush=True)
                rows = run_case(args.binary, model, target, seeds, extra,
                                poses, tmp)
                summary = summarize(os.path.basename(model), target, rows)
                report["cases"][case] = {"rows": rows, "summary": summary}
                cases_rows[case] = rows
                sys.stdout.flush()
                if baseline is not None:
                    cur = next((r for r in rows if r.get("seed") == 0
                                and not r.get("failed")), None)
                    if cur is None:
                        failures.append(f"{case}: seed-0 run failed")
                    elif case not in baseline.get("cases", {}):
                        failures.append(f"{case}: no baseline entry")
                    else:
                        failures += check_case(
                            case, cur["poses"],
                            baseline["cases"][case]["poses"])
    if args.json:
        write_json(args.json, report)
    if args.write_baseline:
        write_json(args.write_baseline, {
            "note": "Deformation gate baseline: per case per pose per "
                    "metric, seed0/min/max over the calibration seed "
                    "spread. Per-platform: regenerate on each platform, "
                    "never mix (see bench/run.py baselines).",
            "seeds": seeds, "jitter": noise.JITTER,
            "cases": build_baseline(cases_rows)})
        print(f"\nwrote {args.write_baseline}")
    if baseline is not None:
        if failures:
            print(f"\nREGRESSIONS vs {args.check}")
            for msg in failures:
                print("  -", msg)
            return 1
        print(f"\nno regressions vs {args.check}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
