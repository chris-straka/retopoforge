#!/usr/bin/env python3
"""Procedural finger-like fixtures for dipole-insertion research.

Generates curved thin tube meshes (NOT game assets - pure procedural
geometry, no AI corpus content) shared by the density/dipole lanes:

  finger-single.obj  one curved tapered tube with a domed tip
  finger-split.obj   two curved tubes side by side with an air gap (2 islands)
  finger-fused.obj   one peanut-cross-section tube: two lobes, shallow
                     valleys front/back (1 island, partial fusion)

Frame: fingers point along +Y, base at y=0, tip near y=3. Bend is in
the YZ plane (tip leans toward +Z). All meshes are closed manifolds
(signed volume > 0, verified after generation).

Density protocol (shared with rust/core/tests/dipole_saturation.rs):
  TIP_Y   = 2.20          masked "inside" region on the output: y > TIP_Y
                          (plus lobe selection for fused/split, see below)
  BASE_Y  = 0.80          control region for finger-single: y < BASE_Y
  TUBE_DX = split-tube center offset; masked tube is x > 0, control x < 0
  Fused lobes: masked lobe x > 0, control lobe x < 0 (both with y > TIP_Y)

Masks: one multiplier per input vertex, value ASK in the masked region,
1.0 elsewhere. Use --ask to also emit mask files (for CLI repros):

  gen_finger_fixtures.py --out /tmp/fing --ask 4.0

writes finger-*.obj plus finger-*-mask4.txt next to them. The committed
fixtures are the OBJs only (default --ask writes no masks).
"""

import argparse
import math
import os
import sys

LENGTH = 3.0
R0 = 0.55
BEND = 0.35  # tip z-offset = BEND * LENGTH
TAPER = 0.35  # radius shrinks 35% base -> tip (before the dome)
DOME_T = 0.85  # dome starts at this fraction of LENGTH
RINGS = 44
SEGS = 20
SPLIT_DX = 0.78  # tube-center |x| for finger-split (gap ~0.3 at base)
FUSED_V = 0.30  # peanut pinch: r *= (1 + V*cos(2*theta)), lobes on +-X
TIP_Y = 2.20
BASE_Y = 0.80


def spine_z(y):
    t = y / LENGTH
    return BEND * LENGTH * t * t


def radius_at(y, theta_phase=0.0):
    """Tapered radius with a hemispherical dome past DOME_T."""
    t = min(max(y / LENGTH, 0.0), 1.0)
    r = R0 * (1.0 - TAPER * t)
    if t > DOME_T:
        u = (t - DOME_T) / (1.0 - DOME_T)
        r *= math.cos(u * math.pi / 2.0)
    return r


def tube_rings(cx, valley=0.0):
    """Ring vertices for one tube centered at x=cx. Returns (verts, rings).

    verts: list of (x, y, z). rings: list of SEGS-long index rows from
    base (y=0) to near-tip; both ends are closed with pole fans by
    close_tube().
    """
    verts = []
    rings = []
    for i in range(RINGS):
        y = LENGTH * i / (RINGS - 1)
        # Pull the last ring just inside the tip so the pole fan is tiny.
        if i == RINGS - 1:
            y = LENGTH * 0.999
        r = radius_at(y)
        zc = spine_z(y)
        ring = []
        for j in range(SEGS):
            th = 2.0 * math.pi * j / SEGS
            rr = r * (1.0 + valley * math.cos(2.0 * th))
            x = cx + rr * math.cos(th)
            z = zc + rr * math.sin(th)
            ring.append(len(verts))
            verts.append((x, y, z))
        rings.append(ring)
    return verts, rings


def close_tube(verts, rings):
    """Side quads (as tri pairs) + base/tip pole fans. Winding: outward."""
    tris = []
    for a, b in zip(rings[:-1], rings[1:]):
        for j in range(SEGS):
            j2 = (j + 1) % SEGS
            # Outward for a +Y tube with theta from +X toward +Z.
            tris.append((a[j], b[j], b[j2]))
            tris.append((a[j], b[j2], a[j2]))
    # Base pole (y=0 plane).
    y0 = verts[rings[0][0]][1]
    cx = sum(verts[k][0] for k in rings[0]) / SEGS
    cz = sum(verts[k][2] for k in rings[0]) / SEGS
    base = len(verts)
    verts.append((cx, y0, cz))
    for j in range(SEGS):
        j2 = (j + 1) % SEGS
        tris.append((base, rings[0][j2], rings[0][j]))
    # Tip pole.
    top = rings[-1]
    y1 = verts[top[0]][1]
    cx = sum(verts[k][0] for k in top) / SEGS
    cz = sum(verts[k][2] for k in top) / SEGS
    tip = len(verts)
    verts.append((cx, y1 + 0.02, cz))
    for j in range(SEGS):
        j2 = (j + 1) % SEGS
        tris.append((tip, top[j], top[j2]))
    return tris


def build_single(cx=0.0, valley=0.0):
    verts, rings = tube_rings(cx, valley)
    tris = close_tube(verts, rings)
    return verts, tris


def build_split():
    verts, tris = [], []

    def add(cx):
        v, r = tube_rings(cx)
        t = close_tube(v, r)  # appends pole verts to v first
        off = len(verts)
        verts.extend(v)
        tris.extend((a + off, b + off, c + off) for (a, b, c) in t)

    add(-SPLIT_DX)
    add(SPLIT_DX)
    return verts, tris


def signed_volume(verts, tris):
    vol = 0.0
    for a, b, c in tris:
        ax, ay, az = verts[a]
        bx, by, bz = verts[b]
        cx, cy, cz = verts[c]
        vol += ax * (by * cz - bz * cy) + bx * (cy * az - cz * ay) + cx * (ay * bz - by * az)
    return vol / 6.0


def check_manifold(verts, tris):
    edges = {}
    for a, b, c in tris:
        for u, v in ((a, b), (b, c), (c, a)):
            key = (u, v) if u < v else (v, u)
            edges[key] = edges.get(key, 0) + 1
    bad = [k for k, n in edges.items() if n != 2]
    return bad


def write_obj(path, verts, tris):
    with open(path, "w") as f:
        f.write("# procedural finger fixture (gen_finger_fixtures.py)\n")
        for x, y, z in verts:
            f.write(f"v {x:.6f} {y:.6f} {z:.6f}\n")
        for a, b, c in tris:
            f.write(f"f {a + 1} {b + 1} {c + 1}\n")


def mask_for(verts, name, ask):
    if name == "finger-single":
        return [ask if y > TIP_Y else 1.0 for (_, y, _) in verts]
    if name == "finger-split":
        return [ask if (y > TIP_Y and x > 0.0) else 1.0 for (x, y, _) in verts]
    if name == "finger-fused":
        return [ask if (y > TIP_Y and x > 0.0) else 1.0 for (x, y, _) in verts]
    raise ValueError(name)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=".", help="output directory")
    ap.add_argument("--ask", type=float, default=None,
                    help="also write *-mask{AASK}.txt density masks")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    fixtures = [
        ("finger-single", build_single()),
        ("finger-split", build_split()),
        ("finger-fused", build_single(valley=FUSED_V)),
    ]
    for name, (verts, tris) in fixtures:
        vol = signed_volume(verts, tris)
        bad = check_manifold(verts, tris)
        ys = [y for (_, y, _) in verts]
        xs = [x for (x, _, _) in verts]
        print(f"{name}: verts={len(verts)} tris={len(tris)} "
              f"vol={vol:.4f} x=[{min(xs):.2f},{max(xs):.2f}] "
              f"y=[{min(ys):.2f},{max(ys):.2f}] bad_edges={len(bad)}")
        assert vol > 0, f"{name}: non-positive volume, winding is inward?"
        assert not bad, f"{name}: {len(bad)} non-manifold edges"
        write_obj(os.path.join(args.out, name + ".obj"), verts, tris)
        if args.ask is not None:
            m = mask_for(verts, name, args.ask)
            tag = ("%.2f" % args.ask).rstrip("0").rstrip(".")
            mp = os.path.join(args.out, f"{name}-mask{tag}.txt")
            with open(mp, "w") as f:
                for v in m:
                    f.write(f"{v}\n")
            n = sum(1 for v in m if v != 1.0)
            print(f"  mask {os.path.basename(mp)}: {n}/{len(m)} masked")
    print(f"TIP_Y={TIP_Y} BASE_Y={BASE_Y} SPLIT_DX={SPLIT_DX} FUSED_V={FUSED_V}")


if __name__ == "__main__":
    main()
