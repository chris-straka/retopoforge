#!/usr/bin/env python3
"""Focused regression tests for bench/rings.py (stdlib only).

Run:  python3 bench/test_rings.py

Covers the joint-ring geometry that has to hold for --guides to see
joint loops: a transverse slice of a limb yields exactly one closed
on-surface loop of the limb's radius; leg pairs keep both loops while
arm slices drop the torso loop; quadruped legs cut perpendicular to
the leg axis; output is byte-deterministic and parses as --guides.
"""

import math
import os
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import deform  # noqa: E402
import rings  # noqa: E402


def check(name, cond):
    if not cond:
        raise AssertionError(f"FAIL: {name}")
    print(f"ok: {name}")


def ztube(radius=deform.TUBE_RADIUS, dx=0.0, axis=(0.0, 0.0, 1.0)):
    """deform's Y-axis tube rewritten along +Z, optionally shifted in x."""
    verts, faces = deform.generate_tube()
    out = []
    for (x, y, z) in verts:
        x, y, z = x * radius / deform.TUBE_RADIUS, y, z
        out.append((x + dx, z, y))
    assert axis == (0.0, 0.0, 1.0)
    tris = []
    for f in faces:
        for k in range(1, len(f) - 1):
            tris.append((f[0], f[k], f[k + 1]))
    return out, tris


def radius_of(loop):
    body = loop[:-1]
    cx = sum(p[0] for p in body) / len(body)
    cy = sum(p[1] for p in body) / len(body)
    return math.sqrt(sum((p[0] - cx) ** 2 + (p[1] - cy) ** 2
                         for p in body) / len(body))


def wall_radius(loop):
    """Max in-plane radius: ring vertices touch the limb wall."""
    body = loop[:-1]
    cx = sum(p[0] for p in body) / len(body)
    cy = sum(p[1] for p in body) / len(body)
    return max(math.hypot(p[0] - cx, p[1] - cy) for p in body)


def main():
    verts, faces = ztube()
    loops = rings.rings_for_joint(verts, faces, "elbow", (0.0, 0.0, 0.0),
                                  (0.0, 0.0, 1.0), drop_largest=True)
    check("tube slice is one loop", len(loops) == 1)
    loop = loops[0]
    check("ring is exactly closed", loop[0] == loop[-1])
    check("ring has two points per wall column (fan-split quads)",
          len(loop) - 1 == 2 * deform.TUBE_SIDES)
    check("ring lies in the cut plane",
          all(abs(p[2]) < 1e-6 for p in loop))
    body = loop[:-1]  # exclude the closing repeat from the centroid
    cx = sum(q[0] for q in body) / len(body)
    cy = sum(q[1] for q in body) / len(body)
    rr = [math.hypot(p[0] - cx, p[1] - cy) for p in body]
    check("ring vertices touch the limb wall",
          abs(max(rr) - deform.TUBE_RADIUS) < 1e-9)
    check("every ring point hugs the limb wall",
          abs(min(rr) - deform.TUBE_RADIUS) < 1e-6)

    # Two legs at one slice: both loops kept.
    v2, f2 = ztube(dx=5.0 * deform.TUBE_RADIUS)
    legs = rings.rings_for_joint(verts + v2,
                                 faces + [(a + len(verts), b + len(verts),
                                           c + len(verts)) for (a, b, c) in f2],
                                 "knee", (0.0, 0.0, 0.0), (0.0, 0.0, 1.0),
                                 drop_largest=True)
    check("leg pair keeps both loops", len(legs) == 2)
    check("leg loops touch the limb wall",
          all(abs(wall_radius(l) - deform.TUBE_RADIUS) < 1e-9 for l in legs))

    # Torso + arms: the largest loop is dropped.
    vt, ft = ztube(radius=3.0 * deform.TUBE_RADIUS,
                   dx=10.0 * deform.TUBE_RADIUS)
    n0, n1 = len(verts), len(verts) + len(v2)
    combo_v = verts + v2 + vt
    combo_f = (faces
               + [(a + len(verts), b + len(verts), c + len(verts))
                 for (a, b, c) in f2]
               + [(a + n1, b + n1, c + n1) for (a, b, c) in ft])
    arms = rings.rings_for_joint(combo_v, combo_f, "elbow", (0.0, 0.0, 0.0),
                                 (0.0, 0.0, 1.0), drop_largest=True)
    check("torso loop dropped, arms kept",
          len(arms) == 2
          and all(abs(wall_radius(l) - deform.TUBE_RADIUS) < 1e-9
                  for l in arms))

    # Quadruped leg: plane perpendicular to a tilted axis.
    m = math
    ang = m.radians(20.0)
    rot = [(x * m.cos(ang) - z * m.sin(ang), y,
            x * m.sin(ang) + z * m.cos(ang)) for (x, y, z) in verts]
    top = (0.0, 0.0, deform.TUBE_LENGTH / 2)
    foot = (0.0, 0.0, -deform.TUBE_LENGTH / 2)
    top = (top[0] * m.cos(ang) - top[2] * m.sin(ang), 0.0,
           top[0] * m.sin(ang) + top[2] * m.cos(ang))
    foot = (foot[0] * m.cos(ang) - foot[2] * m.sin(ang), 0.0,
            foot[0] * m.sin(ang) + foot[2] * m.cos(ang))
    mid = tuple((a + b) / 2 for a, b in zip(top, foot))
    axis = tuple((b - a) for a, b in zip(top, foot))
    n = m.sqrt(sum(c * c for c in axis))
    axis = tuple(c / n for c in axis)
    qloops = rings.rings_for_joint(rot, faces, "leg", mid, axis,
                                   drop_largest=False)
    check("tilted leg cuts one loop", len(qloops) == 1)
    check("leg loop is transverse to the axis",
          all(abs(sum((p[k] - mid[k]) * axis[k] for k in range(3))) < 1e-6
              for p in qloops[0]))
    check("leg loop radius is the limb radius",
          abs(radius_of([(p[0], p[1], 0.0) for p in qloops[0]])
              - deform.TUBE_RADIUS) < 0.2 * deform.TUBE_RADIUS)

    # Determinism + --guides file shape via the CLI.
    with tempfile.TemporaryDirectory(prefix="rings-test-") as tmp:
        mesh = os.path.join(tmp, "limb.obj")
        deform.write_obj(*deform.generate_tube(), mesh)
        lm = os.path.join(tmp, "lm.json")
        with open(lm, "w") as f:
            # deform's tube runs along Y; ring it transversely is the
            # quadruped path (plane perpendicular to the limb axis).
            L = deform.TUBE_LENGTH
            f.write('{"kind": "quadruped", "legs": {'
                    '"limb": {"top": [0,%g,0], "mid": [0,0,0], '
                    '"foot": [0,%g,0]}}}' % (L / 2, -L / 2))
        g1 = os.path.join(tmp, "g1.txt")
        g2 = os.path.join(tmp, "g2.txt")
        here = os.path.dirname(os.path.abspath(__file__))
        for g in (g1, g2):
            r = subprocess.run([sys.executable,
                                os.path.join(here, "rings.py"),
                                "--mesh", mesh, "--landmarks", lm,
                                "--joints", "limb", "--output", g],
                               capture_output=True, text=True)
            check(f"rings CLI exits 0 ({os.path.basename(g)})",
                  r.returncode == 0)
        check("rings output is byte-deterministic",
              open(g1, "rb").read() == open(g2, "rb").read())
        body = [ln for ln in open(g1).read().splitlines()
                if ln.strip() and not ln.startswith("#")]
        check("guides lines are x y z triples",
              all(len(ln.split()) == 3 for ln in body))
        pts = [tuple(float(t) for t in ln.split()) for ln in body]
        check("emitted ring is closed", pts[0] == pts[-1])
        check("emitted ring is transverse (constant y)",
              max(p[1] for p in pts) - min(p[1] for p in pts) < 1e-6)
        rr = math.sqrt(sum(p[0] ** 2 + p[2] ** 2 for p in pts) / len(pts))
        check("emitted ring radius is the tube radius",
              abs(rr - deform.TUBE_RADIUS) < 1e-6)

    # Fail loud, naming the joint.
    try:
        rings.rings_for_joint(verts, faces, "elbow", (99.0, 99.0, 99.0),
                              (0.0, 0.0, 1.0), drop_largest=True)
        check("empty cut fails", False)
    except SystemExit as e:
        check("empty cut names the joint", "elbow" in str(e))

    print("all rings regression tests passed")


if __name__ == "__main__":
    main()
