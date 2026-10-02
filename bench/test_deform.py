#!/usr/bin/env python3
"""Focused regression tests for bench/deform.py (stdlib only).

Run:  python3 bench/test_deform.py

Covers the deformation-gate math that has bitten before: the 2D segment
fallback must not report collinear-but-disjoint edges as crossing (tube
wall facets are coplanar across rows), the clean procedural tube must be
self-intersection-free at rest, and the fixed probe must not flip clean
topology (the gate requires flips == 0).
"""

import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import deform  # noqa: E402


def check(name, cond):
    if not cond:
        raise AssertionError(f"FAIL: {name}")
    print(f"ok: {name}")


def main():
    a, b, c = (0, 0, 0), (1, 0, 0), (0, 1, 0)
    check("smax identity", deform.tri_smax(a, b, c, a, b, c) == 1.0)
    check("smax uniform 2x",
          deform.tri_smax(a, b, c, (0, 0, 0), (2, 0, 0), (0, 2, 0)) == 2.0)
    check("smax rigid rotation",
          deform.tri_smax(a, b, c, (0, 0, 0), (0, 1, 0), (-1, 0, 0)) == 1.0)
    check("smax degenerate rest is None",
          deform.tri_smax(a, a, a, a, b, c) is None)
    q135 = deform.distal_quat({"hinge": 0}, 135)
    rigid135 = [deform.rotate_about_axis(p, (0, 0, 0), 0, math.radians(135))
                for p in (a, b, c)]
    check("rigid 135 never flips",
          not deform.tri_flipped(a, b, c, *rigid135, 1.0, q135))
    q0 = deform.distal_quat({"hinge": 0}, 0)
    check("mirrored flips",
          deform.tri_flipped(a, b, c, (0, 0, 0), (-1, 0, 0), (0, 1, 0),
                             1.0, q0))
    check("unmoved against rotated bone flips",
          deform.tri_flipped(a, b, c, a, b, c, 1.0, q135))
    check("flip degenerate rest",
          not deform.tri_flipped(a, a, a, a, b, c, 0.5, q135))
    check("flip degenerate posed",
          not deform.tri_flipped(a, b, c, a, a, a, 0.5, q135))
    check("seg_tri hit", deform._seg_tri(
        (0, 0, -1), (0, 0, 1), (0, 0, 0), (1, 0, 0), (0, 1, 0)))
    check("seg_tri miss", not deform._seg_tri(
        (2, 2, -1), (2, 2, 1), (0, 0, 0), (1, 0, 0), (0, 1, 0)))
    check("seg2d crossing", deform._seg_seg_2d(
        (0, 0), (1, 1), (0, 1), (1, 0)))
    check("seg2d disjoint", not deform._seg_seg_2d(
        (0, 0), (1, 0), (2, 0), (3, 0)))
    check("seg2d collinear disjoint", not deform._seg_seg_2d(
        (-0.25, 0.259), (-0.125, 0.259), (0.0, 0.259), (0.125, 0.259)))
    check("seg2d touching", deform._seg_seg_2d(
        (0, 0), (1, 0), (1, 0), (1, 1)))

    # Same-column tube wall patches in adjacent rows: coplanar (the wall
    # is an extruded prism) but disjoint; must not count.
    t1 = [(1.0, -0.25, 0.0), (0.9659258262890683, -0.25, 0.25881904510252074),
          (0.9659258262890683, -0.125, 0.25881904510252074)]
    t2 = [(1.0, 0.0, 0.0), (0.9659258262890683, 0.0, 0.25881904510252074),
          (0.9659258262890683, 0.125, 0.25881904510252074)]
    check("coplanar disjoint tris", not deform._tris_intersect(*t1, *t2))
    check("piercing tris", deform._tris_intersect(
        (0, 0, -1), (0, 0, 1), (0, 1, 0),
        (-1, -1, 0), (1, -1, 0), (0, 1, 0)))

    verts, faces = deform.generate_tube()
    poses = deform.load_poses(deform.DEFAULT_POSES)
    rig = deform.fit_rig(verts)
    weights = deform.skin_weights(verts, rig)
    posed = deform.pose_verts(verts, weights, rig, 135)
    worst = 0.0
    for v, p, (w0, w1) in zip(verts, posed, weights):
        if w1 == 1.0:
            q = deform.rotate_about_axis(v, rig["pivot"], rig["hinge"],
                                         math.radians(135))
            worst = max(worst, math.dist(p, q))
        if w1 == 0.0:
            worst = max(worst, math.dist(p, v))
    check("dqs matches rigid at w1=1, identity at w1=0", worst < 1e-9)
    scored = deform.score_output(verts, faces, poses)
    check("tube poses scored", set(scored["poses"]) == {"elbow90", "elbow135"})
    for pname, m in scored["poses"].items():
        check(f"tube {pname} region nonempty", m["region_tris"] > 0
              and not m.get("empty_region"))
        check(f"tube {pname} rest selfint 0", m["selfint_rest"] == 0)
        check(f"tube {pname} flips 0", m["flips"] == 0)
        check(f"tube {pname} stretch sane",
              1.0 <= m["stretch_p95"] <= 10.0)

    # Grid acceleration must agree with brute force on the tube region.
    rig = deform.fit_rig(verts)
    weights = deform.skin_weights(verts, rig)
    _, region = deform.joint_region(faces, weights)
    rtris = []
    for fi in region:
        f = faces[fi]
        for k in range(1, len(f) - 1):
            rtris.append((f[0], f[k], f[k + 1]))
    brute = 0
    for i in range(len(rtris)):
        for j in range(i + 1, len(rtris)):
            if set(rtris[i]) & set(rtris[j]):
                continue
            x, y, z = rtris[i]
            d, e, g = rtris[j]
            if deform._tris_intersect(verts[x], verts[y], verts[z],
                                      verts[d], verts[e], verts[g]):
                brute += 1
    check("grid == brute force",
          brute == deform.selfint_count(rtris, verts) == 0)

    # Closed-slice volume is rigid-invariant and matches the tube segment.
    loops, caps = deform.cap_tris(faces, region)
    v_rest = deform.closed_volume(rtris, loops, caps, verts)
    rigid = [deform.rotate_about_axis(p, (0, 0, 0), 0, math.radians(90))
             for p in verts]
    v_rigid = deform.closed_volume(rtris, loops, caps, rigid)
    check("closed volume rigid-invariant", abs(v_rest - v_rigid) < 1e-9)
    ys = [verts[i][1] for fi in region for i in faces[fi]]
    expect = math.pi * deform.TUBE_RADIUS ** 2 * (max(ys) - min(ys))
    check("closed volume matches tube segment",
          abs(abs(v_rest) - expect) / expect < 0.02)

    # Byte-identical output for identical inputs.
    blob1 = json.dumps(scored, indent=2, sort_keys=True)
    blob2 = json.dumps(deform.score_output(verts, faces, poses),
                       indent=2, sort_keys=True)
    check("deterministic json", blob1 == blob2)

    # Gate logic on synthetic dicts (no remesh needed).
    base = {"elbow90": {
        "flips": {"max": 0}, "stretch_p95": {"max": 1.5},
        "volume_loss_pct": {"max": 10.0}}}
    good = {"elbow90": {"flips": 0, "stretch_p95": 1.4,
                        "volume_loss_pct": 9.0}}
    check("gate passes inside spread",
          deform.check_case("c", good, base) == [])
    bad = {"elbow90": {"flips": 0, "stretch_p95": 1.4,
                       "volume_loss_pct": 9.0}}
    bad["elbow90"] = dict(bad["elbow90"], flips=1)
    check("gate trips on flips above zero spread",
          len(deform.check_case("c", bad, base)) == 1)
    bad["elbow90"] = dict(good["elbow90"], stretch_p95=1.6)
    check("gate trips on stretch beyond spread",
          len(deform.check_case("c", bad, base)) == 1)
    bad["elbow90"] = dict(good["elbow90"], volume_loss_pct=10.2)
    check("gate trips on volume beyond spread",
          len(deform.check_case("c", bad, base)) == 1)
    check("gate absolute without baseline",
          len(deform.check_case("c", {"elbow90": dict(good["elbow90"],
                                                     flips=2)}, None)) == 1)
    check("gate fails closed on no poses",
          deform.check_case("c", {}, base) == ["c: no poses scored"])
    check("summarize tolerates no poses",
          deform.summarize("tube", 1000, [{"seed": 0, "poses": {}}]) == {})
    print("\nall deform regression tests passed")


if __name__ == "__main__":
    main()
