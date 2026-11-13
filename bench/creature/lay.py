"""Blender: lay a cut back onto its source surface (nearest point).

    blender -b --factory-startup -P bench/creature/lay.py -- CUT.obj DENSE.obj OUT.obj

Every vertex moves to the nearest point of the source surface, unless
that would turn one of its faces over (its normal past 90 degrees from
before), in which case it stays. Topology is unchanged. This is what
bench/creature/cut.py does to QuadriFlow's cut; as a tool suffix (`+lay`)
it tests whether a remesher loses only by where it puts its vertices.
"""
import sys

import bpy
import numpy as np
from mathutils.bvhtree import BVHTree


def load(path):
    before = set(bpy.data.objects)
    bpy.ops.wm.obj_import(filepath=path)
    return next(o for o in bpy.data.objects if o not in before)


def normals(v, faces):
    n = np.zeros((len(faces), 3))
    for k, f in enumerate(faces):
        p = v[list(f)]
        q = np.roll(p, -1, axis=0)
        n[k] = [((p[:, 1] - q[:, 1]) * (p[:, 2] + q[:, 2])).sum(),
                ((p[:, 2] - q[:, 2]) * (p[:, 0] + q[:, 0])).sum(),
                ((p[:, 0] - q[:, 0]) * (p[:, 1] + q[:, 1])).sum()]
    return n


def main():
    cut, dense, out = sys.argv[sys.argv.index("--") + 1:][:3]
    bpy.ops.wm.read_factory_settings(use_empty=True)
    src = load(dense)
    sv = [v.co.copy() for v in src.data.vertices]
    bvh = BVHTree.FromPolygons(sv, [tuple(p.vertices) for p in src.data.polygons])
    ob = load(cut)
    faces = [tuple(p.vertices) for p in ob.data.polygons]
    v = np.array([x.co[:] for x in ob.data.vertices])
    n0 = normals(v, faces)
    target = v.copy()
    for i in range(len(v)):
        hit = bvh.find_nearest(v[i])
        if hit[0] is not None:
            target[i] = hit[0][:]
    new = target.copy()
    # keep any vertex whose move turns a face over
    n1 = normals(new, faces)
    bad = (n0 * n1).sum(1) < 0
    for _ in range(3):
        if not bad.any():
            break
        for k in np.nonzero(bad)[0]:
            for i in faces[k]:
                new[i] = v[i]
        n1 = normals(new, faces)
        bad = (n0 * n1).sum(1) < 0
    for i, x in enumerate(ob.data.vertices):
        x.co = new[i]
    for o in bpy.context.scene.objects:
        o.select_set(o is ob)
    bpy.ops.wm.obj_export(filepath=out, export_selected_objects=True, export_uv=False,
                          export_normals=False, export_materials=False)
    print("LAY moved", int((np.linalg.norm(new - v, axis=1) > 0).sum()), "of", len(v))


main()
