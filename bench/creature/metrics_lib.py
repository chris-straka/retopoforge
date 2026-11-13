"""Shared pieces of the creature bench's Blender scripts (metrics, render)."""
import bpy
import numpy as np


def imported(fn):
    before = set(bpy.data.objects)
    fn()
    return [o for o in bpy.data.objects if o not in before]


def coords(ob, mesh=None):
    me = mesh or ob.data
    v = np.empty(len(me.vertices) * 3)
    me.vertices.foreach_get("co", v)
    v = v.reshape(-1, 3)
    m = np.array(ob.matrix_world)
    return v @ m[:3, :3].T + m[:3, 3]


def face_lists(me):
    return [tuple(p.vertices) for p in me.polygons]


def newell(v, faces):
    """Face normals (unit) and centres for arbitrary polygons."""
    n = np.zeros((len(faces), 3))
    c = np.zeros((len(faces), 3))
    a = np.zeros(len(faces))
    for k, f in enumerate(faces):
        p = v[list(f)]
        q = np.roll(p, -1, axis=0)
        nn = np.array([((p[:, 1] - q[:, 1]) * (p[:, 2] + q[:, 2])).sum(),
                       ((p[:, 2] - q[:, 2]) * (p[:, 0] + q[:, 0])).sum(),
                       ((p[:, 0] - q[:, 0]) * (p[:, 1] + q[:, 1])).sum()])
        ln = np.linalg.norm(nn)
        a[k] = ln / 2
        n[k] = nn / ln if ln > 0 else 0
        c[k] = p.mean(0)
    return n, c, a


def with_faces_of(glb_mesh, arm, obj_path):
    """The OBJ's mesh in the GLB's place: weights matched by position."""
    from mathutils.kdtree import KDTree
    objs = imported(lambda: bpy.ops.wm.obj_import(filepath=obj_path))
    c = next(o for o in objs if o.type == "MESH")
    gv = coords(glb_mesh)
    kd = KDTree(len(gv))
    for i, p in enumerate(gv):
        kd.insert(tuple(p), i)
    kd.balance()
    groups = {}
    for v in glb_mesh.data.vertices:
        groups[v.index] = [(glb_mesh.vertex_groups[g.group].name, g.weight) for g in v.groups]
    for vg in glb_mesh.vertex_groups:
        c.vertex_groups.new(name=vg.name)
    cw = coords(c)
    for i, p in enumerate(cw):
        _, j, _ = kd.find(tuple(p))
        for name, wt in groups[j]:
            c.vertex_groups[name].add([i], wt, "REPLACE")
    bpy.data.objects.remove(glb_mesh, do_unlink=True)
    if arm is not None:
        mw = c.matrix_world.copy()
        c.parent = arm
        c.matrix_world = mw
        m = c.modifiers.new("Armature", "ARMATURE")
        m.object = arm
    bpy.context.view_layer.update()
    return c


def weights_of(ob):
    """Vertex group names and a dense (verts x groups) weight matrix."""
    names = [g.name for g in ob.vertex_groups]
    w = np.zeros((len(ob.data.vertices), len(names)))
    for v in ob.data.vertices:
        for g in v.groups:
            w[v.index, g.group] = g.weight
    return names, w


def find_joints(cm, arm, w=None, names=None):
    """Every joint of the rig that holds skin on both sides: a bone with a
    skinned parent. Its ring axis (the bisector in a chain, the bone's own
    direction where a limb leaves the trunk), head, and limb radius (median
    distance from the axis of the vertices the two bones own near it)."""
    if w is None:
        names, w = weights_of(cm)
    cv = coords(cm)
    dom_v = np.where(w.max(1) > 0, w.argmax(1), -1)
    skinned = set(np.nonzero((w > 0.1).any(0))[0])
    gi = {n: i for i, n in enumerate(names)}
    am = arm.matrix_world

    def wpos(p):
        return np.array(am @ p)

    joints = []
    for b in arm.data.bones:
        p = b.parent
        if p is None or b.name not in gi or p.name not in gi:
            continue
        if gi[b.name] not in skinned or gi[p.name] not in skinned:
            continue
        h, t = wpos(b.head_local), wpos(b.tail_local)
        ph, pt = wpos(p.head_local), wpos(p.tail_local)
        lb, lp = np.linalg.norm(t - h), np.linalg.norm(pt - ph)
        if lb == 0 or lp == 0:
            continue
        db, dp = (t - h) / lb, (pt - ph) / lp
        # a joint in a chain (head at the parent's tail) rings the bisector;
        # a limb leaving the trunk (hip, shoulder, tail base) rings its bone
        dbis = db + dp
        if np.linalg.norm(h - pt) <= 0.25 * lp and np.linalg.norm(dbis) > 1e-6:
            dbis = dbis / np.linalg.norm(dbis)
        else:
            dbis = db
        rel = cv - h
        ax = rel @ dbis
        rad = np.linalg.norm(rel - np.outer(ax, dbis), axis=1)
        mine = ((dom_v == gi[b.name]) & (ax >= 0) & (ax <= 0.5 * lb)) | \
               ((dom_v == gi[p.name]) & (ax <= 0) & (ax >= -0.5 * lp))
        if mine.sum() < 8:
            continue
        r = float(np.median(rad[mine]))
        if r <= 0:
            continue
        joints.append({"bone": b.name, "parent": p.name, "head": h, "d": dbis, "db": db, "r": r,
                       "sub": {gi[x.name] for x in [b] + list(b.children_recursive) if x.name in gi}})
    return joints
