#!/usr/bin/env python3
"""The artist bench's fast loop (stdlib only).

    bench/artist/run.py CASES_DIR OUT_DIR --tool NAME [--retopo BIN] [--args "..."]
        [--env K=V ...] [--set fast|full|all] [--jobs 3] [--blender BIN]

Cuts every case's dense.obj with the tool at the artist's quad count,
into OUT_DIR/NAME/<case>.obj, then scores all of them against the
artists' meshes (bench/artist/score.py) into OUT_DIR/NAME.json and
prints the summary. Tools:

  retopo...   this repo's CLI (--retopo BIN, default the release build),
              `--args` and `--env` passed on (any name starting retopo)
  quadriflow  Blender's QuadriFlow (Blender's own tool in the add-on bar)
  decimate    Blender's collapse decimate to twice the quad count

The fast set (about twenty cases of 300 to 4,000 quads) runs in a few
minutes on three cores; `--set full` adds the big bodies and skull to
confirm a gain. A cut that fails or times out scores as a failure.
"""
import argparse
import concurrent.futures as cf
import json
import os
import shlex
import subprocess
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))

QF = '''
import bpy, sys
a = sys.argv[sys.argv.index("--") + 1:]
src, out, quads, how = a[0], a[1], int(a[2]), a[3]
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.wm.obj_import(filepath=src, forward_axis="NEGATIVE_Z", up_axis="Y")
ob = bpy.context.scene.objects[0]
bpy.context.view_layer.objects.active = ob
ob.select_set(True)
if how == "quadriflow":
    for v in ob.data.vertices:
        v.co *= 100.0
    r = bpy.ops.object.quadriflow_remesh(mode="FACES", target_faces=quads, seed=0, use_mesh_symmetry=False,
                                         use_preserve_sharp=False, use_preserve_boundary=False, smooth_normals=False)
    for v in ob.data.vertices:
        v.co /= 100.0
else:
    m = ob.modifiers.new("d", "DECIMATE")
    m.ratio = min(1.0, 2.0 * quads / max(1, len(ob.data.polygons)))
    bpy.ops.object.modifier_apply(modifier=m.name)
bpy.ops.wm.obj_export(filepath=out, export_selected_objects=True, export_uv=False, export_normals=False,
                      export_materials=False, forward_axis="NEGATIVE_Z", up_axis="Y")
'''


def jitter(dense, seed, folder, name):
    """The shape with every vertex moved a hair (1e-5 of the diagonal): the
    spread of scores over seeds is the noise a change has to beat."""
    import random
    os.makedirs(folder, exist_ok=True)
    out = os.path.join(folder, f"{name}.s{seed}.obj")
    if os.path.exists(out):
        return out
    rows = open(dense).read().split("\n")
    vs = [[float(x) for x in r.split()[1:4]] for r in rows if r.startswith("v ")]
    diag = sum((max(v[k] for v in vs) - min(v[k] for v in vs)) ** 2 for k in range(3)) ** 0.5
    rng = random.Random(seed * 7919 + len(vs))
    amp = 1e-5 * diag
    with open(out, "w") as f:
        for r in rows:
            if r.startswith("v "):
                x, y, z = (float(t) for t in r.split()[1:4])
                f.write(f"v {x + rng.uniform(-amp, amp):.9f} {y + rng.uniform(-amp, amp):.9f} {z + rng.uniform(-amp, amp):.9f}\n")
            elif r:
                f.write(r + "\n")
    return out


def cut(case_dir, out, tool, a):
    case = json.load(open(os.path.join(case_dir, "case.json")))
    dense = os.path.join(case_dir, "dense.obj")
    if a.seed:
        dense = jitter(dense, a.seed, os.path.join(a.out, "_jitter"), case["name"])
    quads = case["quads"]
    t0 = time.time()
    if tool.startswith("retopo"):
        argv = [a.retopo, "--input", dense, "--output", out, "--target-quads",
                str(int(round(quads * a.count_scale))), "--quiet"]
        argv += shlex.split(a.args)
        env = dict(os.environ, **dict(x.split("=", 1) for x in a.env))
        r = subprocess.run(argv, capture_output=True, text=True, timeout=600, env=env)
    else:
        script = os.path.join(a.out, "_cut.py")
        open(script, "w").write(QF)
        r = subprocess.run([a.blender, "-b", "--factory-startup", "-P", script, "--", dense, out, str(quads), tool],
                           capture_output=True, text=True, timeout=900)
    return case["name"], r.returncode, round(time.time() - t0, 1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cases")
    ap.add_argument("out")
    ap.add_argument("--tool", required=True)
    ap.add_argument("--retopo", default=os.path.join(ROOT, "rust/target/release/retopo"))
    ap.add_argument("--args", default="")
    ap.add_argument("--env", action="append", default=[])
    ap.add_argument("--set", default="fast")
    ap.add_argument("--count-scale", type=float, default=1.0,
                    help="retopo's --target-quads is the artist's count times this (with --edge-scaling k, use k*k)")
    ap.add_argument("--jobs", type=int, default=3)
    ap.add_argument("--seed", type=int, default=0, help="jitter the shapes (noise floor); 0 = as they are")
    ap.add_argument("--blender", default="blender")
    a = ap.parse_args()
    a.out = os.path.abspath(a.out)
    cuts = os.path.join(a.out, a.tool)
    os.makedirs(cuts, exist_ok=True)
    todo = []
    for name in sorted(os.listdir(a.cases)):
        d = os.path.join(a.cases, name)
        cj = os.path.join(d, "case.json")
        if os.path.exists(cj) and (a.set == "all" or json.load(open(cj)).get("set", "fast") == a.set):
            todo.append(d)
    t0 = time.time()
    with cf.ThreadPoolExecutor(a.jobs) as ex:
        res = list(ex.map(lambda d: cut(d, os.path.join(cuts, os.path.basename(d) + ".obj"), a.tool, a), todo))
    bad = [r for r in res if r[1] != 0]
    secs = round(time.time() - t0, 1)
    js = os.path.join(a.out, a.tool + ".json")
    subprocess.run([a.blender, "-b", "--factory-startup", "-P", os.path.join(HERE, "score.py"), "--",
                    a.cases, cuts, js, "--set", a.set], capture_output=True, text=True, timeout=3600)
    d = json.load(open(js))
    d["summary"].update(tool=a.tool, args=a.args, env=a.env, cut_secs=secs, cut_failures=[r[0] for r in bad])
    json.dump(d, open(js, "w"), indent=1)
    s = d["summary"]
    print(f"{a.tool}: rings {s.get('rings_pct_mean')}% flow {s.get('flow_deg_mean')} poles x{s.get('poles_x_median')} "
          f"near {s.get('poles_near_mean')}% odd {s.get('odd_mean')} count x{s.get('count_x_median')} "
          f"d95 {s.get('d95_median')} dmax {s.get('dmax_median')} | {len(todo)} cases, {secs}s, "
          f"{len(bad)} cut failures {[r[0] for r in bad]}")


if __name__ == "__main__":
    main()
