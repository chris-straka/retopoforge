#!/usr/bin/env python3
"""Creature and prop bench: real rigs, real joints (stdlib only).

    bench/creature/run.py WORK [--subjects a,b] [--tools retopo,quadriflow,...]
        [--tris 10000] [--stages clean,prep,ref,cut,score] [--jobs 3]
        [--tag NAME] [--retopo-args "..."]

WORK/subjects.tsv lists the subjects, one a line:
    name <tab> kind <tab> source [<tab> budget]
kind is `tripo` (a generated model: cleaned with motionforge's
tripo_to_glb, rigged once with skintokens' creature route), `animal` (a
hand-made rigged animal: `source` is its folder with truth.glb and
bare.glb from skintokens' tools/bench/prep_truth.py; the artist's rig is
the reference and the artist's own mesh is scored as tool `artist`), or
`prop` (a GLB, no rig: no bend scores). A budget on the line (an
animal's own triangle count) replaces --tris for that subject. An animal's
low-poly mesh is smoothed two Catmull-Clark levels to make the dense
source.

Stages, each skipped when its output exists (delete to redo):
  clean  WORK/<name>/clean.glb          one mesh, facing forward, on the ground
  prep   WORK/<name>/dense.obj, ref_cut.glb   (bench/creature/prep.py)
  ref    WORK/<name>/ref/<name>.glb, .rom.ron  the reference rig, made once
  cut    WORK/<name>/cand/<tool>@<tris>.obj    each tool at the budget
  score  WORK/<name>/score/<tool>@<tris>.json  transfer + metrics + weights check

Every tool cuts the same dense.obj; every candidate gets the reference's
weights the same way (bench/creature/transfer.py), so only topology
differs. A tool's count is calibrated once toward the budget: if the
first cut's triangles are more than 8% off, it cuts again with the target
scaled by the miss.

Tools: retopo (this repo's CLI; `--retopo-args` adds flags; `--tag` names
the variant, e.g. tool `retopo.skel`), quadriflow (Blender's QuadriFlow,
input decimated to max(6x budget, 150k) as meshkit's clean_copy does),
decimate (Blender collapse: triangles), artist (animals only).

Environment: BLENDER, RETOPO, WEIGHTS, and for the `ref` stage the
creature route's (CREATURE_ROUTE, CREATURE_RIG_BIN, SKINTOKENS_BIN,
MOTIONFORGE_BIN, RFCHECK_BIN, TRIPO_TO_GLB).
"""

import argparse
import concurrent.futures as cf
import json
import os
import shlex
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))


def env(name, default=None):
    v = os.environ.get(name, default)
    if v is None:
        raise SystemExit(f"run.py: set {name}")
    return v


def sh(cmd, log, timeout=3600, extra_env=None):
    t = time.time()
    e = dict(os.environ, **(extra_env or {}))
    with open(log, "a") as f:
        f.write("$ " + " ".join(shlex.quote(c) for c in cmd) + "\n")
        f.flush()
        try:
            r = subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT, timeout=timeout, env=e)
            code = r.returncode
        except subprocess.TimeoutExpired:
            code = 124
        f.write(f"# exit {code} in {time.time() - t:.1f}s\n")
    return code


def blender(script, *a, log, timeout=3600):
    return sh([env("BLENDER"), "-b", "--factory-startup", "-t", os.environ.get("BLENDER_THREADS", "2"),
               "-P", os.path.join(HERE, script), "--", *a], log, timeout)


def obj_census(path):
    q = t = 0
    with open(path) as f:
        for line in f:
            if line.startswith("f "):
                n = len(line.split()) - 1
                t += n - 2
                q += n == 4
    return t, q


# ---------------------------------------------------------------- stages

def stage_clean(s, d, log):
    out = os.path.join(d, "clean.glb")
    if os.path.exists(out):
        return True
    if s["kind"] == "tripo":
        code = sh([env("BLENDER"), "-b", "--factory-startup", "-P", env("TRIPO_TO_GLB"), "--",
                   s["source"], out, "--tris", "300000", "--convention", "+x"], log)
    elif s["kind"] == "animal":
        shutil.copyfile(os.path.join(s["source"], "bare.glb"), out)
        code = 0
    else:
        shutil.copyfile(s["source"], out)
        code = 0
    return code == 0 and os.path.exists(out)


def stage_prep(s, d, log):
    if os.path.exists(os.path.join(d, "dense.obj")):
        return True
    ref = "18000" if s["kind"] == "tripo" else "0"
    sub = "2" if s["kind"] == "animal" else "0"
    blender("prep.py", os.path.join(d, "clean.glb"), d, "--ref-tris", ref, "--subdivide", sub, log=log)
    return os.path.exists(os.path.join(d, "dense.obj"))


def stage_ref(s, d, log):
    rd = os.path.join(d, "ref")
    rig = os.path.join(rd, s["name"] + ".glb")
    if s["kind"] == "prop" or os.path.exists(rig):
        return True
    os.makedirs(rd, exist_ok=True)
    if s["kind"] == "animal":
        shutil.copyfile(os.path.join(s["source"], "truth.glb"), rig)
        # the artist's mesh is its own reference cut (same vertices)
        shutil.copyfile(os.path.join(s["source"], "bare.glb"), os.path.join(d, "ref_cut.glb"))
        return True
    code = sh(["python3", env("CREATURE_ROUTE"), os.path.join(d, "ref_cut.glb"), rd, "--name", s["name"],
               "--clean-mesh", os.path.join(d, "ref_cut.glb"), "--class", "monster", "--no-clips"],
              log, timeout=7200)
    return os.path.exists(rig) and code in (0, 1)


def cut_retopo(dense, out, tris, extra, log):
    q = max(100, tris // 2)
    argv = [env("RETOPO"), "--input", dense, "--output", out, "--target-quads", str(q), "--quiet"]
    argv += shlex.split(extra or "")
    return sh(argv, log, timeout=3600)


def cut_blender(how, dense, out, tris, log):
    return blender("cut.py", how, dense, out, str(tris), log=log)


def one_cut(s, d, tool, tris, a, log):
    cand = os.path.join(d, "cand")
    os.makedirs(cand, exist_ok=True)
    out = os.path.join(cand, f"{tool}@{tris}.obj")
    if os.path.exists(out):
        return out
    dense = os.path.join(d, "dense.obj")
    if tool == "artist":
        code = blender("cut.py", "artist", os.path.join(d, "clean.glb"), out, str(tris), log=log)
        return out if code == 0 and os.path.exists(out) else None
    target = tris
    for attempt in range(2):
        tmp = out + ".tmp.obj"
        if tool.startswith("retopo"):
            code = cut_retopo(dense, tmp, target, a.retopo_args if tool != "retopo" else "", log)
        else:
            code = cut_blender(tool, dense, tmp, target, log)
        if code != 0 or not os.path.exists(tmp):
            return None
        got, _ = obj_census(tmp)
        os.replace(tmp, out)
        if abs(got - tris) <= 0.08 * tris or got == 0:
            break
        target = max(200, int(target * tris / got))
        os.replace(out, out + ".first.obj")
    return out


def one_score(s, d, cand, log):
    name = os.path.basename(cand)[:-4]
    sd = os.path.join(d, "score")
    os.makedirs(sd, exist_ok=True)
    out = os.path.join(sd, name + ".json")
    if os.path.exists(out):
        return out
    res = {"subject": s["name"], "kind": s["kind"], "cand": name}
    rigged = os.path.join(sd, name + ".rigged.glb")
    mj = os.path.join(sd, name + ".metrics.json")
    if s["kind"] == "prop":
        blender("metrics.py", cand, os.path.join(d, "dense.obj"), mj, "--no-poses", log=log)
    else:
        ref = os.path.join(d, "ref", s["name"] + ".glb")
        blender("transfer.py", ref, os.path.join(d, "ref_cut.glb"), cand, rigged, log=log)
        if not os.path.exists(rigged):
            res["error"] = "transfer failed"
            json.dump(res, open(out, "w"), indent=1)
            return out
        blender("metrics.py", rigged, os.path.join(d, "dense.obj"), mj, cand, log=log)
        rom = os.path.join(d, "ref", s["name"] + ".rom.ron")
        argv = [env("WEIGHTS"), "check", rigged, "--json", "--no-clips"]
        if os.path.exists(rom):
            argv += ["--poses", rom]
        r = subprocess.run(argv, capture_output=True, text=True, timeout=3600)
        try:
            w = json.loads(r.stdout)
            res["bend"] = {"score": w.get("score"), "pass": w.get("pass"),
                           "fails": sum(1 for f in w.get("findings", []) if f.get("severity") == "fail"),
                           "bad_verts": sum(x.get("bad", 0) for x in w.get("regions", [])),
                           "energy": round(sum(x.get("energy", 0.0) for x in w.get("regions", [])), 6),
                           "findings": [f"{f['code']}:{f.get('region')}" for f in w.get("findings", [])]}
        except ValueError:
            res["bend"] = {"error": (r.stderr or "")[-300:]}
    if os.path.exists(mj):
        res["metrics"] = json.load(open(mj))
    else:
        res["error"] = "metrics failed"
    json.dump(res, open(out, "w"), indent=1, sort_keys=True)
    return out


def subjects(work):
    rows = []
    with open(os.path.join(work, "subjects.tsv")) as f:
        for line in f:
            if line.strip() and not line.startswith("#"):
                cols = line.rstrip("\n").split("\t")
                n, k, src = cols[:3]
                rows.append({"name": n, "kind": k, "source": os.path.expanduser(src),
                             "tris": int(cols[3]) if len(cols) > 3 and cols[3] else None})
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("work")
    ap.add_argument("--subjects", default="")
    ap.add_argument("--tools", default="retopo,quadriflow,decimate")
    ap.add_argument("--tris", default="10000")
    ap.add_argument("--stages", default="clean,prep,ref,cut,score")
    ap.add_argument("--jobs", type=int, default=3)
    ap.add_argument("--tag", default="", help="variant name: the retopo tool becomes retopo.<tag>")
    ap.add_argument("--retopo-args", default="")
    a = ap.parse_args()
    work = os.path.abspath(a.work)
    subs = subjects(work)
    if a.subjects:
        keep = set(a.subjects.split(","))
        subs = [s for s in subs if s["name"] in keep]
    stages = set(a.stages.split(","))
    tools = [t for t in a.tools.split(",") if t]
    if a.tag:
        tools = [f"retopo.{a.tag}" if t == "retopo" else t for t in tools]
    budgets = [int(x) for x in a.tris.split(",")]

    def prepare(s):
        d = os.path.join(work, s["name"])
        os.makedirs(d, exist_ok=True)
        log = os.path.join(d, "bench.log")
        for st, fn in (("clean", stage_clean), ("prep", stage_prep), ("ref", stage_ref)):
            if st in stages and not fn(s, d, log):
                return f"{s['name']}: {st} failed (see {log})"
        return None

    with cf.ThreadPoolExecutor(a.jobs) as ex:
        for msg in ex.map(prepare, subs):
            if msg:
                print(msg, flush=True)

    jobs = []
    for s in subs:
        d = os.path.join(work, s["name"])
        for tool in tools:
            if tool == "artist" and s["kind"] != "animal":
                continue
            for t in ([s["tris"]] if s["tris"] else budgets):
                jobs.append((s, d, tool, t))

    def go(j):
        s, d, tool, t = j
        log = os.path.join(d, "bench.log")
        cand = os.path.join(d, "cand", f"{tool}@{t}.obj")
        if "cut" in stages:
            cand = one_cut(s, d, tool, t, a, log)
        if not cand or not os.path.exists(cand):
            return f"{s['name']} {tool}@{t}: no cut"
        if "score" in stages:
            p = one_score(s, d, cand, log)
            r = json.load(open(p))
            m = r.get("metrics", {})
            b = r.get("bend", {})
            return (f"{s['name']:20s} {tool}@{t}: tris {m.get('tris')} loops {m.get('joint_loops_mean')} "
                    f"poles {m.get('joint_poles_mean')} flips {m.get('rest_flips')}/{m.get('posed_flips')} "
                    f"dist95 {m.get('dist_p95')} bend {b.get('score')} {r.get('error', '')}")
        return f"{s['name']} {tool}@{t}: cut"

    with cf.ThreadPoolExecutor(a.jobs) as ex:
        for msg in ex.map(go, jobs):
            print(msg, flush=True)


if __name__ == "__main__":
    main()
