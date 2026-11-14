#!/usr/bin/env python3
"""Creature and prop bench: real rigs, real joints (stdlib only).

    bench/creature/run.py WORK [--subjects a,b] [--tools retopo,quadriflow,...]
        [--tris 10000] [--stages clean,prep,ref,cut,score] [--jobs 3]
        [--tag NAME] [--retopo-args "..."]

WORK/subjects.tsv lists the subjects, one a line:
    name <tab> kind <tab> source [<tab> budget]
kind is `character` (one of the owner's generated characters; source is
`MODEL.fbx|RIG.glb`: the download and the human route's rig, fitted onto
our own cut of the body; with $CLIPS set, every candidate gets the
game's clips copied on by bone name (bench/creature/clips_onto.py: the
clip file is on the same standard skeleton) and is checked on them),
`pro` (a professionally made rigged game mesh, a GLB: it is
degraded into what a generator gives, smoothed one Catmull-Clark level
and voxel-fused, every tool remeshes that, and the pro's own mesh and rig
are the truth, scored as tool `artist`), `tripo` (a generated model: cleaned with motionforge's
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
  guides WORK/<name>/ref/aligned.glb, guides/{skeleton,rings}.txt
         the rig in the bench frame; retopo's --skeleton and joint-ring
         --guides made from it (bench/creature/rig_guides.py)
  cut    WORK/<name>/cand/<tool>@<tris>.obj    each tool at the budget
  score  WORK/<name>/score/<tool>@<tris>.json  transfer + metrics + weights check
  remetrics  (only when asked) re-measure existing scores' metrics in place

Every tool cuts the same dense.obj; every candidate gets the reference's
weights the same way (bench/creature/transfer.py), so only topology
differs. A tool's count is calibrated toward the budget: while a cut's
triangles are more than 8% off, it cuts again (up to three tries) with
the target scaled by the miss, and keeps the try closest to the budget.

Tools: retopo (this repo's CLI; `--retopo-args` adds flags; `--tag` names
the variant, e.g. tool `retopo.skel`; `{dir}` in the flags is the
subject's folder: `--retopo-args "--skeleton {dir}/guides/skeleton.txt"`), quadriflow (Blender's QuadriFlow,
input decimated to max(6x budget, 150k) as meshkit's clean_copy does),
decimate (Blender collapse: triangles), artist (animals only). Any tool
`instant` (Instant Meshes, $INSTANT_MESHES) and `quadwild` (QuadWild +
Bi-MDF, $QUADWILD = the release folder) are outside rivals run as black
boxes; `quadwild.rings` is QuadWild with each joint's ring cut into its
input and marked as a feature (bench/creature/qw_rings.py, a `.sharp`
file). Any tool
with `+lay` appended is that tool's cut laid back onto the source
(bench/creature/lay.py), e.g. `retopo+lay`. Any tool with `~sN` appended
(`retopo~s1`) is that tool on the source jittered with seed N: the spread
over seeds is the noise floor a difference has to beat.

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


def sh(cmd, log, timeout=3600, extra_env=None, cwd=None):
    t = time.time()
    e = dict(os.environ, **(extra_env or {}))
    with open(log, "a") as f:
        f.write("$ " + " ".join(shlex.quote(c) for c in cmd) + "\n")
        f.flush()
        try:
            r = subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT, timeout=timeout, env=e, cwd=cwd)
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
    if s["kind"] == "character":
        code = sh([env("BLENDER"), "-b", "--factory-startup", "-P", env("TRIPO_TO_GLB"), "--",
                   s["source"].split("|")[0], out, "--tris", "300000", "--convention", "+x"], log)
    elif s["kind"] == "tripo":
        code = sh([env("BLENDER"), "-b", "--factory-startup", "-P", env("TRIPO_TO_GLB"), "--",
                   s["source"], out, "--tris", "300000", "--convention", "+x"], log)
    elif s["kind"] == "animal":
        shutil.copyfile(os.path.join(s["source"], "bare.glb"), out)
        code = 0
    elif s["kind"] == "pro":
        shutil.copyfile(s["source"], out)
        code = 0
    else:
        shutil.copyfile(s["source"], out)
        code = 0
    return code == 0 and os.path.exists(out)


def stage_prep(s, d, log):
    if os.path.exists(os.path.join(d, "dense.obj")):
        return True
    ref = "18000" if s["kind"] in ("tripo", "character") else "0"
    sub = "2" if s["kind"] == "animal" else "1" if s["kind"] == "pro" else "0"
    fuse = "0.003" if s["kind"] in ("animal", "pro") else "0"
    blender("prep.py", os.path.join(d, "clean.glb"), d, "--ref-tris", ref, "--subdivide", sub, "--fuse", fuse,
            log=log)
    return os.path.exists(os.path.join(d, "dense.obj"))


def stage_ref(s, d, log):
    rd = os.path.join(d, "ref")
    rig = os.path.join(rd, s["name"] + ".glb")
    if s["kind"] == "prop" or os.path.exists(rig):
        return True
    os.makedirs(rd, exist_ok=True)
    if s["kind"] == "character":
        # the human route's rig; transfer.py fits it onto our own cut of the
        # same body (bounding box and quarter turns: different vertices)
        shutil.copyfile(s["source"].split("|")[1], rig)
        return True
    if s["kind"] == "pro":
        shutil.copyfile(s["source"], rig)
        shutil.copyfile(s["source"], os.path.join(d, "ref_cut.glb"))
        return True
    if s["kind"] == "animal":
        shutil.copyfile(os.path.join(s["source"], "truth.glb"), rig)
        # the artist's mesh is its own reference cut (same vertices)
        shutil.copyfile(os.path.join(s["source"], "bare.glb"), os.path.join(d, "ref_cut.glb"))
        return True
    code = sh(["python3", env("CREATURE_ROUTE"), os.path.join(d, "ref_cut.glb"), rd, "--name", s["name"],
               "--clean-mesh", os.path.join(d, "ref_cut.glb"), "--class", "monster", "--no-clips"],
              log, timeout=7200)
    return os.path.exists(rig) and code in (0, 1)


def stage_guides(s, d, log):
    """The reference rig in the bench frame, and retopo inputs made from it."""
    if s["kind"] == "prop":
        return True
    aligned = os.path.join(d, "ref", "aligned.glb")
    if not os.path.exists(aligned):
        blender("transfer.py", os.path.join(d, "ref", s["name"] + ".glb"), os.path.join(d, "ref_cut.glb"),
                os.path.join(d, "ref_cut.glb"), aligned, log=log)
    g = os.path.join(d, "guides")
    if not os.path.exists(os.path.join(g, "rings.txt")):
        blender("rig_guides.py", aligned, os.path.join(d, "dense.obj"), g, log=log)
    return os.path.exists(os.path.join(g, "rings.txt"))


def cut_retopo(dense, out, tris, extra, log, extra_env=None):
    q = max(100, tris // 2)
    d = os.path.dirname(dense)
    argv = [env("RETOPO"), "--input", dense, "--output", out, "--target-quads", str(q), "--quiet"]
    argv += shlex.split((extra or "").replace("{dir}", d))
    e = {k: v.replace("{dir}", d) for k, v in (extra_env or {}).items()}
    return sh(argv, log, timeout=3600, extra_env=e)


def cut_instant(dense, out, tris, log):
    """Instant Meshes (BSD-3, black box) in batch mode: pure quads."""
    return sh([env("INSTANT_MESHES"), "-o", out, "-f", str(max(50, tris // 2)), "-d", dense], log, timeout=3600)


def cut_quadwild(dense, out, tris, log, rings=False):
    """QuadWild + Bi-MDF (GPL-3, black box): its organic prep once per
    subject (cached), then quantization with scaleFact set so the quad
    count lands near the budget (count ~ 1/scaleFact^2, from a probe)."""
    import math
    import re
    qw = env("QUADWILD")
    work = os.path.join(os.path.dirname(dense), "qw_rings" if rings else "qw")
    os.makedirs(work, exist_ok=True)
    src = os.path.join(work, "src.obj")
    rem = os.path.join(work, "src_rem_p0.obj")
    if not os.path.exists(rem):
        if rings:
            # the joint rings cut in and handed to QuadWild as features
            joints = os.path.join(os.path.dirname(dense), "guides", "joints.json")
            blender("qw_rings.py", dense, joints, src, log=log)
            if not os.path.exists(src):
                return 1
        else:
            shutil.copyfile(dense, src)
        sh([os.path.join(qw, "quadwild"), src, "2", "config/prep_config/basic_setup_Organic.txt"], log,
           timeout=7200, cwd=qw)
        if not os.path.exists(rem):
            return 1

    def quantize(scale, tag):
        cfg = open(os.path.join(qw, "config/main_config/flow_noalign_lemon.txt")).read()
        cfg = re.sub(r"(?m)^scaleFact .*$", f"scaleFact {scale:.6g}", cfg)
        cp = os.path.join(work, "main.txt")
        open(cp, "w").write(cfg)
        sh([os.path.join(qw, "quad_from_patches"), rem, tag, cp], log, timeout=7200, cwd=qw)
        res = rem[:-4] + f"_{tag}_quadrangulation_smooth.obj"
        return res if os.path.exists(res) else None

    base_f = os.path.join(work, "base.txt")
    if not os.path.exists(base_f):
        probe = quantize(1.0, "1")
        open(base_f, "w").write(str(obj_census(probe)[1] if probe else 0))
    base = int(open(base_f).read() or 0)
    if base <= 0:
        return 1
    res = quantize(math.sqrt(base / max(1, tris // 2)), str(tris + 2))
    if not res:
        return 1
    shutil.copyfile(res, out)
    return 0


def cut_blender(how, dense, out, tris, log):
    return blender("cut.py", how, dense, out, str(tris), log=log)


def jittered(dense, seed):
    import random
    out = f"{dense[:-4]}.s{seed}.obj"
    if os.path.exists(out):
        return out
    rows = open(dense).read().split("\n")
    vs = [tuple(float(x) for x in r.split()[1:4]) for r in rows if r.startswith("v ")]
    diag = sum((max(v[k] for v in vs) - min(v[k] for v in vs)) ** 2 for k in range(3)) ** 0.5
    rng = random.Random(seed)
    amp = 1e-5 * diag
    with open(out, "w") as f:
        for r in rows:
            if r.startswith("v "):
                x, y, z = (float(t) for t in r.split()[1:4])
                f.write(f"v {x + rng.uniform(-amp, amp):.8f} {y + rng.uniform(-amp, amp):.8f} "
                        f"{z + rng.uniform(-amp, amp):.8f}\n")
            elif r:
                f.write(r + "\n")
    return out


def one_cut(s, d, tool, tris, a, log):
    cand = os.path.join(d, "cand")
    os.makedirs(cand, exist_ok=True)
    out = os.path.join(cand, f"{tool}@{tris}.obj")
    if os.path.exists(out):
        return out
    dense = os.path.join(d, "dense.obj")
    seed = None
    if "~s" in tool:
        # a noise seed: the same tool on the source with every vertex moved
        # a hair (1e-5 of the diagonal), to measure the scores' spread
        seed = int(tool.rsplit("~s", 1)[1])
        dense = jittered(dense, seed)
    if tool.endswith("+lay"):
        base = one_cut(s, d, tool[:-4], tris, a, log)
        if not base:
            return None
        code = blender("lay.py", base, dense, out, log=log)
        return out if code == 0 and os.path.exists(out) else None
    if tool == "artist":
        if s["kind"] not in ("animal", "pro"):
            return None
        code = blender("cut.py", "artist", os.path.join(d, "clean.glb"), out, str(tris), log=log)
        return out if code == 0 and os.path.exists(out) else None
    target = tris
    tries = []
    for attempt in range(3):
        tmp = f"{out}.try{attempt}.obj"
        if tool == "instant":
            code = cut_instant(dense, tmp, target, log)
        elif tool in ("quadwild", "quadwild.rings"):
            code = cut_quadwild(dense, tmp, target, log, rings=tool == "quadwild.rings")
        elif tool.startswith("retopo"):
            plain = tool.split("~s")[0] == "retopo"
            code = cut_retopo(dense, tmp, target, "" if plain else a.retopo_args, log,
                              None if plain else dict(x.split("=", 1) for x in a.retopo_env))
        else:
            code = cut_blender(tool, dense, tmp, target, log)
        if code != 0 or not os.path.exists(tmp):
            break
        got, _ = obj_census(tmp)
        tries.append((abs(got - tris), attempt, tmp))
        if abs(got - tris) <= 0.08 * tris or got == 0:
            break
        target = max(200, int(target * tris / got))
    if not tries:
        return None
    # the try closest to the budget (a tool's count need not grow with its target)
    os.replace(min(tries)[2], out)
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
        blender("metrics.py", rigged, os.path.join(d, "dense.obj"), mj, cand, *pro_args(s, d, cand), log=log)
        rom = os.path.join(d, "ref", s["name"] + ".rom.ron")
        argv = [env("WEIGHTS"), "check", rigged, "--json", "--no-clips"]
        if os.path.exists(rom):
            argv += ["--poses", rom]
        if s["kind"] == "character" and os.environ.get("CLIPS"):
            # the game's clips on this candidate, then the check samples them
            moving = os.path.join(sd, name + ".moving.glb")
            sh(["python3", os.path.join(HERE, "clips_onto.py"), rigged, os.environ["CLIPS"], moving], log)
            if os.path.exists(moving):
                argv = [env("WEIGHTS"), "check", moving, "--json"]
                res["clips"] = True
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


def pro_args(s, d, cand):
    """The pro mesh to compare edge flow with (animal and pro subjects)."""
    if s["kind"] not in ("animal", "pro") or not s["tris"]:
        return []
    pro = os.path.join(d, "cand", f"artist@{s['tris']}.obj")
    rig = os.path.join(d, "score", f"artist@{s['tris']}.rigged.glb")
    out = ["--pro", pro] if os.path.exists(pro) else []
    if os.path.exists(rig) and "artist@" not in os.path.basename(cand):
        out += ["--pro-rigged", rig]
    return out


def remetrics(s, d, cand, log):
    """Re-measure an existing score's metrics (after metrics.py changed),
    keeping its cut, its carried rig and its bend score."""
    name = os.path.basename(cand)[:-4]
    sd = os.path.join(d, "score")
    out = os.path.join(sd, name + ".json")
    if not os.path.exists(out):
        return
    mj = os.path.join(sd, name + ".metrics.json")
    rigged = os.path.join(sd, name + ".rigged.glb")
    if os.path.exists(mj):
        os.remove(mj)
    if s["kind"] == "prop":
        blender("metrics.py", cand, os.path.join(d, "dense.obj"), mj, "--no-poses", log=log)
    elif os.path.exists(rigged):
        blender("metrics.py", rigged, os.path.join(d, "dense.obj"), mj, cand, *pro_args(s, d, cand), log=log)
    res = json.load(open(out))
    if os.path.exists(mj):
        res["metrics"] = json.load(open(mj))
        res.pop("error", None)
    json.dump(res, open(out, "w"), indent=1, sort_keys=True)


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
    ap.add_argument("--stages", default="clean,prep,ref,guides,cut,score")
    ap.add_argument("--jobs", type=int, default=3)
    ap.add_argument("--tag", default="", help="variant name: the retopo tool becomes retopo.<tag>")
    ap.add_argument("--retopo-args", default="")
    ap.add_argument("--retopo-env", action="append", default=[],
                    help="KEY=VALUE for the variant's retopo runs ({dir} is the subject's folder)")
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
        for st, fn in (("clean", stage_clean), ("prep", stage_prep), ("ref", stage_ref),
                       ("guides", stage_guides)):
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
            if tool == "artist" and s["kind"] not in ("animal", "pro"):
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
        if "remetrics" in stages:
            remetrics(s, d, cand, log)
        if "score" in stages or "remetrics" in stages:
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
