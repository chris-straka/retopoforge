#!/usr/bin/env python3
"""Human lane: retopo vs Blender QuadriFlow vs Blender voxel on CC0 people.

    python3 bench/humans.py [--tag NAME] [--targets 2000,5000] [--only a,b]

Inputs: the shared CC0 corpus (8 MPFB2 characters in clothes, built by
~/SWE/blender/weightforge/bench/corpus/fetch.sh into
$FORGE_BENCH/corpus). For each, bench/humans_blender.py makes a scan-style
soup (voxel remesh at 1/200 of height + 0.1% noise: one closed dense
triangle skin with the clothes fused on, like photogrammetry or an AI
generator), and every tool remeshes the soup to each quad target. Scored by
bench/score.py against the noise-free outer skin (quads, non-quads, irregular
vertices, corner-angle error, two-sided surface distance in % of the
diagonal) plus wall time. Writes bench/results/humans-<tag>/
{summary.json,scorecard.md,sheet.png} and copies summary + scorecard to
bench/history/ (tracked).
"""

import argparse
import datetime
import json
import os
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import score as scoring  # noqa: E402

RETOPO = os.environ.get("RETOPO_BINARY", os.path.join(ROOT, "rust", "target", "release", "retopo"))
FORGE = os.environ.get("FORGE_BENCH", os.path.expanduser("~/.cache/forge-bench"))
CORPUS = os.path.join(FORGE, "corpus")
BH = os.path.join(HERE, "humans_blender.py")


def blender_bin():
    for c in (os.environ.get("BLENDER_BINARY"), os.environ.get("BLENDER"), shutil.which("blender"),
              "/Applications/Blender.app/Contents/MacOS/Blender"):
        if c and os.path.exists(c):
            return os.path.realpath(c)
    sys.exit("set $BLENDER_BINARY")


def timed(cmd):
    t0 = time.time()
    p = subprocess.run(cmd, capture_output=True, text=True)
    return p, round(time.time() - t0, 2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", default=datetime.date.today().isoformat())
    ap.add_argument("--targets", default="2000,5000")
    ap.add_argument("--only")
    a = ap.parse_args()
    bl = blender_bin()
    if not os.path.exists(RETOPO):
        subprocess.run(["cargo", "build", "--locked", "--release", "-p", "retopo", "-j2"],
                       cwd=os.path.join(ROOT, "rust"), check=True)
    names = sorted(os.listdir(CORPUS))
    if a.only:
        names = [n for n in names if n in a.only.split(",")]
    targets = [int(x) for x in a.targets.split(",")]
    out = os.path.join(HERE, "results", f"humans-{a.tag}")
    os.makedirs(out, exist_ok=True)
    summary = {"tag": a.tag, "date": datetime.datetime.now().isoformat(timespec="seconds"),
               "commit": subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=ROOT, capture_output=True,
                                        text=True).stdout.strip(), "targets": targets, "characters": {}}
    for n in names:
        w = os.path.join(out, n)
        os.makedirs(w, exist_ok=True)
        clean, scan = os.path.join(w, "clean.obj"), os.path.join(w, "scan.obj")
        if not os.path.exists(scan):
            p, _ = timed([bl, "-b", "--factory-startup", "--python", BH, "--", "prep",
                          os.path.join(CORPUS, n, "ref.glb"), clean, scan])
            if not os.path.exists(scan):
                summary["characters"][n] = {"error": p.stdout[-500:] + p.stderr[-500:]}
                print(f"{n}: prep failed: {summary['characters'][n]['error']}", flush=True)
                continue
        res = {}
        for t in targets:
            row = {}
            o = os.path.join(w, f"retopo_{t}.obj")
            p, dt = timed([RETOPO, "--input", scan, "--output", o, "--target-quads", str(t), "-q"])
            row["retopo"] = {"seconds": dt, "exit": p.returncode}
            o2 = os.path.join(w, f"quadriflow_{t}.obj")
            p, dt2 = timed([bl, "-b", "--factory-startup", "--python", BH, "--", "quadriflow", scan, o2, str(t)])
            row["quadriflow"] = {"seconds": dt2, "exit": p.returncode}
            for tool, path in (("retopo", o), ("quadriflow", o2)):
                if os.path.exists(path):
                    try:
                        row[tool].update(scoring.score(clean, path, target=t))
                    except Exception as e:  # a broken output is a result, not a crash
                        row[tool]["error"] = str(e)[:200]
                else:
                    row[tool]["error"] = "no output"
            res[str(t)] = row
            print(n, t, {k: (v.get("quads"), v.get("irr_pct"), v.get("dist_mean"), v["seconds"]) for k, v in row.items()},
                  flush=True)
        summary["characters"][n] = res
        with open(os.path.join(out, "summary.json"), "w") as fh:
            json.dump(summary, fh, indent=1)
    with open(os.path.join(out, "summary.json"), "w") as fh:
        json.dump(summary, fh, indent=1)
    sheet(bl, out, [n for n in names if "error" not in summary["characters"].get(n, {"error": 1})], max(targets))
    write_scorecard(summary, os.path.join(out, "scorecard.md"))
    hist = os.path.join(HERE, "history")
    os.makedirs(hist, exist_ok=True)
    shutil.copy(os.path.join(out, "summary.json"), os.path.join(hist, f"humans-{a.tag}.json"))
    shutil.copy(os.path.join(out, "scorecard.md"), os.path.join(hist, f"humans-{a.tag}.md"))
    print(open(os.path.join(out, "scorecard.md")).read())


def sheet(bl, out, names, t):
    try:
        from PIL import Image, ImageDraw
    except ImportError:
        return
    rows = [("scan input", "scan.obj"), ("retopoforge", f"retopo_{t}.obj"), ("QuadriFlow (Blender)", f"quadriflow_{t}.obj")]
    cols = names[:4]
    tiles = {}
    for c in cols:
        paths = [os.path.join(out, c, f) for _, f in rows]
        paths = [p for p in paths if os.path.exists(p)]
        dst = os.path.join(out, c, "tiles")
        subprocess.run([bl, "-b", "--factory-startup", "--python", BH, "--", "render", dst] + paths,
                       capture_output=True, text=True)
        tiles[c] = [f"{dst}.tile{i}.png" for i in range(len(paths))]
    tw, th, lw = 360, 480, 170
    img = Image.new("RGB", (lw + tw * len(cols), 30 + th * len(rows)), (40, 40, 40))
    d = ImageDraw.Draw(img)
    d.text((8, 8), f"retopo on scan-style CC0 humans, {t} quad target", fill=(255, 255, 255))
    for ci, c in enumerate(cols):
        d.text((lw + ci * tw + 6, 18), c, fill=(220, 220, 220))
        for ri, f in enumerate(tiles[c]):
            if os.path.exists(f):
                img.paste(Image.open(f).convert("RGB"), (lw + ci * tw, 30 + ri * th))
    for ri, (label, _) in enumerate(rows):
        d.text((8, 30 + ri * th + 8), label, fill=(255, 255, 255))
    img.save(os.path.join(out, "sheet.png"))


def write_scorecard(s, path):
    import statistics as st
    L = [f"# retopoforge human lane `{s['tag']}` ({s['commit']}, {s['date']})", "",
         f"{len(s['characters'])} CC0 MPFB2 characters as scan-style soups (voxel 1/200 height + noise), "
         "scored against the noise-free outer skin. irr = irregular vertices %; ang = mean |corner - 90| deg; "
         "dist = two-sided surface distance, % of diagonal; nonq = non-quad faces.", ""]
    for t in s["targets"]:
        L += [f"## {t} quads", "", "| tool | quads | nonq | irr % | ang deg | dist mean % | dist max % | seconds | ok |",
              "|---|---|---|---|---|---|---|---|---|"]
        for tool in ("retopo", "quadriflow"):
            rows = [c[str(t)][tool] for c in s["characters"].values() if str(t) in c]
            ok = [r for r in rows if "quads" in r]

            def m(k):
                xs = [r[k] for r in ok if r.get(k) is not None]
                return st.mean(xs) if xs else float("nan")
            L.append(f"| {tool} | {m('quads'):.0f} | {m('non_quads'):.0f} | {m('irr_pct'):.1f} | {m('angdev_mean'):.1f} | "
                     f"{m('dist_mean'):.3f} | {m('dist_max'):.2f} | {m('seconds'):.1f} | {len(ok)}/{len(rows)} |")
        L.append("")
    with open(path, "w") as fh:
        fh.write("\n".join(L) + "\n")


if __name__ == "__main__":
    main()
