#!/usr/bin/env python3
"""Black-box QuadWild + Bi-MDF reference lane (stdlib only).

Runs the external GPL-3 QuadWild/Bi-MDF binaries (never vendored, never
linked; build them outside the repo) on bench models, calibrating its
`scaleFact` (it has no target-count knob) toward each quad target, and
scores outputs with bench/score.py so they compare 1:1 with
bench/noise.py rows.

Usage:
  bench/quadwild.py --qw /tmp/qw [--models armadillo.obj] \
      [--targets 1000,5000] [--json out.json]

`--qw` is the quadwild-bimdf checkout (binaries in build/Build/bin,
configs in config/). Prep uses the Organic config; quantization uses
flow_noalign_lemon (Bi-MDF flow solver, no Gurobi).
"""

import argparse
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import score  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CALIBRATION_ROUNDS = 4
TOLERANCE = 0.08


def run_prep(qw, model_path, work):
    stem = os.path.splitext(os.path.basename(model_path))[0]
    local = os.path.join(work, stem + ".obj")
    shutil.copy(model_path, local)
    t0 = time.time()
    proc = subprocess.run(
        [os.path.join(qw, "build/Build/bin/quadwild"), local, "2",
         "config/prep_config/basic_setup_Organic.txt"],
        cwd=qw, capture_output=True, text=True)
    prep_s = time.time() - t0
    rem = os.path.join(work, stem + "_rem_p0.obj")
    if proc.returncode != 0 or not os.path.exists(rem):
        return None, prep_s
    return rem, prep_s


def run_quantize(qw, rem, work, scale_fact, tag):
    with open(os.path.join(qw, "config/main_config/flow_noalign_lemon.txt")) as f:
        cfg = re.sub(r"(?m)^scaleFact .*$", f"scaleFact {scale_fact:.6g}",
                     f.read())
    cfg_path = os.path.join(work, "main.txt")
    with open(cfg_path, "w") as f:
        f.write(cfg)
    t0 = time.time()
    proc = subprocess.run(
        [os.path.join(qw, "build/Build/bin/quad_from_patches"), rem, tag,
         cfg_path], cwd=qw, capture_output=True, text=True)
    secs = time.time() - t0
    out = rem[:-4] + f"_{tag}_quadrangulation_smooth.obj"
    if proc.returncode != 0 or not os.path.exists(out):
        return None, secs
    return out, secs


def count_quads(path):
    with open(path) as f:
        return sum(1 for line in f
                   if line.startswith("f ") and len(line.split()) == 5)


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--qw", required=True)
    ap.add_argument("--models", default="armadillo.obj,beast.obj")
    ap.add_argument("--targets", default="1000,5000")
    ap.add_argument("--json")
    args = ap.parse_args(argv)
    report = {}
    for model in args.models.split(","):
        path = model if os.path.isabs(model) else os.path.join(
            ROOT, "bench", "models", model)
        source = score.load_obj(path)
        with tempfile.TemporaryDirectory(prefix="qw_") as work:
            rem, prep_s = run_prep(args.qw, path, work)
            if rem is None:
                print(f"{model}: prep FAILED ({prep_s:.1f}s)", flush=True)
                continue
            # scaleFact 1 probe; quad count scales ~ 1/scaleFact^2.
            probe, _ = run_quantize(args.qw, rem, work, 1.0, "1")
            base = count_quads(probe) if probe else 0
            for t in args.targets.split(","):
                target = int(t)
                s = math.sqrt(base / target) if base else 1.0
                best = None
                for rnd in range(CALIBRATION_ROUNDS):
                    out, secs = run_quantize(args.qw, rem, work, s,
                                             str(10 * target + rnd + 2))
                    # (the tag must be numeric: it names the output file)
                    if out is None:
                        break
                    q = count_quads(out)
                    best = (out, secs, s, q)
                    if abs(q / target - 1.0) <= TOLERANCE or q == 0:
                        break
                    s *= math.sqrt(q / target)
                if best is None:
                    print(f"{model}@{target}: quantize FAILED", flush=True)
                    continue
                out, secs, s, q = best
                m = score.score(path, out, target, source=source)
                m.update(prep_s=prep_s, quantize_s=secs, scale_fact=s)
                key = f"{os.path.basename(path)}@{target}"
                report[key] = m
                print(key, " ".join(
                    f"{k}={m[k]:.3g}" if isinstance(m[k], float) else
                    f"{k}={m[k]}" for k in
                    ("quads", "non_quads", "irr_pct", "angdev_mean",
                     "dist_mean", "dist_max", "prep_s", "quantize_s")))
                sys.stdout.flush()
    if args.json:
        with open(args.json, "w") as f:
            json.dump(report, f, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
