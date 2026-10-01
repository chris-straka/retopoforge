#!/usr/bin/env python3
"""Matched-count comparison (stdlib only).

Surface distance and irregularity depend on quad count, so engines must be
compared at equal output counts. Reads a bench/quadwild.py JSON report and,
for each case, re-targets `retopo` (secant on --target-quads) until its
output quad count lands within TOLERANCE of the reference count, then
scores both with bench/score.py and prints them side by side.

Usage:
  bench/matched.py --reference bench/results/quadwild-1.json \
      [--binary rust/target/release/retopo] [--json out.json]
"""

import argparse
import json
import os
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import score  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TOLERANCE = 0.05
ROUNDS = 5
COLUMNS = ("quads", "non_quads", "irr_pct", "angdev_mean", "angdev_p95",
           "dist_mean", "dist_max")


def remesh(binary, model, target, out):
    proc = subprocess.run([binary, "--input", model, "--output", out,
                           "--target-quads", str(target), "--quiet"],
                          capture_output=True, text=True)
    if proc.returncode != 0 or not os.path.exists(out):
        return 0
    with open(out) as f:
        return sum(1 for line in f
                   if line.startswith("f ") and len(line.split()) == 5)


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--reference", required=True)
    ap.add_argument("--binary",
                    default=os.path.join(ROOT, "rust/target/release/retopo"))
    ap.add_argument("--json")
    args = ap.parse_args(argv)
    with open(args.reference) as f:
        reference = json.load(f)
    report = {}
    print("| case | engine | target | " + " | ".join(COLUMNS) + " |")
    print("|---|---|---:|" + "---:|" * len(COLUMNS))
    for key, ref in reference.items():
        model_name, _ = key.split("@")
        model = os.path.join(ROOT, "bench", "models", model_name)
        want = ref["quads"]
        source = score.load_obj(model)
        with tempfile.TemporaryDirectory(prefix="matched_") as tmp:
            out = os.path.join(tmp, "out.obj")
            target = want
            got = remesh(args.binary, model, target, out)
            for _ in range(ROUNDS):
                if got and abs(got / want - 1.0) <= TOLERANCE:
                    break
                if not got:
                    break
                target = max(1, round(target * want / got))
                got = remesh(args.binary, model, target, out)
            if not got:
                print(f"| {key} | retopo | {target} | FAILED |")
                continue
            ours = score.score(model, out, None, source=source)
        report[key] = {"retopo_target": target, "retopo": ours,
                       "reference": ref}
        for name, m, t in (("quadwild", ref, "-"), ("retopo", ours, target)):
            cells = " | ".join(
                f"{m[c]:.3g}" if isinstance(m.get(c), float)
                else str(m.get(c, "-")) for c in COLUMNS)
            print(f"| {key} | {name} | {t} | {cells} |", flush=True)
    if args.json:
        with open(args.json, "w") as f:
            json.dump(report, f, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
