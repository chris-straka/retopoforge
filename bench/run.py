#!/usr/bin/env python3
"""retopoforge benchmark / regression harness (stdlib only).

Runs the headless `retopo` CLI over test models x parameter presets,
validates every output mesh, and records metrics as JSON.

Usage:
    bench/fetch_models.sh            # one-time model download (gitignored)
    bench/run.py                     # run all, print table, save results JSON
    bench/run.py --check bench/baseline.json
                                     # run all, fail on regression vs baseline
    bench/run.py --check bench/baseline-linux.json
                                     # same, against the Linux counts
    bench/run.py --no-deform         # skip deformation scoring + gate
    bench/run.py --deform-baseline bench/deform_baseline.json
                                     # use this deform baseline (default:
                                     # deform_<check-basename> next to --check)

Baselines are per-platform: sparse solves differ between Accelerate
(macOS) and libstdc++ (Linux), so counts disagree beyond the gates.
macOS CI checks baseline.json, Linux CI checks baseline-linux.json;
regenerate each on its own platform, never mix. The deform baseline
follows the same rule (bench/deform_baseline.json next to
bench/baseline.json, -linux variant next to baseline-linux.json).

A run regresses when: it exits non-zero, its output mesh fails validation,
its quad count drops >5% below baseline, its non-quad share rises >2pp,
or its deformation scores regress (see below).
Wall time is recorded but never gates (too noisy across machines).

Deformation gate (bench/deform.py, docs/deformation-test.md): every
output mesh is rigged with the fixed 2-bone hinge probe, posed, and
scored for joint distortion (p95 stretch, volume loss, flips). In
--check mode each case's scores must stay inside the deform baseline's
seed spread; flips must not exceed the spread max (0 for feasible
bends). The suite also remeshes the procedural `tube` limb (always run;
its counts are recorded but ungated since baseline.json predates it).
When the deform baseline file is missing (e.g. Linux before its
platform baseline is generated), the check warns loudly and the deform
scores go uncompared: spread gating needs the platform's spread (see
docs/deformation-test.md for the one command that closes the gap).
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import deform  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODELS_DIR = os.path.join(ROOT, "bench", "models")
RESULTS_DIR = os.path.join(ROOT, "bench", "results")
DEFAULT_BINARY = os.path.join(ROOT, "rust", "target", "release", "retopo")

MODELS = ["armadillo.obj", "beast.obj", "nefertiti.obj", "fandisk.obj",
          "xyzrgb_dragon.obj"]
PRESETS = {
    "tiny": ["--target-quads", "1000"],
    "small": ["--target-quads", "5000"],
}
TIMEOUT_SECONDS = 600
QUAD_DROP_TOL = 0.05
NONQUAD_RISE_TOL = 0.02


def count_obj(path):
    """Return (verts, faces) for an input OBJ (positions + face lines)."""
    verts = faces = 0
    with open(path, "r", errors="replace") as f:
        for line in f:
            if line.startswith("v "):
                verts += 1
            elif line.startswith("f "):
                faces += 1
    return verts, faces


def validate_obj(path):
    """Validate a remeshed OBJ. Returns dict of metrics + error list."""
    verts = 0
    quads = 0
    non_quads = 0
    errors = []
    with open(path, "r", errors="replace") as f:
        for lineno, line in enumerate(f, 1):
            if line.startswith("v "):
                verts += 1
            elif line.startswith("f "):
                parts = line.split()[1:]
                try:
                    idx = [int(p.split("/")[0]) for p in parts]
                except ValueError:
                    errors.append(f"line {lineno}: unparsable face")
                    continue
                if len(idx) == 4:
                    quads += 1
                else:
                    non_quads += 1
                if any(i < 1 or i > verts for i in idx):
                    errors.append(f"line {lineno}: index out of range")
                if len(set(idx)) != len(idx):
                    errors.append(f"line {lineno}: degenerate face")
    return {
        "verts": verts,
        "quads": quads,
        "non_quads": non_quads,
        "errors": errors[:20],
        "error_count": len(errors),
    }


def parse_report_stdout(text):
    """Extract Quads/Non-quads/Vertices from the CLI's stdout report."""
    out = {}
    for line in text.splitlines():
        line = line.strip()
        for key in ("Quads", "Non-quads", "Vertices"):
            if line.startswith(key + ":"):
                try:
                    out[key] = int(line.split(":")[1].strip())
                except ValueError:
                    pass
    return out


def run_case(binary, model, preset, extra_args, input_path=None,
             poses=None, score_deform=False):
    if input_path is None:
        input_path = os.path.join(MODELS_DIR, model)
    in_verts, in_faces = count_obj(input_path)
    with tempfile.TemporaryDirectory(prefix="retopo-bench-") as tmp:
        output_path = os.path.join(tmp, "out.obj")
        cmd = [binary, "--input", input_path, "--output", output_path] + extra_args
        start = time.monotonic()
        try:
            proc = subprocess.run(
                cmd, capture_output=True, text=True, timeout=TIMEOUT_SECONDS
            )
            wall = time.monotonic() - start
            result = {
                "model": model,
                "preset": preset,
                "args": extra_args,
                "exit_code": proc.returncode,
                "wall_seconds": round(wall, 3),
                "input_verts": in_verts,
                "input_faces": in_faces,
            }
            if proc.returncode == 0 and os.path.exists(output_path):
                result.update(validate_obj(output_path))
                reported = parse_report_stdout(proc.stdout)
                # Cross-check the CLI's self-reported counts against ours.
                for key, ours in (("Quads", result["quads"]),
                                  ("Non-quads", result["non_quads"]),
                                  ("Vertices", result["verts"])):
                    if key in reported and reported[key] != ours:
                        result.setdefault("errors", []).append(
                            f"cli reported {key}={reported[key]}, mesh has {ours}"
                        )
                        result["error_count"] = result.get("error_count", 0) + 1
                result["stderr_tail"] = proc.stderr.strip().splitlines()[-2:]
                if score_deform:
                    # Fail closed: a mesh we cannot score must not pass.
                    try:
                        scored = deform.score_file(output_path, poses)
                    except Exception as e:  # noqa: BLE001 -- recorded, fails case
                        result.setdefault("errors", []).append(
                            f"deform scoring failed: {e}")
                        result["error_count"] = result.get("error_count", 0) + 1
                        scored = {"poses": {}}
                    result["deform"] = scored.get("poses", {})
            else:
                result["verts"] = 0
                result["quads"] = 0
                result["non_quads"] = 0
                result["errors"] = (proc.stderr.strip().splitlines()[-5:] or ["no output"])
                result["error_count"] = len(result["errors"])
            return result
        except subprocess.TimeoutExpired:
            return {
                "model": model,
                "preset": preset,
                "args": extra_args,
                "exit_code": None,
                "wall_seconds": TIMEOUT_SECONDS,
                "input_verts": in_verts,
                "input_faces": in_faces,
                "verts": 0,
                "quads": 0,
                "non_quads": 0,
                "errors": ["timeout"],
                "error_count": 1,
            }


def _target_of(args):
    """Extract the --target-quads value from a preset's arg list."""
    for i, a in enumerate(args):
        if a == "--target-quads" and i + 1 < len(args):
            return args[i + 1]
    return "?"


def check_regressions(results, baseline, deform_baseline=None):
    """Compare results to baseline. Returns list of failure strings.

    deform_baseline is the bench/deform.py baseline ({cases: ...}) or
    None, in which case deform scores go uncompared (warned in main).
    """
    base = {(r["model"], r["preset"]): r for r in baseline["results"]}
    failures = []
    for r in results:
        key = (r["model"], r["preset"])
        where = f"{r['model']}/{r['preset']}"
        if r["exit_code"] != 0:
            failures.append(f"{where}: exit={r['exit_code']} errors={r['errors'][:2]}")
            continue
        if r["error_count"]:
            failures.append(f"{where}: {r['error_count']} mesh errors: {r['errors'][:2]}")
        if "deform" in r and r["deform"] is not None \
                and deform_baseline is not None:
            case = f"{r['model']}@{_target_of(r['args'])}"
            entry = deform_baseline.get("cases", {}).get(case)
            if entry is None:
                failures.append(f"{where}: no deform baseline entry "
                                f"for {case}")
            else:
                failures += deform.check_case(case, r["deform"],
                                              entry["poses"])
        if key not in base:
            continue
        b = base[key]
        if b["quads"] > 0 and r["quads"] < b["quads"] * (1 - QUAD_DROP_TOL):
            failures.append(
                f"{where}: quads {r['quads']} < baseline {b['quads']} (-5% tol)"
            )
        b_total = b["quads"] + b["non_quads"]
        r_total = r["quads"] + r["non_quads"]
        if b_total > 0 and r_total > 0:
            if r["non_quads"] / r_total - b["non_quads"] / b_total > NONQUAD_RISE_TOL:
                failures.append(
                    f"{where}: non-quad share rose "
                    f"{b['non_quads'] / b_total:.1%} -> {r['non_quads'] / r_total:.1%}"
                )
    return failures


def main(argv):
    binary = DEFAULT_BINARY
    check_path = None
    deform_baseline_path = None
    deform_on = True
    i = 0
    while i < len(argv):
        if argv[i] == "--binary" and i + 1 < len(argv):
            binary = argv[i + 1]
            i += 2
        elif argv[i] == "--check" and i + 1 < len(argv):
            check_path = argv[i + 1]
            i += 2
        elif argv[i] == "--deform-baseline" and i + 1 < len(argv):
            deform_baseline_path = argv[i + 1]
            i += 2
        elif argv[i] == "--no-deform":
            deform_on = False
            i += 1
        else:
            print(f"unknown arg: {argv[i]}", file=sys.stderr)
            return 2
    if not os.path.exists(binary):
        print(f"binary not found: {binary} (build with cargo first: cargo build --locked --release -p retopo)", file=sys.stderr)
        return 2
    poses = []
    if deform_on:
        if os.path.exists(deform.DEFAULT_POSES):
            poses = deform.load_poses(deform.DEFAULT_POSES)
        else:
            print(f"deform poses not found: {deform.DEFAULT_POSES} "
                  f"(deform scoring disabled)", file=sys.stderr)
            deform_on = False
    models = [m for m in MODELS if os.path.exists(os.path.join(MODELS_DIR, m))]
    missing = [m for m in MODELS if m not in models]
    for m in missing:
        print(f"skip (missing, run bench/fetch_models.sh): {m}", file=sys.stderr)
    if not models:
        print("no models found, aborting", file=sys.stderr)
        return 2

    tube_tmp = tempfile.mkdtemp(prefix="retopo-bench-tube-")
    tube_input = os.path.join(tube_tmp, "tube.obj")
    deform.write_obj(*deform.generate_tube(), tube_input)
    results = []
    for model in models:
        for preset, args in PRESETS.items():
            print(f"run {model} {preset}...", flush=True)
            results.append(run_case(binary, model, preset, args,
                                    poses=poses, score_deform=deform_on))
    for preset, args in PRESETS.items():
        print(f"run tube {preset}...", flush=True)
        results.append(run_case(binary, "tube", preset, args,
                                input_path=tube_input, poses=poses,
                                score_deform=deform_on))
    shutil.rmtree(tube_tmp, ignore_errors=True)

    os.makedirs(RESULTS_DIR, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    out_path = os.path.join(RESULTS_DIR, f"{stamp}.json")
    with open(out_path, "w") as f:
        json.dump({"binary": binary, "results": results}, f, indent=2)

    print(f"\n{'model':<16}{'preset':<8}{'time':>8}{'quads':>8}{'nonq':>6}{'verts':>8}  status")
    failed = 0
    for r in results:
        ok = r["exit_code"] == 0 and r["error_count"] == 0
        failed += not ok
        print(
            f"{r['model']:<16}{r['preset']:<8}{r['wall_seconds']:>8.1f}"
            f"{r['quads']:>8}{r['non_quads']:>6}{r['verts']:>8}  "
            f"{'OK' if ok else 'FAIL ' + str(r['errors'][:1])}"
        )
    print(f"\nsaved {out_path}")

    if deform_on:
        order = [p["name"] for p in poses]
        print("\ndeform (stretch_p95 / vol% / flips per pose):")
        for r in results:
            d = r.get("deform")
            if not d:
                continue
            cells = "; ".join(
                f"{p}: {d[p]['stretch_p95']}/{d[p]['volume_loss_pct']}/"
                f"{d[p]['flips']}" for p in order if p in d)
            print(f"  {r['model']}/{r['preset']}: {cells}")

    if check_path:
        with open(check_path) as f:
            baseline = json.load(f)
        deform_baseline = None
        if deform_on:
            if deform_baseline_path is None:
                sibling = "deform_" + os.path.basename(check_path)
                deform_baseline_path = os.path.join(
                    os.path.dirname(check_path) or ".", sibling)
            if os.path.exists(deform_baseline_path):
                with open(deform_baseline_path) as f:
                    deform_baseline = json.load(f)
            else:
                print(f"\nWARNING: deform baseline not found: "
                      f"{deform_baseline_path}\n"
                      f"deform scores go uncompared; generate it with:\n"
                      f"  bench/deform.py --write-baseline "
                      f"{deform_baseline_path}")
        failures = check_regressions(results, baseline, deform_baseline)
        if failures:
            print("\nREGRESSIONS vs", check_path)
            for msg in failures:
                print("  -", msg)
            return 1
        print(f"\nno regressions vs {check_path}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
