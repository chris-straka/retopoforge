#!/usr/bin/env python3
"""retopoforge profiler (stdlib only).

Profiles the headless `retopo` CLI on one production-size mesh and prints
wall time, peak RSS, output counts, and the engine's per-phase breakdown
(parsed from the phase report on stderr).

Usage:
    bench/profile.py [--model xyzrgb_dragon.obj] [--target-quads 50000]
                     [--binary rust/target/release/retopo] [--repeat 3] [--json out.json]

Threading: the engine exposes no thread-count knob (no CLI flag, no TBB
global_control; TBB_NUM_THREADS has no effect), so there is no thread
sweep. The phase report's "Cores kept busy" line is the scaling evidence;
see docs/perf.md.
"""
import json
import os
import re
import subprocess
import sys
import tempfile
import time

try:
    import resource
except ImportError:
    resource = None

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODELS_DIR = os.path.join(ROOT, "bench", "models")
DEFAULT_BINARY = os.path.join(ROOT, "rust", "target", "release", "retopo")

PHASE_RE = re.compile(r"^(.*?):\s+([0-9]+(?:\.[0-9]+)?)\s+ms$")
CORES_RE = re.compile(r"Cores kept busy across the parallel phase:\s+([0-9.]+)")
TIMEOUT_SECONDS = 1200


def peak_rss_mb():
    """Peak RSS of the largest reaped child, in MiB. None when unavailable."""
    if resource is None:
        return None
    maxrss = resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss
    if maxrss <= 0:
        return None
    # macOS reports bytes, Linux reports KiB.
    if sys.platform == "darwin":
        return round(maxrss / 1024 / 1024, 1)
    return round(maxrss / 1024, 1)


def count_output(path):
    """Return (verts, quads, non_quads) for a remeshed OBJ."""
    verts = quads = non_quads = 0
    with open(path, "r", errors="replace") as f:
        for line in f:
            if line.startswith("v "):
                verts += 1
            elif line.startswith("f "):
                if len(line.split()) - 1 == 4:
                    quads += 1
                else:
                    non_quads += 1
    return verts, quads, non_quads


def parse_phase_report(stderr_text):
    """Parse phase lines ("  Name: 123.4 ms") out of CLI stderr.

    The CLI prints the phase table on stderr under --verbose only;
    dedupe by name, last occurrence wins.
    Returns (phases, cores_busy, islands_line, simplifier_line).
    """
    phases = {}
    order = []
    cores_busy = None
    islands_line = None
    simplifier_line = None
    for raw in stderr_text.splitlines():
        line = raw.strip()
        if line.startswith("Islands:"):
            islands_line = line
        elif line.startswith("Mesh simplifier:"):
            simplifier_line = line
        m = CORES_RE.search(line)
        if m:
            cores_busy = float(m.group(1))
            continue
        m = PHASE_RE.match(line)
        if m:
            name = m.group(1).strip()
            if name not in phases:
                order.append(name)
            phases[name] = float(m.group(2))
    return [(n, phases[n]) for n in order], cores_busy, islands_line, simplifier_line


def run_once(binary, model, target_quads):
    input_path = os.path.join(MODELS_DIR, model)
    with tempfile.TemporaryDirectory(prefix="retopo-profile-") as tmp:
        output_path = os.path.join(tmp, "out.obj")
        cmd = [binary, "--input", input_path, "--output", output_path,
               "--target-quads", str(target_quads), "--verbose"]
        start = time.monotonic()
        proc = subprocess.run(cmd, capture_output=True, text=True,
                              timeout=TIMEOUT_SECONDS)
        wall = time.monotonic() - start
        result = {
            "exit_code": proc.returncode,
            "wall_seconds": round(wall, 3),
            "peak_rss_mb": peak_rss_mb(),
        }
        if proc.returncode != 0:
            result["error"] = (proc.stderr.strip().splitlines()[-3:] or ["no output"])
            return result
        verts, quads, non_quads = count_output(output_path)
        phases, cores, islands, simplifier = parse_phase_report(proc.stderr)
        result.update({
            "verts": verts,
            "quads": quads,
            "non_quads": non_quads,
            "phases": [{"name": n, "ms": ms} for n, ms in phases],
            "cores_busy": cores,
            "islands_line": islands,
            "simplifier_line": simplifier,
        })
        return result


def median(values):
    ordered = sorted(values)
    return ordered[len(ordered) // 2]


def main(argv):
    binary = DEFAULT_BINARY
    model = "xyzrgb_dragon.obj"
    target_quads = 50000
    repeat = 3
    json_path = None
    i = 0
    while i < len(argv):
        if argv[i] == "--binary" and i + 1 < len(argv):
            binary, i = argv[i + 1], i + 2
        elif argv[i] == "--model" and i + 1 < len(argv):
            model, i = argv[i + 1], i + 2
        elif argv[i] == "--target-quads" and i + 1 < len(argv):
            target_quads, i = int(argv[i + 1]), i + 2
        elif argv[i] == "--repeat" and i + 1 < len(argv):
            repeat, i = int(argv[i + 1]), i + 2
        elif argv[i] == "--json" and i + 1 < len(argv):
            json_path, i = argv[i + 1], i + 2
        else:
            print(f"unknown arg: {argv[i]}", file=sys.stderr)
            return 2
    if not os.path.exists(binary):
        print(f"binary not found: {binary} (build with: cargo build --release -p retopo)", file=sys.stderr)
        return 2
    if not os.path.exists(os.path.join(MODELS_DIR, model)):
        print(f"model not found: {model} (run bench/fetch_models.sh)", file=sys.stderr)
        return 2

    runs = []
    for n in range(repeat):
        print(f"run {n + 1}/{repeat} {model} target-quads={target_quads}...",
              flush=True)
        runs.append(run_once(binary, model, target_quads))
        r = runs[-1]
        if r["exit_code"] != 0:
            print(f"run {n + 1} failed: {r.get('error')}", file=sys.stderr)
            return 1

    walls = [r["wall_seconds"] for r in runs]
    rss = [r["peak_rss_mb"] for r in runs if r["peak_rss_mb"] is not None]
    rep = runs[0]  # phase breakdown shape is stable; show run 1, medians below
    print(f"\nmodel: {model}  target-quads: {target_quads}  "
          f"output: {rep['quads']} quads + {rep['non_quads']} non-quads, "
          f"{rep['verts']} verts")
    # ru_maxrss is a cumulative max over reaped children, so take the max:
    # it is the true peak reached during profiling.
    print(f"wall: {median(walls):.1f}s (median of {len(runs)})  "
          f"peak RSS: {max(rss) if rss else 'n/a'} MiB")
    if rep["islands_line"]:
        print(rep["islands_line"])
    if rep["simplifier_line"]:
        print(rep["simplifier_line"])
    if rep["cores_busy"] is not None:
        print(f"cores kept busy: {rep['cores_busy']:.2f} "
              f"(of {os.cpu_count()} logical CPUs)")
    print(f"\n{'phase':<48}{'ms':>12}")
    for p in rep["phases"]:
        print(f"{p['name']:<48}{p['ms']:>12.1f}")

    # Top bottlenecks: leaf stages only (skip wall-clock rollups and Total).
    rollups = {"Total", "Isotropic phase wall clock",
               "Parameterize phase wall clock", "Parallel phase wall clock",
               "Parameterize (accumulated)", "Quad extract (accumulated)",
               "Isotropic remesh (accumulated)",
               "Adaptive target length field (accumulated)"}
    leaves = [p for p in rep["phases"] if p["name"] not in rollups]
    leaves.sort(key=lambda p: p["ms"], reverse=True)
    print("\ntop bottlenecks (leaf stages):")
    for p in leaves[:5]:
        print(f"  {p['ms']:>10.1f} ms  {p['name']}")

    if json_path:
        with open(json_path, "w") as f:
            json.dump({"binary": binary, "model": model,
                       "target_quads": target_quads, "runs": runs}, f, indent=2)
        print(f"\nsaved {json_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
