#!/usr/bin/env python3
"""Free-baseline comparison harness: retopo vs Blender voxel vs QuadriFlow.

Pits the headless `retopo` CLI against two free baselines:

  (a) Blender's native voxel remesher, run headless
      (<blender> --background --factory-startup --python bench/blender_voxel.py)
      -- the Blender GUI is never launched.
  (b) the standalone QuadriFlow binary when available (best-effort:
      $QUADRIFLOW_BINARY, PATH, or a /tmp build; SKIP with a printed
      reason otherwise -- the harness still exits 0 on retopo+voxel).

For each model x quad-target case every tool's OBJ output is parsed
directly for: quads, non-quads, verts, wall time, boundary-edge count,
non-manifold-edge count. A markdown table goes to stdout.

Usage:
    bench/fetch_models.sh          # one-time model download (gitignored)
    python3 bench/compare.py       # defaults: armadillo+fandisk x 1000,5000
    python3 bench/compare.py --targets 1000,5000,10000 --models fandisk.obj

Exit code is 0 when every retopo and Blender-voxel case succeeds;
QuadriFlow skip/failure is non-fatal and recorded in the notes column.
"""

import os
import shutil
import subprocess
import sys
import tempfile
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODELS_DIR = os.path.join(ROOT, "bench", "models")
BLENDER_HELPER = os.path.join(ROOT, "bench", "blender_voxel.py")

DEFAULT_MODELS = ["armadillo.obj", "fandisk.obj"]
DEFAULT_TARGETS = [1000, 5000]
DEFAULT_BLENDER = "/Applications/Blender.app/Contents/MacOS/Blender"
QUADRIFLOW_REPO = "https://github.com/hjwdzh/QuadriFlow"
TIMEOUT_SECONDS = 600
QUADRIFLOW_BUILD_TIMEOUT = 1500


def resolve_retopo(arg):
    if arg:
        return arg
    return os.environ.get(
        "RETOPO_BINARY",
        os.path.join(ROOT, "rust", "target", "release", "retopo"))


def resolve_blender(arg):
    if arg:
        return arg
    return os.environ.get("BLENDER_BINARY", DEFAULT_BLENDER)


def quadriflow_build_dir():
    return os.environ.get("QUADRIFLOW_BUILD_DIR", "/tmp/quadriflow-build")


def try_build_quadriflow(build_dir):
    """Best-effort clone+build of QuadriFlow. Returns (binary or None, note)."""
    for dep in ("git", "cmake"):
        if shutil.which(dep) is None:
            return None, f"SKIP: cannot build quadriflow ({dep} not on PATH)"
    if not os.path.isdir(build_dir):
        print("quadriflow not found; attempting /tmp build...", flush=True)
        proc = subprocess.run(
            ["git", "clone", "--depth", "1", QUADRIFLOW_REPO, build_dir],
            capture_output=True, text=True, timeout=300)
        if proc.returncode != 0:
            tail = proc.stderr.strip().splitlines()[-1:]
            return None, f"SKIP: quadriflow clone failed ({'; '.join(tail)})"
    src_build = os.path.join(build_dir, "build")
    binary = os.path.join(src_build, "quadriflow")
    if os.path.exists(binary):
        return binary, f"built earlier at {binary}"
    # Fresh configure each attempt: a failed cmake leaves a stale cache.
    shutil.rmtree(src_build, ignore_errors=True)
    os.makedirs(src_build, exist_ok=True)
    cfg = subprocess.run(
        ["cmake", "..", "-DCMAKE_BUILD_TYPE=release",
         "-DCMAKE_POLICY_VERSION_MINIMUM=3.5"],
        cwd=src_build, capture_output=True, text=True,
        timeout=QUADRIFLOW_BUILD_TIMEOUT)
    if cfg.returncode != 0:
        tail = (cfg.stderr.strip() or cfg.stdout.strip()).splitlines()[-2:]
        return None, f"SKIP: quadriflow cmake failed ({'; '.join(tail)})"
    jobs = str(os.cpu_count() or 4)
    make = subprocess.run(
        ["cmake", "--build", ".", "--config", "release", "-j", jobs],
        cwd=src_build, capture_output=True, text=True,
        timeout=QUADRIFLOW_BUILD_TIMEOUT)
    if make.returncode != 0 or not os.path.exists(binary):
        return None, "SKIP: quadriflow build failed (see build log)"
    return binary, f"built at {binary}"


def resolve_quadriflow(arg, no_build):
    """Returns (binary path or None, note string). Never raises."""
    if arg and os.path.exists(arg):
        return arg, ""
    if arg:
        return None, f"SKIP: --quadriflow {arg} not found"
    env = os.environ.get("QUADRIFLOW_BINARY")
    if env:
        if os.path.exists(env):
            return env, ""
        return None, f"SKIP: $QUADRIFLOW_BINARY={env} not found"
    found = shutil.which("quadriflow")
    if found:
        return found, ""
    binary = os.path.join(quadriflow_build_dir(), "build", "quadriflow")
    if os.path.exists(binary):
        return binary, ""
    if no_build:
        return None, "SKIP: quadriflow not on PATH (--no-quadriflow-build)"
    try:
        return try_build_quadriflow(quadriflow_build_dir())
    except subprocess.TimeoutExpired:
        return None, "SKIP: quadriflow build timed out"
    except Exception as e:  # noqa: BLE001 -- best-effort lane, never fatal
        return None, f"SKIP: quadriflow build error ({e})"


def measure_obj(path):
    """Parse an OBJ directly. Returns dict of mesh metrics."""
    verts = 0
    quads = 0
    non_quads = 0
    faces = []
    with open(path, "r", errors="replace") as f:
        for line in f:
            if line.startswith("v "):
                verts += 1
            elif line.startswith("f "):
                idx = []
                for p in line.split()[1:]:
                    try:
                        idx.append(int(p.split("/")[0]))
                    except ValueError:
                        idx = []
                        break
                if len(idx) < 3:
                    continue
                faces.append(idx)
                if len(idx) == 4:
                    quads += 1
                else:
                    non_quads += 1
    # Resolve relative (negative) indices, then count edge uses.
    edge_uses = {}
    for face in faces:
        norm = [i if i > 0 else verts + 1 + i for i in face]
        n = len(norm)
        for k in range(n):
            e = (norm[k], norm[(k + 1) % n])
            e = (e[0], e[1]) if e[0] < e[1] else (e[1], e[0])
            edge_uses[e] = edge_uses.get(e, 0) + 1
    boundary = sum(1 for c in edge_uses.values() if c == 1)
    non_manifold = sum(1 for c in edge_uses.values() if c >= 3)
    return {
        "quads": quads,
        "non_quads": non_quads,
        "verts": verts,
        "boundary_edges": boundary,
        "nonmanifold_edges": non_manifold,
    }


def run_tool(cmd, output_path, timeout):
    """Run one tool command. Returns (wall_seconds, error or None)."""
    start = time.monotonic()
    try:
        proc = subprocess.run(
            cmd, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return timeout, f"timeout after {timeout}s: {' '.join(cmd[:2])}"
    wall = time.monotonic() - start
    if proc.returncode != 0:
        tail = (proc.stderr.strip() or proc.stdout.strip()).splitlines()[-2:]
        return wall, f"exit {proc.returncode}: {'; '.join(tail)}"
    if not os.path.exists(output_path):
        return wall, "no output file produced"
    return wall, None


def run_case(tool, cmd, target_label, timeout):
    with tempfile.TemporaryDirectory(prefix="compare-") as tmp:
        output_path = os.path.join(tmp, "out.obj")
        full_cmd = [c.replace("{OUTPUT}", output_path) for c in cmd]
        wall, error = run_tool(full_cmd, output_path, timeout)
        row = {"tool": tool, "wall_seconds": round(wall, 3)}
        if error is None:
            try:
                row.update(measure_obj(output_path))
            except OSError as e:
                error = f"unreadable output: {e}"
            row["notes"] = ""
        if error is not None:
            row.update({"quads": 0, "non_quads": 0, "verts": 0,
                        "boundary_edges": 0, "nonmanifold_edges": 0,
                        "notes": error})
        row["target_label"] = target_label
        return row


def print_table(rows):
    print("| model | target | tool | quads | non-quads | verts | "
          "wall s | boundary | non-manif | notes |")
    print("|---|---|---|---:|---:|---:|---:|---:|---:|---|")
    for r in rows:
        print(f"| {r['model']} | {r['target']} | {r['tool']} | "
              f"{r['quads']} | {r['non_quads']} | {r['verts']} | "
              f"{r['wall_seconds']:.1f} | {r['boundary_edges']} | "
              f"{r['nonmanifold_edges']} | {r['notes']} |")


def parse_args(argv):
    opts = {"models": list(DEFAULT_MODELS),
            "targets": list(DEFAULT_TARGETS),
            "binary": None, "blender": None, "quadriflow": None,
            "no_quadriflow_build": False, "timeout": TIMEOUT_SECONDS}
    i = 0
    while i < len(argv):
        a = argv[i]
        if a == "--help":
            print(__doc__)
            sys.exit(0)
        elif a == "--models" and i + 1 < len(argv):
            opts["models"] = argv[i + 1].split(",")
            i += 2
        elif a == "--targets" and i + 1 < len(argv):
            opts["targets"] = [int(t) for t in argv[i + 1].split(",")]
            i += 2
        elif a == "--binary" and i + 1 < len(argv):
            opts["binary"] = argv[i + 1]
            i += 2
        elif a == "--blender" and i + 1 < len(argv):
            opts["blender"] = argv[i + 1]
            i += 2
        elif a == "--quadriflow" and i + 1 < len(argv):
            opts["quadriflow"] = argv[i + 1]
            i += 2
        elif a == "--no-quadriflow-build":
            opts["no_quadriflow_build"] = True
            i += 1
        elif a == "--timeout" and i + 1 < len(argv):
            opts["timeout"] = int(argv[i + 1])
            i += 2
        else:
            print(f"unknown arg: {a} (see --help)", file=sys.stderr)
            sys.exit(2)
    return opts


def main(argv):
    opts = parse_args(argv)
    retopo = resolve_retopo(opts["binary"])
    if not os.path.exists(retopo):
        print(f"binary not found: {retopo} "
              f"(build first, or set $RETOPO_BINARY)", file=sys.stderr)
        return 2
    blender = resolve_blender(opts["blender"])
    if not os.path.exists(blender):
        print(f"blender not found: {blender} "
              f"(set $BLENDER_BINARY)", file=sys.stderr)
        return 2
    if not os.path.exists(BLENDER_HELPER):
        print(f"helper missing: {BLENDER_HELPER}", file=sys.stderr)
        return 2
    models = [m for m in opts["models"]
              if os.path.exists(os.path.join(MODELS_DIR, m))]
    missing = [m for m in opts["models"] if m not in models]
    for m in missing:
        print(f"skip (missing, run bench/fetch_models.sh): {m}",
              file=sys.stderr)
    if not models:
        print("no models found, aborting", file=sys.stderr)
        return 2

    quad_binary, quad_note = resolve_quadriflow(
        opts["quadriflow"], opts["no_quadriflow_build"])
    if quad_note:
        print(f"quadriflow: {quad_note}", flush=True)

    rows = []
    failed = 0
    for model in models:
        input_path = os.path.join(MODELS_DIR, model)
        for target in opts["targets"]:
            lanes = [
                ("retopo",
                 [retopo, "--input", input_path, "--output", "{OUTPUT}",
                  "--target-quads", str(target)]),
                ("blender-voxel",
                 [blender, "--background", "--factory-startup",
                  "--python", BLENDER_HELPER, "--",
                  input_path, "{OUTPUT}", str(target)]),
            ]
            if quad_binary:
                lanes.append(
                    ("quadriflow",
                     [quad_binary, "-i", input_path, "-o", "{OUTPUT}",
                      "-f", str(target)]))
            for tool, cmd in lanes:
                print(f"run {model} target={target} {tool}...",
                      file=sys.stderr, flush=True)
                row = run_case(tool, cmd, str(target), opts["timeout"])
                row["model"] = model
                row["target"] = target
                rows.append(row)
                if row["notes"] and tool in ("retopo", "blender-voxel"):
                    failed += 1

    print_table(rows)
    if not quad_binary:
        print(f"\nquadriflow: {quad_note}")
    elif quad_note:
        print(f"\nquadriflow: {quad_note}")
    if failed:
        print(f"\n{failed} retopo/blender-voxel case(s) failed",
              file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
