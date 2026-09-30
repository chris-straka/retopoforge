# retopoforge

retopoforge is a fork of [AutoRemesher](https://github.com/huxingyi/autoremesher)
(MIT, by Jeremy HU) restructured around a **headless engine**: a C++ core
library, a `retopo` CLI, a Blender extension, and a benchmark/regression
harness. There is no desktop app — Blender is the UI. Upstream is kept as
the `upstream` git remote as a read-only reference; this fork has
structurally diverged, so upstream engine fixes are ported by hand when
relevant, never git-merged.

## Install (Homebrew)

```bash
brew tap chris-straka/retopoforge https://github.com/chris-straka/retopoforge
brew install chris-straka/retopoforge/retopoforge
```

This taps the repo itself (the formula lives in `Formula/`) and builds
the `retopo` CLI from source (cmake, ninja, llvm, tbb are pulled in
automatically), linking it onto your PATH as `retopo`. `brew update`
keeps the tap current; `brew upgrade retopoforge` rebuilds on updates.

## Build from source (one CMake build for everything)

```bash
# macOS prerequisites (macOS-only product; Linux exists in CI for
# compile + test validation, Windows parked)
brew install cmake tbb llvm ninja

# macOS builds use Homebrew LLVM (AppleClang lacks C++ named modules);
# Ninja is required (the only macOS generator supporting C++ modules)
cmake -S . -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/macos-llvm.cmake \
  -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

This builds the `retopo` CLI (`build/cli/retopo`) and the test suite.

## CLI usage

```bash
./build/cli/retopo --help
./build/cli/retopo --input bench/models/armadillo.obj \
    --output /tmp/armadillo-remeshed.obj --report /tmp/armadillo-report.txt \
    --target-quads 5000
```

Flags: `--input`/`-i` and `--output`/`-o` (required), `--report`,
`--target-quads` (default 50000), `--edge-scaling` (1.0–4.0),
`--sharp-edge` (30–180°), `--smooth-normal` (0–180°),
`--adaptivity`/`--anisotropy` (0–1), `--model-type organic|hardsurface`,
`--symmetry off|auto|x|y|z` (default `off`), `--guides <file>`,
`--density <file>`, `--features <file>`, `--uvs on|off` (default `off`),
`--lods <q0,q1,...>`,
`--quiet`, `--help`/`-h`, `--version`/`-v`. Non-indexed triangle soup is
welded on load; `--quiet` silences progress output (warnings, errors, and
the report still print). `--symmetry auto` detects the dominant mirror
plane (x/y/z pin it) and falls back to unconstrained output when the
input scores below threshold. `--guides` takes a polyline file (one
`x y z` point per line, blank lines separate polylines, `#` comments)
and bends quad edge flow along the curves; single-file and `--lods`
runs only. `--density` takes a mask file (one multiplier per input
vertex, `1.0` = unchanged, clamped to 0.25–4.0) for local detail
control; strong localized refinement saturates (~2.3x realized for
4x asks), mild masks realize nearly fully; single-file and `--lods`
runs only. `--features` takes a polyline file in the same format as
`--guides` and marks crisp hard-surface edges (wins ties over guides);
single-file and `--lods` runs only. `--uvs on` emits remeshed UVs from
the internal
parameterization (`vt` + `v/vt` corners for OBJ, `TEXCOORD_0` for GLB),
normalized 0..1 per island. `--input`/`--output` accept `.glb` as well
as `.obj` (positions + faces; batch dirs and `--lods` chains keep each
file's extension). The input model comes from
`bench/fetch_models.sh` (see Benchmarks).

Multi-output: `--lods 10000,5000,2000` emits a full LOD chain in one run
(`<stem>_lod0.obj`, `<stem>_lod1.obj`, ... next to `--output`, overriding
`--target-quads`); pointing `--input` at a directory remeshes every
`.obj` and `.glb` in it (non-recursive) with `--output` as the
directory:

```bash
./build/cli/retopo --input bench/models/armadillo.obj --output /tmp/hero.obj --lods 10000,5000,2000
./build/cli/retopo --input assets/ --output assets-retopo/
```

## Tests

```bash
ctest --test-dir build --output-on-failure
```

Fourteen unit tests cover engine components (vectors, mesh container,
solvers, OBJ reader, welding, symmetry, guide curves, density, sharp
constraints, input validation); ten CLI tests drive the built binary end
to end (round-trip, `--lods`/batch multi-output, `--quiet`, GLB
input/output, symmetry, guides, UVs, density, nasty-corpus, sharp
features). The CLI tests remesh `bench/models/` fixtures (except the two
hermetic robustness suites), so fetch the models first (see Benchmarks).

## Benchmarks

```bash
bench/fetch_models.sh          # one-time download of test models (gitignored)
bench/run.py                   # run suite, validate meshes, save results JSON
bench/run.py --check bench/baseline.json   # fail on regression vs baseline
bench/profile.py               # profile one production-size mesh (docs/perf.md)
```

The suite runs `build/cli/retopo` over five models × two presets
(`--target-quads` 1000/5000), validates every output mesh, and records
timings plus quad counts. A run regresses when it exits non-zero, its
mesh fails validation, its quad count drops >5% below baseline, or its
non-quad share rises >2pp; wall time is recorded but never gates.
`bench/profile.py` profiles a single mesh instead: wall time, peak RSS,
and the engine's per-phase breakdown — see
[docs/perf.md](docs/perf.md). The shipped LOD rung strategy is
[docs/lod-strategy.md](docs/lod-strategy.md).

## Blender extension

Quad remeshing inside Blender 4.2+, driven by the `retopo` CLI: select
mesh objects, open the *RetopoForge* tab in the 3D Viewport sidebar
(N-panel), tune the parameters, hit **Remesh Selected**. Each object is
exported to a temp OBJ under the identity transform, the CLI remeshes
it, and the result lands back on the original object in a single undo
step; temp files are removed afterwards. New topology cannot carry UVs
or vertex colors — the panel says so. The panel also offers **Generate
LODs** (one `--lods` chain per selected object from the comma-separated
rung field, each rung imported as a `<object>_lod<N>` sibling), per-object
settings recall (each remesh saves its parameters; the next run on the
same object restores them), and a Bake Assist box (high-to-low texture
bake from the active object to the selected one).

Install:

1. Build the CLI (see Build above).
2. Zip the `blender/retopoforge/` directory (the folder containing
   `blender_manifest.toml`):

   ```bash
   (cd blender && zip -r /tmp/retopoforge-addon.zip retopoforge -x '*/__pycache__/*')
   ```

3. In Blender: *Edit → Preferences → Extensions → Install from Disk*,
   pick the zip, enable *RetopoForge*.
4. If the `retopo` binary is not on `PATH`, set its location in the
   add-on preferences (the *Retopo CLI* path — the panel header shows
   whether the binary was found). The panel's **Reload Scripts** button
   picks up extension updates without restarting Blender.

Headless test (uses the Blender app binary directly — the
`~/.local/bin/blender` shim has a broken Python environment):

```bash
/Applications/Blender.app/Contents/MacOS/Blender --background \
    --factory-startup --python blender/tests/test_headless.py
```

The test registers the extension, remeshes a transformed subdivided
cube, and asserts the mesh was replaced, is mostly quads, keeps the
object transform bit-exact, records a report, and leaves no temp
objects. It skips (exit 0) when no `retopo` binary is available.

The extension is GPL-3.0-or-later, as Blender requires; the C++ engine
stays MIT — the extension talks to it only as a subprocess over OBJ
files. See [docs/architecture.md](docs/architecture.md) and
[blender/README.md](blender/README.md).

## Layout

- `core/` — headless engine, built as the `retopo_core` static library:
  18 C++23 named modules `retopo.core.*` (interface in `core/*.cppm`,
  implementation in `core/*.cpp`, including the `symmetry`
  mirror-constraint, `guides` guide-curve, and `density` local-density
  modules), plus the two components not yet
  converted: `core/autoremesher.h/.cpp` (pipeline orchestrator) and
  `core/objreader.h/.cpp` (OBJ loader), reached via the
  `<AutoRemesher/...>` forwarders in `core/include/`.
- `cli/` — `retopo` CLI (`cli/main.cpp` + `cli/glb.*`).
- `Formula/` — Homebrew formula for the CLI.
- `blender/` — Blender extension (`blender/retopoforge/`) driving the
  CLI over a temp-OBJ round-trip, with a headless test in
  `blender/tests/`.
- `bench/` — harness (`bench/run.py`, models/results gitignored,
  `bench/baseline.json` committed) plus the `bench/profile.py` profiler.
- `docs/` — architecture, engine-vs-Exoside gap, perf profile, and LOD
  strategy notes.
- `tests/` — unit tests plus the CLI round-trip test.
- `thirdparty/` — vendored Eigen, isotropicremesher, meshoptimizer
  (TBB comes from the system install).

See [docs/architecture.md](docs/architecture.md) for the module graph
and the engine/CLI/addon split.

## Direction

1. Headless engine + CLI + benchmarks (this fork's foundation, done)
2. Blender addon driving the CLI (done, extension v0.2.0)
3. C++23 modules + idiom modernization, gated by the benchmark suite
4. Incremental engine improvements toward Exoside parity (see
   [docs/exoside-gap.md](docs/exoside-gap.md))

## Attribution

Retopoforge is a fork of [AutoRemesher](https://github.com/huxingyi/autoremesher)
by Jeremy HU (Dust3D Project) and contributors, used under the MIT
license — see [LICENSE](LICENSE). The core remeshing engine is principally
Jeremy's work; this fork restructures it around a headless library
and adds the `retopo` CLI, the Blender extension, and the benchmark
harness. (The upstream Qt desktop shell was removed; Blender is the UI.)

- Upstream repository: <https://github.com/huxingyi/autoremesher> (tracked as
  the `upstream` git remote)
- Support Jeremy's work:
  [donate via PayPal](https://www.paypal.com/cgi-bin/webscr?cmd=_donations&business=GHALWLWXYGCU6&item_name=Support+me+coding+in+my+spare+time&currency_code=AUD&source=url)
- Contributors: [AUTHORS](AUTHORS) and [CONTRIBUTORS](CONTRIBUTORS);
  third-party licenses:
  [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) and
  [ACKNOWLEDGEMENTS.html](ACKNOWLEDGEMENTS.html)
