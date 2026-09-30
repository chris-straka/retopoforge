# retopoforge

retopoforge is a fork of [AutoRemesher](https://github.com/huxingyi/autoremesher)
(MIT, by Jeremy HU) restructured around a **Qt-free headless engine**: a C++
core library, a `retopo` CLI, a Qt6 desktop app, a Blender extension, and a
benchmark/regression harness. Upstream is kept as the `upstream` git remote
as a read-only reference; this fork has structurally diverged, so upstream
engine fixes are ported by hand when relevant, never git-merged.

## Build (one CMake build for everything)

```bash
# macOS prerequisites (Linux/Windows parked for now, macOS-only)
brew install cmake tbb qtbase llvm ninja

# macOS builds use Homebrew LLVM (AppleClang lacks C++ named modules);
# Ninja is required (the only macOS generator supporting C++ modules)
cmake -S . -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/macos-llvm.cmake \
  -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

This builds the `retopo` CLI (`build/cli/retopo`) and the Qt6 desktop app
(`build/app/retopoforge.app` on macOS). For a headless-only build without
Qt installed, add `-DRETOPOFORGE_BUILD_QT_APP=OFF` to the configure line.

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
`--adaptivity`/`--anisotropy` (0–1), `--model-type organic|hardsurface`, `--lods <q0,q1,...>`, `--quiet`,
`--help`/`-h`, `--version`/`-v`. Non-indexed triangle soup is welded on
load; `--quiet` silences progress output (warnings, errors, and the
report still print). The input model comes from
`bench/fetch_models.sh` (see Benchmarks).

Multi-output: `--lods 10000,5000,2000` emits a full LOD chain in one run
(`<stem>_lod0.obj`, `<stem>_lod1.obj`, ... next to `--output`, overriding
`--target-quads`); pointing `--input` at a directory remeshes every `.obj`
in it (non-recursive) with `--output` as the directory:

```bash
./build/cli/retopo --input bench/models/armadillo.obj --output /tmp/hero.obj --lods 10000,5000,2000
./build/cli/retopo --input assets/ --output assets-retopo/
```

`retopo` must stay Qt-free —
this must print `Qt-free: OK`:

```bash
otool -L build/cli/retopo | grep -i qt || echo "Qt-free: OK"
```

## Tests

```bash
ctest --test-dir build --output-on-failure
```

Unit tests cover the converted core modules; `test_cli_roundtrip`
remeshes `bench/models/armadillo.obj`, so fetch the models first (see
Benchmarks).

## Benchmarks

```bash
bench/fetch_models.sh          # one-time download of test models (gitignored)
bench/run.py                   # run suite, validate meshes, save results JSON
bench/run.py --check bench/baseline.json   # fail on regression vs baseline
```

The suite runs `build/cli/retopo` over four models × two presets
(`--target-quads` 1000/5000), validates every output mesh, and records
timings plus quad counts. A run regresses when it exits non-zero, its
mesh fails validation, its quad count drops >5% below baseline, or its
non-quad share rises >2pp; wall time is recorded but never gates.

## Blender extension

Quad remeshing inside Blender 4.2+, driven by the `retopo` CLI: select
mesh objects, open the *RetopoForge* tab in the 3D Viewport sidebar
(N-panel), tune the parameters, hit **Remesh Selected**. Each object is
exported to a temp OBJ under the identity transform, the CLI remeshes
it, and the result lands back on the original object in a single undo
step; temp files are removed afterwards. New topology cannot carry UVs
or vertex colors — the panel says so.

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

- `core/` — Qt-free engine, built as the `retopo_core` static library:
  15 C++23 named modules `retopo.core.*` (interface in `core/*.cppm`,
  implementation in `core/*.cpp`), plus the two components not yet
  converted: `core/autoremesher.h/.cpp` (pipeline orchestrator) and
  `core/objreader.h/.cpp` (OBJ loader), reached via the
  `<AutoRemesher/...>` forwarders in `core/include/`.
- `cli/` — Qt-free `retopo` CLI (`cli/main.cpp`).
- `app/` — Qt6 desktop app: `Q_OBJECT` widgets in headers plus the
  converted plain components as 10 `retopo.app.*` modules (`app/*.cppm`).
- `blender/` — Blender extension (`blender/retopoforge/`) driving the
  CLI over a temp-OBJ round-trip, with a headless test in
  `blender/tests/`.
- `bench/` — harness (`bench/run.py`, models/results gitignored,
  `bench/baseline.json` committed).
- `tests/` — unit tests plus the CLI round-trip test.
- `thirdparty/` — vendored Eigen, isotropicremesher, meshoptimizer
  (TBB comes from the system install).

See [docs/architecture.md](docs/architecture.md) for the module graph
and the engine/CLI/app/addon split.

## Direction

1. Headless engine + CLI + benchmarks (this fork's foundation, done)
2. Blender addon driving the CLI (done, extension v0.1.0)
3. Qt6 desktop app on the unified CMake build (done, was qmake/Qt5 upstream)
4. C++23 modules + idiom modernization, gated by the benchmark suite
5. Incremental engine improvements toward Exoside parity (see
   [docs/exoside-gap.md](docs/exoside-gap.md))

## Attribution

Retopoforge is a fork of [AutoRemesher](https://github.com/huxingyi/autoremesher)
by Jeremy HU (Dust3D Project) and contributors, used under the MIT
license — see [LICENSE](LICENSE). The core remeshing engine is principally
Jeremy's work; this fork restructures it around a Qt-free headless library
and adds the `retopo` CLI and benchmark harness.

- Upstream repository: <https://github.com/huxingyi/autoremesher> (tracked as
  the `upstream` git remote)
- Support Jeremy's work:
  [donate via PayPal](https://www.paypal.com/cgi-bin/webscr?cmd=_donations&business=GHALWLWXYGCU6&item_name=Support+me+coding+in+my+spare+time&currency_code=AUD&source=url)
- Contributors: [AUTHORS](AUTHORS) and [CONTRIBUTORS](CONTRIBUTORS);
  third-party licenses:
  [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) and
  [ACKNOWLEDGEMENTS.html](ACKNOWLEDGEMENTS.html)
