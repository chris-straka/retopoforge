# retopoforge

retopoforge is a fork of [AutoRemesher](https://github.com/huxingyi/autoremesher)
(MIT, by Jeremy HU) restructured around a **Qt-free headless engine**: a C++
core library, a `retopo` CLI, a benchmark/regression harness, and (planned) a
Blender addon. Upstream is kept as the `upstream` git remote for merging future
fixes.

## Build (one CMake build for everything)

```bash
# macOS prerequisites (Linux: cmake + TBB + Qt6 from your package manager)
brew install cmake tbb qtbase

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

This builds the `retopo` CLI (`build/cli/retopo`) and the Qt6 desktop app
(`build/app/autoremesher.app` on macOS). For a headless-only build without
Qt installed: `cmake -S . -B build -DRETOPOFORGE_BUILD_QT_APP=OFF`.

## CLI usage

```bash
./build/cli/retopo --input armadillo.obj --output remeshed.obj \
    --report report.txt --target-quads 5000
./build/cli/retopo --help   # all options (same flags as upstream --input mode)
```

## Benchmarks

```bash
bench/fetch_models.sh          # one-time download of test models (gitignored)
bench/run.py                   # run suite, validate meshes, save results JSON
bench/run.py --check bench/baseline.json   # fail on regression vs baseline
```

## Direction

1. Headless engine + CLI + benchmarks (this fork's foundation, done)
2. Blender addon driving the CLI
3. Qt6 desktop app on the unified CMake build (done, was qmake/Qt5 upstream)
4. C++23 modules + idiom modernization, gated by the benchmark suite
5. Incremental engine improvements toward Exoside parity

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
