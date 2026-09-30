# retopoforge — agent notes

Fork of huxingyi/autoremesher (MIT): Qt-free headless C++ engine + `retopo`
CLI + benchmark harness + (planned) Blender addon. The `upstream` git remote
tracks the original repo for merging future fixes.

## Standing rules

- Commit and push to `origin/main` on your own after each completed chunk
  of work. Do not wait for the user to approve commits or pushes.
  (Explicit standing authorization from the project owner.)
- Never force-push, rebase, amend published commits, or otherwise rewrite
  published history.
- The Qt desktop app must keep building — verify it or leave its inputs
  untouched whenever shared sources change.
- Never launch GUI binaries without explicit user approval. Offscreen
  smoke tests count as launches: batch them, and prefer exit-code-checked
  `--help` / headless runs over open-ended GUI sessions.

## Build

- Everything: `cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=cmake/macos-llvm.cmake -DCMAKE_BUILD_TYPE=Release && cmake --build build -j`
  produces `build/cli/retopo` and the Qt6 app (`build/app/autoremesher[.app]`).
  Needs TBB + Qt6 + LLVM; macOS: `brew install cmake tbb qtbase llvm`.
  (AppleClang lacks named-modules support, so all macOS builds use LLVM;
  required once the first .cppm file lands.)
- Headless only (no Qt): add `-DRETOPOFORGE_BUILD_QT_APP=OFF` to configure.
- Qt app smoke test (ask first): `QT_QPA_PLATFORM=offscreen` + `--help`
  (must exit 0), plus a headless `--input` remesh compared against the
  `bench/baseline.json` counts for the same model/preset.

## Checks

- `bench/run.py --check bench/baseline.json` must pass after engine or CLI
  changes (fetch models once with `bench/fetch_models.sh`).
- `retopo` must stay Qt-free: `otool -L build/cli/retopo | grep -i qt`
  (macOS) must print nothing.
- New CLI flags must also appear in `--help` and the README.

## Layout

- `core/` = Qt-free engine, built as `retopo_core` (`core/include/`
  holds the public `<AutoRemesher/...>` forwarding headers).
- `cli/` = Qt-free CLI. `bench/` = harness (models/results gitignored,
  `baseline.json` committed).
- `app/` = Qt GUI shell (sources, `shaders/`, `resources/`, `resources.qrc`).
- `thirdparty/` = vendored deps (Eigen, TBB, meshoptimizer,
  isotropicremesher, tinyobjloader, QtAwesome, QtWaitingSpinner).
