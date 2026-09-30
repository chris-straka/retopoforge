# retopoforge — agent notes

Fork of huxingyi/autoremesher (MIT): Qt-free headless C++ engine + `retopo`
CLI + benchmark harness + (planned) Blender addon. The `upstream` git remote
tracks the original repo as a read-only reference only. This fork has
structurally diverged (C++23 modules, deleted headers, new layout) and is
ahead of upstream in engineering — NEVER git-merge upstream into this tree,
it will conflict destructively. Port individual upstream engine fixes by
hand when relevant, and only with `bench/run.py --check` green.

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
- Watch `upstream` for engine fixes worth hand-porting: periodically
  `git fetch upstream` and review new `upstream/master` commits. Never merge
  (diverged tree — port individual fixes by hand, bench green).

## Build

- Everything: `cmake -S . -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/macos-llvm.cmake -DCMAKE_BUILD_TYPE=Release && cmake --build build`
  produces `build/cli/retopo` and the Qt6 app (`build/app/autoremesher[.app]`).
  Needs TBB + Qt6 + LLVM + Ninja; macOS: `brew install cmake tbb qtbase llvm ninja`.
  (Ninja is mandatory: the only macOS generator with C++ modules support.
  AppleClang cannot build this tree at all once `.cppm` files exist.)

## Modules conversion pattern (established by the positionkey pilot)

- One named module per component: `retopo.core.snake_name`. Interface in
  `core/<name>.cppm`, implementation stays in `core/<name>.cpp`.
- Interface unit shape: copyright header, then `module;` + third-party and
  not-yet-converted includes (global fragment), then
  `export module retopo.core.<name>;`, then `export`ed declarations.
- Implementation unit shape: `module;` + includes, then `module <name>;`,
  then the definitions (includes AFTER the module decl attach to the
  module itself — always use the leading `module;` fragment).
- Importers swap `#include <AutoRemesher/X>` for the `import`, and keep
  direct includes/imports for everything else they use (no transitive
  reliance — the build enforces it).
- Delete the old `.h` and its `core/include/AutoRemesher/` forwarder.
- CMake: list the `.cppm` in the target's `FILE_SET CXX_MODULES`, and set
  `CXX_SCAN_FOR_MODULES ON` on every target with importers (plain `.cpp`
  files are otherwise compiled unscanned: no BMI flags, no ordering).
- Every conversion commit must keep `bench/run.py --check` green.
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
  isotropicremesher). OBJ loading and the spinner are native
  (`core/objreader.*`, `app/spinnerwidget.*`). QtAwesome was removed
  (dead code, zero call sites).
