# retopoforge — agent notes

Fork of huxingyi/autoremesher (MIT): Qt-free headless C++ engine + `retopo`
CLI + benchmark harness + (planned) Blender addon. The `upstream` git remote
tracks the original repo for merging future fixes.

## Standing rules

- Commit and push to `origin/master` on your own after each completed chunk
  of work. Do not wait for the user to approve commits or pushes.
  (Explicit standing authorization from the project owner.)
- Never force-push, rebase, amend published commits, or otherwise rewrite
  published history.
- The Qt desktop app (`autoremesher.pro`) must keep building — verify it or
  leave its inputs untouched whenever shared sources change.

## Build

- Headless: `cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j`
  produces `build/cli/retopo` (needs TBB; macOS: `brew install cmake tbb`).
- Qt app: `qmake && make` (Qt 5/6). For a quick `.pro` parse check, run
  `qmake /path/to/autoremesher.pro` from an empty shadow directory.

## Checks

- `bench/run.py --check bench/baseline.json` must pass after engine or CLI
  changes (fetch models once with `bench/fetch_models.sh`).
- `retopo` must stay Qt-free: `otool -L build/cli/retopo | grep -i qt`
  (macOS) must print nothing.
- New CLI flags must also appear in `--help` and the README.

## Layout

- `src/AutoRemesher/` + `thirdparty/{isotropicremesher,meshoptimizer}`
  + `include/` = Qt-free core, built as `retopo_core`.
- `cli/` = Qt-free CLI. `bench/` = harness (models/results gitignored,
  `baseline.json` committed).
- Top-level `src/*.cpp`, `shaders/`, `resources/` = Qt GUI shell.
