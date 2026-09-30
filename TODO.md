# retopoforge TODO / Roadmap

Ordered by priority: Blender-first workflow, Exoside-beating last.
Standing rule for all refactors: `bench/run.py --check bench/baseline.json`
must report no regressions with identical counts, and new code adds zero
new warnings.

## In flight

- [ ] Idioms wave (`retopo-idioms`): solvers + mesh + shell modernization
      lanes and the unit-test lane; coordinator verifies each branch and
      joins to `main`
- [ ] Blender addon v1 scaffold (between lane reports)

## Blender addon (the workflow goal)

- [x] Extension-format package (`blender_manifest.toml`) for Blender 4.2+/5.x
- [x] Operator: remesh selected objects via the `retopo` CLI subprocess,
      temp-OBJ round-trip invisible to the user
- [x] Panel: organic/hardsurface presets, target quads, sharp/smooth angles,
      adaptivity, anisotropy, model type
- [x] Evaluated-mesh export (modifiers applied, toggleable) in local space
      under identity transform (double-safe); restore transforms on import
- [x] Replace-active-mesh in a single undo step vs spawn-new-object modes
- [x] Modal operator with progress indication (no UI freeze on long remeshes)
- [x] Stats report display in the panel
- [x] Multi-object loop over the selection; temp files removed afterwards
- [x] Headless verification (`blender/tests/test_headless.py`, all passing)
- [x] Honest UV / vertex-color data-loss notice in the UI
- [ ] Zip install path verified (`package_install_files`); user-facing
      release packaging (signed zip? extensions.blender.org listing?)
- [ ] (future) Blender sharp-edge marks as feature constraints — needs a
      CLI `--features` input flag first (engine change)

## App phase 2: Qt shell headers to modules

- [x] Converted the 11 plain `app/*.h` to `retopo.app.*` modules,
      bench-identical, zero warnings (mocs unity workaround extended
      with the app PCM dir + module-object ordering edges)
- [ ] Q_OBJECT widgets (14 headers) stay as headers — toolchain limit,
      not effort: moc emits member definitions plus Qt includes that
      cannot coexist inside module purview (pilot: one module warns,
      two modules hard-error). Revisit only if moc gains module support.
- [ ] Macro-only `version.h` stays (macros don't export from modules)
- [ ] (needs user decision) QtAwesome replacement: what replaces the
      FontAwesome icons?

## C++ follow-ups (after the idioms join)

- [ ] API-shape modernization, solo (cross-file, not lane-safe):
      `string_view` params, `std::span`, `std::expected` returns
- [ ] Expand `tests/`: solver golden tests, CLI round-trip tests;
      wire `ctest` into CI
- [ ] Binary rename `autoremesher` → `retopoforge` (binaries, bundle, docs)
- [ ] Upstream watch: evaluate the Sept-2026 Kwizatz PRs for porting —
      input validation (#58), parameterizer success flag (#57), dense
      face map (#56); unique_ptr (#60) and Qt6/MinGW (#59) already covered
      by our tree

## Quality / release

- [ ] README refresh (module layout, build, tests, Blender addon)
- [ ] Architecture doc (engine / CLI / app / addon split, module graph)
- [ ] macOS bundle CI: sign + verify path via `ci/macos_bundle.sh`
- [ ] Bigger bench models for a real perf signal
- [ ] Linux CI revive (parked); Windows stays parked (mac-only scope)

## Exoside parity (last)

- [ ] Feature comparison pass vs Exoside (the $100 benchmark)
- [ ] Sharp / feature constraints end-to-end
- [ ] Perf: profile the CLI on production-size meshes, check TBB scaling
- [ ] UX polish in the addon and the app
