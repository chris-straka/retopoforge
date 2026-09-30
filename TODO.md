# retopoforge TODO / Roadmap

Owner context: solo-dev action game in a Twilight-Princess-like stylized
look, shipping on desktop and mobile. Asset pipeline is AI image-to-3D
generation (dense textured triangle soup, often messy: holes,
non-manifold spots, unwelded verts, no rig) remeshed into clean
quad-dominant game characters and props, textured and rigged downstream
in Blender/Godot. Optimize for organic quality and robustness on nasty
AI inputs over hard-surface features.
Platform strategy: ONE LOD chain per asset; desktop and mobile pick
different rungs (mobile clamps to lower rungs / tighter screen-size
thresholds). Never author separate per-platform models.
The owner's game assets must never be committed to this repo — not even
file names or paths in tracked files. Local test corpus lives in the
game project next door; refer to it only as "the owner's AI corpus".

Ordered by owner value: game pipeline first, Exoside-beating last.
Standing rule for all refactors: `bench/run.py --check bench/baseline.json`
must report no regressions with identical counts, and new code adds zero
new warnings.

## Game-asset pipeline (owner's core loop)

- [ ] AI-soup sliver output (BLOCKER for the owner's use): dense AI soup
      remeshes into microscopic slivers (median face area ~1e-8 at
      q5000; total surface ~2% of sane) at all adaptivity settings and
      input orientations. Coverage improves with target but stays far
      below sane. Repro: `bench/models/ai-soup-repro.obj` (local-only,
      gitignored — never commit). Suspect sizing-field breakdown on
      noisy soup; the count-accuracy fix below may cure both — re-test
      slivers right after it lands.
- [ ] Target-count accuracy: `--target-quads` currently undershoots badly
      (10k asked, ~4.3k produced on the owner's corpus). Research done:
      root cause is the unnormalized adaptivity field in
      `Parameterizer::computeFaceScalingField` (no budget
      renormalization; flat regions pin at 3x = 1/9 density). Fix =
      renormalize to preserve integral(area/s^2), then regen
      `bench/baseline.json` (counts rise everywhere). Mobile budgets are
      exact — and Exoside's count is approximate too, so exact counts
      would be a genuine edge, not catch-up.
- [x] Weld-on-load in the CLI: `weldPositionsAndTriangles` (meshopt
      remap, degenerate-tris drop) runs on every load; de-indexed
      armadillo went from 99,978 islands / 305 s / empty output to
      1 island / 0.3 s / baseline counts
- [x] `--quiet` CLI flag: silences CLI-owned progress/info; report
      block, warnings, and errors always print (engine-owned stderr
      still prints — needs an engine touch to fully silence)
- [x] Loud island-failure accounting: report block + `--report` file
      carry `Islands:` / `Failed islands:`, stderr warns; exit 0 kept
      on partial success so pipelines keep surviving output (CLI-side
      bbox heuristic — exact attribution needs an engine counter)
- [x] Batch mode: remesh a whole asset folder in one CLI invocation
      (`--input` dir + `--output` dir, per-file report, failed-files
      list, exit 1 on partial failure)
- [ ] Robustness pass over the owner's AI corpus (holes proven OK;
      still to probe: non-manifold soup, floating parts, multi-component
      meshes, 1M-tri scale perf). Record results as local-only notes,
      never asset names in tracked files.

## LOD chains (desktop + mobile from one chain)

- [x] Multi-resolution output: CLI `--lods` emits the full chain
      (`<stem>_lod<N>.obj` + per-rung report lines)
- [x] Blender one-click "Generate LODs" driving CLI `--lods`
      (rungs as `<name>_lodN` siblings; sync operator — modal-ize if
      long chains freeze the UI annoyingly)
- [ ] Document the rung strategy: which chain rungs serve desktop vs
      mobile, triangle budgets per rung for hero/prop/environment
      assets, and how Godot's import-time auto-LOD interacts with
      hand-authored chains.

## Texturing (AI output is textured; ours is bare)

- [ ] (future) Emit remeshed UVs from the internal parameterization so
      game assets can be textured without a second auto-UV pass
      (Exoside's UV behavior is undocumented — possible leapfrog)
- [ ] Blender bake assist: automate the standard high→low bake
      (import high-poly source + remeshed low, Smart UV Project the low,
      bake diffuse/normal from high) as a one-click addon step
- [x] GLB input (and ideally output) for the CLI to cut the manual
      GLB→OBJ conversion out of the loop (cgltf input + hand-written
      writer; batch and `--lods` keep the extension)

## Character quality (engine work that serves the game)

- [ ] Face animation flow (owner's top quality complaint): automatic
      fields follow curvature, not animation flow — eye/mouth loops
      that deform cleanly need user-drawn guide curves as frame-field
      constraints (draw flow lines in Blender → CLI carries them to the
      engine). Until then: document the iterate loop (preset, sharp
      angle, adaptivity, head-only passes) plus the manual-cleanup
      workflow. Honest scope: no automatic remesher emits
      animator-grade face topology; the goal is 80% + fast cleanup.
- [ ] Symmetry constraints (characters are the main subject)
- [ ] Sharp / feature constraints end-to-end (weapons and hard-surface
      props need crisp edges; Blender sharp-edge marks as constraints
      needs a CLI `--features` input flag first)
- [ ] Local density control (face/hands detail without blowing the
      total budget)

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
- [x] Iterate loop: per-object settings recall (last-used params
      auto-restore per object with an INFO note, so do-overs are one
      click, not retyping)

## App phase 2: Qt shell headers to modules

- [x] Converted the 11 plain `app/*.h` to `retopo.app.*` modules,
      bench-identical, zero warnings (mocs unity workaround extended
      with the app PCM dir + module-object ordering edges)
- [ ] Q_OBJECT widgets (14 headers) stay as headers — toolchain limit,
      not effort: moc emits member definitions plus Qt includes that
      cannot coexist inside module purview (pilot: one module warns,
      two modules hard-error). Revisit only if moc gains module support.
- [ ] Macro-only `version.h` stays (macros don't export from modules)
- [x] QtAwesome replacement: dead code — zero live call sites, so
      deleted outright (no SVG set needed): removed
      `thirdparty/QtAwesome`, `SpinnableAwesomeButton`, and the
      `Theme::initAwesome*` helpers.

## C++ follow-ups

- [ ] API-shape modernization, solo (cross-file, not lane-safe):
      `string_view` params, `std::span`, `std::expected` returns
- [x] Expand `tests/`: solver golden tests, CLI round-trip tests;
      wire `ctest` into CI (CLI round-trip + SurfaceMesh tests done,
      CI wired; solver goldens still open — see below)
- [x] Solver golden tests (CLS 9 groups + MILS 5 groups, 1e-6
      tolerance, theory-derived expectations)
- [x] Binary rename `autoremesher` → `retopoforge` (binaries, bundle, docs)
- [x] Upstream watch: Sept-2026 Kwizatz PRs evaluated — all already
      present (fork contains upstream/master tip 3cb2012c): #56 dense
      face map, #57 success flag, #58 input validation, #59 Qt6/MinGW,
      #60 unique_ptr. No ports needed.

## Quality / release

- [ ] README refresh (module layout, build, tests, Blender addon)
- [ ] Architecture doc (engine / CLI / app / addon split, module graph)
- [ ] macOS bundle CI: sign + verify path via `ci/macos_bundle.sh`
- [ ] Bigger bench models for a real perf signal
- [x] Linux CI revive (ubuntu-24.04 + distro clang, ctest + bench
      gated); Windows stays parked. NOTE: unproven until release CI
      runs it — fix forward if red.

## Exoside parity (last)

- [x] Feature comparison pass vs Exoside (the $100 benchmark —
      `docs/exoside-gap.md`, cited, UNVERIFIED marks where needed)
- [ ] Guide system phase 1 (their headline differentiator; L–XL)
- [ ] Density painting (0.25x–4x local density)
- [ ] Perf at scale: profile the CLI on production-size meshes, check
      TBB scaling, measured black-box comparison vs QR (EULA-aware:
      their outputs stay out of the repo and out of training data)
- [ ] UX polish in the addon and the app
- [ ] DCC breadth (other hosts) — last of last
