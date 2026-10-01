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
must report no regressions (quality bar: good remeshes, not
bit-identical counts), and new code adds zero new warnings.

## Rust port (active workstream, `exp/rust-solvers`)

Strangler-fig rewrite: Rust mirrors C++ module-by-module, each proven by
a differential oracle (C++ dump + committed fixture + Rust replay) before
joining. Equality is scaffolding, not the goal — the oracles become the
regression net for post-switch improvements. Bar per lane: oracle green
(exact/bitwise where deterministic; scale-aware 1e-6 + zero structural
mismatches where float order legitimately varies; robustness-only sections
with demonstrated mechanism where backend noise flips rounding — the
QPX/FFX lists), `cargo test` green, fmt clean, timing recorded. Never
push main from lanes (explicit refspec only); game assets never committed.

- [x] Solvers first (calibration): CLS + MILS in `retopo_solvers` (faer),
      9+5 goldens, 200+200 differential cases — verdict: viable
- [x] Core batch, 16 modules: double_utils, progress, obj_reader,
      mesh_separator, vector2+vector3 (FMA-exact, bitwise), position_key,
      surface_mesh, density, symmetry, isotropic_remesher (+kernel),
      quad_parameterizer, guides, frame_field, singularity_simplifier
      (sincos-fusion root cause, bitwise oracle), parameterizer
      (PPX-documented, vendored singularity deduped at join)
- [ ] Finisher running: quad_extractor (largest module, mid-port)
- [x] CLI glb IO joined (std-only reader, byte-identical writer;
      14/235 transform-path cases scale-aware, fmuladd has no bitwise
      contract there)
- [ ] Queued behind deps: autoremesher engine (needs quad_extractor;
      parameterizer joined), cli/main (needs engine; its end-to-end
      differential run is the acceptance gate for the whole port)
- [ ] Switch: gate Rust `cargo test` in CI, write the rewrite verdict,
      point the Blender addon + Homebrew formula at the Rust binary
- [ ] Post-switch superiority batch (equality proved — now beat C++):
      sizing-aware MILS rounding driven by the QPX/FFX flip maps, CLI UX
      redesign (flags/errors/progress), single-island parallelism in Rust

## Game-asset pipeline (owner's core loop)

- [x] AI-soup sliver output (was BLOCKER): fixed by weld-on-load +
      renorm together — soup repro now yields 4879 quads on a 5000
      target, median face ~1e-4, 82% surface retained (was: median
      ~1e-8, ~2% surface). Repro kept at `bench/models/ai-soup-repro.obj`
      (local-only, gitignored — never commit).
- [x] Target-count accuracy: renormalized `computeFaceScalingField` to
      preserve integral(area/s^2); baseline regen (counts rise
      everywhere, e.g. armadillo small 2856→4566). Test pins are now
      tolerance-based (0.5% + 8): macOS Accelerate MT solves jitter a
      few quads run-to-run, and Linux SimplicialLDLT may differ —
      revisit per-platform exact pins if Linux CI disagrees beyond
      tolerance.
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
- [x] Robustness pass over the owner's AI corpus: holes, non-manifold
      soup, floating parts, multi-component meshes all probed via a
      committed synthetic nasty-corpus (11 fixtures) + procedural
      scale inputs; loader OOB/NaN validation, engine entry guards,
      multi-mode island accounting, honest exit codes. 1M-tri scale
      covered by the dragon profile (`docs/perf.md`). A sweep over the
      owner's real corpus stays a manual step (local-only, never asset
      names in tracked files).

## LOD chains (desktop + mobile from one chain)

- [x] Multi-resolution output: CLI `--lods` emits the full chain
      (`<stem>_lod<N>.obj` + per-rung report lines)
- [x] Blender one-click "Generate LODs" driving CLI `--lods`
      (rungs as `<name>_lodN` siblings; sync operator — modal-ize if
      long chains freeze the UI annoyingly)
- [x] Document the rung strategy (`docs/lod-strategy.md`): measured
      rung map + budgets + Godot import auto-LOD interaction

## Texturing (AI output is textured; ours is bare)

- [x] Emit remeshed UVs from the internal parameterization so game
      assets can be textured without a second auto-UV pass: CLI
      `--uvs on|off` (default off), OBJ `vt` + `v/vt` corners, GLB
      `TEXCOORD_0`, normalized 0..1 per island (multi-island UVs
      overlap; a global atlas needs a second pass)
- [x] Blender bake assist: one-click high→low bake (Smart UV the
      low, Cycles CPU diffuse + tangent normal, PNGs next to the .blend;
      render settings restored afterwards)
- [x] GLB input (and ideally output) for the CLI to cut the manual
      GLB→OBJ conversion out of the loop (cgltf input + hand-written
      writer; batch and `--lods` keep the extension)
- [ ] Full PBR bake: bake every map the source has (albedo, roughness,
      metallic, AO, emissive), not just diffuse + normal; cage support
- [ ] One-click end-to-end: remesh → UV → bake all maps in one action
- [ ] Better low-poly UVs: proper unwrap + pack with texel-density
      control (replacing Smart UV), or the engine-side global atlas
      (see Engine backlog)
- [ ] Direct UV projection: nearest-point UV copy where the remesh hugs
      the source (keeps original seams, skips re-bake)
- [ ] Vertex-color / attribute transfer for non-textured AI outputs

## Character quality (engine work that serves the game)

- [x] Face animation flow (owner's top quality complaint): engine +
      CLI + Blender guide mechanism done (frame-field locks, `--guides`
      file, edge-selection stroke export with recall) + iterate-loop
      doc (`docs/face-flow-iterate.md`: preset/sharp/adaptivity passes,
      guides, density masks, symmetry, RetopoFlow cleanup). Current
      state is an automatic pass plus fast cleanup; one-click faces
      remain the goal.
- [x] Symmetry constraints (characters are the main subject):
      vote-based plane detection + frame-field/vertex symmetrization,
      default off, CLI `--symmetry off|auto|x|y|z`; positional only
      (quad connectivity is not mirrored)
- [x] Sharp / feature constraints end-to-end (weapons and hard-surface
      props need crisp edges): engine `setSharpPolylines` (snapping
      post-resample, sharp-first frame locks winning ties over guides,
      corner marks, curl anchors) + CLI `--features` sharing the guide
      file format. Blender sharp-marks export done (see Engine backlog).
      Follow-up:
      corner singularities under crossing sharps distort (full cage
      over-constrains); keep corner-mark radius small.
- [x] Local density control (face/hands detail without blowing the
      total budget): engine `setDensityMultipliers` + CLI `--density`
      mask file, budget-preserving renormalization; strong localized
      refinement saturates (~2.3x for 4x asks — integer-grid pole
      saturation), mild masks realize nearly fully, coarsening fully.
      Full 4x needs density-aware pole placement (future engine work).
      Blender vertex-group (weight-paint) export done (see Exoside parity).
- [ ] Hands: fused fingers are fused in the AI input, so no remesher
      setting can unfuse them — detect + warn + assist instead. Staged,
      AFTER the Rust port (build Rust-first, no mirror oracle needed):
      (1) mark-mode + fusion check: owner selects hand verts in
      Blender (or auto-detect extremities later), tool measures finger
      crotch depth / webbing and warns when fingers are stuck together,
      with per-mesh suppress + a mittens mode (skip finger detection,
      just refine hands); (2) auto-assist: feed a hand density mask +
      valley guides into the same run automatically (no geometry edits,
      failure mode is "no better" never "mangled"); (3) regional
      pre-pass cleanup (weld/dust/nonmanifold repair scoped to the hand
      region only). Never automatic finger surgery (cutting soup apart
      invents worse artifacts). Test with procedural fused-vs-split
      finger-tube fixtures (no game assets committed).

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
- [x] Zip install path verified (`package_install_files`) end to end
      in an isolated config (0.2.0)
- [ ] Release packaging (signed zip? extensions.blender.org listing?) —
      DEFERRED by owner 2026-09-30: no Apple $99 fee, no listing for now
- [x] Iterate loop: per-object settings recall (last-used params
      auto-restore per object with an INFO note, so do-overs are one
      click, not retyping)

## App phase 2: deleted (2026-09-30)

The Qt desktop shell was removed outright — Blender is the UI, the CLI
is the headless interface. (`app/`, the `retopo.app.*` modules, the
Q_OBJECT/moc notes, and `version.h` are gone; git history keeps them.)
This section stays as the record: 11 plain headers were modularized
and QtAwesome deleted as dead code before the removal.

## C++ follow-ups

- [x] API-shape modernization, solo (cross-file, not lane-safe):
      `string_view` for inspection-only paths (glb extension checks,
      `lodOutputPath`); `std::span` for read-only vector params
      (quad-cover scaling/guidance, density normalize/resample input).
      Deliberately not converted: filename params at fopen/ifstream
      boundaries (NUL termination), output/resized vectors (span can't
      resize), thirdparty-mirroring pointer members,
      `std::expected` (bool+warn/err loader pattern ripples callers
      and tests for no behavior gain).
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

- [x] README refresh (module layout, build, tests, Blender addon)
      — literal-tested, covers 16 modules + `--symmetry` + profiler
- [x] Architecture doc (engine / CLI / app / addon split, module graph)
      — covers the 16-module graph incl. symmetry
- [x] macOS bundle CI: unsigned bundle build + self-containment verify
      in `development.yml`; Developer-ID signing stubbed (needs owner
      `APPLE_DEVELOPER_IDENTITY` secret)
- [x] Bigger bench models for a real perf signal (Stanford XYZ Dragon,
      250k faces; 2nd mesh skipped — no verifiable license)
- [x] Linux CI revive (ubuntu-24.04 + distro clang, ctest + bench
      gated); Windows stays parked. NOTE: unproven until release CI
      runs it — fix forward if red.

## Exoside parity (last)

- [x] Feature comparison pass vs Exoside (the $100 benchmark —
      `docs/exoside-gap.md`, cited, UNVERIFIED marks where needed)
- [x] Guide system phase 1 (their headline differentiator):
      user polylines as frame-field hard constraints end to end
      (engine setter + CLI `--guides`); Blender-side drawing is
      phase 2
- [x] Density painting (0.25x–4x local density): engine + CLI mask
      file + Blender vertex-group export with min/max mapping and
      recall all done
- [x] Perf at scale: profiled (`docs/perf.md`, dragon 50k: ~23s,
      ~1.5 GiB); TBB scaling checked — no thread knob exists, batch
      mode is the parallelism story
- [x] Measured comparison vs free baselines (owner-scratched the
      paid QR comparison): native Voxel remesh in-repo + standalone
      QuadriFlow binary — same inputs, quad counts, timings,
      thin-feature behavior. NOTE: Blender 5.x removed the Quadriflow
      modifier mode (only BLOCKS/SMOOTH/SHARP/VOXEL remain), so the
      quad baseline must be the standalone QuadriFlow build, not a
      modifier
- [ ] Thin-feature detail allocation (fingers, face): owner-verified
      15k hero still starves thin regions; needs the density-aware
      pole placement research below, validated on finger-like fixtures
- [x] Pole pinch cleanup: stray non-manifold verts at sphere poles
      (2 verts found in character hair via Select Non-Manifold);
      find and fix the degenerate-cap source
- [ ] UX polish in the addon (needs the owner's eyes on real meshes)
- [ ] DCC breadth (other hosts) — last of last

## Engine backlog (from lane follow-ups, wave 5+)

- [x] C++23 completion: convert the last two headers (`autoremesher`,
      `objreader`) to named modules; delete the `<AutoRemesher/...>`
      forwarders. No moc excuse remains.
- [x] Global UV atlas: `--uvs on` normalizes 0..1 per island, so
      multi-island UVs overlap. Pack islands into one atlas (or emit
      per-island UDIM offsets).
- [x] Blender sharp-marks export: sharp-marked edges → `--features`
      file with recall, shared chain tracer with guides (byte-identical)
- [x] Engine island drop counter: replace the CLI's bbox island
      attribution heuristic with a real per-island output/empty count
      from the engine.
- [x] Quiet through the engine: `--quiet` still leaks engine-owned
      stderr (progress + phase report). Plumb the flag down.
- [ ] Corner singularities under crossing sharps: full closed cages
      over-constrain and distort. Fix the corner-mark radius/strength
      handling (currently documented as "keep it small").
- [ ] Density-aware pole placement (research): strong localized
      refinement saturates (~2.3x for 4x asks) because poles are
      sizing-unaware. Placing poles for the density field would unlock
      the full 4x. Post-switch: feed it the port's QPX/FFX
      robustness-only lists — they map exactly where rounding noise
      flips integer decisions today.
- [x] Tetra non-monotonic collapse (research, time-boxed): tiny inputs
      collapse non-monotonically with target count (empty at 8 and 2,
      OK at 4). Probe whether a principled floor exists; report-only
      fallback.
- [ ] Single-island parallelism (research): one island uses ~1 core;
      top bottleneck is "merging shared five edge faces" (5.5s on
      dragon-50k). Profile-guided; quality-gated (no --check regressions).
      Natural post-switch Rust work (fearless concurrency).
