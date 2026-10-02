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

## Rust port (landed on main; was `exp/rust-solvers`)

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
- [x] Core batch, 18 modules: double_utils, progress, obj_reader,
      mesh_separator, vector2+vector3 (FMA-exact, bitwise), position_key,
      surface_mesh, density, symmetry, isotropic_remesher (+kernel),
      quad_parameterizer, guides, frame_field, singularity_simplifier
      (sincos-fusion root cause, bitwise oracle), parameterizer
      (PPX-documented, vendored singularity deduped at join),
      quad_extractor (libc++ hash emulation, 227-case bitwise oracle;
      PositionKey/box-tree vendors deduped at join), autoremesher engine
      (295-case oracle, sincos-bisection fix, meshopt stays C++ via FFI)
- [x] Main lane joined (acceptance gate green: 50 arg + 20 IO +
      37 remesh with adaptive C++ census; stderr story specced, not
      restored — see gap precedent; hex-float/nan(payload) specified
      divergence)
- [x] CLI glb IO joined (std-only reader, byte-identical writer;
      14/235 transform-path cases scale-aware, fmuladd has no bitwise
      contract there)
- [x] Port complete: 20/20 (18 core + 2 solvers) + glb; main-lane
      end-to-end run is the acceptance gate for the whole port
- [x] Switch: Rust `cargo test --locked` gated in CI, verdict in
      `docs/rust-switch-verdict.md`, Blender addon + Homebrew formula
      (0.3.0, rust-only) + bench default point at the Rust binary,
      `rust/Cargo.lock` committed, C++ frozen as oracle reference
- [x] Post-switch superiority batch (equality proved — now beat C++):
      sizing-aware MILS rounding: SCOPED OUT, verdict NO (see
      `docs/mils-rounding-scope.md` — progressive schedule degrades the
      production path: fewer retry fires but unrecoverable folds,
      dist_max tail 1.7x, medians wrong-way; third negative after
      greedy-rejected and schedule-flat). CLI UX redesign LANDED
      (chunks 0-2: version 0.3.0, errors teach + fail fast, grouped
      help + `--verbose` gating + stream split; spec was on branch
      lane/cli-ux-spec). Single-island parallelism LANDED (lane
      par-single-island: O(1) containers + scoped-thread data-parallel
      loops; dragon-50k >20min -> 22.3s, bitwise-identical).

## Next work (2026-10-01; read docs/direction.md first)

Owner's map of all projects and the cross-project order:
`~/Games/hll/tools/roadmap.md`. Gate every engine change with
`bench/noise.py` + `bench/score.py` distributions (single runs are below
the noise floor), plus `bench/matched.py` against `bench/quadwild.py`
when comparing back ends.

- [x] Remove remaining triangles/pentagons: final `cleanup_residual_routes`
      pass (pentagon-start route collapses into sinks, generalized from
      `cleanup_triangles`). Owner's character 203->64 non-quads (-68%),
      beast 49->10, fandisk/nefertiti/armadillo residuals roughly halved;
      oracles re-pinned via UPDATE_QUADEXT/UPDATE_ARDIFF (30 + 52 cases,
      every rewrite reduces non-quads). Loop-locked loners remain (no
      straight route to a sink; need defect migration, not more routes).
- [x] Normalize input scale by a power of two on load: engine entry
      scales sub-unit inputs by exact 2^k (IEEE-exponent math, libm-free)
      into diag [1,2), unscales all position outputs (exact round trip).
      diag >= 1 bitwise-identical (bench corpus 1.3-658 untouched). Tiny
      beast x1e-6: 210 quads (yield 0.04) -> 5268 healthy, x1e-7 empty ->
      5191; tiny noise.py distributions match full-scale. Oracle: only
      case 285 (tiny) 0->33 quads re-pinned, +15 stale ECX CONN lines
      (report-only drift since item 1); regen hook extended to all 11
      output sections with per-section count triggers.
- [x] Rounding: TRIED true greedy rounding (closest-quarter/half per
      round, MIQ-style), REJECTED on the gate. Matched-count beast:
      quarters improve irr 5.84->5.10 but regress dist_max 2.09->2.32
      (8/8 seeds worse) at +33% time; halves worse still (dist_mean
      +10%, dist_max +35%). Greediness trades accuracy for regularity
      monotonically — wrong direction (worst-case is our weak metric vs
      QuadWild). One-shot stays. Experiment reverted (lives in history).
- [x] Drop the libc++ order emulation: `CxxSet`/`CxxMap` (~860 lines)
      deleted, extractor fully `BTreeMap`/`BTreeSet` + dead 28k-line
      fixture sections stripped. Oracles re-baselined via permissive
      snapshot regen (extractor 80+value cases 124->120 non-quads,
      engine 170 cases 185->183; totals neutral, 1 marginal empty
      traded for 2 un-empties). noise.py A/B overlaps on every metric;
      dragon-50k 25.6s before=after, RSS unchanged. Regen now supports
      exact-compare snapshots + PROG re-pin (monotonicity-checked) in
      permissive mode.
- [x] Reduce tiling chaos from decimation: snap decimator f32 inputs
      to a 1e-6-diagonal grid (noise 1e-9 vanishes except rare boundary
      flips; grid 18x below the tightest bench min-edge). Decimated
      face overlap 77%->91% across seeds; armadillo @1000 quads spread
      [501-749]->[534-639]; @5000 quality spreads tighten (angdev width
      halved); medians neutral everywhere incl. dragon; no-decimation
      beast bit-identical (control). Oracle: 7 decimating cases
      re-pinned (+4 PHASE count tokens by hand); macOS baseline
      regenerated (Linux baseline NEEDS a Linux regen — counts shifted).
      Kept RETOPO_DUMP_DECIMATED probe for the meshoptimizer item.
- [x] Migrate meshoptimizer to Rust (`retopo_core::decimator`, f64
      internals) and FLIP the default (FFI, `build.rs` cc step, and
      `thirdparty/` deleted; pure-Rust tree): f32 transcription proven
      bit-faithful seed-for-seed; f64 kills rounding flips ->
      decimated overlap 99.7-100%. The flip blocker (beast@1000 lands
      badly, dist 7->30) was resolved by the coverage retry (item 6b,
      not item 7): re-ran gate is 8-seed native-vs-meshopt noise on all
      5 models x 1000/5000 + deform spreads — beast all-green, 3 cases
      bit-identical, stability wins (armadillo@5000 3 tilings vs 8,
      beast deform spread collapses). Known cost, all sub-coverage-bar
      extremity starvation also present on meshopt (different sites):
      armadillo@1000 dist_max +50%/deform vol ~2x/flips +30,
      armadillo@5000 flips +30, dragon stretch +5-8%, nefertiti angdev
      +12% @1000. Re-pinned: ardiff fixture (7 decimating cases,
      nonquads 4->10 on micro-fixtures, 1 PHASE token by hand),
      bench/baseline.json + bench/deform_baseline.json (macOS);
      bench/baseline-linux.json NEEDS a Linux regen; deform linux
      baseline likewise. Coverage gap noted: input-side extremity
      drops upstream of the working mesh are invisible to the
      working->output check (armadillo fingertip-class misses).
- [x] Coverage check + deterministic retry (before item 7): every
      island fails coverage when >= 25 working verts sit beyond 3x the
      nominal quad width (resolution-relative; absolute bars misfire on
      coarse-healthy outputs, see `docs/coverage-retry.md`); failure
      re-runs the island with jitter seeds 1..3 (1e-3 diag amplitude —
      noise-floor jitter cannot move the rounding) keeping the first
      full-coverage result, else attempt 0; reported like a failed
      island (`Warning:` + `coverage_reports()`). Gates: beast@1000-
      native recovers the appendage 8/8 seeds (first retry, exact-zero
      residuals, dist 4.8-8.2 vs 13.5-30.6), other bench cases
      byte-identical, contract + full suite green (62 ok), bench gate
      green, permanent `coverage_retry.rs` regression (skips without
      the corpus; proven to fail with retries off). Follow-ups landed:
      unrecoverable suite case pinned explicitly (diff id 58 — since
      retired by the corner-mark cage fix: its collapse was its own
      sharps pinning the slab; docs say why); retry jitter confined
      to the parameterization
      (extractor embeds unjittered; median output-to-working 2.2e-15,
      pinned); per-region patch bar (>= 10 connected beyond 3 widths)
      for thin drops the floor misses (claw fixture; suite grid case +
      CLI grid goldens legitimately recover and re-baseline). Input-side
      verdict added (original input verts vs output, same 3-width bar,
      connectivity-only — dragon scatter forbids a count floor; side
      pinned in `CoverageReport.input_side`): catches extremities the
      working mesh keeps only as stretched-triangle surface
      (thin-claw e2e fixture fires input-side-only and re-passes;
      `CoverageIndex` grid/scan === brute force pinned, with
      termination guards — scan unless h > bar/64 — after a
      collapsed-h output hung the suite). Fallback chain: first
      both-quiet wins, else earliest working-quiet (`kept_attempt`) so
      input-side never downgrades working coverage (armadillo noise
      seeds keep pre-input-side winners byte-identically). The retry
      re-rolls the tiling, it does not fix the fold: beast stays a
      first-attempt-coverage target for the patch back end.
- [x] Untangling stage in the current engine: score-gated NO —
      post-rounding Garanzha barrier (fixed integers) halves IGM flips
      but never converges (60-round cap everywhere) and harms output
      quality (fandisk@5000 8-seed: angdev 6.4->9.7, dist 0.034->0.040
      with disjoint ranges; native beast angles +23%); prototype
      removed, evidence in `docs/untangle-verdict.md`. NOTE (bisected, see
      `docs/beast-knife-edge-bisection.md`): beast@1000-native drops the
      whole left appendage (x < -62, 416 working tris -> 0 quads, closed
      mesh, dist 30 vs 7) through a NEW chain — singularity count lands
      systematically high (120-122 vs old 98-117 spread) -> degenerate
      layout (93 near-zero-area uvs vs <=62) -> appendage uv folds 100%
      inside body uv (old: 53%) -> extractor traces it once onto body
      tris. Untangling-as-prototyped (local flip barrier, fixed integers)
      can NOT fix this (global fold, locally valid); fix directions are
      singularity control, integer-layout feasibility, or extraction
      robustness. Use this case + the deformation score (item 9) to gate
      robustness before flipping the decimator default.
- [x] Joint loops: rigforge landmarks -> guide rings via `--guides`
      (`bench/rings.py` + `bench/test_rings.py`, 20 checks): biped
      z-slices (torso loop dropped at 3+ loops) + quadruped per-leg
      transverse planes from real detector schema; rings byte-
      deterministic, CLI-accepted, scores hold (tube + twisted tube +
      rigforge synthetic quadruped with real detector landmarks: rings
      land exactly on leg mids). Frame caveat in the docstring (Blender-
      world vs file axes); bench corpus has no detector-compatible
      subject (beast fails the signature loud).
- [x] Deformation score in bench (rig + pose + joint distortion) as the
      main gate (`bench/deform.py` + `bench/deform_baseline.json`, wired
      into `bench/run.py --check`; see `docs/deformation-test.md`)
- [ ] Build the patch back end as a selectable extension (`--backend patch`,
      clean-room QuadWild + Bi-MDF from the papers), only after the items
      above; the current engine stays the default. Target: beast@1000-
      native must cover the appendage on the FIRST attempt (no retry) —
      the coverage retry (`docs/coverage-retry.md`) papers over a fold
      the patch extractor should never produce. STATUS (kept on branch
      `agent/patch-backend`, see `docs/patch-revival.md` there): revived
      — planned greedy fill + caps + projection index turned the 30-min
      beast hang into 4.3 s, tube smoke 0.6 s in CI, first-attempt
      appendage coverage PROVEN and pinned — but the landing protocol
      fails (92/144 non-quads; matched corpus three-way loses all four
      criteria: fallback-heavy exact-equality quantization strands ~2/3
      of faces into subdivision, 16-19x over-yield, 55% irregular).
      To land: quantization agreement + all-quad odd fills.
- [ ] Humanoid faces/hands: out of scope here (wrapforge)
- Experiments with code: branch `exp/igm-validity` (flip census,
  rounding schedule, untangling)

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
- [x] Full PBR bake: bake every map the source has (albedo, roughness,
      metallic, AO, emissive), not just diffuse + normal; cage support.
      LANDED: per-map toggles (default on), used-socket detection with
      skip notes, metallic via temp-duplicate Metallic→Emission rewire
      (no native bake type), cage picker with face-count validation.
      Headless phases A/B/C green (skip notes, 6 non-uniform PNGs,
      dup cleanup, cage bake + wrong-topology cancel).
- [x] One-click end-to-end: remesh → UV → bake all maps in one action.
      LANDED (`retopoforge.remesh_and_bake`, "Remesh + Bake All"):
      active HIGH → sync remesh (keep-original forced + restored) →
      Smart-UV + full PBR bake to the `_retopo` result (HIGH unhidden
      for the bake raycast, active restored). Headless section green
      (6 PNGs, diffuse saw HIGH, report has both legs).
- [x] Better low-poly UVs: proper unwrap + pack with texel-density
      control (replacing Smart UV), or the engine-side global atlas
      (see Engine backlog). LANDED (Blender side; engine atlas was
      already done): LOW UVs mode Smart (default) vs Unwrap + Pack
      (angle-based/conformal + average-islands-scale + pack margin),
      measured px/unit always reported, nonzero target enforced by
      post-pack uniform scale when it fits the tile, honest unmet
      note otherwise (pack normalizes scale — probed). Headless
      phase D green (tile fit, 8px/unit enforced exactly, absurd
      target refused).
- [x] Direct UV projection: nearest-point UV copy where the remesh hugs
      the source (keeps original seams, skips re-bake). LANDED
      (`retopoforge.project_uvs`): BVH nearest-face per LOW face,
      barycentric UV interpolation per corner (per-face mapping keeps
      seams), range gate as bbox-diag fraction, report counts.
      Headless green (rewrite + tile bounds, majority projected,
      far-away keeps UVs, uv-less HIGH cancels).
- [x] Vertex-color / attribute transfer for non-textured AI outputs.
      LANDED (`retopoforge.transfer_colors`): HIGH active color layer
      (point or corner domain) onto LOW via the shared
      `_face_correspondence` projection core (factored out of UV
      projection), same-named CORNER layer, range gate + report
      counts. Headless green (non-uniform transfer, far-away fill,
      color-less HIGH cancels).
## Rigging (separate repos, see docs/rigging-strategy.md)

Character rigging lives outside this repo: `~/SWE/rigforge` (Rigify
fork, heroes/control rigs) and `~/SWE/unirig-mac` (ML rigger Mac
port, creature volume/auto-placement). Toolbox siblings, no contest.

- [ ] Pipeline end-to-end: remeshed mesh → rigged character
      (orchestrates the rig repos; only after their comparison +
      bake tooling settle — until then this repo stays the mesh stage)

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
      Density-aware pole placement has now landed (default-on dipoles,
      `docs/dipole-production.md` — 4x faceAbs up to 2.04x on fingers).
      Blender vertex-group (weight-paint) export done (see Exoside parity).
- [ ] Hands (DEFERRED to last, 2026-10-01: owner's call — build
      once everything else is tip-top, if at all): fused fingers are
      fused in the AI input, so no remesher setting can unfuse them.
      Loop is detect -> propose -> owner reviews/edits -> remesh runs
      with approved assist (build Rust-first, no mirror oracle needed):
      (1) auto-detect first: extremity finder proposes hand regions,
      scale-free webbing metric (valley depth vs finger width) scores
      them, warning fires with region highlighted; mark-mode stays as
      manual override only, per-mesh suppress + a mittens mode (skip
      detection, just refine hands); (2) propose assist as an overlay,
      never applied blind: hand density mask ("spend more quads here")
      shown as heatmap + valley guide polylines ("run edges along
      here") shown as lines — owner tweaks, redraws, mittens, or
      approves; honest output for zero-valley fusion is "mitten or
      separate by hand," never a fake fix; (3) regional pre-pass
      cleanup (weld/dust/nonmanifold repair scoped to the hand region
      only). Assist helps PARTIAL fusion only (shallow valleys the
      remesher would smooth over); true fusion needs real geometry
      edits by the owner. Never automatic finger surgery (cutting soup
      apart invents worse artifacts). Test with procedural
      fused-vs-split finger-tube fixtures (no game assets committed).

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
      15k hero still starves thin regions; the density-aware pole
      placement below has now landed (default-on dipoles, validated on
      finger-like fixtures) — needs owner eyes on whether the hero's
      thin regions fill in with a sharp mask
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
- [x] Corner singularities under crossing sharps: full closed cages
      over-constrain and distort — FIXED (see `docs/corner-marks.md`):
      explicit marks are alignment-only now (`AlignU`/`AlignV`: UV
      equality kept, integer period + curl anchor dropped). Box cage
      8-seed: quads 437->563, irr 13.7->4.0, dist max 6.9->4.3
      (disjoint ranges); single lines hold; damage was proportional
      to mark count (channel ablation), crossings need nothing extra.
      Oracles: 5 quadparam + 32 parameterizer demotions, 45-case
      ardiff re-pin (zero SHARPS=0 changers), case-58 coverage pin
      retired, new `box-cage` CLI regression (fails pre-fix).
- [x] Case-156 C++ heap-OOB read on DENSITY-3 inputs (found by port
      forensics, engine bisection): C++ values derive from UB —
      EPX-by-UB in the oracle, never match; exclude the input class in
      main-lane e2e. No C++ fix (being replaced). MOOT 2026-10-01:
      main-lane e2e was the retired C++ differential (`e2e_diff.rs`,
      deleted with the C++ tree); the Rust-only contract has no C++
      comparison to exclude anything from. Verdict doc keeps the
      historical record.
- [x] Merge-five-faces path divergence (found by e2e forensics;
      verdict PORT BUG, fixed on `lane/merge-five`): was NOT a path
      divergence — the Rust merge always ran (trace: pentagons 5:2→0
      and 5:4→0 with no print), only its stderr print was gated on the
      parked `progress_handler` (`extract()` parks it in an `Arc`
      while the merge runs, so `diagnose()` saw `None`). Fix gates the
      print on the passed `progress` handle (`Some` exactly when the
      outer handler exists — the C++ gate). e2e allowlist arm removed,
      counts match via census/robust tiers (mode-sensitive, as on the
      C++ side itself). Seven-splits stay allowlisted: their gates are
      verified identical with the handler intact, so that 5/0 is
      genuinely state-driven — separate follow-up.
- [x] Density-aware pole placement (research → production
      2026-10-01, `lane/dipole-production`,
      `docs/dipole-production.md`): was "strong localized refinement
      saturates (~2.3x for 4x asks) because poles are sizing-unaware".
      Shipped automatic dipole rings off raw density steps (per-ring
      line-ending dose, offset-everywhere placement rule, sliver/speck
      guards), CLI `--dipoles off|auto` defaulting to auto, quirks
      fixed, oracles re-tiered. Finger fixtures (re-tiered baselines
      in `docs/dipole-fixtures-baseline.md`): 4x faceAbs off→auto
      single 1.70→2.04, split 1.57→1.70, fused 1.33→1.52; all six
      sharp rows improve, unmasked/mild runs bit-identical via gating.
- [x] Dipole-spike quirks (flagged + fixed 2026-10-01, production
      pass): (1) pole fans opposed the side quads — fan order fixed,
      directed-edge pairing asserted in generator + harness, fixtures
      regenerated (verts identical); (2) "silent cover failure"
      forensically cleared — instrumented runs show parameterize +
      extract both succeed (0-quad island is area starvation) and the
      `Failed islands` counter DOES catch it; harness now prints +
      asserts per-island counts, CLI/golden tests pin the accounting.
- [x] Tetra non-monotonic collapse (research, time-boxed): tiny inputs
      collapse non-monotonically with target count (empty at 8 and 2,
      OK at 4). Probe whether a principled floor exists; report-only
      fallback.
- [x] Single-island parallelism (research, landed 2026-10-01 on
      lane/par-single-island): profile showed the Rust bottleneck was NOT
      the C++ one — O(n^2) Cxx container emulation made dragon-50k take
      >20min (99.9% of samples in insert_fresh). Fixed with O(1)
      exact-order containers + sorted per-round fixpoint indexes +
      scoped-thread data-parallel loops (frame field, smooth/project,
      remap phases). Dragon-50k: >20min -> 22.3s (C++: 22.7s);
      dragon-5k: 7.9s -> 1.8s. Bitwise-identical outputs (10 bench
      cases), strict run-to-run determinism, bench + full cargo test green.
- [x] beast/tiny 680-vs-779 mode split (found at switch, 2026-10-01;
      verdict MODE 2026-10-01, no code change): stage-dump bisection
      (adopted dead lane's env-gated harness) shows decimate +
      resample dumps byte-identical, frame field at solver noise
      (1570/2608 vecs differ, max 1.2e-14), then gross UV split.
      Mechanism: singularity-simplifier greedy cancellation consumes
      252 hop-tied candidates in sort order — Rust's stable sort keeps
      index order, C++'s libc++ introsort scrambles ties (past the
      documented <24-candidate regime where C++ order is "unspecified
      anyway"). Singularities/charges identical 287/287 — the 1e-14
      field noise flips NO integer; pure order divergence. Fidelity:
      symmetric vertex-to-surface means 0.24% vs 0.27% of bbox diag,
      in->out identical to 4 digits, areas within 0.6% — equal
      quality, different tilings. Baselines stay Rust numbers.
      Follow-up (open): Linux-Rust gives 772 (vs 680 mac-Rust) — same
      deterministic sort, so the candidate/singularity SET likely
      differs via libm-noise integer flip or meshopt codegen; needs a
      Linux-side bisection. Both modes valid + baselined; not blocking.
