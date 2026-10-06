# retopoforge

retopoforge is a fork of [AutoRemesher](https://github.com/huxingyi/autoremesher)
(MIT, by Jeremy HU) restructured around a **headless engine**: a Rust core
library, a `retopo` CLI, a Blender extension, and a benchmark/regression
harness. There is no desktop app — Blender is the UI. The engine was
ported 1:1 from the original C++, proven by differential oracles, and the
C++ tree was then removed (see
[docs/rust-switch-verdict.md](docs/rust-switch-verdict.md); the history
keeps it). Upstream is kept as
the `upstream` git remote as a read-only reference; this fork has
structurally diverged, so upstream engine fixes are ported by hand when
relevant, never git-merged.

## Install (Homebrew)

```bash
brew tap chris-straka/retopoforge https://github.com/chris-straka/retopoforge
brew install chris-straka/retopoforge/retopoforge
```

This taps the repo itself (the formula lives in `Formula/`) and builds
the `retopo` CLI from source (rust is pulled in automatically), linking
it onto your PATH as `retopo`. `brew update` keeps the tap current;
`brew upgrade retopoforge` rebuilds on updates.

## Build from source (cargo; macOS-only product)

```bash
brew install rust
cargo build --locked --release -p retopo
```

Run it from `rust/` (the cargo workspace). This builds the `retopo` CLI
(`rust/target/release/retopo`); pure Rust, no C++ (the vendored
meshoptimizer was replaced by the native `retopo_core::decimator`).

## CLI usage

```bash
./rust/target/release/retopo --help
./rust/target/release/retopo --input bench/models/armadillo.obj \
    --output /tmp/armadillo-remeshed.obj --report /tmp/armadillo-report.txt \
    --target-quads 5000
```

Flags: `--input`/`-i` and `--output`/`-o` (required), `--report`,
`--target-quads` (default 50000), `--edge-scaling` (1.0–4.0),
`--sharp-edge` (30–180°), `--smooth-normal` (0–180°),
`--adaptivity`/`--anisotropy` (0–1), `--model-type organic|hardsurface`,
`--symmetry off|auto|x|y|z` (default `off`), `--guides <file>`,
`--density <file>`, `--features <file>`, `--skeleton <file>`, `--uvs [on|off]` (default `off`,
bare `--uvs` means on),
`--lods <q0,q1,...>`,
`--quiet`/`-q`, `--verbose`, `--help`/`-h`, `--version`/`-v`.
Non-indexed triangle soup is
welded on load; `--quiet` silences progress output (warnings, errors, and
the report still print), `--verbose` adds the phase-timing table and
engine diagnostics on stderr (mutually exclusive with `--quiet`).
Progress, warnings, errors, and the closing `done:` line go to stderr;
stdout carries only results (the report block, `FILE`/`LOD` rung lines).
`--symmetry auto` detects the dominant mirror
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
single-file and `--lods` runs only. `--skeleton` takes the rig (one
`hx hy hz tx ty tz parent bend_degrees` bone per line, parent = earlier
line index or -1) and sizes the edges at creases between sibling limbs
(see Skeleton density); it multiplies with `--density`; single-file and
`--lods` runs only. `--uvs on` emits remeshed UVs from
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
./rust/target/release/retopo --input bench/models/armadillo.obj --output /tmp/hero.obj --lods 10000,5000,2000
./rust/target/release/retopo --input assets/ --output assets-retopo/
```

Exit codes: 0 success, 2 usage error (bad flags, missing options,
unreadable inputs — nothing runs), 1 runtime failure (an input failed
to load or remesh). Inputs are validated before any work starts, so a
bad flag or missing file fails fast without touching outputs.
Out-of-range numerics clamp to their documented range with a
`retopo: warning:` rather than failing.

## Tests

```bash
cd rust && cargo test --locked --release
```

Module goldens replay committed C++ reference dumps
(`tests/fixtures/*_diff.txt`, recorded before the C++ tree was removed)
against every engine component. The CLI contract test
(`rust/cli/tests/cli_contract.rs`) drives the built binary over 52
arg-matrix + 20 IO-failure + 37 remesh cases against goldens in
`tests/fixtures/cli_golden/`: exact output for parser and IO paths;
run-to-run determinism plus counts within 0.5% + 8 for remeshes. After
an intended CLI or engine change, regenerate and review the diff:

```bash
UPDATE_GOLDENS=1 cargo test --release -p retopo --test cli_contract
```

## Benchmarks

```bash
bench/fetch_models.sh          # one-time download of test models (gitignored)
bench/run.py                   # run suite, validate meshes, save results JSON
bench/run.py --check bench/baseline.json   # fail on regression vs baseline
bench/profile.py               # profile one production-size mesh (docs/perf.md)
bench/deform.py --check bench/deform_baseline.json  # deformation gate alone
bench/rings.py --mesh subject.obj --landmarks lm.json \
  --joints elbow,knee --output guides.txt  # joint rings -> --guides file
```

The suite runs `rust/target/release/retopo` over five models × two presets
(override with `bench/run.py --binary`)
(`--target-quads` 1000/5000), validates every output mesh, and records
timings plus quad counts. A run regresses when it exits non-zero, its
mesh fails validation, its quad count drops >5% below baseline, its
non-quad share rises >2pp, or its deformation scores (joint p95
stretch, volume loss, flips vs `bench/deform_baseline.json`) regress;
wall time is recorded but never gates.
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

### genforge adapter (repair-topology)

genforge's `gen character` chain rebuilds a character that fails its
budget check (P_VERTS, P_TRIS, P_TEX_SIZE, P_TEX_NORMAL) with

```bash
tools/genforge_adapter.sh repair-topology IN.glb OUT.glb RESULT.json \
    [--class humanoid|quadruped|custom]
```

Each skinned mesh over budget (or without a normal map) is remeshed by
the installed `retopo` CLI to a target from the class budget (rfcheck's
hero 10k verts / 15k tris / 1024 px; monster 12k / 20k / 1024),
smart-UV'd, baked (color + normal from the original, Cycles CPU), and
reskinned from the original (nearest-face weight transfer, 4
influences, normalized); the remesh gets the input's armature as
`--skeleton` (see Skeleton density), and the adapter runs this
checkout's `rust/target/release/retopo` (or `$RETOPO_BIN`) before the
one on PATH, since an older installed CLI lacks the flag; the armature, bone names and clips are kept
(glTF import without bind-pose guessing, so a translated root node
round-trips exactly). Textures over budget on untouched meshes are
scaled down. `RESULT.json` follows genforge's adapter schema
(`{"ok", "outputs", "tool": "retopoforge", ...}`; exit 0 within budget,
1 still over or a step failed, 2 error with no result). Test:

```bash
/Applications/Blender.app/Contents/MacOS/Blender --background \
    --factory-startup --python blender/tests/test_genforge_adapter.py
```

Measured 2026-10-05 on a real 22-bone, 9-clip textured character from
the owner's AI corpus: 10,395 -> 6,657 glTF verts (12,616 -> 11,074
tris; 6,632 on an earlier run: the tiling varies run to run), 2048 px
albedo -> 1024 px color + normal, rfcheck hero-class
clean, 6 s on an M4; turntable matches the original's colors.

The extension is GPL-3.0-or-later, as Blender requires; the Rust engine
stays MIT — the extension talks to it only as a subprocess over OBJ
files. See [docs/architecture.md](docs/architecture.md) and
[blender/README.md](blender/README.md).

## Skeleton density (`--skeleton`)

Deformation-aware density for rigged characters. The skin weights a
rigger or weight fixer produces blend across a joint over a roughly
fixed number of edge rings (SkinTokens' transition and weightforge's
repair bands are both about four rings), so the blend's width in space
is that ring count times the local edge length. Where a limb's crease
runs deep, the blend is then too narrow for the sweep and the crease
edges tear. Measured on a 2 m humanoid from the owner's AI corpus
(2026-10-05; SkinTokens rig, motionforge standardize, `weights fix`,
then the weightforge gate, three remesh targets each):

| groin topology | thigh stretch fails | score after fix |
|---|---|---|
| default | 3 of 3 runs (5-11 verts) | 72.6-73.2 |
| 1.6x more quads everywhere* | yes (27 verts) | 67.5 |
| denser groin and armpits (x4 zones)* | yes, fix refused | 41.6 |
| `--skeleton` (groin edges 1.2-1.3x longer) | 1 of 3 (3 verts) | 75.7-78.0 |

(*one run each, on the earlier Python SkinTokens runtime, whose
default-topology runs scored the same 69-73.)

More rings at the groin made it worse; wider, even rings fixed it.
Coarsening every limb root (armpits too) moved the failure into
collapsed arm volume and cost a test mannequin 6 points, so the rule
only sizes the crease between **sibling limbs** (two legs under one
pelvis, four legs under one body), where one limb swings against a
still neighbour:

1. For each bone whose parent has another child, slice the mesh
   perpendicular to the bone, stepping from its pivot toward its tail,
   until the loop around the bone holds no surface nearer to a sibling
   limb than to this one. That distance is the crease depth (none when
   the limb separates within its own radius of the pivot, or never).
2. The crease asks for edges no shorter than
   `bend * depth / ((2.5 - 1) * 4)` (radians; 2.5 = the gate's per-edge
   stretch, 4 = blend rings). Where the nominal quad edge
   (`sqrt(area / target)`) is shorter, a sphere around the pivot
   (radius 2x the depth, full strength over the inner half) gets
   density `(nominal / wanted)^2`, floored at 0.25. The density
   pipeline keeps the total quad budget.

Everything comes from bone lengths, the hierarchy, bend ranges and the
mesh; nothing is in units or tuned per character. On the weightforge
mannequin (legs leave the body at the hip) no crease qualifies and the
output is byte-identical; on a rigged quadruped the hind legs ask for
x0.77 and its same-rig scores stay level (31.4-33.8 off, 31.8-34.7 on).
`--verbose` lists each crease and its ask. The Blender extension writes
the file from the object's armature (Density panel, *Skeleton Density*;
bend ranges from bone-name roles, 45 degrees otherwise) and genforge's
repair-topology adapter turns it on. Limits: limb-to-trunk creases
(armpits) are left alone, and the humanoid still fails one small arm
finding in each run (arm_forward volume or arm_up stretch, 4-10 verts);
that is a weights or corrective-shape job, not topology.

## Layout

- `rust/` — the product: `retopo_core` + `retopo_solvers` libraries and
  the `retopo` CLI binary (`rust/cli`), ported from the original C++ by
  differential oracles (see `docs/rust-switch-verdict.md`).
- `Formula/` — Homebrew formula for the CLI.
- `blender/` — Blender extension (`blender/retopoforge/`) driving the
  CLI over a temp-OBJ round-trip, with a headless test in
  `blender/tests/`.
- `bench/` — harness (`bench/run.py`, models/results gitignored,
  `bench/baseline.json` committed), the `bench/profile.py` profiler,
  the `bench/score.py` quality scorecard, and the `bench/noise.py`
  noise-floor harness.
- `docs/` — architecture, engine-vs-Exoside gap, perf profile, and LOD
  strategy notes.
- `tests/fixtures/` — golden data for the Rust tests (reference dumps,
  procedural OBJ/GLB fixtures, CLI contract goldens).

See [docs/architecture.md](docs/architecture.md) for the module graph
and the engine/CLI/addon split.

## Direction

1. Headless engine + CLI + benchmarks (this fork's foundation, done)
2. Blender addon driving the CLI (done, extension v0.3.0)
3. Rust rewrite of the engine + CLI, proven by differential oracles
   (done — shipped in 0.3.0, C++ tree removed afterwards, see
   [docs/rust-switch-verdict.md](docs/rust-switch-verdict.md))
4. Post-switch superiority: sizing-aware rounding, single-island
   parallelism, CLI UX, and the 6-7x perf gap — then incremental
   engine improvements toward Exoside parity (see
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
- Upstream authors and contributors are credited in the upstream
  repository; third-party licenses:
  [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)
