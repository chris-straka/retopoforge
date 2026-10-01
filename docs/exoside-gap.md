# Exoside Quad Remesher gap analysis

Date: 2026-09-29. Our-side inputs read for this doc: the CLI usage
(`cli/main.cpp`, `--help` text and defaults), the Blender panel
(`blender/retopoforge/__init__.py`, extension v0.1.0), and the TODO roadmap
(`TODO.md`). No build was run (research-only task).

Their-side inputs: exoside.com pages (overview, buy, doc/FAQ, download),
the QuadRemesher 1.2 UserDoc PDF, the What's New PDF (covers 1.1–1.4),
the EULA text, plus press and forum reviews. Every non-obvious claim below
carries its source URL. Anything I could not verify is marked UNVERIFIED.

Version note: the FAQ page links a "1.3 User Documentation" PDF, but the
file served at that URL is currently byte-identical (same MD5) to the 1.2
UserDoc, so all parameter documentation below is cited to the 1.2 doc.
Latest release I could verify is 1.4 (July 2025) from the What's New PDF.

## 1. Feature set

They have: one-click quad remesh of any tri/poly input mesh into a
quad-dominant mesh with a user-specified target count
([user doc B.1–B.2](https://exoside.com/quadremesherdata/QuadRemesher_1.2_UserDoc.pdf)).
Beyond the core remesh: curvature-adaptive quad sizing with an
adaptive-count toggle, vertex-color density painting (0.25x–4x local
density), a whole edge-loop guide system (material boundaries, smoothing
groups, harden/soften edges, normals splitting/creasing, automatic
hard-edge detection by angle, polygon/primitive-group boundaries, BRep
edges/faces in Fusion 360), symmetry options, Keep-Materials in the
retopo object, FaceSets-to-Materials conversion, undo/redo, abortable
runs with progress bar, and (Fusion 360 only) BRep/T-Spline reverse-
engineering output plus a quad-preserving .obj export button
([user doc B.3–B.4](https://exoside.com/quadremesherdata/QuadRemesher_1.2_UserDoc.pdf),
[What's New PDF](https://exoside.com/quadremesherdata/QuadRemesher_WhatsNew.pdf)).
User-drawn guide curves and edge selection sets were listed as "WIP, not
in 1.0" in the user doc and are not mentioned in any 1.1–1.4 What's New
entry, so they are presumably still absent (UNVERIFIED for 1.4).

We have: the same core loop (tri/quad-soup OBJ in, quad-dominant OBJ
out) behind two front ends: headless `retopo` CLI (`cli/main.cpp`) and
the Blender extension driving the CLI over a temp-OBJ round-trip
(`blender/retopoforge/__init__.py`, `blender_manifest.toml`). (The
upstream Qt desktop shell was removed 2026-09-30; Blender is the UI.)
Engine lineage is the AutoRemesher stack (frame field + QuadCover-style
parameterization; the upstream changelog notes "Replace MIQ with QuadCover").

Gap: their headline differentiators we lack entirely are the guide
system (especially auto hard-edge detection and normals-creasing
guides), symmetry, local density painting, and material preservation.
Difficulty: guides are the hard one (L–XL: needs feature-constrained
parameterization plumbed from CLI flag through engine; our TODO already
notes the Blender sharp-edge-marks idea "needs a CLI `--features` input
flag first (engine change)"). Symmetry is M (detect plane + constrain
solve). Density painting is M (per-vertex sizing field input; engine
already takes adaptivity/anisotropy scalars). Keep-materials is S–M
(mostly Blender round-trip work once the CLI can carry face labels).

## 2. Parameters and presets

They have: Target Quad Count (approximate, not exact), Adaptive Size
0–100% (default 50%), Adaptive Quad Count on/off (off = respect the
count, on = count defines flat-area size and curves get extra quads),
density-paint slider 0.25–4, Detect-Hard-Edges-by-angle toggle
(detects edges above ~30°), plus the per-source guide toggles above
([user doc B.2–B.4](https://exoside.com/quadremesherdata/QuadRemesher_1.2_UserDoc.pdf)).
No named presets (organic/hard-surface/etc.) appear anywhere in the
user doc or What's New. Max TargetQuadCount is 4M on 3ds Max (raised
from 600K in 1.1; other hosts UNVERIFIED)
([What's New PDF](https://exoside.com/quadremesherdata/QuadRemesher_WhatsNew.pdf)).

We have: 7 parameters — `--target-quads` (default 50000),
`--edge-scaling` (1.0–4.0), `--sharp-edge` (default 90°, 30–180°),
`--smooth-normal` (default 0°, 0–180°), `--adaptivity` and
`--anisotropy` (0–1), `--model-type organic|hardsurface`
(`cli/main.cpp:45-80`), all mirrored 1:1 in the Blender panel
(`blender/retopoforge/__init__.py:68-134`, `:392-427`). The
organic/hardsurface model type is effectively a two-preset system,
which is more preset surface than they ship.

Gap: small and mostly in our favor on presets. Missing knobs that
matter: an adaptive-count mode (their count-vs-quality toggle) and a
hard-edge angle detector (our `--sharp-edge` is a threshold input to
the solve, but there is no auto-detected guide loop output; verify
against engine behavior before claiming parity). Their 4M max target
vs our unbounded int is not a real gap. Difficulty: adaptive-count
semantics S–M (post-solve scaling decision); auto hard-edge guides L
(same engine work as section 1).

## 3. Input/output formats

They have: no file I/O of their own — each plugin remeshes the host
DCC's in-memory mesh (any triangle/polygon mix, including sculpts and
scan data; quads out)
([CG Channel](https://www.cgchannel.com/2019/10/check-out-neat-automatic-retopology-tool-quadremesher/)).
So their "format support" is whatever the host imports/exports (FBX,
OBJ, Alembic, …). The one file-format feature is Fusion 360's .obj
export button that preserves quads as quads (1.3)
([What's New PDF](https://exoside.com/quadremesherdata/QuadRemesher_WhatsNew.pdf)).

We have: OBJ in, OBJ out, everywhere — CLI (`cli/main.cpp:63-64`),
Blender extension round-trips temp OBJs
(`blender/retopoforge/__init__.py:167-180`).
Known gaps already in our TODO: no weld-on-load (AI triangle soup
exploded into 3684 islands on a test mesh), noisy progress on soup
input, silent island drops with exit 0.

Gap: through a DCC they accept anything; standalone we accept only
OBJ, and hostile OBJ at that (unwelded soup). Difficulty: weld-on-
load + degenerate-tri dropping is S (meshopt remap in loader, already
scoped in TODO). Additional formats (STL/PLY/glTF) are S each via a
small parser but low value while Blender-first (the addon inherits
Blender's importers for free). Whether they handle non-indexed soup
better than us is UNVERIFIED (their doc only warns that extreme noise
and degenerate triangles can disturb hard-edge detection).

## 4. DCC integrations

They have: 3ds Max (2016+), Maya (2012+, not LT), Blender (2.8–4.x),
Modo (10–16), Houdini (17+), Fusion 360, Cinema 4D R14–2023 (legacy
users only after the C4D R26 dispute), Nomad Sculpt (desktop + iPad,
iPad sold via App Store), and Shapelab Max (covered by the ALL-
softwares license); Windows + macOS everywhere, Linux for
Maya/Blender/Modo/Houdini; plugins "designed to be compatible with
all future versions of the host softwares without upgrading"
([FAQ](https://exoside.com/quadremesher/quadremesher-doc/),
[overview](https://exoside.com/quadremesher/),
[buy page](https://exoside.com/quadremesher/quadremesher-buy/)).

We have: Blender 4.2+/5.x extension v0.1.0 (subprocess + temp OBJ,
identity-transform export, evaluated-mesh toggle, multi-object loop,
modal operator with progress + ESC cancel, stats report, explicit
UV/vertex-color loss notice —
`blender/retopoforge/__init__.py:190-427`) and the headless CLI.
macOS-only build scope per `README.md`.

Gap: breadth (we have 1 DCC + CLI; they have 8+ hosts).
But depth-per-host matters more for our owner: our Blender addon
already does things theirs does not advertise — multi-object batch
(their doc requires selecting "one and only-one mesh",
[user doc B.1](https://exoside.com/quadremesherdata/QuadRemesher_1.2_UserDoc.pdf))
and replace-in-place in one undo step (theirs hides the source and
spawns a new object). Difficulty: each new DCC is M–L (new plugin
scaffold + host API quirks + testing matrix); not recommended while
Blender-first. Houdini/CLI parity is the closest fit if ever needed
since our headless CLI already exists.

## 5. Output-quality claims

They claim: "clean all-quad meshes", "evenly spaced quads suitable
for animation", from sculpts or raw scan data
([CG Channel](https://www.cgchannel.com/2019/10/check-out-neat-automatic-retopology-tool-quadremesher/));
one-click auto-retopo examples (booleans, text, skull) on the
[overview page](https://exoside.com/quadremesher/). Pedigree: author
Maxime Rouca "developed the technology behind ZRemesher … along with
… Decimation Master"
([CG Channel](https://www.cgchannel.com/2019/10/check-out-neat-automatic-retopology-tool-quadremesher/)).
Independent reviewers consistently call it the best Blender auto-
retopo: "indisputably better than Quadriflow"
([BlenderArtists](https://blenderartists.org/t/quad-remesher-auto-retopologizer/1170913?page=6)),
"the best auto-retopo tool you will find for Blender"
([BlenderArtists](https://blenderartists.org/t/quad-remesher-auto-retopologizer/1170913?page=27)),
"the resulting mesh … needs the least amount of manual work … usable
out of the box"
([BlenderArtists](https://blenderartists.org/t/the-big-blender-sculpt-mode-thread-part-1/1150731?page=470)),
"turns a dense or triangle-heavy mesh into clean quads"
([Gachoki 2026 review](https://gachoki.com/best-blender-addons-for-retopology/)).
No numeric quality metrics (edge-flow scores, distortion numbers)
are published anywhere I could find.

We claim: quad-dominant output with honest quad/non-quad/vertex
counts in every CLI report (`cli/main.cpp:346-353`), a committed
`bench/baseline.json` regression gate, and mesh validation in
`bench/run.py`. No head-to-head comparison against QuadRemesher
exists in our tree.

Gap: unmeasured. Their quality reputation rests on the ZRemesher
lineage plus six years of user consensus, not numbers — and we have
neither numbers nor a comparison. Difficulty: running a blind
comparison on our bench models + the Andras game meshes is S (we
would need a $15.99 subscription or trial for the reference outputs;
note the EULA bans using outputs for competing-product development
and ML training — see section 8 — so get the comparison protocol
right before buying). Closing a measured quality gap is L–XL
depending on what the numbers say; do not guess before measuring.

## 6. Performance claims

They claim: no published benchmarks, timings, or poly-throughput
figures anywhere on exoside.com or in either PDF (verified by
reading). What's New cites only relative speedups: 4–5% faster
engine in 1.1, "improved execution speed for big meshes" in 1.2,
"improve speed / memory usage on some heavy meshes" in 1.4
([What's New PDF](https://exoside.com/quadremesherdata/QuadRemesher_WhatsNew.pdf)).
A 2026 review characterizes it as "seconds rather than minutes"
([Gachoki](https://gachoki.com/best-blender-addons-for-retopology/)).
Indirect scale signal: 4M max target quads on 3ds Max (1.1). No
public statement on threading/multicore use (UNVERIFIED).

We claim: multithreaded engine (TBB dependency), per-run wall time
in the CLI report (`cli/main.cpp:352`), bench harness timing. Our
TODO admits the signal is weak: "Bigger bench models for a real
perf signal" and "profile the CLI on production-size meshes, check
TBB scaling" are both open.

Gap: unmeasured, same as quality. Difficulty: S to get a real
signal (bigger bench models + profile, already TODO items); M–L to
fix whatever profiling finds. Do not claim perf parity either way
until both sides run the same meshes on the same machine.

## 7. UX

They have: in-DCC panel, single REMESH IT button, one mesh per run,
source hidden + new object spawned, progress bar, abort, online-help
and version-check buttons (Blender 1.3), standard per-host install
(drag-drop mzp/lpk/zip, Fusion installer), email + license-key
activation with offline fallback and a license manager for
deactivate/move
([user doc A–B](https://exoside.com/quadremesherdata/QuadRemesher_1.2_UserDoc.pdf),
[What's New PDF](https://exoside.com/quadremesherdata/QuadRemesher_WhatsNew.pdf)).

We have: Blender panel mirroring all 7 params with binary-found
status, multi-object loop, modal non-freezing operator with ESC
cancel, per-object stats, replace-in-undo-step or spawn-copy modes,
honest data-loss notice
(`blender/retopoforge/__init__.py:392-427`); CLI with `--help`/`--version`,
progress %, and report file. Install is Homebrew formula +
build-from-source + extension-zip-from-disk; no activation at all.

Gap: near parity on Blender UX, with different trade-offs (we batch
+ replace; they guide + keep materials). Genuine UX gaps: no
in-Blender density painting or guide toggles (blocked on engine
features, not UI), no progress % inside Blender during a run (we
show current-object counts only), release packaging/zip-install path
unverified (open TODO). Difficulty: S for the UI-only items once
engine features land; packaging verification S.

## 8. Pricing/licensing

They charge (VAT excluded, via reseller Verifone/2Checkout): per-
DCC perpetual Pro $109.90, perpetual Indie (non-commercial) $59.90,
subscription $15.99/3 months; ALL-softwares (excl. CAD) Pro $139.90
/ sub $22.99; CAD (Fusion 360) Pro $199.90 / Indie $129.90 / sub
$29.90; ALL incl. CAD Pro $239.90 / Indie $159.90 / sub $39.90;
30-day non-commercial trial; upgrades via an upgrade page
([buy page](https://exoside.com/quadremesher/quadremesher-buy/),
[download page](https://exoside.com/quadremesher/quadremesher-download/)).
License terms: single-computer activation plus a second home/
portable copy (not simultaneous); Indie barred above $100k revenue,
4+ people, or $150k raised in 5 years; no refunds after activation;
no reverse engineering; no competing-product development; no ML/AI
training on meshes produced with the tool; no SaaS/hosting use;
SDK use needs a separate deal
([EULA](https://www.exoside.com/quadremesherdata/QuadRemesher_EULA.txt),
[FAQ](https://exoside.com/quadremesher/quadremesher-doc/)).

We charge: nothing. MIT engine + GPL Blender extension, no
activation, fully scriptable and CI-safe.

Gap: none — this is our structural advantage. The EULA's
competing-product and ML-training clauses are the one caution: any
benchmark-against-QR work must be a clean black-box comparison, and
QR outputs must stay out of training data and out of our repo.

## 9. Ordered gap list (highest user value first)

Ranked for our owner: solo-dev Godot game, AI image-to-3D organic
meshes → clean quad characters, Blender-first. "Value" = closer to
shippable game assets per unit of effort.

1. Robustness on nasty inputs (weld-on-load, degenerate-tri drop,
   loud island accounting, `--quiet`) — already TODO-scoped, S.
   Unblocks the owner's actual meshes; value is immediate.
2. Measured quality comparison vs QR on bench + Andras meshes —
   S (+$16 sub or trial). Unblocks every later quality decision;
   do this before any L/XL engine bet. Mind the EULA clauses.
3. Guide system, phase 1: auto hard-edge detection by angle +
   normals-creasing guides — L–XL. The core QR differentiator for
   anything with edges; phase 2 (material/smoothing-group guides)
   after.
4. Symmetry support — M. High value for characters; reviewers
   single out symmetry handling as a QR-vs-QuadriFlow divider
   ([BlenderArtists](https://blenderartists.org/t/quad-remesher-auto-retopologizer/1170913?page=27)).
5. Local density control (paint or vertex-group driven) — M. Matters
   for face/hands detail vs body on game characters.
6. Attribute survival: remeshed UVs + material slots through the
   round-trip — M–L. QR keeps materials (1.21) but I found no
   source saying it emits UVs (UNVERIFIED) — UV output may be a
   leapfrog, not a catch-up. Our TODO already wants it.
7. Adaptive-count (quality-priority) mode — S–M. Cheap once density
   control exists.
8. Perf at scale: bigger bench models, profile, TBB scaling check
   — S to measure (already TODO), M–L to fix.
9. Blender UX finish: live progress %, verified zip-install path,
   release packaging — S total.
10. DCC breadth beyond Blender — M–L each. Last; only if a paying
    need appears. Our headless CLI is already the seed of a
    Houdini-style integration.

Anti-gaps (do not "fix"): free/OSS licensing, headless CLI + CI
harness, multi-object batch, replace-in-place undo, two model-type
presets. These are leads to keep.
