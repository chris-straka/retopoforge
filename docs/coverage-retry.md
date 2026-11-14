# Coverage check + deterministic retry

Companion defect: `docs/beast-knife-edge-bisection.md`
(beast@1000-native drops the whole left appendage through a
region-scale uv fold)

## What

After extraction, every island checks its working verts against the
extracted quads. An island **fails coverage** when a region-scale part
of the working mesh sits far from any quad:

 - distance bar: 3x the nominal quad width (`diag/sqrt(nquads)`)
 - region bar: >= 25 working verts beyond it
(`COVERAGE_WIDTH_MULTIPLE`, `COVERAGE_MIN_REGION_VERTS`)
 - per-region bar: largest working-triangle-connected patch beyond it
holds >= 10 verts (`COVERAGE_MIN_PATCH_VERTS`) — catches thin
dropped features the count floor misses (a 16-vert claw tip with
only 16 beyond the bar trips it; the 25-floor stays quiet)

The verdict has a second side: the ORIGINAL input verts vs the same
output (`input_coverage_failed`). Same 3-width bar, but
connectivity-only — no anywhere count floor: input noise
legitimately strands isolated verts beyond the bar (dragon: 123
singletons at every bar), while genuine drops are connected. The
input side catches extremities the working mesh keeps only as
stretched-triangle surface (sparse working verts the working-side
check cannot see); it runs through `CoverageIndex` (grid over 150k
input verts at ms-scale, exact linear scan under 256 fan tris) and
is skipped when working-side already failed (the attempt retries
regardless) unless `RETOPO_COVERAGE_LOG` is set. The firing side is
pinned in `CoverageReport.input_side` (`input verts` vs `working
verts` in the `Warning:` line). Termination guards: build routes to
scan unless `h > bar/64` (ring exit then fires at `r < 64`), and
grid build bails to scan past 1M cells — a repeated-corner output
collapses h toward 0, which hung `differential_replay` (25+ min)
before the guard; both arms test the same predicate, so Grid ===
Scan === brute force, pinned by the routing + agreement lib tests.

On failure the island re-runs parameterize+extract over deterministically
jittered vertices (seeds 1..3, SplitMix64 pattern keyed on position bits
so weld-duplicates move together, amplitude 1e-3 of the island diagonal)
and keeps the **first full-coverage result** (both sides quiet). When no
retry covers fully, the earliest working-quiet attempt is kept
(`kept_attempt`; attempt 0 when it passed working-side) — the input
side can upgrade the winner but never downgrade working-side coverage
below unretried. Empty outputs never retry (the failed-island path
owns them). Outcomes surface via
`AutoRemesher::coverage_reports()` and a `Warning:` stderr line in the
failed-island style.

Jitter steers only the parameterization: the extractor always embeds in
the unjittered working mesh (identical input on attempt 0), so a
recovered output carries no jitter displacement — median
output-to-working distance is 2.2e-15 on beast@1000-native (vs 5.6e-2
with jittered embedding), pinned by `coverage_retry.rs`. The coverage
verdict likewise measures against the unjittered mesh every attempt.

## Finer re-run and the extremity check (CLI default)

Jitter steers the parameterization, but a thin separate part (a rib,
a key's bow, a ring) whose quads are wider than the part itself
collapses on every seed: the artist bench's ribcage kept 6-10 quad
boxes for 9 of its 24 ribs. So after the retries, an island that is
still short, or that came out empty, and that is at most
`FINER_RERUN_MAX_SHARE` (half) of the mesh's area, is remeshed again
as its own `AutoRemesher` from its input triangles (same settings, no
symmetry, no odd-face pairing, no re-run of its own). Targets, cheapest
first: `FINER_RERUN_DENSITY` = 3x, then 6x, its area share of the
target, then, for a closed part whose tube radius (2 x volume / area)
needs more, what keeps `FINER_RERUN_AROUND` = 4 square quads round it;
each capped at the island's input triangle count. A thin plate (a
blade) also reads as a thin tube, which is why the tube size comes
last. The engine's yield varies by shape (hard pieces up to 2.5x the
target), so a covered re-run that overshoots its ask by more than
`FINER_RERUN_MAX_OVERSHOOT` (1.25x) runs once more aimed at the miss,
kept when it still covers. The first covered run replaces the island
(`CoverageReport.recovered`, `finer_rerun` = its multiple of the fair
share). When none covers, the candidate (the kept attempt or a re-run)
with the smallest mean distance from the island's input vertices to its
quads is kept: each coverage verdict is relative to its own quad width,
so only an absolute measure compares runs of different density. The
key's body showed why: cut alone from a 7-digit OBJ it covers, inside
the whole key at full float precision it collapses to a 6-quad box.

The extremity check, on with the re-run, adds one failure to the
verdict (folded into the input side): input vertices sticking out past
the output by more than `EXTENT_WIDTH_MULTIPLE` (0.5) nominal quad
widths along any of 13 directions (axes, face and body diagonals, both
ways). The 3-width bars cannot see an end rounded off by about a quad
(the femur's ball: 6% of its length gone). On the artist bench's floor
outputs every case with a lost end receded 0.58-2.1 widths, every
healthy organic case 0.02-0.25. A whole-mesh island that fails it gets
the jittered retries only (more quads there would break the budget).

The library default is off (the original engine, and the differential
oracles); the CLI turns both on (`--no-finer-rerun` turns them off).
The CLI's count-matching second run never replaces the first when it
drops a part the first kept or adds a coverage shortfall.

## Why resolution-relative + count (calibration)

Absolute bars fail both ways on the measured data:

 - worst-gap/diag: healthy tiny outputs reach 0.097 (dragon@1000) while
   the failure sits at 0.307 — a 0.10 bar has 3% margin and noisy seeds
   cross it (armadillo seed: 0.175 with dist_max 17.7%).
 - fraction beyond 10% diag: healthy unjittered dragon@1000 holds 7.7%
   of verts beyond 5%; a (10%, 1%) bar fires on 330/359 suite
   island-runs (ultra-coarse committed fixtures deviate ~1 quad width,
   healthy for their resolution).

Normalizing by the nominal quad width separates cleanly (c3 = verts
beyond 3 widths):

 | set | islands | max c3 | fires (>= 25) |
 | bench 10 (unjittered) | 10 | 0 | 0 |
 | suite (diff+golden) | 359 | 69 | 1 (retries fail 230/235/192 -> attempt 0 kept, goldens unchanged) |
 | default tiny noise seeds | 24 | 80 | 1 |
 | native beast@1000 seeds | 8 | 156 | 7 (c3 67-156; covered seed: 0) |

## Why amplitude 1e-3 (native-beast unjittered, per-attempt c3)

 | amplitude | attempts 0-3 c3 | outcome |
 | 1e-9 (noise floor) | 155/155/155/155 | no recovery (attempt 1 repeats attempt 0's counts) |
 | 1e-6 | 155/155/155/144 | no recovery |
 | 1e-4 | 155/155/155/0 | recovers on seed 3; but 7/8 seeds (partial fold resists: 67 -> 58/69/75) |
 | 1e-3 | 155/0 | 8/8 recover on the first retry, exact-zero residuals |

Jitter lands post-decimation (all snaps), so it must be big enough to
move uv rounding: recovered outputs sit inside the healthy tiling
spread (dist_mean 0.82-1.09 vs 0.68-1.01 unretried, all dist_max < 8.2
vs 13.5-30.6 dropped).

## Gates (all green at landing)

 - beast@1000-native: appendage recovered on all 8 noise seeds
 - bench 10 unjittered: byte-identical pre/post (`cmp`, no retry fires)
 - `bench/run.py --check`: no regressions
 - full `cargo test --release`: 62 ok, zero warnings; CLI contract green
 - regression: `rust/core/tests/coverage_retry.rs`
   (beast@1000-native: retry fires, recovers, output min_x < -100;
   skips loudly without the corpus; proven to fail with retries off).
   The firing suite fixture needs no new test: its golden implicitly
   pins fallback-0 (keeping a partial retry would change the output).

## Per-region bar calibration (small-feature fixture)

`body_with_claw` (in-module procedural fixture: 3x3x3 body, thin
0.5x0.5 claw stitched into one island): the pipeline covers the claw
(no retry fires on the real outputs), so drops are simulated by
clipping quads over the claw. A protrusion-3.0 clip leaves 16
connected tip verts beyond 3 widths — under the 25-floor (miss),
caught by the 10-patch bar. A protrusion-2.0 clip leaves 2 (worst gap
2.1 widths) — quiet by design: sub-bar deviations are fidelity
(healthy dragon deviates 2.5 widths), not coverage; coverage catches
missing parts, `score.py` dist and item 9 catch fidelity.

Rejected alternatives (measured): 2-width bars fire all over the
suite's ultra-coarse fixtures (coherent whole-region deviation);
gap cliffs confuse healthy spikes (2.4) with short drops (~2.2);
1-width patch aspect merges claw with body deviations (1.27).
Largest connected beyond-3w patch: 0 on bench 10, <= 3 on the
suite outside firing cases, <= 2 on noise seeds outside firing
cases — vs 10+ on every genuine small drop (claw tip 16, grid 11,
suite grid case 24, noisy seeds 10-16, all recovered). Bar at 10
clears both sides. (Measure patch size over UNSORTED gaps; an
early probe sorted first and reported garbage — the verdict always
used vertex order.)

## The unrecovered suite case (diff id 58, island 2) — retired

This case used to be the only suite island that fires and cannot
recover: a triangle-soup fragment (`EPX 58 KIND soup`, 14 verts / 3
tris, 3 sharps) whose island 2 resampled to a thin flat slab (437
verts, 826 tris, zero singularities). Attempt 0 traced 80 quads over
part of it; 69 verts sat beyond 3 widths; the uv layout collapsed in
v (a 19 x 0.5 strip, 251/826 near-zero-area uvs). All three retries
failed worse (quads 101/131/119) — the collapse looked structural.

The corner-mark cage fix (`docs/corner-marks.md`) revealed the real
cause: the 3 explicit sharps pinned integer coordinates across the
slab, over-constraining the cover into the collapsed layout. With
alignment-only marks the first attempt covers and the case stays
retry-free (pinned explicitly in `auto_remesher_diff.rs`). Lesson:
"structural, unrecoverable" meant "pinned by its own sharps". Case 99
still fires and now recovers fully (12 -> 0 uncovered, was 24 -> 11).

## Input-side calibration (thin-claw fixture)

Seed-0 input-side probe (input verts vs output, 3-width bar): bench
worst-healthy is dragon scatter (123/391 beyond, all isolated
singletons at 3w — hence connectivity-only); armadillo's fingertip
miss sits at 0.93x (sub-bar downstream fidelity, not a drop — the
working mesh covers it as stretched-triangle surface 1.3u away).
Beast/nefertiti/fandisk worst 0.3-0.45x. Nothing fires at seed 0;
on noise seeds armadillo@1000 tilings strand real input patches and
the verdict fires (working-side first where both fail).

`thin_claw_fires_input_side_and_recovers` (`coverage_retry.rs`,
procedural, no corpus): dense 0.25-wide claw at 50 quads — decimation
thins it past working-side visibility while 49 connected input verts
strand; input-side-only fire, retry 2 re-passes with zero residuals.
Recovery at 50 quads is bar-relative (the claw sits below resolving
power), documented in the test. Index-vs-brute-force agreement is
pinned by `input_side_matches_brute_force`, with the Grid/Scan
routing pinned by `grid_path_matches_brute_force` and
`degenerate_output_scans_and_matches_brute_force`.

Fallback-chain proof: without the earliest-working-quiet rule the
input side vetoes working-side winners (armadillo@1000 seeds 1/2/5/7
kept attempt 0 with 13 working verts out); with it those seeds keep
their pre-input-side winners byte-identically while reporting the
input shortfall (`kept attempt N (working-side cover; ...)`).

## Notes for later work

 - Stage dumps (`RETOPO_DUMP_STAGES`) always show attempt 0; per-attempt
   coverage lands in `coverage.log` (`attempt=` field).
 - The retry re-rolls the tiling; it does not fix the fold. The beast
   case stays a target for the patch back end (item 10): a patch-based
   extractor should cover the appendage on the first attempt.
 - Item 7 (untangling) is unaffected: the fold is globally consistent
   (locally valid), outside a flip barrier's reach.
