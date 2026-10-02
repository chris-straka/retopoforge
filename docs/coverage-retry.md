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

On failure the island re-runs parameterize+extract over deterministically
jittered vertices (seeds 1..3, SplitMix64 pattern keyed on position bits
so weld-duplicates move together, amplitude 1e-3 of the island diagonal)
and keeps the **first full-coverage result**. When no retry covers fully,
attempt 0 is kept (never a partial retry). Empty outputs never retry
(the failed-island path owns them). Outcomes surface via
`AutoRemesher::coverage_reports()` and a `Warning:` stderr line in the
failed-island style.

Jitter steers only the parameterization: the extractor always embeds in
the unjittered working mesh (identical input on attempt 0), so a
recovered output carries no jitter displacement — median
output-to-working distance is 2.2e-15 on beast@1000-native (vs 5.6e-2
with jittered embedding), pinned by `coverage_retry.rs`. The coverage
verdict likewise measures against the unjittered mesh every attempt.

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

## The unrecovered suite case (diff id 58, island 2)

The only suite island that fires and cannot recover: a triangle-soup
fragment (`EPX 58 KIND soup`, 14 verts / 3 tris) whose island 2
resamples to a thin flat slab (437 verts, 826 tris, x 3.2 x y 0.43 x
z 3.4, zero singularities). Attempt 0 traces 80 quads over part of it
(output spans x[-1.05,0.28] z[0.22,1.70] of working x[-1.31,1.88]
z[-1.36,1.99]); 69 verts sit beyond 3 widths. Its uv layout collapses
in v (a 19 x 0.5 strip, 251/826 near-zero-area uvs), so extraction has
no full cover to trace.

All three retries fail worse (quads 101/131/119, c3 230/235/192, frac
0.55-0.62 vs attempt 0's 0.45): the collapse is structural, not a
knife-edge fold — 1e-3 jitter re-rolls within the same degenerate
layout family (more fragmented tracing, less coverage), never near
the bar. Fallback-0 keeps attempt 0, so the committed golden is
byte-stable; the diff test pins the exact report (island 2, 3
retries, unrecovered, 69/69) and fails loudly if any other case ever
fires. Islands 0/1 of the same case (frac 0.60/0.31, c3 = 0) show the
25-vert floor correctly quieting scattered spikes on tiny islands.

## Notes for later work

 - Stage dumps (`RETOPO_DUMP_STAGES`) always show attempt 0; per-attempt
   coverage lands in `coverage.log` (`attempt=` field).
 - The retry re-rolls the tiling; it does not fix the fold. The beast
   case stays a target for the patch back end (item 10): a patch-based
   extractor should cover the appendage on the first attempt.
 - Item 7 (untangling) is unaffected: the fold is globally consistent
   (locally valid), outside a flip barrier's reach.
