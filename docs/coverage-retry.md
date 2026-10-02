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

 On failure the island re-runs parameterize+extract over deterministically
 jittered vertices (seeds 1..3, SplitMix64 pattern keyed on position bits
 so weld-duplicates move together, amplitude 1e-3 of the island diagonal)
 and keeps the **first full-coverage result**. When no retry covers fully,
 attempt 0 is kept (never a partial retry). Empty outputs never retry
 (the failed-island path owns them). Outcomes surface via
 `AutoRemesher::coverage_reports()` and a `Warning:` stderr line in the
 failed-island style.

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

## Notes for later work

 - Stage dumps (`RETOPO_DUMP_STAGES`) always show attempt 0; per-attempt
   coverage lands in `coverage.log` (`attempt=` field).
 - The retry re-rolls the tiling; it does not fix the fold. The beast
   case stays a target for the patch back end (item 10): a patch-based
   extractor should cover the appendage on the first attempt.
 - Item 7 (untangling) is unaffected: the fold is globally consistent
   (locally valid), outside a flip barrier's reach.
