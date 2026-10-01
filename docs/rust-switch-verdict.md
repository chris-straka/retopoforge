# Rust switch verdict: viable, landed

The strangler-fig rewrite is complete. All of `core/` + `cli/` now runs
as Rust (`retopo_core`, `retopo_solvers`, `retopo` binary), proven against
the C++ by differential oracles at every level, and the Rust binary is the
shipped product (Homebrew 0.3.0, Blender add-on default). The C++ tree is
frozen in place as the oracle reference — it still builds and the e2e
suite runs it on every `cargo test`.

## Numbers

- 20/20 modules ported (18 core + 2 solvers) + glb + CLI; 60 Rust test
  suites green, `cargo fmt --check` clean, zero warnings.
- Acceptance gate (`rust/cli/tests/e2e_diff.rs`): 50 arg-matrix cases
  byte-exact, 20 IO-failure paths, 37 remesh cases over a flag×mesh
  matrix with adaptive C++ census (N=5→30) — green 3× consecutive.
- Timing (grid32 target-2000): Rust release ~700ms vs C++ Release
  ~113ms (~6-7x). Threading model (scoped threads + serial faer vs
  TBB + Accelerate), not algorithmic. Recorded, not a gate.

## How equality was proved (and where it bends)

- Bitwise where IEEE-deterministic (vectors, isoremesher, singularity,
  quad extractor incl. a libc++ hash-table emulation).
- Scale-aware 1e-6 + zero structural mismatches where float order
  legitimately varies (solvers, parameterizer, frame field).
- Robustness-only tiers with demonstrated mechanisms where backend
  noise flips integer decisions (CLS.../QPX/FFX/PPX at module level;
  CASE/ECX/EPX at engine level; strict/tol/any-mode/EPX at e2e).
- Two C++ bugs found by the port, both EPX-by-UB, never matched:
  heap-OOB reads on DENSITY-3 inputs (cases 41/156 class).
- C++ engine self-nondeterminism is pervasive (TBB wobble flips
  extractor cliffs run-to-run; 122/295 engine cases flip counts).
  The e2e harness censuses C++ and accepts Rust matching ANY
  demonstrated mode; Rust itself is verified deterministic.

## Known asterisks (all documented, none silent)

- meshoptimizer stays C++ behind a small audited FFI (`build.rs` + `cc`;
  builds offline from cache, system-clang fallback). Porting the
  decimator is all risk for no gain.
- Stderr tells a new, specced story: duplicate phase dumps and
  unplumbable module one-liners dropped (content preserved); exits
  and failure behavior identical. Full audit in the e2e report.
- Specified divergences: hex-float/`nan(payload)` numerics (both sides
  exit nonzero), lossy non-UTF8 argv, `RETOPO_VERSION` still 0.1.0 in
  both binaries (pre-existing wart, packaging is 0.3.0).
- One systematic path divergence with identical output: Rust's
  extractor skips five-merges C++ takes (30/30 vs 0/777 diagnostic).
  Same mesh today; post-switch diagnosis item (TODO).
- Linux Rust path (`sincos` link) compiles by cfg but has no CI cover;
  product is Mac-only. The C++ sanitize job is unchanged.

## What this unlocks (post-switch superiority batch)

Equality was scaffolding. With the oracles as the regression net:
sizing-aware MILS rounding driven by the flip maps, single-island
parallelism in Rust, CLI UX redesign, and the 6-7x perf gap.
