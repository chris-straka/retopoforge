# Rust port conventions (lane contract)

Strangler-fig port of `core/` to the `rust/` workspace. Each lane ports its
modules and proves them with a differential oracle. This doc is the contract;
`docs/rust-solvers-experiment.md` is the worked example (solvers wave).

## Layout

- `rust/` workspace: `solvers` (`retopo_solvers`, done), `core`
  (`retopo_core`, this port).
- One submodule per C++ module, named after the C++ file snake_cased:
  `core/objreader.*` -> `retopo_core::obj_reader`. Keep public API names
  snake_cased 1:1 (`addEnergy` -> `add_energy`).
- C++ dump tools live in `tests/` (`solver_diff_dump.cpp` pattern);
  fixtures in `tests/fixtures/`; Rust replay tests in
  `rust/core/tests/`.

## Port style: line-by-line mirror

- Mirror the C++ structure (same functions, same order, same thresholds);
  comment deliberate restructures (borrowck, faer API gaps) at the site.
- Preserve every numeric threshold and tolerance constant exactly.
- No `.unwrap()`/`.expect()` in library code on the solve path; return
  `Option`/`Result` like the C++ `bool` returns (mirror success/failure).
- `cargo fmt` clean before every push.

## Oracle (required per module)

1. Replicated goldens: mirror the existing C++ test groups 1:1 at the
   same tolerance.
2. Randomized differential: C++ dump tool (seeded splitmix64, `%.17g`
   doubles) -> committed fixture -> Rust replay test. Minimum 200 cases
   per module or pair. Replay asserts ok flags + values + any structural
   facts (ranks, counts, iterations).
3. Timing: bit-identical systems regenerated from the same seed on both
   sides, release-vs-Release, 3 samples each.

Generation rules (learned the hard way):

- Generate downstream-plausible systems: full variable coverage, and
  overdetermined where downstream is (underdetermined systems measure
  ridge-regime FP noise, not logic).
- Constraints that must agree across backends must be consistent
  (generate from a known target); segregate inconsistent inputs into a
  robustness-only section (port must solve everything C++ solves;
  values may differ by backend selection rules).
- Values assert at scale-aware `1e-6 * max(1, |expected|)`; logic bugs
  show up O(1) and still trip it by orders of magnitude.

## Pitfalls (all bitten once already)

- Gram accumulation: unordered pairs once (ordered pairs folded into one
  triangle double-count off-diagonals).
- Fuzzer loops: cap row width at variable count (duplicate rejection
  hangs on 1-2 variable systems).
- Borrowck clones: never clone a whole matrix per call; use
  explicit-scratch associated functions (a 22x slowdown hid here).
- `Box<dyn Fn>` aliases are `'static`: test closures capturing locals
  need `Arc` (e.g. `Arc<Mutex<..>>` for handlers).
- FP contraction: Clang fuses `a*b-c*d` to FMA by default (no
  fast-math needed), flipping ulp-level decisions vs strict Rust.
  Any port of FP-decision code must check the C++ IR for
  `llvm.fmuladd` at every multiply-add site and replicate with
  explicit `mul_add` — and the oracle needs near-degenerate
  adversarial inputs to catch it (plain goldens pass).
- Relative ridge / mean-diagonal scaling: preserve exactly.
- Backend transcendental fusion: LLVM fuses adjacent `cos(x)`/`sin(x)`
  into one `sincos` libm call — invisible in opt IR (still separate
  `llvm.cos`/`llvm.sin` there; only disassembly shows `sincos_stret`) —
  and macOS libm `sincos` rounds sine 1 ulp off standalone `sin` on some
  inputs. Audit adjacent transcendentals via disassembly (`nm` for
  `sincos`), and use the shared `double_utils::joint_sin_cos`
  (`f64::sin_cos` provably does not fuse on rustc 1.98).

## Branches and done-means

- Lane branches `exp/rs-<module>` off `exp/rust-solvers`; push with an
  explicit refspec, never main.
- Done per lane: `cargo test` (debug + release) green, oracle max diff
  reported, timing ratio recorded, replica goldens green, pushed.
- Report: divergence number, timing table, effort notes, any known gaps
  with the same honesty bar as the solvers verdict.
