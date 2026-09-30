# Rust solvers experiment (calibration, not commitment)

Date: 2026-09-30. Branch: `exp/rust-solvers`. Question: what does a port
of the numeric core actually cost, and how far do the answers diverge?

## Scope

Port exactly two modules to the `rust/solvers` crate (faer 0.24):

- `core/constrainedleastsquares.*` (421 lines, Eigen sparse) -> `src/constrained.rs`
- `core/mixedintegerleastsquares.*` (547 lines, pure std over CLS) -> `src/mixed_integer.rs`

Nothing else. No engine changes, no build-system integration, no FFI.
`main` is untouched; the experiment lives and dies on this branch.

## Solver mapping (Eigen -> faer, verified against faer 0.24.4 sources)

| C++ (Eigen)                              | Rust (faer)                                   |
|------------------------------------------|-----------------------------------------------|
| SimplicialLDLT on normal equations       | sparse LLT (`sp_cholesky`) + AMD              |
| AccelerateLLT fast path                  | none (measure the gap honestly)               |
| SparseQR/COLAMD rank + column selection  | dense `ColPivQr` on the constraint-support submatrix (exact, see below) |
| SparseLU on KKT system                   | sparse LU (`sp_lu`)                           |

Lagrange rank note: faer's sparse QR is not rank-revealing, so the port
collects the union of variables appearing in constraints (small in
practice), builds the dense support submatrix, and uses dense
column-pivoted QR for rank + independent selection. Zero columns outside
the support cannot change the rank, so this is exact — but it must be
proven by the differential tests, including a rank-deficient case.

## Oracle (differential tests, the quality bar)

Two layers, all green:

1. Replicated goldens: 9 CLS + 5 MILS groups mirroring
   `tests/test_constrainedleastsquares.cpp` /
   `tests/test_mixedintegerleastsquares.cpp` at 1e-6
   (`rust/solvers/tests/{constrained,mixed_integer}_golden.rs`).
2. Randomized differential (`tests/solver_diff_dump.cpp` ->
   `tests/fixtures/solver_diff.txt` -> `rust/solvers/tests/differential.rs`):
   seeded splitmix64 systems solved by C++, replayed by Rust:
   - 200 CLS consistent cases (constraints pinned to a known target, 1-8
     vars, overdetermined + full variable coverage like downstream, wide
     rows force the Lagrange path, plus RHS-update and clear re-solves),
   - 200 MILS cases (homogeneous equalities, periods, builder API, loop
     to convergence, asserts ok/converged/iters/kernel sizes/values),
   - 40 CLSX inconsistent-constraint cases (robustness-only, see gap 1).

Values assert at scale-aware 1e-6 x max(1, |x|): the two Cholesky
backends' forward error grows with |x|, and O(1) logic divergences
still trip it by orders of magnitude (proven: it caught a Gram
double-count and a subset-selection divergence during development).

Timing: bit-identical cover-sized systems regenerated from the same
splitmix64 stream on both sides (`tests/timing.rs`), release-vs-Release.

## Verdict criteria

- effort log (hours per module, friction notes),
- max divergence on goldens + randomized cases,
- runtime ratio on the timing case (note: no Accelerate on the Rust side),
- recommendation: strangler-fig full port, port-with-restructure, or stop.

## First findings (pre-port)

- MILS is pure std over CLS: the port risk concentrates in CLS's three
  Eigen sparse factorizations, not in MILS logic.
- faer 0.24 sparse covers LLT/LU/QR structurally; only QR-rank needs the
  support-submatrix restructure above.

## Results (2026-09-30, all suites green)

Divergence (strict oracle, 400 cases + replicas):

- Max |diff| 2.4e-8 (pure backend FP noise; tolerance is scale-aware
  1e-6). Zero ok-flag, convergence, iteration-count, kernel-size, or
  builder-API mismatches. MILS iteration counts match exactly on all
  200 cases, so the integer-rounding cliff never triggered differentially.

Known gap 1 (quantified, unreachable downstream): on INCONSISTENT hard
constraints the Lagrange path must drop a dependent constraint, and
SPQR+COLAMD (C++) vs dense ColPiv (Rust) can drop different ones ->
different values. CLSX section: 58 both-solved, 56 value-agree, max
diff 13.0 on the 2 divergent solves, zero robustness regressions (Rust
solves everything C++ solves). Downstream constraints (cover equalities
+ MILS pins) are consistent by construction, so this path is unreachable
in practice. A full port should replace selection with a deterministic
greedy rule on both sides (C++ golden churn on affected cases TBD).

Known gap 2 (closed during calibration): the first port doubled every
off-diagonal Gram entry (ordered-pair accumulation folded into the upper
triangle) and mis-scaled weights; the oracle caught both. Lesson: the
replica goldens alone were insufficient (no multi-var-energy reduced
case); the randomized differential is the real bar.

Runtime (cover-sized, 3 samples each, same systems, iters match):

| Stage                                  | C++ Release+Accelerate | Rust release+faer |
|----------------------------------------|------------------------|-------------------|
| CLS 2000v/4000E/200C                   | ~95-108 ms             | ~26 ms (3.7x faster) |
| MILS finalize (500v/300eq)             | ~0.12 ms               | ~0.15 ms (parity) |
| MILS loop (2 solves)                   | ~0.31 ms               | ~0.35 ms (parity) |

Note: an early MILS finalize was 22x slower from a per-constraint full
`m2` matrix clone (borrowck workaround); fixed by explicit-scratch
associated fn. No Accelerate on the Rust side and it still wins CLS by
~4x — faer's sparse Cholesky beats Accelerate here; the C++ side pays
Eigen sparse A^T A + Accelerate overhead.

Effort log: ~1 session for both modules + oracle (~1k lines ported,
681-line MILS port vs 547 C++). Friction points, in order of cost:
(1) divergent first port needed a line-by-line mirror rewrite after two
`cargo test` stalls; (2) fuzzer bugs of mine (duplicate-variable
infinite loop, unconstrained rank-deficient systems measuring FP noise
instead of logic); (3) tolerance had to go scale-aware for |x| ~ 20
ridge/underdetermined cases; (4) one borrowck-driven clone (see above).
No faer API gaps: sp_cholesky/sp_lu/ColPivQr covered all three Eigen
factorizations.

## Verdict: strangler-fig viable, solvers first as done here

- Cost projects linearly: the numeric core ports at ~1k lines/session
  with the oracle catching every divergence class seen so far.
- Risk concentrates in (a) the Lagrange subset-selection rule (gap 1:
  needs a joint C++/Rust decision, not a mirror), and (b) performance
  traps from borrowck-driven clones (caught by the timing harness).
- Recommendation: proceed strangler-fig — keep this crate + oracle on
  the branch, port next leaf module (parameterizer input structs, no
  Eigen), re-run the oracle each step. Do NOT port the mesh/Eigen-dense
  core until the leaf layer is fully green.
