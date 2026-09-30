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

`src/*_tests.rs` replicate 1:1 the C++ golden groups at the same 1e-6
tolerance (9 CLS groups in `tests/test_constrainedleastsquares.cpp`, 5
MILS groups in `tests/test_mixedintegerleastsquares.cpp`), plus:

- randomized diagonal/known-answer systems (hand-verifiable LS solutions),
- a rank-deficient Lagrange case (validates the support-submatrix restructure),
- a cover-sized timing case (thousands of vars) for faer-vs-Eigen runtime.

Pass criteria: all replicated goldens within 1e-6 max-abs-diff, zero
structural mismatches (rank, convergence, fixed/free classification).
Any systematic divergence is a finding, not a failure — it calibrates
the full-port estimate.

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
