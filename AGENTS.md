# retopoforge — agent notes

Fork of huxingyi/autoremesher (MIT): a Rust engine (`retopo_core` +
`retopo_solvers`), the `retopo` CLI, a Blender extension driving the CLI,
and a benchmark/quality harness. The engine was ported 1:1 from the
original C++ (proven by differential oracles, see
`docs/rust-switch-verdict.md`); the C++ tree has since been removed. The
`upstream` git remote tracks the original repo as a read-only reference
only — NEVER git-merge upstream into this tree (fully diverged). Port
individual upstream engine fixes by hand when relevant, bench green.

## Standing rules

- Commit and push to `origin/main` on your own after each completed chunk
  of work. Do not wait for the user to approve commits or pushes.
  (Explicit standing authorization from the project owner.)
- Never force-push, rebase, amend published commits, or otherwise rewrite
  published history.
- The owner's game assets must never be committed — not even file names
  or paths in tracked files. Refer to them only as "the owner's AI
  corpus". `bench/models/` and `bench/results/` are gitignored.
- Never launch GUI binaries without explicit user approval. Headless
  Blender (`--background --factory-startup`) is fine.

## Build

- `cd rust && cargo build --locked --release -p retopo` produces
  `rust/target/release/retopo`. Rust only; `thirdparty/meshoptimizer` (C++)
  is compiled in via `rust/core/build.rs` (`cc` crate).

## Checks

- `cd rust && cargo fmt --all --check && cargo test --locked --release`
  must be green with zero warnings.
- CLI contract goldens (`tests/fixtures/cli_golden/`): after an intended
  CLI or engine output change, regenerate with
  `UPDATE_GOLDENS=1 cargo test --release -p retopo --test cli_contract`
  and review the diff before committing.
- `bench/run.py --check bench/baseline.json` must pass after engine or CLI
  changes (fetch models once with `bench/fetch_models.sh`).
- Quality claims need distributions, not single runs: the tiling is
  sensitive to sub-visible input noise (see
  `docs/igm-validity-spike.md`). Use `bench/noise.py` + `bench/score.py`.
- New CLI flags must also appear in `--help` and the README.

## Layout

- `rust/core/` engine, `rust/solvers/` least-squares solvers,
  `rust/cli/` the `retopo` binary + CLI contract test.
- `blender/` = Blender extension (GPL, talks to the CLI as a subprocess).
- `bench/` = harness (`run.py` regression gate, `score.py` scorecard,
  `noise.py` noise floor, `compare.py` free baselines, `profile.py`).
- `tests/fixtures/` = golden data for the Rust tests.
- `thirdparty/meshoptimizer/` = the only vendored dependency.
