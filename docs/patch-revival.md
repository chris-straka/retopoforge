# Patch backend revival + landing verdict: KEEP ON BRANCH

Revival (this branch): the lane's `fill_ngon` backtracking search (try
every cut, Dijkstra + Coons fill per attempt, recurse, roll back) was
exponential in side count — beast@1000 hung 30+ min. Replaced with:

- greedy plan-then-emit: the single cut sequence is planned from
  side-count arithmetic (first-valid-choice per level) and emitted
  infallibly — no tentative geometry, no rollback. Planning success is
  exactly the old search's first success, minus the exponential
  re-emission;
- hard caps with per-face-subdivision fallback (coverage by
  construction): 128 plannable sides, 2048 plan-search nodes (the
  rotation search backtracks, so depth alone cannot bound it), 16384
  grid points per Coons piece — counted as `capped_patches`;
- exact closest-point index for projection (grid + scan fallback for
  small/degenerate face sets), built per patch; grid ≡ brute-force
  distance pinned by test;
- tube-scale e2e smoke test (`tube_smoke_finishes_under_10s`,
  procedural bench-scale tube, runs in CI): 0.6 s.

Beast@1000 with `--backend patch`: 4.3 s (was: hang).

## Landing protocol (2026-10-02, macOS)

- Default path byte-identical: CLI contract 4/4 + `run.py --check`
  green on the branch (goldens regen'd for the lane's `--backend` help
  stanza, which the lane never pinned).
- `--backend patch` 0 non-quads: FAIL — 92 on beast@1000 (odd-patch
  triangle arms are structural, by design).
- beast@1000 first-attempt appendage coverage: PASS — 2694 output
  verts at x < -62, no retry exists in this back end. The structural
  target holds: no fold, no retry, appendage present.
- Matched three-way on the corpus-scale character
  (corpus_v5_150k, anchor = QuadWild flow-lane count 10690):

| engine | target | quads | non_quads | irr_pct | angdev_mean | angdev_p95 | dist_mean | dist_max |
| quadwild-flow | - | 10690 | 0 | 9.9 | 21.6 | 51.5 | 0.154 | 2.06 |
| default | 9917 | 14770 | 78 | 8.3 | 11.7 | 46.1 | 0.136 | 2.36 |
| default | 7493 | 7879 | 42 | 9.3 | 12.3 | 43.9 | 0.196 | 2.55 |
| patch | 543 | 10350 | 144 | 54.7 | 28.0 | 67.0 | 0.300 | 3.58 |

(Default yield is bimodal here — the coverage retry flips 7.9k ↔
14.7k across secant targets, so no target lands within 5% of the
anchor; both bands are reported. Patch converged round 1 at −3.2%.)

Verdict: KEEP ON BRANCH. Patch loses on all four landing criteria:
non-quads (144 vs 42–78), worst-case distance (3.58 vs 2.36–2.55),
irregular verts (54.7% vs ~9%), angles (28.0 vs ~12).

## Why it loses (for the next revival)

1. Fallback-heavy: exact-opposite-equality cutting on noisy
   quantization leaves ~2/3 of working faces to per-face
   3-quad subdivision (beast: 5300/8000 faces) → 16–19×
   over-yield and 55% irregular verts. Needs quantization
   agreement (near-match resolution without T-junctions, or a
   global integer solve).
2. Triangle arms by design (odd n-gons, conflict diagonals) → 0
   non-quads is unreachable without all-quad odd fills (center
   irregular vertex).
3. Projection onto the coarse working mesh caps surface fidelity
   (dist_mean 2× default).

The branch stays as the first-attempt-coverage reference: beast
appendage coverage without retry is proven and pinned
(`beast_1000_covers_appendage_first_attempt`).
