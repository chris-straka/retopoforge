# Sizing-aware MILS rounding: scope check verdict is NO

Date: 2026-10-01. Question from the post-switch batch (TODO.md):
is sizing-aware mixed-integer rounding, driven by parameterizer /
frame-field flip maps, worth building? Verdict: no — rounding order
moves output quality the wrong way and breaks retry recovery. No code
on main; the experiment gate lived in a scratch worktree only.

Note: the "QPX/FFX flip maps" named in the batch item exist nowhere in
the tree (no QPX/FFX/MILS references outside TODO.md). The closest
existing machinery is the `RETOPO_IGM_DEBUG` inversion census and the
`RETOPO_ROUND_SCHEDULE` progressive-rounding gate on `exp/igm-validity`
(see `docs/igm-validity-spike.md` §3). This check ports that gate onto
current main (native decimator, coverage retry on) and asks whether
rounding order affects the beast@1000-native fold — the case any
rounding work would have to fix.

## Experiment

`RETOPO_ROUND_SCHEDULE=0.1,0.2,0.3,0.4` (same thresholds as the spike)
vs one-shot, beast@1000 × 8 noise seeds, retry ON (production path).
Gate verified inert when unset (byte-identical output). First-attempt
coverage is read off the stderr coverage warning (no warning = attempt
0 clean); quality via `bench/score.py`.

| config | retry fired | unrecovered | dist_max med [min-max] | irr med | angdev med |
|---|---|---|---|---|---|
| one-shot | 7/8 (all retry-1, 0 uncovered) | 0/8 | 6.71 [5.21-7.94] | 14.53 | 18.31 |
| schedule | 2/8 | 2/8 (kept original) | 7.19 [7.18-13.67] | 16.56 | 18.89 |

## Reading

- The schedule *looks* better on first-attempt coverage (6/8 clean vs
  1/8) but its failures are catastrophic and unrecoverable: 2/8 seeds
  ship the fold (dist_max 13.67), where one-shot recovers 7/7 via
  retry 1 with exact-zero residuals.
- Quality medians move the wrong way (irr +14%, dist_max +7% med with
  a 1.7x tail), consistent with the two prior negatives: true greedy
  rounding was REJECTED (dist_max worse 8/8 seeds, +33% time; TODO
  "Rounding"), and the meshopt-era schedule cut inversions 12-25%
  with the scorecard flat inside the noise floor (spike §3).
- Mechanism, same as the spike: 65-97% of inversions pre-date rounding
  (the least-squares cover has no injectivity term), so ordering only
  rearranges the rounding-induced remainder — while freezing
  early decisions removes the retry's escape route. The extractor
  compensates inversion-count changes without scorecard movement.

## Conclusion

There is no signal for ordering work: the progressive schedule, the
simplest flip-reducing ordering, degrades the production path it would
have to improve. A sizing-aware / flip-map-driven variant would steer
the same stage with finer weights, but the stage provably moves quality
the wrong way. Do not build it. The fold-class defects stay owned by
the patch back end (integer-layout feasibility by construction), which
already proves first-attempt appendage coverage on its branch.
