# retopoforge log: what was tried, kept and dropped

Newest round first. Scores come from `bench/creature/` (README there):
the owner's generated creatures and HLH props, plus hand-made rigged
animals as the topology reference. Subjects are named by kind only.

## The keep rule (owner, 2026-11-13, replaces the first one)

"Some tools are better for different characters", "we might just be
doing the benchmarks wrong", "we should be improving and not trying and
dropping":

1. A method that wins on some models stays. The tool learns to choose per
   model by measurable reasons, and the log says where each method wins.
2. A promising method that loses gets worked on: find why, fix it, run it
   again. Parking one needs the reason and what it would take.
3. When the score and the pictures disagree, the benchmark is fixed first.

The comparison itself (`bench/creature/table.py --compare A B`) still
reports, per score, on how many models B is better and worse; that now
feeds rule 1 (where B wins) instead of a single keep/drop verdict.

The main score comes from professional ground truth where there is one:
hand-made game creatures and characters (CC0/CC-BY, benchmark use only),
turned into what a generator gives (dense, triangulated, parts fused),
remeshed, and compared with the pro mesh and bent with the pro's rig.

## Round 1 (2026-11-13): review, bench, baseline

See `docs/review-2026-11-13.md` for the review.

### Baseline: retopo against plain decimate on the owner's creatures

10,000-triangle budget, each creature rigged once by the creature route,
the same weights carried onto every cut. First four creatures (the fifth
and the hand-made animals were still scoring):

| creature | bend retopo / decimate | worst distance % | open edges | normal dev. deg |
|---|---|---|---|---|
| 1 (furred biped) | 49.7 / 68.1 | 5.33 / 1.17 | 14 / 0 | 32.7 / 21.2 |
| 2 (rooted quadruped) | 64.8 / 75.1 | 9.51 / 0.42 | 78 / 0 | 39.4 / 20.3 |
| 3 (horned biped) | 83.0 / 93.3 | 4.44 / 0.38 | 0 / 0 | 25.3 / 12.8 |
| 4 (spined quadruped) | 68.5 / 85.0 | 4.26 / 0.48 | 0 / 0 | 33.1 / 18.8 |

Retopo loses on every one. The pictures show why: single vertices thrown
far off the body (long spikes), small holes, and spines and fur melted
into blobs. Where the surface is plain (the rooted creature's hind leg),
retopo's rings bend cleaner than decimate's slivers, but the checker sees
the spikes and lost shape first. QuadriFlow (Blender) refused three of the
five creatures even on a closed voxel shell; on the one it cut, it scored
72.2 (between the two). Loops at joints are near zero for every tool
(retopo 0 to 0.4 rings per joint), so nothing on the route makes joint
loops today.

Also found: the horned creature is one connected piece, yet retopo split
it into three islands and dropped two (review finding 11, island split by
face winding); the spined quadruped's six "islands" were 3-vertex specks.

Fifth creature (mountain boss, biped): retopo 74.2, QuadriFlow 75.1,
decimate 79.9; surface p95 3.45 / 3.03 / 0.13%.

Hand-made animals (8, artist's rig and mesh as the reference, each tool
at the artist's own count, 576-4,004 triangles): the bend checker scores
the artists' own rigs 3.7-21.9, so on low-poly animals the bend score
says little (the bench-shelf session saw the same on pro characters).
What the lane does show: retopo collapses thin low-poly limbs at these
budgets (surface p95 1.3-17.5%; the fox came back at 180 of 576
triangles), while decimate stays within 0.04-0.23%.

HLH props (10, each at its own count): decimate at the own count is the
prop itself (0 error); retopo moved the surface 0.31-11.6% (p95) and lost
parts (ribcage 1,370 of 4,160 triangles); QuadriFlow 0.14-4.6%. Props
are made at budget by the props kit, so none should go through a
remesher. Route decision: `docs/direction.md`.

### Outside rivals not run this round

Instant Meshes (BSD-3, official 2019 Linux build) and QuadWild+Bi-MDF
(GPL-3, official v0.0.2 Linux binaries) were downloaded to f
(`~/rfwork/third_party/`, URLs, versions and sha256 in its NOTE.md), but
running downloaded binaries was refused by the agent's permission check.
They wait for the owner. Tripo's own retopology needs paid credits: not
run.

### Tried: retopo's cut laid back onto the source (bench only, `retopo+lay`)

Each vertex moved to the nearest source point unless that turns a face
over. Bend on the five creatures: 49.7 -> 53.7, 64.8 -> 66.4, 83.0 ->
83.6, 68.5 -> 70.3, 74.2 -> 77.1 (better on all five); mean surface
distance 0.27-0.70% -> lower on all five; surface p95 and worst distance
almost unchanged (1.40 -> 1.37 on the first). So where retopo puts its
vertices costs a little; most of its loss is shape it never captured.
Kept as the reason to build the engine version (`exp/settle`: the same
projection onto the island's input, plus pulling stray vertices home).

### Found: the horns are lost after the working mesh

The mountain boss lost both horns (worst distance 15.5% of its size).
Its working mesh (`RETOPO_DUMP_STAGES`, the isotropic stage's output,
24k triangles) still has both horns and the spines, so they are lost in
rounding or extraction, not in decimation. Two suspects from the review:
one-shot rounding collapsing a thin part (finding 6) and extraction
keeping only the largest piece (finding 8). The second is cheap to test:
`RETOPO_KEEP_PIECES` on `exp/settle`.

Also learned: the long thin lines in the pictures are hair strands in
the Tripo source itself (decimate shows them too), not retopo outliers.
