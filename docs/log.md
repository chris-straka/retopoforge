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

## Round 2 (2026-11-13, running)

New lanes (owner's request for quality benchmarks):

- **pro**: 14 professional rigged meshes (Quaternius UBC male and female,
  knight, eight Quaternius/Khronos animals: CC0; Cesium Man: CC-BY 4.0;
  Blender Studio's Phileas: CC-BY; the owner's MPFB base: CC0), benchmark
  use only. Each is smoothed one level and voxel-fused (0.3% of its size)
  into what a generator gives; every tool remeshes that at the pro's own
  triangle count, and the result is compared with the pro: rings where
  the pro rings (`pro-loops%`), edge flow against the pro's quads
  (`flow`, 0-45 degrees), poles, and bending on the pro's own rig.
- **character**: four of the owner's generated characters, each cut by
  every tool, carrying the human route's rig (fitted onto our cut), then
  given the game's clips (`clips_humanoid.glb`, `motionforge animate`)
  and checked on those clips.

Experiments in the queue: settle onto the input (`exp/settle`,
RETOPO_SETTLE), keep detached pieces (RETOPO_KEEP_PIECES), hard joint
rings (`exp/joint-rings`), symmetry snap fix (`exp/symmetry-snap`),
`--skeleton` density, soft ring guides, and the parked patch back end
(`agent/patch-backend`, `--backend patch`), which won worst-case distance
on 10-02 (2.27% against 5.21%+) and is measured here on creatures before
any work to land it.

Not possible this round: QuadWild with forced joint rings (the downloaded
QuadWild binaries may not be run without the owner's say).

### Settle onto the input (engine, `exp/settle`, RETOPO_SETTLE): working, partial

Four creatures so far: bend 49.7 -> 53.7, 64.8 -> 66.4, 83.0 -> 83.6,
68.5 -> 70.2; surface p95 down a little on each. Identical to the
bench-only `retopo+lay`, so the stray-vertex pass found nothing the
plain projection didn't fix on these; the gain is vertex placement.

### Symmetry (`--symmetry x`) and the snap fix (`exp/symmetry-snap`)

The engine accepted the mirror plane on one of five creatures (the horned
one); on the other four it judged them not symmetric enough and ran
plain (identical numbers). On the horned one: old snap bend 85.1, posed
flips 1,021, surface p95 2.06%; bounded mutual snap 84.2, 297, 1.74%;
no symmetry 83.0, 286, 1.72%. The old snap's damage (vertices dragged to
far partners) shows as the flips and the shape error; the fix removes it
and keeps most of symmetry's bend gain. One model only: kept on its
branch until more symmetric subjects (the pro lane's humans) are scored.

### Bench faults found and fixed this round (rule 3: fix the benchmark first)

- **QuadriFlow "refused" creatures and characters; the fault was ours.**
  Blender's QuadriFlow rejects any mesh with an edge shorter than 1e-4
  units, with the message "needs to be manifold". A dense creature one
  unit tall always has such edges. The bench now runs QuadriFlow at 100
  units across and scales back. Round 1's "QuadriFlow refused three of
  five" is withdrawn; re-run in progress.
- **The game's clips had not run on the characters.** `motionforge
  animate` refused the game's clip file (it holds only a skeleton). The
  clip file is on the same standard skeleton as the rigs, so the clips
  are now copied onto each candidate by bone name
  (`bench/creature/clips_onto.py`, 31 clips) and `weights check` samples
  them. The first character table (below) is on the standard pose set
  only.
- **The pro lane had thrown the pros' quads away.** A GLB stores
  triangles; the first try at pairing them back (Blender's Tris to Quads)
  left half the vertices irregular and found no loops on meshes that
  plainly have them. An exporter writes each quad as two consecutive
  triangles, so pairing neighbours in file order recovers the artist's
  quads exactly: 99-100% quads on the Quaternius humans and Cesium Man,
  98% on Phileas, 86% on the knight. The MPFB base turned out to be a
  decimated copy (half triangles): not a topology truth, removed.
- **The bend checker says little on pro meshes** (it scores the artists'
  own rigs 3-70, median 7). The pro lane's bend measure is now the truth
  itself: every joint is bent the same way on the candidate and on the
  pro mesh (same rig), and the distance between the two bent surfaces is
  the score (`bent-vs-pro%`).
- A mesh lost its parent's transform when unparented (prep and cuts);
  kept now.

### First character table (standard poses only, before the fixes above)

Four of the owner's characters at 14,000 triangles, the human route's
rig: retopo 42.0 / 68.4 / 63.7 / 55.5, decimate 53.6 / 69.4 / 62.1 /
70.2. Decimate ahead on three, retopo by 1.6 on one. Retopo's surface
error (p95) 0.37-1.47% against decimate's 0.04-0.06%. Re-running on the
game's clips with QuadriFlow included.

### Hard joint rings, first version (`exp/joint-rings`): no loops yet

Rings were cut into the working mesh and survive the isotropic stage
(694 ring vertices in, 694 out on a test animal; 14 of 14 rings pinned),
but only about 45% of ring edges got an alignment mark (the feature path
marks an edge only within 10 degrees of the cross field), so each ring is
in pieces and no loop closes (loops at joints 0 on all five creatures).
Bend rose on four of five creatures (median 68.5 -> 73.8) while the
joint picture got messier (more irregular vertices at the ring): score
and picture disagree, and a single run cannot tell a 5-point move from
noise. So: (1) the bend score's spread is being measured (three jittered
seeds of the baseline), (2) version 2 marks every ring edge by its ring
tag. Stays in work.

### `--skeleton` density and symmetry on creatures

`--skeleton` changed one creature of four so far (64.8 -> 66.6). Symmetry
was accepted on one of five. Neither moves the set; both stay available
(they are per-model tools by the owner's rule), nothing to tune yet.

### Instant Meshes (first two creatures)

Bend 26.3 and 44.6 against retopo's 49.7 and 64.8 and decimate's 68.1
and 75.1; surface p95 1.08 and 1.0%. Behind both so far.

## Round 3 (2026-11-13): what should retopoforge be?

Direction from the orchestrator: a game character or creature is judged
by how it bends at the joints, its silhouette and its triangle budget,
not by how square its faces are. Before more engine work, measure that
and decide the tool's shape.

**Scores** (per candidate, after rigging; `bench/creature/run.py --stages
... after`):

- `tears%`: weightforge's headline, the share of the surface inside
  failing findings while the body moves: on the hero's game clips for
  the owner's characters (`weights check --motion hero.glb`), on the
  creature's own range of motion for creatures;
- `outline%`: silhouette error against the download, eight views
  (`silhouette.py`: area where the two outlines differ, as a share of
  the source's outline);
- triangles, held to the budget (10,000 creatures, 14,000 characters).

Quad quality (loops, poles, edge flow) stays in the tables as secondary.

**Candidates**: retopoforge as it is, Blender's QuadriFlow, Blender's
Decimate (today's route), and each with a joint-ring pass
(`ring_pass.py`): three planes cut across each limb joint (-0.5, 0, +0.5
limb radii), placed from the reference skeleton, trunk joints skipped;
the base cut is made smaller so the ringed mesh lands on the same budget.
On one creature the rings cost 9% of the triangles on retopo's quads and
22% on decimate's triangles before the base is re-cut.

Engine experiments (settle, hard rings v2, keep-pieces, patch back end)
and the wider rival runs were stopped to free the machine; their branches
and partial results stand as logged above.
