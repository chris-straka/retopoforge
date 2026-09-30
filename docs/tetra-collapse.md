# Tetra collapse diagnosis: tiny-mesh empty output at some `--target-quads`

`tests/fixtures/tetra.glb` (unit trirectangular tetrahedron, 4 verts / 4 tris,
area 2.366) remeshes to an EMPTY mesh at some `--target-quads` values and
succeeds at others, non-monotonically. This doc traces every pipeline stage,
names the stage that wipes the mesh with file:line evidence, and recommends a
fix shape with a risk estimate. No code fix is included: nothing found keeps
`bench/run.py --check` counts identical by inspection, so the doc is the
deliverable.

## Repro

```sh
cmake -S . -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/macos-llvm.cmake \
  -DCMAKE_BUILD_TYPE=Release -DRETOPOFORGE_BUILD_QT_APP=OFF
cmake --build build --target retopo
for t in 2 4 8; do
  ./build/cli/retopo --input tests/fixtures/tetra.glb --output /tmp/t$t.obj \
    --target-quads $t
done
# target 2 -> exit 1 "failed to write", target 4 -> exit 0 (3 quads),
# target 8 -> exit 1 "failed to write"
```

The OBJ control `tests/fixtures/nasty-single-tetra.obj` reproduces identically
(2/8 fail, 4 works), so the GLB loader is not implicated.

Full target sweep (exit 0 = mesh written, exit 1 = empty output):

| target | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 | 20 | 50 | 200 | 50000 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| result | F | F | F | P | F | F | F | F | F | P | P | F | F | P | F | P | P | P | P | P |

(P = pass with 3/6/6/6/8/6/33/110/15797 quads respectively; F = exit 1.)

A failing run ends loudly, not silently:

```
Warning: 1 of 1 islands produced no output and were dropped from the mesh
Error: failed to write /tmp/t8.obj
```

(`test_cli_glb.cpp` case (f) pins this loud failure for `--lods 8,4`.)

## Stage-by-stage trace

Per-stage counts from a probe linked against `libretopo_core` (public
accessors only: `isotropicTriangles()`, `islandOutputQuadCounts()`):

| target | voxelSize | iso verts/tris | island quads | out verts/quads |
|---|---|---|---|---|
| 2 | 1.169 | 4 / 4 | [0] | 0 / 0 |
| 4 | 0.826 | 8 / 12 | [3] | 5 / 3 |
| 8 | 0.584 | 14 / 24 | [0] | 0 / 0 |
| 12 | 0.477 | 22 / 40 | [0] | 0 / 0 |
| 16 | 0.413 | 32 / 60 | [8] | 9 / 8 |

`AutoRemesher::remesh()` returns `true` in every case: no stage errors out.
The isotropic mesh is never wiped (tri counts grow as the target rises).
The wipe happens strictly inside quad extraction: the island enters the
parameterizer/extractor with a healthy triangle mesh and comes out with
zero quads.

Instrumented-build dumps (`-DAUTO_REMESHER_DEV=1`, a separate `build-dev`
dir; dumps land in the run cwd) show where inside extraction:

| target | connections (v/l) | edges after cleanup (v/l) | mesh pre-cleanup (faces) | final (v/f) |
|---|---|---|---|---|
| 2 | 4 / 6 | 1 / 0 | 0 | 0 / 0 |
| 4 | 18 / 39 | 10 / 34 | 10 | 5 / 3 |
| 8 | 18 / 34 | 3 / 0 | 0 | 0 / 0 |
| 12 | 40 / 79 | 3 / 2 | 0 | 0 / 0 |

(`l` lines are dumped both directions, so undirected edges = l/2.)

Connection tracing works fine at every target (isoline intersections are
found). The graph cleanup between connections and mesh extraction deletes
the entire graph.

## Failing stage: QuadExtractor edge-graph cleanup

Call sequence in `QuadExtractor::extract()`, `core/quadextractor.cpp:132-138`:

```cpp
extractEdges(connections, &edgeConnectMap);          // 133
if (collapseShortEdges(&crossPoints, &edgeConnectMap)) // 134
    simplifyGraph(edgeConnectMap);
collapseTriangles(&crossPoints, &edgeConnectMap);    // 136
if (removeSingleEndpoints(&crossPoints, &edgeConnectMap)) // 137
    simplifyGraph(edgeConnectMap);
```

Replayed step by step on the dumped connection graphs (simulation of the
exact functions below; all three failing targets reproduce the dumped
post-cleanup graph node-for-node):

Target 8 (18 nodes / 18 edges raw = three disjoint pure cycles of 5, 5, 8
nodes; every node valence 2):

1. `simplifyGraph` (`core/quadextractor.cpp:325-354`) strips valence-2
   nodes. A pure N-cycle degenerates into a single open edge (2 nodes /
   1 edge): the reconnection at lines 348-351 inserts into adjacency
   `unordered_set`s, so the parallel edges that would preserve loop-ness
   dedup into one. Result: 6 nodes / 3 edges, three disjoint open edges.
2. `collapseShortEdges` (`:449-477`) and `collapseTriangles` (`:388-447`)
   are no-ops (no short edges, no 3-cycles left).
3. `removeSingleEndpoints` (`:356-386`) strips each open chain from its
   endpoints down to one isolated vertex. Result: 3 nodes / 0 edges.
4. `extractMesh` (`:508+`) closes faces by walking `edgeConnectMap`
   adjacency (`:594-595`); with 0 edges no face loop can form, so
   `m_remeshedPolygons` stays empty (pre-cleanup dump: 0 faces).
5. The vertex compaction at `:223-243` then drops all 18 unused cross
   points, leaving 0 vertices / 0 faces.

Targets 2 and 12 are the same mechanism with variations: target 2 is one
4-cycle (`4n/4e -> 2n/1e -> 1n/0e`); target 12 additionally loses cycles to
`collapseTriangles` (`8n/10e -> 6n/6e`) before the endpoint strip leaves a
single open edge (`3n/1e`), which likewise cannot form a face.

Downstream, the empty island is dropped at the merge
(`core/autoremesher.cpp:1204-1208`, `if (quads.empty()) continue;`),
`m_islandOutputQuadCounts` stays `[0]`, and the CLI refuses to write an
empty mesh (`cli/main.cpp:640-641`), producing the exit-1 above. Note
`QuadExtractor::extract()` unconditionally returns `true`
(`core/quadextractor.cpp:311`); `remesh()` likewise returns `true`.

## Why non-monotonic (and platform-dependent)

Pass/fail depends only on whether the rounded integer-isoline graph
contains branch points or cycles that survive the cleanup above -- a
discrete quantization property of the parameterizer's cover rounding at a
given voxel scale, not a monotone function of the target count. Small
voxel-size changes re-quantize the lattice and flip the graph topology
(e.g. target 11 works, 12 fails, 14 works again).

This also explains the existing platform note in
`tests/test_cli_glb.cpp:196-200` ("collapses to empty on some platforms
(Linux)"): `simplifyGraph` iterates an `unordered_map`, whose order -- and
hence which nodes survive cycle degeneration -- varies by platform and
standard library.

## Suspect verdicts

- `initializeVoxelSize` vs feature size (`core/autoremesher.cpp:311-319`):
  ruled out. Voxel sizes are sane relative to the tetra edges (1.0 and
  1.414): 1.169 at target 2 down to 0.413 at target 16, and the isotropic
  remesh output is healthy at every target.
- Decimation overshoot on the 4-tri input (`decimateIfTooDense`,
  `core/autoremesher.cpp:331-456`): ruled out. The phase report prints
  `Mesh simplifier: SKIPPED (no island above 8x target triangle count)`
  on every tetra run; the 8x guard at `:354` never fires on 4 triangles.
- Empty-island drop (`core/autoremesher.cpp:1204-1208`): mechanism, not
  root cause. The island enters parameterization non-empty (24 tris at
  target 8); the extractor yields zero quads and the merge skip converts
  that into empty output. Given an upstream zero-quad island, the drop
  plus the loud warning is correct behavior.

## Fix recommendation

Recommended: make `simplifyGraph` (`core/quadextractor.cpp:325-354`)
cycle-aware -- do not strip a valence-2 vertex when its removal would
destroy a cycle (both neighbors already adjacent, i.e. the vertex sits on
a triangle; or more generally the vertex belongs to an all-valence-2
connected component, which is by definition a pure cycle). Preserved loops
give `extractMesh` closed rings to extract faces from. This touches only
graphs that currently degenerate, so large-model output should be
unchanged -- but that must be proven, not assumed (see risk).

Alternative (narrower blast radius): in `extract()`, if `extractMesh`
yields zero faces while `connections` was non-empty, retry mesh extraction
on the pre-cleanup graph (skip the simplify/strip passes). This path only
triggers on inputs that fail today, so bench counts cannot move on
currently-passing models; output quality on tiny inputs would need review.

Risk estimate for either engine change: LOW-MEDIUM. Both alter extraction
only in cases that currently produce degenerate graphs, and bench models
are large meshes whose graphs carry branch points -- but small pure-cycle
components can occur on real models too, so `bench/run.py --check
bench/baseline.json` must come back count-identical plus full `ctest`
green, per the lane brief. If either gate moves, the fallback is
documentation only: tiny closed meshes need a target whose lattice
resolves (for the tetra: 4 works; bulky targets >= ~10 mostly work) --
the failure is already loud (non-zero exit, named cause), so no silent
corruption is at stake.

## Method appendix

- Stage table: `probe.cpp` (project library `tetra-probe/`, not committed),
  compiled against `build/libretopo_core.a` + `build/.../retopo.core.*.pcm`
  with `-framework Accelerate -ltbb`; uses public accessors only, no core
  modifications.
- Graph replays: `graphsim.py` (project library `tetra-probe/`),
  faithful re-implementation of `simplifyGraph` / `collapseShortEdges` /
  `collapseTriangles` / `removeSingleEndpoints`; post-cleanup node/edge
  counts match the instrumented dumps exactly at targets 2, 4, 8, 12.
- Instrumented build: separate `build-dev/` dir configured with
  `-DCMAKE_CXX_FLAGS="-DAUTO_REMESHER_DEV=1 -DAUTO_REMESHER_DEBUG=1"`;
  neither macro is defined by the normal build (both test false). No repo
  file was modified for instrumentation; `build/` and `build-dev/` are
  gitignored.
