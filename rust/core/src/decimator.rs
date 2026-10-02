//! Noise-stable Rust replacement for the vendored meshoptimizer decimator.
//!
//! Line-by-line port of the subset of meshoptimizer (zeux/meshoptimizer,
//! MIT) the pipeline calls: `generateVertexRemap` / `remapIndexBuffer` /
//! `remapVertexBuffer` (`indexgenerator.cpp`) and `meshopt_simplifyEdge`
//! (`simplifier.cpp`) as invoked through `meshopt_simplifyWithAttributes`
//! with no attributes, unbounded `target_error`, and `options =
//! meshopt_SimplifyRegularize`.
//!
//! Status: the default (and only) decimator since the item-6 flip —
//! the coverage retry (item 6b) robustified the downstream against this
//! port's re-tiling (beast@1000 recovers 8/8), so the meshoptimizer FFI,
//! its `build.rs` cc step, and `thirdparty/` were deleted.
//!
//! Unused upstream features are omitted, not stubbed: attribute quadrics,
//! sparse remap, component pruning, permissive (complex/fringe) collapse,
//! border locking, fold preservation, and the internal position/attribute
//! solve. The collapse-rule tables and [`VertexKind`] enum keep all six
//! upstream kinds so the transcribed indexing stays 1:1.
//!
//! ## Deliberate deviations from upstream
//!
//! 1. **f64 internals.** Upstream computes quadrics and errors in `f32`;
//!    this port widens the rescaled positions to `f64` (exact) and runs
//!    all quadric math, errors, and comparisons in `f64`, with the same
//!    op order. Sort keys narrow each error back to `f32` so the collapse
//!    buckets keep upstream's exact granularity.
//!
//!    Rationale (measured, not theorized): input noise here is ~1e-9
//!    absolute, ~60x smaller than the f32 rounding quantum, so in f32 the
//!    noise enters decisions mostly through *rounding flips* — perturbed
//!    values crossing rounding boundaries at ~ulp scale, 60x bigger than
//!    the noise itself. Evidence: the vendored C++ build (which contracts
//!    ~490 fused multiply-adds, i.e. far fewer rounding sites) held
//!    91-98% decimated-face overlap across noise seeds; the same C++ with
//!    `-ffp-contract=off` dropped to 73-99%, matching an f32 transcription
//!    seed-for-seed exactly (which also proves the transcription itself is
//!    bit-faithful). In f64 the rounding quantum (2^-52) sits ~1e8x below
//!    the noise, so rounding flips vanish and only true near-ties (gaps
//!    below ~1e-9, plus discrete snap-grid boundary flips) can flip.
//! 2. **Deterministic maps**: the upstream open-addressing hash tables are
//!    lookup-only for their results (canonical = first occurrence in
//!    iteration order), so `BTreeMap` computes the identical remaps without
//!    a hasher.
//!
//! Everything else (quadric math op order, adjacency counting sort,
//! classification, collapse rules, flip guards, exact direction and break
//! comparisons) transcribes upstream exactly. Bit-identity with upstream
//! is neither expected (f64 values, no FMA fusion to match) nor required:
//! the default pipeline stays on FFI meshopt, and this port holds its own
//! unit-test pins below.

use std::collections::BTreeMap;

// ---------------------------------------------------------------------------
// Weld (`indexgenerator.cpp`)
// ---------------------------------------------------------------------------

/// Canonical "no remap" sentinel, mirrors upstream `~0u`.
const NO_REMAP: u32 = u32::MAX;

/// Welds bitwise-identical vertices, mirroring
/// `meshopt_generateVertexRemap` for 12-byte vertices over an index buffer.
///
/// Iterates `indices` in order; the first occurrence of each bitwise-distinct
/// position (raw `memcmp` equality: `-0.0 != +0.0`, bitwise-equal NaNs unify)
/// becomes canonical and takes the next compact id. Unreferenced vertices
/// keep [`NO_REMAP`]. Returns the per-vertex remap and the welded count.
pub fn generate_vertex_remap(indices: &[u32], positions: &[f32]) -> (Vec<u32>, usize) {
    let vertex_count = positions.len() / 3;
    let mut remap = vec![NO_REMAP; vertex_count];
    // Canonical id per position-bits triple, in first-occurrence order.
    let mut table: BTreeMap<[u32; 3], u32> = BTreeMap::new();
    let mut next_vertex = 0u32;
    for &index in indices {
        let vertex = index as usize;
        if remap[vertex] != NO_REMAP {
            continue;
        }
        let key = [
            positions[vertex * 3].to_bits(),
            positions[vertex * 3 + 1].to_bits(),
            positions[vertex * 3 + 2].to_bits(),
        ];
        let canonical = *table.entry(key).or_insert_with(|| {
            let id = next_vertex;
            next_vertex += 1;
            id
        });
        remap[vertex] = canonical;
    }
    (remap, next_vertex as usize)
}

/// Applies a weld remap to an index buffer in place, mirroring
/// `meshopt_remapIndexBuffer` (which supports in-place remap).
pub fn remap_index_buffer_in_place(indices: &mut [u32], remap: &[u32]) {
    for destination in indices.iter_mut() {
        let source = *destination as usize;
        debug_assert_ne!(remap[source], NO_REMAP);
        *destination = remap[source];
    }
}

/// Gathers welded vertices, mirroring `meshopt_remapVertexBuffer` for
/// 12-byte vertices. Vertices whose remap is [`NO_REMAP`] are skipped.
pub fn remap_vertex_buffer(positions: &[f32], remap: &[u32], unique_count: usize) -> Vec<f32> {
    let mut destination = vec![0.0f32; unique_count * 3];
    for (vertex, &target) in remap.iter().enumerate() {
        if target != NO_REMAP {
            let source = vertex * 3;
            let target = target as usize * 3;
            destination[target..target + 3].copy_from_slice(&positions[source..source + 3]);
        }
    }
    destination
}

// ---------------------------------------------------------------------------
// Simplifier (`simplifier.cpp`)
// ---------------------------------------------------------------------------

/// Mirrors `meshopt_SimplifyVertex_Lock` (`meshoptimizer.h`: `1 << 0`).
const VERTEX_LOCK: u8 = 1 << 0;
/// Mirrors `meshopt_SimplifyVertex_Priority` (`meshoptimizer.h`: `1 << 2`).
/// The pipeline marks sharp-edge vertices with this bit.
pub(crate) const VERTEX_PRIORITY: u8 = 1 << 2;

/// Upstream `VertexKind` discriminants, kept in upstream order.
const KIND_MANIFOLD: u8 = 0;
const KIND_BORDER: u8 = 1;
const KIND_SEAM: u8 = 2;
const KIND_COMPLEX: u8 = 3;
const KIND_FRINGE: u8 = 4;
const KIND_LOCKED: u8 = 5;
const KIND_COUNT: usize = 6;

/// Upstream `kCanCollapse` table, transcribed verbatim.
const CAN_COLLAPSE: [[u8; KIND_COUNT]; KIND_COUNT] = [
    [1, 1, 1, 1, 1, 1],
    [0, 1, 0, 0, 1, 1],
    [0, 0, 1, 0, 0, 1],
    [0, 0, 0, 1, 1, 1],
    [0, 0, 0, 0, 1, 0],
    [0, 0, 0, 0, 0, 0],
];

/// Upstream `kHasOpposite` table, transcribed verbatim.
const HAS_OPPOSITE: [[u8; KIND_COUNT]; KIND_COUNT] = [
    [1, 1, 1, 1, 1, 1],
    [1, 0, 1, 0, 0, 0],
    [1, 1, 1, 0, 0, 1],
    [1, 0, 0, 0, 0, 0],
    [1, 0, 0, 0, 0, 0],
    [1, 0, 1, 0, 0, 0],
];

/// Point-quadric weight factor for `meshopt_SimplifyRegularize` (`1 << 4`),
/// the only options flag the pipeline passes (see upstream
/// `fillVertexQuadrics`: light = 1e-2, regularize = 1e-1, default = 1e-7).
const REGULARIZE_FACTOR: f64 = 1e-1;

/// Collapse sort key width: top 12 bits of exponent+mantissa, as upstream.
const SORT_BITS: u32 = 12;
/// Collapse sort bin count: exponent range [-127, 32), as upstream.
const SORT_BINS: usize = 2048 + 512;

/// Sort key of a collapse error, matching upstream `sortEdgeCollapses`.
/// The f64 error narrows to f32 (deterministic round-to-nearest) so the
/// buckets keep upstream's exact granularity (8 exponent + 4 mantissa
/// bits over exponent range [-127, 32)).
fn collapse_key(error: f64) -> usize {
    let bits = (error as f32).to_bits();
    let key = (bits.wrapping_shl(1) >> (32 - SORT_BITS)) as usize;
    key.min(SORT_BINS - 1)
}

/// Upstream `EdgeAdjacency::Edge`: the other two corners of a triangle as
/// seen from one corner (`next` follows face winding).
#[derive(Clone, Copy)]
struct Edge {
    next: u32,
    prev: u32,
}

/// Upstream `EdgeAdjacency`: per-vertex ranges into a shared edge list.
struct EdgeAdjacency {
    offsets: Vec<u32>,
    data: Vec<Edge>,
}

/// Upstream `Quadric`:
/// `a00*x^2 + a11*y^2 + a22*z^2 + 2*a10*xy + 2*a20*xz + 2*a21*yz +
/// 2*b0*x + 2*b1*y + 2*b2*z + c`, with cumulative weight `w`.
#[derive(Clone, Copy, Default)]
struct Quadric {
    a00: f64,
    a11: f64,
    a22: f64,
    a10: f64,
    a20: f64,
    a21: f64,
    b0: f64,
    b1: f64,
    b2: f64,
    c: f64,
    w: f64,
}

/// Upstream `Collapse`: `v0 -> v1` edge collapse. Upstream overlays `bidi`
/// and `error` in a union (pick writes the flag, rank overwrites it with
/// the error); here they are separate fields with the same lifecycle.
#[derive(Clone, Copy, Default)]
struct Collapse {
    v0: u32,
    v1: u32,
    bidi: bool,
    error: f64,
}

/// Normalizes `v` in place, mirroring upstream `normalize`; returns the
/// original length. Zero vectors are left untouched.
fn normalize(v: &mut [f64; 3]) -> f64 {
    let length = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if length > 0.0 {
        v[0] /= length;
        v[1] /= length;
        v[2] /= length;
    }
    length
}

/// Upstream `quadricAdd(Quadric&, const Quadric&)`.
fn quadric_add(q: &mut Quadric, r: &Quadric) {
    q.a00 += r.a00;
    q.a11 += r.a11;
    q.a22 += r.a22;
    q.a10 += r.a10;
    q.a20 += r.a20;
    q.a21 += r.a21;
    q.b0 += r.b0;
    q.b1 += r.b1;
    q.b2 += r.b2;
    q.c += r.c;
    q.w += r.w;
}

/// Upstream `quadricEval`.
fn quadric_eval(q: &Quadric, v: &[f64; 3]) -> f64 {
    let mut rx = (q.b0 + q.a10 * v[1]) * 2.0;
    let mut ry = (q.b1 + q.a21 * v[2]) * 2.0;
    let mut rz = (q.b2 + q.a20 * v[0]) * 2.0;

    rx += q.a00 * v[0];
    ry += q.a11 * v[1];
    rz += q.a22 * v[2];

    q.c + rx * v[0] + ry * v[1] + rz * v[2]
}

/// Upstream position-only `quadricError`.
fn quadric_error(q: &Quadric, v: &[f64; 3]) -> f64 {
    let r = quadric_eval(q, v);
    let s = if q.w == 0.0 { 0.0 } else { 1.0 / q.w };
    r.abs() * s
}

/// Upstream `quadricFromPlane`.
fn quadric_from_plane(q: &mut Quadric, a: f64, b: f64, c: f64, d: f64, w: f64) {
    let aw = a * w;
    let bw = b * w;
    let cw = c * w;
    let dw = d * w;

    q.a00 = a * aw;
    q.a11 = b * bw;
    q.a22 = c * cw;
    q.a10 = a * bw;
    q.a20 = a * cw;
    q.a21 = b * cw;
    q.b0 = a * dw;
    q.b1 = b * dw;
    q.b2 = c * dw;
    q.c = d * dw;
    q.w = w;
}

/// Upstream `quadricFromPoint`.
fn quadric_from_point(q: &mut Quadric, x: f64, y: f64, z: f64, w: f64) {
    q.a00 = w;
    q.a11 = w;
    q.a22 = w;
    q.a10 = 0.0;
    q.a20 = 0.0;
    q.a21 = 0.0;
    q.b0 = -x * w;
    q.b1 = -y * w;
    q.b2 = -z * w;
    q.c = (x * x + y * y + z * z) * w;
    q.w = w;
}

/// Upstream `quadricFromTriangle`.
fn quadric_from_triangle(
    q: &mut Quadric,
    p0: &[f64; 3],
    p1: &[f64; 3],
    p2: &[f64; 3],
    weight: f64,
) {
    let p10 = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]];
    let p20 = [p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]];

    // normal = cross(p1 - p0, p2 - p0)
    let mut normal = [
        p10[1] * p20[2] - p10[2] * p20[1],
        p10[2] * p20[0] - p10[0] * p20[2],
        p10[0] * p20[1] - p10[1] * p20[0],
    ];
    let area = normalize(&mut normal);

    let distance = normal[0] * p0[0] + normal[1] * p0[1] + normal[2] * p0[2];

    // we use sqrtf(area) so that the error is scaled linearly; this tends to improve silhouettes
    quadric_from_plane(
        q,
        normal[0],
        normal[1],
        normal[2],
        -distance,
        area.sqrt() * weight,
    );
}

/// Upstream `quadricFromTriangleEdge`.
fn quadric_from_triangle_edge(
    q: &mut Quadric,
    p0: &[f64; 3],
    p1: &[f64; 3],
    p2: &[f64; 3],
    weight: f64,
) {
    let p10 = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]];

    // edge length; keep squared length around for projection correction
    let lengthsq = p10[0] * p10[0] + p10[1] * p10[1] + p10[2] * p10[2];
    let length = lengthsq.sqrt();

    // p20p = length of projection of p2-p0 onto p1-p0; note that p10 is unnormalized so we need to correct it later
    let p20 = [p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]];
    let p20p = p20[0] * p10[0] + p20[1] * p10[1] + p20[2] * p10[2];

    // perp = perpendicular vector from p2 to line segment p1-p0
    // note: since p10 is unnormalized we need to correct the projection; we scale p20 instead to take advantage of normalize below
    let mut perp = [
        p20[0] * lengthsq - p10[0] * p20p,
        p20[1] * lengthsq - p10[1] * p20p,
        p20[2] * lengthsq - p10[2] * p20p,
    ];
    normalize(&mut perp);

    let distance = perp[0] * p0[0] + perp[1] * p0[1] + perp[2] * p0[2];

    // note: the weight is scaled linearly with edge length; this has to match the triangle weight
    quadric_from_plane(q, perp[0], perp[1], perp[2], -distance, length * weight);
}

/// Rebuilds `adjacency` for `indices` (`result` in upstream), mirroring
/// upstream `updateEdgeAdjacency`: a counting sort of corners by vertex
/// (mapped through `remap` when present, as in later passes).
fn update_edge_adjacency(
    adjacency: &mut EdgeAdjacency,
    indices: &[u32],
    vertex_count: usize,
    remap: Option<&[u32]>,
) {
    let offsets = &mut adjacency.offsets;
    let data = &mut adjacency.data;

    // fill edge counts (upstream writes through `offsets + 1`)
    for slot in offsets.iter_mut().skip(1) {
        *slot = 0;
    }
    for &index in indices {
        let vertex = match remap {
            Some(remap) => remap[index as usize],
            None => index,
        } as usize;
        debug_assert!(vertex < vertex_count);
        offsets[vertex + 1] += 1;
    }

    // fill offset table
    let mut offset = 0u32;
    for i in 0..vertex_count {
        let count = offsets[i + 1];
        offsets[i + 1] = offset;
        offset += count;
    }
    debug_assert_eq!(offset as usize, indices.len());

    // fill edge data
    for triangle in indices.as_chunks::<3>().0 {
        let (mut a, mut b, mut c) = (triangle[0], triangle[1], triangle[2]);
        if let Some(remap) = remap {
            a = remap[a as usize];
            b = remap[b as usize];
            c = remap[c as usize];
        }

        let head = offsets[a as usize + 1] as usize;
        data[head] = Edge { next: b, prev: c };
        offsets[a as usize + 1] += 1;

        let head = offsets[b as usize + 1] as usize;
        data[head] = Edge { next: c, prev: a };
        offsets[b as usize + 1] += 1;

        let head = offsets[c as usize + 1] as usize;
        data[head] = Edge { next: a, prev: b };
        offsets[c as usize + 1] += 1;
    }

    // finalize offsets
    offsets[0] = 0;
    debug_assert_eq!(offsets[vertex_count] as usize, indices.len());
}

/// Builds the position remap (each vertex -> the first vertex with an equal
/// position) and the cyclic wedge loops, mirroring upstream
/// `buildPositionRemap` without sparse support.
///
/// Upstream compares positions with float `==` (so `-0.0 == +0.0`, while
/// NaN never equals anything, not even itself); the map key normalizes
/// negative zero bits for the same result, and NaN positions map to
/// themselves without entering the table.
fn build_position_remap(positions: &[f32], vertex_count: usize) -> (Vec<u32>, Vec<u32>) {
    fn normalized_bits(v: f32) -> u32 {
        let bits = v.to_bits();
        if bits == 0x8000_0000 { 0 } else { bits }
    }

    let mut table: BTreeMap<[u32; 3], u32> = BTreeMap::new();
    let mut remap = vec![0u32; vertex_count];
    for i in 0..vertex_count {
        let x = positions[i * 3];
        let y = positions[i * 3 + 1];
        let z = positions[i * 3 + 2];
        if x.is_nan() || y.is_nan() || z.is_nan() {
            remap[i] = i as u32;
            continue;
        }
        let key = [normalized_bits(x), normalized_bits(y), normalized_bits(z)];
        remap[i] = *table.entry(key).or_insert(i as u32);
    }

    // build wedge table: for each vertex, which other vertex is the next wedge that also maps to the same vertex?
    // entries in table form a (cyclic) wedge loop per vertex; for manifold vertices, wedge[i] == remap[i] == i
    let mut wedge: Vec<u32> = (0..vertex_count as u32).collect();
    for i in 0..vertex_count {
        if remap[i] != i as u32 {
            let r = remap[i] as usize;
            wedge[i] = wedge[r];
            wedge[r] = i as u32;
        }
    }
    (remap, wedge)
}

/// Upstream two-argument `hasEdge`: does a directed adjacency edge `a -> b`
/// exist? (The remap/wedge variant only serves the omitted permissive
/// classification.)
fn has_edge(adjacency: &EdgeAdjacency, a: u32, b: u32) -> bool {
    let start = adjacency.offsets[a as usize] as usize;
    let end = adjacency.offsets[a as usize + 1] as usize;
    adjacency.data[start..end].iter().any(|edge| edge.next == b)
}

/// Classifies every vertex, mirroring upstream `classifyVertices` with the
/// pipeline's configuration: no permissive mode (complex/fringe truly
/// unreachable), no border locking. `vertex_lock` is `None` when the
/// pipeline passes null (no sharp-edge preservation requested).
///
/// `loop_`/`loopback` double as the open-edge scratch maps (`openout` is
/// `loop_`, `openinc` is `loopback`), exactly as upstream, and keep the
/// open-edge targets for the collapse passes.
#[allow(clippy::too_many_arguments)] // Port keeps the C++ parameter list 1:1.
fn classify_vertices(
    result: &mut [u8],
    loop_: &mut [u32],
    loopback: &mut [u32],
    vertex_count: usize,
    adjacency: &EdgeAdjacency,
    remap: &[u32],
    wedge: &[u32],
    vertex_lock: Option<&[u8]>,
) {
    loop_.fill(NO_REMAP);
    loopback.fill(NO_REMAP);

    // incoming & outgoing open edges: ~0u if no open edges, i if there are more than 1
    // note that this is the same data as required in loop[] arrays; loop[] data is only used for border/seam by default
    let openinc = loopback;
    let openout = loop_;

    for i in 0..vertex_count {
        let vertex = i as u32;

        let start = adjacency.offsets[vertex as usize] as usize;
        let end = adjacency.offsets[vertex as usize + 1] as usize;

        for edge in &adjacency.data[start..end] {
            let target = edge.next;

            if target == vertex {
                // degenerate triangles have two distinct edges instead of three, and the self edge
                // is bi-directional by definition; this can break border/seam classification by "closing"
                // the open edge from another triangle and falsely marking the vertex as manifold
                // instead we mark the vertex as having >1 open edges which turns it into locked/complex
                openinc[vertex as usize] = vertex;
                openout[vertex as usize] = vertex;
            } else if !has_edge(adjacency, target, vertex) {
                let incoming = &mut openinc[target as usize];
                *incoming = if *incoming == NO_REMAP {
                    vertex
                } else {
                    target
                };
                let outgoing = &mut openout[vertex as usize];
                *outgoing = if *outgoing == NO_REMAP {
                    target
                } else {
                    vertex
                };
            }
        }
    }

    for i in 0..vertex_count {
        if remap[i] == i as u32 {
            if wedge[i] == i as u32 {
                // no attribute seam, need to check if it's manifold
                let openi = openinc[i];
                let openo = openout[i];

                // note: we classify any vertices with no open edges as manifold
                // this is technically incorrect - if 4 triangles share an edge, we'll classify vertices as manifold
                // it's unclear if this is a problem in practice
                if openi == NO_REMAP && openo == NO_REMAP {
                    result[i] = KIND_MANIFOLD;
                } else if openi != NO_REMAP
                    && openo != NO_REMAP
                    && remap[openi as usize] == remap[openo as usize]
                    && openi != i as u32
                {
                    // classify half-seams as seams (the branch below would mis-classify them as borders)
                    // half-seam is a single vertex that connects to both vertices of a potential seam
                    // treating these as seams allows collapsing the "full" seam vertex onto them
                    result[i] = KIND_SEAM;
                } else if openi != i as u32 && openo != i as u32 {
                    result[i] = KIND_BORDER;
                } else {
                    result[i] = KIND_LOCKED;
                }
            } else if wedge[wedge[i] as usize] == i as u32 {
                // attribute seam; need to distinguish between Seam and Locked
                let w = wedge[i] as usize;
                let openiv = openinc[i];
                let openov = openout[i];
                let openiw = openinc[w];
                let openow = openout[w];

                // seam should have one open half-edge for each vertex, and the edges need to "connect" - point to the same vertex post-remap
                if openiv != NO_REMAP
                    && openiv != i as u32
                    && openov != NO_REMAP
                    && openov != i as u32
                    && openiw != NO_REMAP
                    && openiw != w as u32
                    && openow != NO_REMAP
                    && openow != w as u32
                    && remap[openiv as usize] == remap[openow as usize]
                    && remap[openov as usize] == remap[openiw as usize]
                    && remap[openiv as usize] != remap[openov as usize]
                {
                    result[i] = KIND_SEAM;
                } else {
                    // mismatched seam endpoints or disconnected seam; we don't have classification available
                    result[i] = KIND_LOCKED;
                }
            } else {
                // more than one vertex maps to this one; we don't have classification available
                result[i] = KIND_LOCKED;
            }
        } else {
            debug_assert!(remap[i] < i as u32);

            result[i] = result[remap[i] as usize];
        }
    }

    if let Some(vertex_lock) = vertex_lock {
        // vertex_lock may lock any wedge, not just the primary vertex, so we need to lock the primary vertex and relock any wedges
        for i in 0..vertex_count {
            if vertex_lock[i] & VERTEX_LOCK != 0 {
                result[remap[i] as usize] = KIND_LOCKED;
            }
        }

        for i in 0..vertex_count {
            if result[remap[i] as usize] == KIND_LOCKED {
                result[i] = KIND_LOCKED;
            }
        }
    }
}

/// Copies positions into `result` normalized into the unit box and returns
/// the extent, mirroring upstream `rescalePositions` without sparse
/// support. The strict comparisons keep the first minimum/maximum on ties;
/// the transcriptions below preserve upstream NaN behavior (comparisons
/// are false, so NaN never displaces an extreme).
fn rescale_positions(positions: &[f32], vertex_count: usize) -> (Vec<[f64; 3]>, f64) {
    let mut result = vec![[0.0; 3]; vertex_count];

    let mut minv = [f64::MAX; 3];
    let mut maxv = [-f64::MAX; 3];

    for i in 0..vertex_count {
        // f32 -> f64 widening is exact; all downstream math runs in f64.
        let v = [
            positions[i * 3] as f64,
            positions[i * 3 + 1] as f64,
            positions[i * 3 + 2] as f64,
        ];
        result[i] = v;

        for j in 0..3 {
            let vj = v[j];
            minv[j] = if minv[j] > vj { vj } else { minv[j] };
            maxv[j] = if maxv[j] < vj { vj } else { maxv[j] };
        }
    }

    let mut extent = 0.0;
    for j in 0..3 {
        let range = maxv[j] - minv[j];
        extent = if range < extent { extent } else { range };
    }

    let scale = if extent == 0.0 { 0.0 } else { 1.0 / extent };
    for v in result.iter_mut() {
        v[0] = (v[0] - minv[0]) * scale;
        v[1] = (v[1] - minv[1]) * scale;
        v[2] = (v[2] - minv[2]) * scale;
    }

    (result, extent)
}

/// Accumulates one face quadric per corner, mirroring upstream
/// `fillFaceQuadrics` without volume gradients (those serve the omitted
/// attribute solve).
fn fill_face_quadrics(
    vertex_quadrics: &mut [Quadric],
    indices: &[u32],
    vertex_positions: &[[f64; 3]],
    remap: &[u32],
) {
    for triangle in indices.as_chunks::<3>().0 {
        let i0 = triangle[0] as usize;
        let i1 = triangle[1] as usize;
        let i2 = triangle[2] as usize;

        let mut q = Quadric::default();
        quadric_from_triangle(
            &mut q,
            &vertex_positions[i0],
            &vertex_positions[i1],
            &vertex_positions[i2],
            1.0,
        );

        let target = remap[i0] as usize;
        let existing = vertex_quadrics[target];
        let mut merged = existing;
        quadric_add(&mut merged, &q);
        vertex_quadrics[target] = merged;

        let target = remap[i1] as usize;
        let mut merged = vertex_quadrics[target];
        quadric_add(&mut merged, &q);
        vertex_quadrics[target] = merged;

        let target = remap[i2] as usize;
        let mut merged = vertex_quadrics[target];
        quadric_add(&mut merged, &q);
        vertex_quadrics[target] = merged;
    }
}

/// Adds stabilizing point quadrics, mirroring upstream `fillVertexQuadrics`
/// with `meshopt_SimplifyRegularize` (see [`REGULARIZE_FACTOR`]).
/// Priority-locked vertices (sharp edges) keep full weight.
fn fill_vertex_quadrics(
    vertex_quadrics: &mut [Quadric],
    vertex_positions: &[[f64; 3]],
    vertex_count: usize,
    remap: &[u32],
    vertex_lock: Option<&[u8]>,
) {
    for i in 0..vertex_count {
        if remap[i] != i as u32 {
            continue;
        }

        // increase regularization weight for vertices marked as priority; for now we only examine the primary vertex
        let priority = vertex_lock
            .map(|lock| lock[i] & VERTEX_PRIORITY != 0)
            .unwrap_or(false);

        let p = vertex_positions[i];
        let w = vertex_quadrics[i].w * if priority { 1.0 } else { REGULARIZE_FACTOR };

        let mut q = Quadric::default();
        quadric_from_point(&mut q, p[0], p[1], p[2], w);

        let mut merged = vertex_quadrics[i];
        quadric_add(&mut merged, &q);
        vertex_quadrics[i] = merged;
    }
}

/// Adds border/seam edge quadrics, mirroring upstream `fillEdgeQuadrics`.
/// The fringe/complex guards are transcribed verbatim although those kinds
/// never occur without permissive mode.
fn fill_edge_quadrics(
    vertex_quadrics: &mut [Quadric],
    indices: &[u32],
    vertex_positions: &[[f64; 3]],
    remap: &[u32],
    vertex_kind: &[u8],
    loop_: &[u32],
    loopback: &[u32],
) {
    const NEXT: [usize; 4] = [1, 2, 0, 1];

    for triangle in indices.as_chunks::<3>().0 {
        for e in 0..3 {
            let i0 = triangle[e] as usize;
            let i1 = triangle[NEXT[e]] as usize;

            let k0 = vertex_kind[i0];
            let k1 = vertex_kind[i1];

            // early out: manifold vertices never lie on a border
            if k0 == KIND_MANIFOLD || k1 == KIND_MANIFOLD {
                continue;
            }

            // one of the vertices must be border/seam/fringe; additional checks help validate the border
            if k0 != KIND_BORDER
                && k0 != KIND_SEAM
                && k0 != KIND_FRINGE
                && k1 != KIND_BORDER
                && k1 != KIND_SEAM
                && k1 != KIND_FRINGE
            {
                continue;
            }

            // if i0/i1 are border/seam we check if they are on the same edge loop
            // note that we need to add the error even for edges that connect e.g. border & locked
            // if we don't do that, the adjacent border->border edge won't have correct errors for corners
            if (k0 == KIND_BORDER || k0 == KIND_SEAM) && loop_[i0] != i1 as u32 {
                continue;
            }

            if (k1 == KIND_BORDER || k1 == KIND_SEAM) && loopback[i1] != i0 as u32 {
                continue;
            }

            // fringe vertices don't have proper edge loops, so we need to add edge quadrics to fringe-fringe or fringe-locked edges
            // note that border-fringe edges are still processed, guarded by the loop check above
            if (k0 == KIND_FRINGE || k1 == KIND_FRINGE)
                && (k0 == KIND_COMPLEX || k1 == KIND_COMPLEX)
            {
                continue;
            }

            let i2 = triangle[NEXT[e + 1]] as usize;

            // we try hard to maintain border edge geometry; seam edges can move more freely
            // due to topological restrictions on collapses, seam quadrics slightly improve collapse structure but aren't critical
            const EDGE_WEIGHT_SEAM: f64 = 0.5; // applied twice due to opposite edges
            const EDGE_WEIGHT_BORDER: f64 = 10.0;

            let edge_weight = if k0 == KIND_SEAM || k1 == KIND_SEAM {
                EDGE_WEIGHT_SEAM
            } else {
                EDGE_WEIGHT_BORDER
            };

            let mut q = Quadric::default();
            quadric_from_triangle_edge(
                &mut q,
                &vertex_positions[i0],
                &vertex_positions[i1],
                &vertex_positions[i2],
                edge_weight,
            );

            let mut qt = Quadric::default();
            quadric_from_triangle(
                &mut qt,
                &vertex_positions[i0],
                &vertex_positions[i1],
                &vertex_positions[i2],
                edge_weight,
            );

            // mix edge quadric with triangle quadric to stabilize collapses in both directions; both quadrics inherit edge weight so that their error is added
            qt.w = 0.0;
            quadric_add(&mut q, &qt);

            let target = remap[i0] as usize;
            let mut merged = vertex_quadrics[target];
            quadric_add(&mut merged, &q);
            vertex_quadrics[target] = merged;

            let target = remap[i1] as usize;
            let mut merged = vertex_quadrics[target];
            quadric_add(&mut merged, &q);
            vertex_quadrics[target] = merged;
        }
    }
}

/// Upper-bounds the per-pass collapse list, mirroring upstream
/// `boundEdgeCollapses`.
fn bound_edge_collapses(
    adjacency: &EdgeAdjacency,
    vertex_count: usize,
    index_count: usize,
    vertex_kind: &[u8],
) -> usize {
    let mut dual_count = 0usize;

    for i in 0..vertex_count {
        let k = vertex_kind[i];
        let e = adjacency.offsets[i + 1] - adjacency.offsets[i];

        dual_count += if k == KIND_MANIFOLD || k == KIND_SEAM {
            e as usize
        } else {
            0
        };
    }

    debug_assert!(dual_count <= index_count);

    // pad capacity by 3 so that we can check for overflow once per triangle instead of once per edge
    (index_count - dual_count / 2) + 3
}

/// Collects collapsible directed edges, mirroring upstream
/// `pickEdgeCollapses`. Returns the number of entries written to the
/// front of `collapses`.
fn pick_edge_collapses(
    collapses: &mut [Collapse],
    indices: &[u32],
    remap: &[u32],
    vertex_kind: &[u8],
    loop_: &[u32],
    loopback: &[u32],
) -> usize {
    const NEXT: [usize; 3] = [1, 2, 0];

    let collapse_capacity = collapses.len();
    let mut collapse_count = 0usize;

    for triangle in indices.as_chunks::<3>().0 {
        // this should never happen as boundEdgeCollapses should give an upper bound for the collapse count, but in an unlikely event it does we can just drop extra collapses
        if collapse_count + 3 > collapse_capacity {
            break;
        }

        for e in 0..3 {
            let i0 = triangle[e];
            let i1 = triangle[NEXT[e]];

            // this can happen either when input has a zero-length edge, or when we perform collapses for complex
            // topology w/seams and collapse a manifold vertex that connects to both wedges onto one of them
            // we leave edges like this alone since they may be important for preserving mesh integrity
            if remap[i0 as usize] == remap[i1 as usize] {
                continue;
            }

            let k0 = vertex_kind[i0 as usize];
            let k1 = vertex_kind[i1 as usize];

            // the edge has to be collapsible in at least one direction
            if CAN_COLLAPSE[k0 as usize][k1 as usize] == 0
                && CAN_COLLAPSE[k1 as usize][k0 as usize] == 0
            {
                continue;
            }

            // manifold and seam edges should occur twice (i0->i1 and i1->i0) - skip redundant edges
            if HAS_OPPOSITE[k0 as usize][k1 as usize] != 0
                && remap[i1 as usize] > remap[i0 as usize]
            {
                continue;
            }

            // two vertices are on a border or a seam, but there's no direct edge between them
            // this indicates that they belong to two different edge loops and we should not collapse this edge
            // loop[] and loopback[] track half edges so we only need to check one of them
            if (k0 == KIND_BORDER || k0 == KIND_SEAM)
                && k1 != KIND_MANIFOLD
                && loop_[i0 as usize] != i1
            {
                continue;
            }
            if (k1 == KIND_BORDER || k1 == KIND_SEAM)
                && k0 != KIND_MANIFOLD
                && loopback[i1 as usize] != i0
            {
                continue;
            }

            // edge can be collapsed in either direction - we will pick the one with minimum error
            // note: we evaluate error later during collapse ranking, here we just tag the edge as bidirectional
            if CAN_COLLAPSE[k0 as usize][k1 as usize] != 0
                && CAN_COLLAPSE[k1 as usize][k0 as usize] != 0
            {
                collapses[collapse_count] = Collapse {
                    v0: i0,
                    v1: i1,
                    bidi: true,
                    error: 0.0,
                };
                collapse_count += 1;
            } else {
                // edge can only be collapsed in one direction
                let forward = CAN_COLLAPSE[k0 as usize][k1 as usize] != 0;
                let e0 = if forward { i0 } else { i1 };
                let e1 = if forward { i1 } else { i0 };

                collapses[collapse_count] = Collapse {
                    v0: e0,
                    v1: e1,
                    bidi: false,
                    error: 0.0,
                };
                collapse_count += 1;
            }
        }
    }

    collapse_count
}

/// Scores collapses and fixes bidirectional collapse directions, mirroring
/// upstream `rankEdgeCollapses` without attributes (so without the seam /
/// complex wedge error aggregation, which only aggregates attribute
/// errors).
fn rank_edge_collapses(
    collapses: &mut [Collapse],
    vertex_positions: &[[f64; 3]],
    vertex_quadrics: &[Quadric],
    remap: &[u32],
) {
    for collapse in collapses.iter_mut() {
        let i0 = collapse.v0;
        let i1 = collapse.v1;
        let bidi = collapse.bidi;

        let ei = quadric_error(
            &vertex_quadrics[remap[i0 as usize] as usize],
            &vertex_positions[i1 as usize],
        );
        let ej = if bidi {
            quadric_error(
                &vertex_quadrics[remap[i1 as usize] as usize],
                &vertex_positions[i0 as usize],
            )
        } else {
            f64::MAX
        };

        // pick edge direction with minimal error (branchless)
        let rev = bidi && ej < ei;

        collapse.v0 = if rev { i1 } else { i0 };
        collapse.v1 = if rev { i0 } else { i1 };
        collapse.error = if ej < ei { ej } else { ei };
    }
}

/// Orders collapses by error with a stable counting sort on 12-bit error
/// keys, mirroring upstream `sortEdgeCollapses`. Within a bucket, pick
/// order (face order) is preserved, which is noise-independent.
fn sort_edge_collapses(sort_order: &mut [u32], collapses: &[Collapse]) {
    debug_assert_eq!(sort_order.len(), collapses.len());

    // fill histogram for counting sort
    let mut histogram = vec![0u32; SORT_BINS];
    for collapse in collapses {
        histogram[collapse_key(collapse.error)] += 1;
    }

    // compute offsets based on histogram data
    let mut histogram_sum = 0usize;
    for slot in histogram.iter_mut() {
        let count = *slot as usize;
        *slot = histogram_sum as u32;
        histogram_sum += count;
    }
    debug_assert_eq!(histogram_sum, collapses.len());

    // compute sort order based on offsets
    for (i, collapse) in collapses.iter().enumerate() {
        let key = collapse_key(collapse.error);
        sort_order[histogram[key] as usize] = i as u32;
        histogram[key] += 1;
    }
}

/// Upstream `hasTriangleFlip`: does triangle ABC flip when C is replaced
/// with D?
fn has_triangle_flip(a: &[f64; 3], b: &[f64; 3], c: &[f64; 3], d: &[f64; 3]) -> bool {
    let eb = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let ec = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
    let ed = [d[0] - a[0], d[1] - a[1], d[2] - a[2]];

    let nbc = [
        eb[1] * ec[2] - eb[2] * ec[1],
        eb[2] * ec[0] - eb[0] * ec[2],
        eb[0] * ec[1] - eb[1] * ec[0],
    ];
    let nbd = [
        eb[1] * ed[2] - eb[2] * ed[1],
        eb[2] * ed[0] - eb[0] * ed[2],
        eb[0] * ed[1] - eb[1] * ed[0],
    ];

    let ndp = nbc[0] * nbd[0] + nbc[1] * nbd[1] + nbc[2] * nbd[2];
    let abc = nbc[0] * nbc[0] + nbc[1] * nbc[1] + nbc[2] * nbc[2];
    let abd = nbd[0] * nbd[0] + nbd[1] * nbd[1] + nbd[2] * nbd[2];

    // scale is cos(angle); somewhat arbitrarily set to ~75 degrees
    // note that the "pure" check is ndp <= 0 (90 degree cutoff) but that allows flipping through a series of close-to-90 collapses
    ndp <= 0.25 * (abc * abd).sqrt()
}

/// Upstream `hasTriangleFlips` (collapse-remap variant): would collapsing
/// `i0 -> i1` flip any surviving triangle around `i0`? (The single-position
/// variant only serves the omitted attribute solve.)
fn has_triangle_flips(
    adjacency: &EdgeAdjacency,
    vertex_positions: &[[f64; 3]],
    collapse_remap: &[u32],
    i0: u32,
    i1: u32,
) -> bool {
    debug_assert_eq!(collapse_remap[i0 as usize], i0);
    debug_assert_eq!(collapse_remap[i1 as usize], i1);

    let v0 = vertex_positions[i0 as usize];
    let v1 = vertex_positions[i1 as usize];

    let start = adjacency.offsets[i0 as usize] as usize;
    let end = adjacency.offsets[i0 as usize + 1] as usize;

    for edge in &adjacency.data[start..end] {
        let a = collapse_remap[edge.next as usize];
        let b = collapse_remap[edge.prev as usize];

        // skip triangles that will get collapsed by i0->i1 collapse or already got collapsed previously
        if a == i1 || b == i1 || a == b {
            continue;
        }

        // early-out when at least one triangle flips due to a collapse
        if has_triangle_flip(
            &vertex_positions[a as usize],
            &vertex_positions[b as usize],
            &v0,
            &v1,
        ) {
            return true;
        }
    }

    false
}

/// Upstream `getComplexTarget`: guides complex collapses towards the
/// correct wedge using loop metadata. Transcribed for the complex branch
/// of [`perform_edge_collapses`], which is unreachable without permissive
/// mode.
fn get_complex_target(v: u32, target: u32, remap: &[u32], loop_: &[u32], loopback: &[u32]) -> u32 {
    let r = remap[target as usize];

    // use loop metadata to guide complex collapses towards the correct wedge
    // this works for edges on attribute discontinuities because loop/loopback track the single half-edge without a pair, similar to seams
    if loop_[v as usize] != NO_REMAP && remap[loop_[v as usize] as usize] == r {
        loop_[v as usize]
    } else if loopback[v as usize] != NO_REMAP && remap[loopback[v as usize] as usize] == r {
        loopback[v as usize]
    } else {
        target
    }
}

/// Executes one pass of collapses in sorted order, mirroring upstream
/// `performEdgeCollapses`. Returns the number of executed edge collapses.
/// (The `error_limit` break never fires: the call site passes infinite
/// error.)
#[allow(clippy::too_many_arguments)]
fn perform_edge_collapses(
    collapse_remap: &mut [u32],
    collapse_locked: &mut [u8],
    collapses: &[Collapse],
    collapse_order: &[u32],
    remap: &[u32],
    wedge: &[u32],
    vertex_kind: &[u8],
    loop_: &[u32],
    loopback: &[u32],
    vertex_positions: &[[f64; 3]],
    adjacency: &EdgeAdjacency,
    triangle_collapse_goal: usize,
    error_limit: f64,
    result_error: &mut f64,
) -> usize {
    let collapse_count = collapses.len();
    let mut edge_collapses = 0usize;
    let mut triangle_collapses = 0usize;

    // most collapses remove 2 triangles; use this to establish a bound on the pass in terms of error limit
    // note that edge_collapse_goal is an estimate; triangle_collapse_goal will be used to actually limit collapses
    let mut edge_collapse_goal = triangle_collapse_goal / 2;

    for i in 0..collapse_count {
        let c = &collapses[collapse_order[i] as usize];

        if c.error > error_limit {
            break;
        }

        if triangle_collapses >= triangle_collapse_goal {
            break;
        }

        // we limit the error in each pass based on the error of optimal last collapse; since many collapses will be locked
        // as they will share vertices with other successful collapses, we need to increase the acceptable error by some factor
        let error_goal = if edge_collapse_goal < collapse_count {
            1.5 * collapses[collapse_order[edge_collapse_goal] as usize].error
        } else {
            f64::MAX
        };

        // on average, each collapse is expected to lock 6 other collapses; to avoid degenerate passes on meshes with odd
        // topology, we only abort if we got over 1/6 collapses accordingly.
        if c.error > error_goal
            && c.error > *result_error
            && triangle_collapses > triangle_collapse_goal / 6
        {
            break;
        }

        let i0 = c.v0;
        let i1 = c.v1;

        let r0 = remap[i0 as usize];
        let r1 = remap[i1 as usize];

        let kind = vertex_kind[i0 as usize];

        // we don't collapse vertices that had source or target vertex involved in a collapse
        // it's important to not move the vertices twice since it complicates the tracking/remapping logic
        // it's important to not move other vertices towards a moved vertex to preserve error since we don't re-rank collapses mid-pass
        if collapse_locked[r0 as usize] != 0 || collapse_locked[r1 as usize] != 0 {
            continue;
        }

        if has_triangle_flips(adjacency, vertex_positions, collapse_remap, r0, r1) {
            // adjust collapse goal since this collapse is invalid and shouldn't factor into error goal
            edge_collapse_goal += 1;

            continue;
        }

        debug_assert_eq!(collapse_remap[r0 as usize], r0);
        debug_assert_eq!(collapse_remap[r1 as usize], r1);

        if kind == KIND_COMPLEX || kind == KIND_FRINGE {
            collapse_remap[i0 as usize] = i1;

            // remap all vertices in the complex to the target vertex, using the same ranking that we used to evaluate the collapses
            let mut v = wedge[i0 as usize];
            while v != i0 {
                collapse_remap[v as usize] = get_complex_target(v, i1, remap, loop_, loopback);
                v = wedge[v as usize];
            }
        } else if kind == KIND_SEAM {
            // for seam collapses we need to move the seam pair together; this is a bit tricky since we need to rely on edge loops as target vertex may be locked (and thus have more than two wedges)
            let s0 = wedge[i0 as usize];
            let s1 = if loop_[i0 as usize] == i1 {
                loopback[s0 as usize]
            } else {
                loop_[s0 as usize]
            };
            debug_assert_eq!(wedge[s0 as usize], i0); // s0 may be equal to i0 for half-seams
            debug_assert_ne!(s1, NO_REMAP);
            debug_assert_eq!(remap[s1 as usize], r1);

            // additional asserts to verify that the seam pair is consistent
            debug_assert!(kind != vertex_kind[i1 as usize] || s1 == wedge[i1 as usize]);
            debug_assert!(loop_[i0 as usize] == i1 || loopback[i0 as usize] == i1);
            debug_assert!(loop_[s0 as usize] == s1 || loopback[s0 as usize] == s1);

            // note: this should never happen due to the assertion above, but when disabled if we ever hit this case we'll get a memory safety issue; for now play it safe
            let s1 = if s1 != NO_REMAP {
                s1
            } else {
                wedge[i1 as usize]
            };

            collapse_remap[i0 as usize] = i1;
            collapse_remap[s0 as usize] = s1;
        } else {
            debug_assert_eq!(wedge[i0 as usize], i0);

            collapse_remap[i0 as usize] = i1;
        }

        // note: we technically don't need to lock r1 if it's a locked vertex, as it can't move and its quadric won't be used
        // however, this results in slightly worse error on some meshes because the locked collapses get an unfair advantage wrt scheduling
        collapse_locked[r0 as usize] = 1;
        collapse_locked[r1 as usize] = 1;

        // border edges collapse 1 triangle, other edges collapse 2 or more
        // fringe edges often collapse 1 but may also collapse 2 because they don't respect loops, so we conservatively assume 2
        triangle_collapses += if kind == KIND_BORDER { 1 } else { 2 };
        edge_collapses += 1;

        *result_error = if *result_error < c.error {
            c.error
        } else {
            *result_error
        };
    }

    edge_collapses
}

/// Merges source quadrics into collapse targets, mirroring upstream
/// `updateQuadrics` without attributes (without attributes,
/// `vertex_error == result_error`, so no error recomputation is needed).
fn update_quadrics(
    collapse_remap: &[u32],
    vertex_count: usize,
    vertex_quadrics: &mut [Quadric],
    remap: &[u32],
) {
    for i in 0..vertex_count {
        if collapse_remap[i] == i as u32 {
            continue;
        }

        let i0 = i as u32;
        let i1 = collapse_remap[i];

        let r0 = remap[i0 as usize];
        let r1 = remap[i1 as usize];

        // ensure we only update vertex_quadrics once: primary vertex must be moved if any wedge is moved
        if i0 == r0 {
            let source = vertex_quadrics[r0 as usize];
            let mut merged = vertex_quadrics[r1 as usize];
            quadric_add(&mut merged, &source);
            vertex_quadrics[r1 as usize] = merged;
        }
    }
}

/// Rewrites edge loops through the pass remap, mirroring upstream
/// `remapEdgeLoops`.
fn remap_edge_loops(loop_: &mut [u32], collapse_remap: &[u32]) {
    for i in 0..loop_.len() {
        // note: this is a no-op for vertices that were remapped
        // ideally we would clear the loop entries for those for consistency, even though they aren't going to be used
        // however, the remapping process needs loop information for remapped vertices, so this would require a separate pass
        if loop_[i] != NO_REMAP {
            let l = loop_[i];
            let r = collapse_remap[l as usize];

            // i == r is a special case when the seam edge is collapsed in a direction opposite to where loop goes
            loop_[i] = if i as u32 == r {
                if loop_[l as usize] != NO_REMAP {
                    collapse_remap[loop_[l as usize] as usize]
                } else {
                    NO_REMAP
                }
            } else {
                r
            };
        }
    }
}

/// Rewrites the index buffer through the pass remap and drops degenerate
/// triangles, mirroring upstream `remapIndexBuffer` (the `indices` flavor
/// used on `result`). Returns the surviving index count.
fn remap_index_buffer(indices: &mut [u32], collapse_remap: &[u32], remap: &[u32]) -> usize {
    let mut write = 0usize;

    let mut i = 0;
    while i + 3 <= indices.len() {
        let v0 = collapse_remap[indices[i] as usize];
        let v1 = collapse_remap[indices[i + 1] as usize];
        let v2 = collapse_remap[indices[i + 2] as usize];

        // we never move the vertex twice during a single pass
        debug_assert_eq!(collapse_remap[v0 as usize], v0);
        debug_assert_eq!(collapse_remap[v1 as usize], v1);
        debug_assert_eq!(collapse_remap[v2 as usize], v2);

        // collapse zero area triangles even if they are not topologically degenerate
        // this is required to cleanup manifold->seam collapses when a vertex is collapsed onto a seam pair
        // as well as complex collapses and some other cases where cross wedge collapses are performed
        let r0 = remap[v0 as usize];
        let r1 = remap[v1 as usize];
        let r2 = remap[v2 as usize];

        if r0 != r1 && r0 != r2 && r1 != r2 {
            indices[write] = v0;
            indices[write + 1] = v1;
            indices[write + 2] = v2;
            write += 3;
        }
        i += 3;
    }

    write
}

/// Edge-collapse simplification of an indexed triangle mesh, mirroring
/// upstream `meshopt_simplifyEdge` as the pipeline calls it: no
/// attributes, no sparse/prune/permissive/lock-border options, unbounded
/// `target_error` (never error-limited), regularize on.
///
/// `positions` holds the welded mesh (flat `f32` triples),
/// `vertex_lock` carries `VERTEX_PRIORITY` bits for sharp-edge vertices
/// (`None` = no sharp-edge preservation), and `target_index_count` is the
/// triangle target times three. Returns the decimated index buffer
/// (indices into `positions`) and the linear result error.
pub fn simplify(
    positions: &[f32],
    indices: &[u32],
    vertex_lock: Option<&[u8]>,
    target_index_count: usize,
) -> (Vec<u32>, f64) {
    debug_assert_eq!(indices.len() % 3, 0);
    debug_assert!(target_index_count <= indices.len());
    if let Some(lock) = vertex_lock {
        debug_assert_eq!(lock.len(), positions.len() / 3);
    }

    let vertex_count = positions.len() / 3;
    let index_count = indices.len();

    let mut result = indices.to_vec();

    // build adjacency information
    let mut adjacency = EdgeAdjacency {
        offsets: vec![0u32; vertex_count + 1],
        data: vec![Edge { next: 0, prev: 0 }; index_count],
    };
    update_edge_adjacency(&mut adjacency, &result, vertex_count, None);

    // build a position remap that maps each vertex to the one with identical position
    // wedge table stores next vertex with identical position for each vertex
    let (remap, wedge) = build_position_remap(positions, vertex_count);

    // classify vertices; vertex kind determines collapse rules, see CAN_COLLAPSE
    let mut vertex_kind = vec![0u8; vertex_count];
    let mut loop_ = vec![NO_REMAP; vertex_count];
    let mut loopback = vec![NO_REMAP; vertex_count];
    classify_vertices(
        &mut vertex_kind,
        &mut loop_,
        &mut loopback,
        vertex_count,
        &adjacency,
        &remap,
        &wedge,
        vertex_lock,
    );

    let (vertex_positions, _extent) = rescale_positions(positions, vertex_count);

    let mut vertex_quadrics = vec![Quadric::default(); vertex_count];

    fill_face_quadrics(&mut vertex_quadrics, &result, &vertex_positions, &remap);
    fill_vertex_quadrics(
        &mut vertex_quadrics,
        &vertex_positions,
        vertex_count,
        &remap,
        vertex_lock,
    );
    fill_edge_quadrics(
        &mut vertex_quadrics,
        &result,
        &vertex_positions,
        &remap,
        &vertex_kind,
        &loop_,
        &loopback,
    );

    let collapse_capacity =
        bound_edge_collapses(&adjacency, vertex_count, index_count, &vertex_kind);

    let mut edge_collapses = vec![Collapse::default(); collapse_capacity];
    let mut collapse_order = vec![0u32; collapse_capacity];
    let mut collapse_remap = vec![0u32; vertex_count];
    let mut collapse_locked = vec![0u8; vertex_count];

    let mut result_count = index_count;
    let mut result_error = 0.0;

    // target_error input is linear; we need to adjust it to match quadricError units.
    // the call site passes unbounded error with no absolute-error scaling,
    // so the limit is infinite and the error break below never fires.
    let error_limit = f64::MAX * f64::MAX;
    debug_assert!(error_limit.is_infinite());

    while result_count > target_index_count {
        // note: throughout the simplification process adjacency structure reflects welded topology for result-in-progress
        update_edge_adjacency(
            &mut adjacency,
            &result[..result_count],
            vertex_count,
            Some(&remap),
        );

        let edge_collapse_count = pick_edge_collapses(
            &mut edge_collapses,
            &result[..result_count],
            &remap,
            &vertex_kind,
            &loop_,
            &loopback,
        );
        debug_assert!(edge_collapse_count <= collapse_capacity);

        // no edges can be collapsed any more due to topology restrictions
        if edge_collapse_count == 0 {
            break;
        }

        rank_edge_collapses(
            &mut edge_collapses[..edge_collapse_count],
            &vertex_positions,
            &vertex_quadrics,
            &remap,
        );

        sort_edge_collapses(
            &mut collapse_order[..edge_collapse_count],
            &edge_collapses[..edge_collapse_count],
        );

        let triangle_collapse_goal = (result_count - target_index_count) / 3;

        for (i, slot) in collapse_remap.iter_mut().enumerate() {
            *slot = i as u32;
        }
        collapse_locked.fill(0);

        let collapses = perform_edge_collapses(
            &mut collapse_remap,
            &mut collapse_locked,
            &edge_collapses[..edge_collapse_count],
            &collapse_order[..edge_collapse_count],
            &remap,
            &wedge,
            &vertex_kind,
            &loop_,
            &loopback,
            &vertex_positions,
            &adjacency,
            triangle_collapse_goal,
            error_limit,
            &mut result_error,
        );

        // no edges can be collapsed any more due to hitting the error limit or triangle collapse limit
        if collapses == 0 {
            break;
        }

        update_quadrics(&collapse_remap, vertex_count, &mut vertex_quadrics, &remap);

        // note: we update loops following edge collapses, but after this we might still have stale loop data
        // this can happen when a triangle with a loop edge gets collapsed along a non-loop edge
        // that works since a loop that points to a vertex that is no longer connected is not affecting collapse logic
        remap_edge_loops(&mut loop_, &collapse_remap);
        remap_edge_loops(&mut loopback, &collapse_remap);

        result_count = remap_index_buffer(&mut result[..result_count], &collapse_remap, &remap);
    }

    result.truncate(result_count);

    // result_error is quadratic; we need to remap it back to linear
    (result, result_error.sqrt())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 6x6 wavy grid (50 triangles) for golden pins.
    fn grid_mesh() -> (Vec<f32>, Vec<u32>) {
        let n = 6usize;
        let mut positions = Vec::new();
        let mut indices = Vec::new();
        for j in 0..n {
            for i in 0..n {
                let x = i as f32 / (n - 1) as f32;
                let y = j as f32 / (n - 1) as f32;
                positions.push(x);
                positions.push(y);
                positions.push((x * 9.0).sin() * (y * 7.0).cos() * 0.15);
            }
        }
        for j in 0..n - 1 {
            for i in 0..n - 1 {
                let a = (j * n + i) as u32;
                let b = (j * n + i + 1) as u32;
                let c = ((j + 1) * n + i) as u32;
                let d = ((j + 1) * n + i + 1) as u32;
                indices.extend_from_slice(&[a, b, c, b, d, c]);
            }
        }
        (positions, indices)
    }

    fn weld_grid() -> (Vec<f32>, Vec<u32>, usize) {
        let (positions, indices) = grid_mesh();
        let (remap, unique) = generate_vertex_remap(&indices, &positions);
        let mut welded_indices = indices.clone();
        remap_index_buffer_in_place(&mut welded_indices, &remap);
        let welded_positions = remap_vertex_buffer(&positions, &remap, unique);
        (welded_positions, welded_indices, unique)
    }

    #[test]
    fn weld_first_occurrence_wins_with_memcmp_equality() {
        // v2 duplicates v0, v3 is unreferenced, v4 differs from v0 only
        // by a negative zero (distinct under memcmp equality).
        let positions = vec![
            0.0, 0.0, 0.0, // v0
            1.0, 0.0, 0.0, // v1
            0.0, 0.0, 0.0, // v2 (dup of v0)
            0.0, 1.0, 0.0, // v3 (unreferenced)
            -0.0, 0.0, 0.0, // v4 (-0.0: bitwise distinct from v0)
        ];
        let indices = vec![2, 1, 0, 0, 1, 2, 4, 1, 0];
        let (remap, unique) = generate_vertex_remap(&indices, &positions);
        // First occurrence in index-buffer order takes the next id: 2 -> 0,
        // 1 -> 1, 0 -> 0 (dup of 2), 4 -> 2 (-0.0 is not +0.0 bitwise).
        assert_eq!(remap, vec![0, 1, 0, u32::MAX, 2]);
        assert_eq!(unique, 3);

        let mut remapped = indices.clone();
        remap_index_buffer_in_place(&mut remapped, &remap);
        assert_eq!(remapped, vec![0, 1, 0, 0, 1, 0, 2, 1, 0]);

        let gathered = remap_vertex_buffer(&positions, &remap, unique);
        assert_eq!(
            gathered,
            vec![
                0.0, 0.0, 0.0, // id 0: v2's bits (first occurrence)
                1.0, 0.0, 0.0, // id 1: v1
                -0.0, 0.0, 0.0, // id 2: v4 (negative zero preserved)
            ]
        );
        assert_eq!(gathered[6].to_bits(), 0x8000_0000);
    }

    #[test]
    fn simplify_grid_matches_golden() {
        let (positions, indices, unique) = weld_grid();
        assert_eq!((unique, indices.len()), (36, 150));
        let target = indices.len() / 4;
        let (out, error) = simplify(&positions, &indices, None, target);
        // Golden pin (f64 port): exact output on the 6x6 wavy grid.
        // Regenerate deliberately if the algorithm changes; the values
        // were sanity-checked (face count near target, valid indices,
        // finite error) when pinned.
        let golden = vec![
            6, 20, 1, 6, 8, 20, 1, 13, 0, 1, 20, 13, 8, 16, 20, 8, 10, 16, 10, 35, 16, 0, 13, 30,
            13, 31, 30, 13, 20, 31, 20, 16, 33, 16, 35, 33, 20, 33, 31,
        ];
        assert_eq!(out, golden);
        assert_eq!(error.to_bits(), 0x3FC2_BB21_1BC0_FCFAu64);
        assert!(out.iter().all(|&v| (v as usize) < unique));
    }

    #[test]
    fn simplify_is_deterministic() {
        let (positions, indices, _) = weld_grid();
        let target = indices.len() / 4;
        let (out_a, err_a) = simplify(&positions, &indices, None, target);
        let (out_b, err_b) = simplify(&positions, &indices, None, target);
        assert_eq!(out_a, out_b);
        assert_eq!(err_a.to_bits(), err_b.to_bits());
    }

    #[test]
    fn simplify_honors_locks() {
        let (positions, indices, unique) = weld_grid();
        let target = indices.len() / 4;

        // Priority (sharp-edge) lock: runs clean and deterministic.
        let mut priority = vec![0u8; unique];
        priority[7] |= VERTEX_PRIORITY;
        let (out_pa, _) = simplify(&positions, &indices, Some(&priority), target);
        let (out_pb, _) = simplify(&positions, &indices, Some(&priority), target);
        assert_eq!(out_pa, out_pb);
        assert!(out_pa.iter().all(|&v| (v as usize) < unique));

        // Hard lock: a locked vertex is never a collapse source, so it
        // always survives decimation.
        let mut lock = vec![0u8; unique];
        lock[13] |= VERTEX_LOCK;
        let (out_l, _) = simplify(&positions, &indices, Some(&lock), target);
        assert!(out_l.contains(&13));
        assert!(out_l.iter().all(|&v| (v as usize) < unique));
    }
}
