//! Line-by-line mirror of `core/autoremesher.*` (`retopo.core.auto_remesher`).
//!
//! The engine: input validation, symmetry-plane resolution, voxel sizing,
//! island splitting, per-island isotropic resampling (with the adaptive
//! target-length field and meshopt pre-decimation), per-island
//! parameterization + quad extraction, output merging, the optional UV
//! atlas, and the phase-report / progress-permille infrastructure.
//!
//! All numeric stages delegate to the joined `retopo_core` siblings
//! (`MeshSeparator`, `Density`, `IsotropicRemesher`, `Parameterizer`,
//! `QuadExtractor`, `Symmetry`) exactly where the C++ imports dictate —
//! zero vendoring. The only third-party call the C++ engine makes that is
//! not a joined module is meshoptimizer (`decimate_if_too_dense`), which
//! stays C++ via FFI: `rust/core/build.rs` compiles the vendored
//! `thirdparty/meshoptimizer/src/{simplifier,indexgenerator}.cpp` with the
//! same Release optimization the CMake build uses, and the calls below
//! transcribe the C++ call sites argument by argument. A native port of
//! the 3k-line simplifier could not hold the oracle's structural facts
//! (decimation picks topology); FFI keeps them identical by construction.
//!
//! Threading (mandated): `std::thread` scoped threads mirroring the C++
//! TBB structure — one worker per island chunk for the island build, the
//! isotropic phase (one `IsotropicRemesher` per island), and the
//! parameterization phase (one `Parameterizer` + one `QuadExtractor` per
//! island) — plus chunked workers for the data-parallel loops inside
//! `mark_sharp_edge_vertices` and the adaptive field. No rayon,
//! offline std-only. Every parallel loop is per-index independent, so any
//! chunking gives identical results; only cross-island progress-callback
//! order varies run to run, exactly as in the C++ (see the oracle note on
//! `update_progress`).
//!
//! Deliberate restructures (all commented at the site):
//! - Progress state (`thread_progress`, weights, statuses, the permille
//!   gate, stage times, handler + tag) lives in an `Arc<ProgressState>`
//!   instead of bare members: `make_stage_progress` must return a
//!   `'static` [`ProgressHandler`], so the closure owns an `Arc` clone
//!   where the C++ captures `this`.
//! - Statuses are owned `String`s compared by content, where the C++
//!   stores `const char*` and compares pointers. All real step names are
//!   static literals, so content and pointer equality agree on every flow
//!   (proven by the single-island exact progress pinning in the oracle).
//! - `ParameterizationThread` keeps owned output vectors instead of the
//!   live `Parameterizer`/`QuadExtractor`: the C++ thread struct is
//!   self-referential (the extractor borrows the thread's own captured
//!   vectors), which Rust cannot express. The merge phase reads the same
//!   values from the owned copies.
//! - `island_build` uses a `BTreeMap` where the C++ uses
//!   `std::unordered_map` (insert/lookup only, never iterated —
//!   deterministic either way; the B-tree is deterministic by
//!   construction).
//! - Module-level `eprintln!` diagnostics are omitted per the project
//!   stderr-gap memo (quad/frame/parameterizer precedent, documented at
//!   each site): the bool returns and the progress-handler calls that
//!   carry the same information are kept. The main lane owns the stderr
//!   audit.
//! - `AUTO_REMESHER_DEBUG` blocks are omitted: the flag is never defined
//!   by the build, so they compile out on the C++ side too.
//! - `std::min`/`std::max` on `f64` transcribe via [`cxx_min`]/[`cxx_max`]
//!   (density-lane precedent): `f64::min`/`max` return the non-NaN
//!   operand where the C++ returns the first argument.
//! - The C++ `try/catch` around `parameterize()` has no counterpart: the
//!   port signals failure through the `bool` return (lane contract), so
//!   there is nothing to catch.
//! - `parallel_sort` on the mark-sharp edge list transcribes as
//!   `sort_unstable`: every (edge, face) pair is distinct (the C++ comment
//!   says so), so any correct sort yields the same order.
//! - `std::nth_element` for the curvature percentile transcribes as
//!   `select_nth_unstable`: both place the rank-`k` value at `k`, so the
//!   reference value is identical.
//!
//! FMA audit (mandatory, done): seven multiply-add forms in this
//! module's own expressions fuse under Clang at -O3 (verified in IR, brew
//! Clang 23.1.2, ARM64 — sites A..G, each commented at its transcription
//! with the exact `llvm.fmuladd` shape). All seven transcribe with
//! explicit `mul_add`. Every other local expression is fusion-free by
//! shape (division or calls break the patterns), and cross-call fusion is
//! impossible without LTO (the CMake build uses none). The single libm
//! transcendental this module evaluates itself is `acos` (isolated calls
//! — no adjacent sin/cos pair exists on either side, so `sincos` fusion
//! is impossible and `double_utils::joint_sin_cos` is unneeded). All
//! heavy FP goes through the joined siblings, which carry their own
//! audits.

use crate::density::Density;
use crate::isotropic_remesher::IsotropicRemesher;
use crate::mesh_separator::MeshSeparator;
use crate::parameterizer::Parameterizer;
use crate::progress::ProgressHandler;
use crate::quad_extractor::QuadExtractor;
use crate::quad_parameterizer::DipoleConfig;
use crate::symmetry::{Symmetry, SymmetryPlane};
use crate::vector2::Vector2;
use crate::vector3::Vector3;
use std::collections::BTreeMap;
use std::ffi::c_void;
use std::sync::atomic::{AtomicI64, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Instant;

/// Mirrors `AutoRemesherProgressHandler`: `tag` is the opaque pointer from
/// [`AutoRemesher::set_tag`], `progress` runs 0..1, `status` names the
/// current step. A `fn` pointer (not a trait object) so the type stays
/// `Send + Sync` and `Copy` like the C++ pointer.
pub type AutoRemesherProgressHandler = fn(*mut c_void, f32, &str);

/// Mirrors `ModelType`. Write-only on both sides (the C++ `remesh()` never
/// reads `m_modelType`); kept so the setter chain type-checks 1:1.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ModelType {
    /// Mirrors `ModelType::Organic` (the default).
    #[default]
    Organic,
    /// Mirrors `ModelType::HardSurface`.
    HardSurface,
}

// --- meshoptimizer FFI (decimate_if_too_dense only) ---
//
// Signatures transcribe `thirdparty/meshoptimizer/src/meshoptimizer.h`
// (`size_t` -> `usize`, `unsigned int` -> `u32`). The C++ engine links the
// same two translation units through `retopo_core`; the Rust side compiles
// them in `rust/core/build.rs` at the same optimization level, so both
// sides run identical decimation code.

/// Mirrors `meshopt_SimplifyRegularize` (`meshoptimizer.h`: `1 << 4`).
const MESHOPT_SIMPLIFY_REGULARIZE: u32 = 1 << 4;
/// Mirrors `meshopt_SimplifyVertex_Priority` (`meshoptimizer.h`: `1 << 2`).
const MESHOPT_SIMPLIFY_VERTEX_PRIORITY: u8 = 1 << 2;

unsafe extern "C" {
    fn meshopt_generateVertexRemap(
        destination: *mut u32,
        indices: *const u32,
        index_count: usize,
        vertices: *const c_void,
        vertex_count: usize,
        vertex_size: usize,
    ) -> usize;
    fn meshopt_remapIndexBuffer(
        destination: *mut u32,
        indices: *const u32,
        index_count: usize,
        remap: *const u32,
    );
    fn meshopt_remapVertexBuffer(
        destination: *mut c_void,
        vertices: *const c_void,
        vertex_count: usize,
        vertex_size: usize,
        remap: *const u32,
    );
    #[allow(clippy::too_many_arguments)]
    fn meshopt_simplifyWithAttributes(
        destination: *mut u32,
        indices: *const u32,
        index_count: usize,
        vertex_positions: *const f32,
        vertex_count: usize,
        vertex_positions_stride: usize,
        vertex_attributes: *const f32,
        vertex_attributes_stride: usize,
        attribute_weights: *const f32,
        attribute_count: usize,
        vertex_lock: *const u8,
        target_index_count: usize,
        target_error: f32,
        options: u32,
        result_error: *mut f32,
    ) -> usize;
}

const PARALLEL_PHASE_BEGIN: f32 = 0.03;
const PARALLEL_PHASE_END: f32 = 0.95;

// How an island's own 0..1 progress splits across its three stages, from
// the measured cost of each on a typical model. The phase report prints
// the real accumulated times, so these can be re-checked against a run.
const ISLAND_RESAMPLE_END: f32 = 0.17;
const ISLAND_PARAMETERIZE_END: f32 = 0.50;
// Quad extraction runs from islandParameterizeEnd to 1.0.

const DECIMATE_TRIGGER_RATIO: f64 = 8.0;
const DECIMATE_TARGET_RATIO: f64 = 4.0;

// Minimum input support for the symmetry plane: below this fraction of
// mirrored vertices the run falls back to unconstrained output rather
// than snapping an asymmetric mesh onto a plane it does not have.
const MIN_SYMMETRY_SCORE: f64 = 0.75;

/// `std::min<double>` semantics: `(b < a) ? b : a` — a NaN `a` stays NaN.
/// (`f64::min` returns the non-NaN operand instead; density-lane precedent.)
#[inline]
fn cxx_min(a: f64, b: f64) -> f64 {
    if b < a { b } else { a }
}

/// `std::max<double>` semantics: `(a < b) ? b : a` — a NaN `a` stays NaN.
/// (`f64::max` returns the non-NaN operand instead; density-lane precedent.)
#[inline]
fn cxx_max(a: f64, b: f64) -> f64 {
    if a < b { b } else { a }
}

/// Chunk length for one `parallel_each` worker: the C++ sides use
/// `tbb::parallel_for` over the whole range and let TBB partition; here
/// the range splits into one chunk per worker thread. All uses are
/// per-index independent, so the partitioning is unobservable.
fn worker_chunk_len(n: usize) -> usize {
    debug_assert!(n > 0);
    let workers = thread::available_parallelism().map_or(1, |p| p.get());
    n.div_ceil(workers).max(1)
}

/// Mirrors `tbb::parallel_for(tbb::blocked_range<size_t>(0, items.len()),
/// ...)`: calls `f(i, &mut items[i])` for every index, spread over scoped
/// worker threads that each own disjoint `chunks_mut` slices.
fn parallel_each<T, F>(items: &mut [T], f: F)
where
    T: Send,
    F: Fn(usize, &mut T) + Send + Sync,
{
    if items.is_empty() {
        return;
    }
    let chunk_len = worker_chunk_len(items.len());
    // Share the worker body by reference: a `move` closure per chunk would
    // otherwise try to move `f` into the first thread.
    let f = &f;
    thread::scope(|s| {
        for (chunk_index, chunk) in items.chunks_mut(chunk_len).enumerate() {
            let base = chunk_index * chunk_len;
            s.spawn(move || {
                for (k, item) in chunk.iter_mut().enumerate() {
                    f(base + k, item);
                }
            });
        }
    });
}

/// Two-slice variant of [`parallel_each`] for loops that mutate two vectors
/// in lockstep (face normals + areas, normals + neighbor rings). Both
/// slices share the chunking, so worker `k` owns the same index range of
/// each. Panics on length mismatch like a `zip` that must stay aligned —
/// every call site passes same-length slices by construction.
fn parallel_each_zip2<T, U, F>(a: &mut [T], b: &mut [U], f: F)
where
    T: Send,
    U: Send,
    F: Fn(usize, &mut T, &mut U) + Send + Sync,
{
    debug_assert_eq!(a.len(), b.len(), "parallel zip slices must align");
    if a.is_empty() {
        return;
    }
    let chunk_len = worker_chunk_len(a.len());
    let f = &f;
    thread::scope(|s| {
        for (chunk_index, (chunk_a, chunk_b)) in a
            .chunks_mut(chunk_len)
            .zip(b.chunks_mut(chunk_len))
            .enumerate()
        {
            let base = chunk_index * chunk_len;
            s.spawn(move || {
                for (k, (x, y)) in chunk_a.iter_mut().zip(chunk_b.iter_mut()).enumerate() {
                    f(base + k, x, y);
                }
            });
        }
    });
}

fn mark_sharp_edge_vertices(
    vertices: &[Vector3],
    indices: &[u32],
    sharp_edge_radians: f64,
    vertex_lock: &mut [u8],
) {
    let face_count = indices.len() / 3;

    let mut face_normals = vec![Vector3::default(); face_count];
    parallel_each(&mut face_normals, |i, normal| {
        *normal = Vector3::normal(
            &vertices[indices[i * 3] as usize],
            &vertices[indices[i * 3 + 1] as usize],
            &vertices[indices[i * 3 + 2] as usize],
        );
    });

    // The C++ worker is one face with a serial 3-edge inner loop; here the
    // worker is one flat (face, corner) slot — each worker owns a distinct
    // slot either way, so the values are identical.
    let mut edges = vec![(0u64, 0u32); face_count * 3];
    parallel_each(&mut edges, |slot, edge| {
        let i = slot / 3;
        let j = slot % 3;
        let mut first = indices[i * 3 + j];
        let mut second = indices[i * 3 + (j + 1) % 3];
        if first > second {
            std::mem::swap(&mut first, &mut second);
        }
        *edge = ((u64::from(first) << 32) | u64::from(second), i as u32);
    });
    // Every (edge, face) pair is distinct, so the parallel sort produces
    // the same order the serial one did (C++ comment, kept): any correct
    // sort of distinct keys yields one order, and `sort_unstable` is
    // deterministic for a given input.
    edges.sort_unstable();

    for i in 0..edges.len().saturating_sub(1) {
        if edges[i].0 != edges[i + 1].0 {
            continue;
        }
        if Vector3::angle(
            &face_normals[edges[i].1 as usize],
            &face_normals[edges[i + 1].1 as usize],
        ) < sharp_edge_radians
        {
            continue;
        }
        vertex_lock[(edges[i].0 >> 32) as usize] |= MESHOPT_SIMPLIFY_VERTEX_PRIORITY;
        vertex_lock[(edges[i].0 & 0xffff_ffff) as usize] |= MESHOPT_SIMPLIFY_VERTEX_PRIORITY;
    }
}

// Global UV atlas: shelf-packs the per-island UV ranges (each already
// normalized to 0..1 by QuadExtractor::computeRemeshedVertexUvs, so
// islands would otherwise stack on each other) into one shared 0..1
// atlas. `spans` holds each merged island's (start, count) slice of
// `uvs`, in merge order.
//
// (C++ contract comment, kept:) Packing inside 0..1 was chosen over
// per-island UDIM offsets because the shipped --uvs contract requires
// every UV inside 0..1 (test_cli_uvs asserts it, and GLB TEXCOORD_0
// consumers expect normalized coordinates); UDIM tiles (u >= 1) would
// break both. Deterministic shelf packing: islands sort by box height
// (ties by width, then merge order), fill rows left to right inside a
// strip ceil(sqrt(N)) boxes wide, then one uniform scale fits the shelves
// into 0..1 with a fixed 2-texel-at-1k gutter between boxes. A single
// island returns untouched, so one-island output keeps today's exact UVs.
// Geometry is never touched (only the UV array is rewritten), and the
// --uvs off path never calls this.
const ATLAS_GUTTER: f64 = 2.0 / 1024.0;

fn pack_island_uvs_into_atlas(uvs: &mut [Vector2], spans: &[(usize, usize)]) {
    if uvs.is_empty() {
        return;
    }
    let mut islands = Vec::new();
    for (i, span) in spans.iter().enumerate() {
        if span.1 > 0 {
            islands.push(i);
        }
    }
    if islands.len() <= 1 {
        return;
    }

    #[derive(Clone, Copy, Default)]
    struct AtlasBox {
        min_u: f64,
        min_v: f64,
        width: f64,
        height: f64,
    }
    let mut boxes = vec![AtlasBox::default(); spans.len()];
    let mut max_width = 0.0;
    for &island in &islands {
        let begin = spans[island].0;
        let end = (begin + spans[island].1).min(uvs.len());
        if begin >= end {
            continue;
        }
        let mut min_u = uvs[begin].x();
        let mut max_u = min_u;
        let mut min_v = uvs[begin].y();
        let mut max_v = min_v;
        for p in &uvs[begin + 1..end] {
            min_u = cxx_min(min_u, p.x());
            max_u = cxx_max(max_u, p.x());
            min_v = cxx_min(min_v, p.y());
            max_v = cxx_max(max_v, p.y());
        }
        boxes[island] = AtlasBox {
            min_u,
            min_v,
            width: max_u - min_u,
            height: max_v - min_v,
        };
        max_width = cxx_max(max_width, boxes[island].width);
    }

    // Stable sort with the literal C++ comparator (descending height,
    // then descending width, then ascending merge order). NaN box
    // extents make the comparator inconsistent on both sides (C++
    // `stable_sort` and Rust `sort_by` then both pick an unspecified but
    // deterministic order); sane UVs never produce them.
    islands.sort_by(|&a, &b| {
        if boxes[a].height != boxes[b].height {
            if boxes[b].height < boxes[a].height {
                std::cmp::Ordering::Less
            } else {
                std::cmp::Ordering::Greater
            }
        } else if boxes[a].width != boxes[b].width {
            if boxes[b].width < boxes[a].width {
                std::cmp::Ordering::Less
            } else {
                std::cmp::Ordering::Greater
            }
        } else {
            a.cmp(&b)
        }
    });

    // A strip ceil(sqrt(N)) boxes wide keeps the shelves roughly square
    // for the common equal-box case (every island spans the full unit
    // square before packing).
    let columns = (islands.len() as f64).sqrt().ceil();
    let strip_width = if max_width > 0.0 {
        columns * max_width
    } else {
        1.0
    };
    struct Shelf {
        members: Vec<usize>,
        width: f64,
        height: f64,
    }
    let mut shelves = vec![Shelf {
        members: Vec::new(),
        width: 0.0,
        height: 0.0,
    }];
    for &island in &islands {
        // `shelves` always holds at least one shelf (it starts with one
        // and only grows), so the back index is always valid.
        let back = shelves.len() - 1;
        if !shelves[back].members.is_empty()
            && shelves[back].width + boxes[island].width > strip_width
        {
            shelves.push(Shelf {
                members: Vec::new(),
                width: 0.0,
                height: 0.0,
            });
        }
        let back = shelves.len() - 1;
        shelves[back].members.push(island);
        shelves[back].width += boxes[island].width;
        let grown = cxx_max(shelves[back].height, boxes[island].height);
        shelves[back].height = grown;
    }

    // The gutter is fixed in atlas units; only shrink it when the box
    // count alone would overflow the unit square (hundreds of islands).
    let mut widest_shelf = 0usize;
    let mut total_height = 0.0;
    for shelf in &shelves {
        widest_shelf = widest_shelf.max(shelf.members.len());
        total_height += shelf.height;
    }
    let mut gutter = ATLAS_GUTTER;
    let gaps = (widest_shelf.saturating_sub(1)).max(shelves.len().saturating_sub(1));
    if gaps > 0 && gutter * gaps as f64 > 0.5 {
        gutter = 0.5 / gaps as f64;
    }

    // FMA audit (site E): Clang fuses `1.0 - gutter * count` to
    // llvm.fmuladd(-gutter, count, 1.0) (negation first, then one fused
    // op); the division stays separate. Transcribed explicitly at both
    // occurrences below.
    let mut scale = 1.0;
    for shelf in &shelves {
        if shelf.width <= 0.0 {
            continue;
        }
        scale = cxx_min(
            scale,
            (-gutter).mul_add((shelf.members.len() - 1) as f64, 1.0) / shelf.width,
        );
    }
    if total_height > 0.0 {
        scale = cxx_min(
            scale,
            (-gutter).mul_add((shelves.len() - 1) as f64, 1.0) / total_height,
        );
    }
    scale = cxx_min(1.0, cxx_max(1e-9, scale));

    let mut y = 0.0;
    for shelf in &shelves {
        let mut x = 0.0;
        for &island in &shelf.members {
            let bx = boxes[island];
            let begin = spans[island].0;
            let end = (begin + spans[island].1).min(uvs.len());
            for slot in &mut uvs[begin..end] {
                // FMA audit (site F): Clang fuses to
                // llvm.fmuladd(u - min, scale, origin). Transcribed
                // explicitly.
                let u = (slot.x() - bx.min_u).mul_add(scale, x);
                let v = (slot.y() - bx.min_v).mul_add(scale, y);
                // `std::min(1.0, std::max(0.0, u))`, transcribed
                // literally (NOT `clamp`: clamp keeps a NaN self where the
                // C++ yields 0.0, and clippy's manual_clamp must not
                // "fix" this).
                *slot = Vector2::new(cxx_min(1.0, cxx_max(0.0, u)), cxx_min(1.0, cxx_max(0.0, v)));
            }
            // FMA audit (site G): Clang fuses the inner `(w * scale) +
            // gutter` to llvm.fmuladd(w, scale, gutter), then adds the
            // running origin separately. Transcribed explicitly.
            x += bx.width.mul_add(scale, gutter);
        }
        y += shelf.height.mul_add(scale, gutter);
    }
}

// Per-island durations are accumulated in microseconds: a mesh split into
// hundreds of islands spends well under a millisecond on most of them, and
// truncating each one to whole milliseconds loses the bulk of the total.
#[derive(Debug, Default)]
pub struct DecimationStats {
    pub time_us: AtomicI64,
    pub islands_decimated: AtomicUsize,
    pub islands_considered: AtomicUsize,
    pub triangles_before: AtomicUsize,
    pub triangles_after: AtomicUsize,
}

struct StageTime {
    name: String,
    order: f32,
    microseconds: i64,
}

/// The mutex-guarded progress members (`m_threadProgress` and friends plus
/// the handler/tag pair). The C++ keeps these as bare members and captures
/// `this` in the stage closures; here they live behind an `Arc` so the
/// `'static` [`ProgressHandler`] closures can own them.
struct ProgressData {
    handler: Option<AutoRemesherProgressHandler>,
    tag: *mut c_void,
    thread_progress: Vec<f32>,
    thread_progress_weights: Vec<f32>,
    // Owned strings where the C++ stores borrowed `const char*` (see the
    // module docs); `None` mirrors `nullptr`.
    thread_status: Vec<Option<String>>,
    // The weighted sum of thread_progress, kept incrementally so that a
    // fine-grained update stays O(1) rather than a scan of every island.
    progress_sum: f64,
    reported_permille: i32,
    reported_status: Option<String>,
}

// SAFETY: `ProgressData` crosses threads only behind a `Mutex` (inside
// `ProgressState`), and the tag is an opaque cookie the handler interprets
// — the same contract as the C++ `void*`.
unsafe impl Send for ProgressData {}

struct ProgressState {
    progress_mutex: Mutex<ProgressData>,
    stage_timing_mutex: Mutex<Vec<StageTime>>,
}

impl ProgressState {
    fn new() -> Self {
        Self {
            progress_mutex: Mutex::new(ProgressData {
                handler: None,
                tag: std::ptr::null_mut(),
                thread_progress: Vec::new(),
                thread_progress_weights: Vec::new(),
                thread_status: Vec::new(),
                progress_sum: 0.0,
                reported_permille: -1,
                reported_status: None,
            }),
            stage_timing_mutex: Mutex::new(Vec::new()),
        }
    }

    /// Locks, ignoring poisoning: a poisoned mutex means a worker thread
    /// panicked, which the port treats as unreachable (workers only fail
    /// through `bool` returns, like the C++); the C++ `std::mutex` has no
    /// poisoning, so carrying on mirrors it.
    fn lock_progress(&self) -> std::sync::MutexGuard<'_, ProgressData> {
        self.progress_mutex
            .lock()
            .unwrap_or_else(|e| e.into_inner())
    }

    fn lock_stages(&self) -> std::sync::MutexGuard<'_, Vec<StageTime>> {
        self.stage_timing_mutex
            .lock()
            .unwrap_or_else(|e| e.into_inner())
    }

    /// Mirrors the handler-taking half of `updateProgress` (called with the
    /// lock already held, like the C++ `lock_guard` that spans the call).
    fn update_locked(
        data: &mut ProgressData,
        thread_index: usize,
        progress: f32,
        status: Option<&str>,
    ) {
        if data.handler.is_none() {
            return;
        }
        if thread_index >= data.thread_progress.len() {
            return;
        }
        if let Some(name) = status
            && !name.is_empty()
        {
            data.thread_status[thread_index] = Some(name.to_string());
        }
        if progress > data.thread_progress[thread_index] {
            // FMA audit (site B): Clang fuses to
            // llvm.fmuladd(delta, weight, sum) (both f32 operands
            // extended first, then one fused op). Transcribed explicitly.
            data.progress_sum = f64::from(progress - data.thread_progress[thread_index]).mul_add(
                f64::from(data.thread_progress_weights[thread_index]),
                data.progress_sum,
            );
            data.thread_progress[thread_index] = progress;
        }

        // FMA audit (site C): Clang fuses to
        // llvm.fmuladd(clamped, end - begin, begin). Transcribed explicitly.
        let overall = cxx_min(1.0, cxx_max(0.0, data.progress_sum)).mul_add(
            f64::from(PARALLEL_PHASE_END - PARALLEL_PHASE_BEGIN),
            f64::from(PARALLEL_PHASE_BEGIN),
        );

        // Steps now report many times per island, so only wake the UI when
        // the bar would actually move or the status line would change.
        let permille = (overall * 1000.0) as i32;
        let island_status = data.thread_status[thread_index].clone();
        if permille == data.reported_permille && island_status == data.reported_status {
            return;
        }
        data.reported_permille = permille;
        data.reported_status = island_status;

        // With several islands in flight, the run as a whole is only as far
        // along as its slowest island, so that is the step worth naming.
        let mut slowest = thread_index;
        for i in 0..data.thread_progress.len() {
            if data.thread_progress[i] < data.thread_progress[slowest] {
                slowest = i;
            }
        }
        let name = data.thread_status[slowest].clone().unwrap_or_default();
        let tag = data.tag;
        if let Some(handler) = data.handler {
            handler(tag, overall as f32, &name);
        }
    }

    /// Mirrors the direct `m_progressHandler(m_tag, progress, status)` calls
    /// (entry/exit announcements, never gated by the permille filter).
    fn report_direct(&self, progress: f32, status: &str) {
        let data = self.lock_progress();
        if let Some(handler) = data.handler {
            handler(data.tag, progress, status);
        }
    }

    /// The shared body of `accumulate_stage_time`, callable both through
    /// `&AutoRemesher` and from the stage closures that own only the `Arc`.
    fn accumulate(&self, name: &str, order: f32, microseconds: i64) {
        if name.is_empty() {
            return;
        }
        let mut stages = self.lock_stages();
        for it in stages.iter_mut() {
            if it.name == name {
                it.microseconds += microseconds;
                return;
            }
        }
        stages.push(StageTime {
            name: name.to_string(),
            order,
            microseconds,
        });
    }
}

struct IslandContext {
    vertices: Vec<Vector3>,
    triangles: Vec<Vec<usize>>,
    voxel_size: f64,
    scaling: f64,
    adaptivity: f64,
    anisotropy: f64,
    sharp_edge_degrees: f64,
    smooth_normal_degrees: f64,
    symmetry_plane: SymmetryPlane,
    // NOTE: no stored guide/sharp polyline pointers (the C++
    // `&m_guidePolylines` / `&m_sharpPolylines`): their only consumer is
    // the parameterization worker, which reads `&self.guide_polylines` /
    // `&self.sharp_polylines` (always `Some`, possibly empty — never null
    // like the C++) from the scope capture instead. Storing the borrows
    // here would hold `&self` across the serial `&mut self` merge phase.
    // Density mask slice on the island's input vertices (empty = off),
    // plus the same mask carried onto the resampled island vertices.
    density: Vec<f64>,
    resampled_density: Vec<f64>,
}

impl Default for IslandContext {
    fn default() -> Self {
        Self {
            vertices: Vec::new(),
            triangles: Vec::new(),
            voxel_size: 0.0,
            scaling: 0.0,
            adaptivity: 0.0,
            anisotropy: 0.0,
            sharp_edge_degrees: 0.0,
            smooth_normal_degrees: 0.0,
            symmetry_plane: SymmetryPlane::default(),
            density: Vec::new(),
            resampled_density: Vec::new(),
        }
    }
}

struct ParameterizationThread<'a> {
    island_index: usize,
    island: &'a IslandContext,
    captured_uvs: Vec<Vec<Vector2>>,
    captured_original_uvs: Vec<Vec<Vector2>>,
    captured_extracted_connection_moved: Vec<u8>,
    captured_singular_vertices: Vec<Vector3>,
    captured_singular_vertex_indices: Vec<usize>,
    captured_extracted_connections: Vec<(Vector3, Vector3)>,
    captured_vertex_uvs: Vec<Vector2>,
    // Owned copies of the extractor outputs: the C++ merge phase reads
    // them off the live `thread.remesher`, which cannot be expressed here
    // (see the module docs on the self-referential thread struct).
    // `remeshed_vertices`/`remeshed_quads` stay empty when the island
    // produced nothing, which subsumes both C++ skip cases (null
    // remesher after a failed extract, and empty quads).
    remeshed_vertices: Vec<Vector3>,
    remeshed_quads: Vec<Vec<usize>>,
    // Dipole flips applied on this island (captured off the parameterizer
    // even when extraction later yields nothing).
    dipoles_placed: usize,
    // Copied from AutoRemesher::m_computeRemeshedUvs before the parallel
    // loop (the worker below is not a member, so it cannot read it).
    compute_vertex_uvs: bool,
    // NOTE: no `auto_remesher` back-pointer (the C++ `AutoRemesher*`): the
    // worker closures capture `&AutoRemesher` from the scope directly.
}

pub struct AutoRemesher {
    vertices: Vec<Vector3>,
    triangles: Vec<Vec<usize>>,
    remeshed_vertices: Vec<Vector3>,
    remeshed_quads: Vec<Vec<usize>>,
    remeshed_vertex_uvs: Vec<Vector2>,
    compute_remeshed_uvs: bool,
    decimated_vertices: Vec<Vector3>,
    decimated_triangles: Vec<Vec<usize>>,
    decimated: bool,
    isotropic_vertices: Vec<Vector3>,
    isotropic_triangles: Vec<Vec<usize>>,
    isotropic_triangle_uvs: Vec<Vec<Vector2>>,
    isotropic_original_triangle_uvs: Vec<Vec<Vector2>>,
    isotropic_extracted_connection_moved: Vec<u8>,
    isotropic_singular_vertices: Vec<Vector3>,
    isotropic_extracted_connections: Vec<(Vector3, Vector3)>,
    progress: Arc<ProgressState>,
    phase_report: Vec<String>,
    island_output_quad_counts: Vec<usize>,
    island_dipole_counts: Vec<usize>,
    dipoles: DipoleConfig,
    scaling: f64,
    target_triangle_count: usize,
    voxel_size: f64,
    adaptivity: f64,
    anisotropy: f64,
    sharp_edge_degrees: f64,
    smooth_normal_degrees: f64,
    model_type: ModelType,
    symmetry_enabled: bool,
    symmetry_axis: i32,
    symmetry_plane: SymmetryPlane,
    guide_polylines: Vec<Vec<Vector3>>,
    sharp_polylines: Vec<Vec<Vector3>>,
    density_multipliers: Vec<f64>,
    quiet: bool,
}

// NOTE: `AutoRemesher` is shared across scoped worker threads (`&self`
// captured by the worker closures) through its automatic `Send + Sync`
// impls: the only worker-mutated state sits behind the `ProgressState`
// mutexes, and every other field is read-only during the parallel phases
// (same sharing pattern as the C++ `const` reads from TBB workers).

impl AutoRemesher {
    pub const DEFAULT_SHARP_EDGE_DEGREES: f64 = 90.0;

    /// Mirrors the constructor (copies its inputs like the C++ member
    /// initializers do; isotropic-remesher precedent).
    #[must_use]
    pub fn new(vertices: &[Vector3], triangles: &[Vec<usize>]) -> Self {
        Self {
            vertices: vertices.to_vec(),
            triangles: triangles.to_vec(),
            remeshed_vertices: Vec::new(),
            remeshed_quads: Vec::new(),
            remeshed_vertex_uvs: Vec::new(),
            compute_remeshed_uvs: false,
            decimated_vertices: Vec::new(),
            decimated_triangles: Vec::new(),
            decimated: false,
            isotropic_vertices: Vec::new(),
            isotropic_triangles: Vec::new(),
            isotropic_triangle_uvs: Vec::new(),
            isotropic_original_triangle_uvs: Vec::new(),
            isotropic_extracted_connection_moved: Vec::new(),
            isotropic_singular_vertices: Vec::new(),
            isotropic_extracted_connections: Vec::new(),
            progress: Arc::new(ProgressState::new()),
            phase_report: Vec::new(),
            island_output_quad_counts: Vec::new(),
            island_dipole_counts: Vec::new(),
            // Product default for dipole insertion (no C++ counterpart;
            // see set_dipoles). Off until offset rings validate end to
            // end — the saturation harness opts in explicitly per row.
            dipoles: DipoleConfig::off(),
            scaling: 0.0,
            target_triangle_count: 0,
            voxel_size: 0.0,
            adaptivity: 1.0,
            anisotropy: 1.0,
            sharp_edge_degrees: Self::DEFAULT_SHARP_EDGE_DEGREES,
            smooth_normal_degrees: 0.0,
            model_type: ModelType::Organic,
            symmetry_enabled: false,
            symmetry_axis: -1,
            symmetry_plane: SymmetryPlane::default(),
            guide_polylines: Vec::new(),
            sharp_polylines: Vec::new(),
            density_multipliers: Vec::new(),
            quiet: false,
        }
    }

    /// Mirrors `setTargetTriangleCount`.
    pub fn set_target_triangle_count(&mut self, target_triangle_count: usize) {
        self.target_triangle_count = target_triangle_count;
    }

    /// Mirrors `setScaling`.
    pub fn set_scaling(&mut self, scaling: f64) {
        self.scaling = scaling;
    }

    /// Mirrors `setProgressHandler` (the C++ pointer is nullable; `None`
    /// mirrors `nullptr`, the default).
    pub fn set_progress_handler(&mut self, progress_handler: Option<AutoRemesherProgressHandler>) {
        self.progress.lock_progress().handler = progress_handler;
    }

    /// Mirrors `setTag`.
    pub fn set_tag(&mut self, tag: *mut c_void) {
        self.progress.lock_progress().tag = tag;
    }

    /// Mirrors `setQuiet` (see the C++ contract comment on the `.cppm`:
    /// silences engine-owned progress chatter on stderr while warnings and
    /// errors still print; quiet also skips downstream progress-handler
    /// installation, so a quiet run collects no per-stage timings and
    /// `phase_report` omits its leaf-stage lines).
    pub fn set_quiet(&mut self, quiet: bool) {
        self.quiet = quiet;
    }

    /// Mirrors `quiet`.
    #[must_use]
    pub fn quiet(&self) -> bool {
        self.quiet
    }

    /// Mirrors `setModelType` (write-only on both sides).
    pub fn set_model_type(&mut self, model_type: ModelType) {
        self.model_type = model_type;
    }

    /// Mirrors `setGradientAdaptivity`.
    pub fn set_gradient_adaptivity(&mut self, adaptivity: f64) {
        self.adaptivity = adaptivity;
    }

    /// Mirrors `setSharpEdgeDegrees`.
    pub fn set_sharp_edge_degrees(&mut self, degrees: f64) {
        self.sharp_edge_degrees = degrees;
    }

    /// Mirrors `setAnisotropy`.
    pub fn set_anisotropy(&mut self, anisotropy: f64) {
        self.anisotropy = anisotropy;
    }

    /// Mirrors `setSmoothNormalDegrees`.
    pub fn set_smooth_normal_degrees(&mut self, degrees: f64) {
        self.smooth_normal_degrees = degrees;
    }

    /// Mirrors `setSymmetryEnabled` (see the C++ contract comment:
    /// mirror-symmetry constraints for organic remeshing, disabled by
    /// default; when off, the pipeline is byte-for-byte the unmodified
    /// one).
    pub fn set_symmetry_enabled(&mut self, enabled: bool) {
        self.symmetry_enabled = enabled;
    }

    /// Mirrors `setSymmetryPlane` (`axis` selects the plane normal,
    /// -1 = auto-detect).
    pub fn set_symmetry_plane(&mut self, axis: i32) {
        self.symmetry_axis = axis;
    }

    /// Mirrors `setDensityMultipliers` (see the C++ contract comment:
    /// per-input-vertex multipliers, clamped to [0.25, 4.0], non-finite
    /// becoming 1.0; empty/all-1.0/wrong-sized disables the modulation
    /// entirely and the run is bit-identical to one without any field).
    pub fn set_density_multipliers(&mut self, multipliers: &[f64]) {
        if multipliers.len() != self.vertices.len() {
            self.density_multipliers.clear();
            return;
        }
        self.density_multipliers = Density::normalize_field(multipliers);
    }

    /// Mirrors `symmetryPlaneAxis` (the plane the last `remesh` actually
    /// used; -1 = symmetry was off/skipped).
    #[must_use]
    pub fn symmetry_plane_axis(&self) -> i32 {
        self.symmetry_plane.axis
    }

    /// Mirrors `symmetryPlaneOffset`.
    #[must_use]
    pub fn symmetry_plane_offset(&self) -> f64 {
        self.symmetry_plane.offset
    }

    /// Mirrors `symmetryPlaneScore`.
    #[must_use]
    pub fn symmetry_plane_score(&self) -> f64 {
        self.symmetry_plane.score
    }

    /// Mirrors `setGuidePolylines` (see the C++ contract comment: user
    /// guide-curve constraints for the frame field; default off/empty is
    /// byte-for-byte the unmodified pipeline).
    pub fn set_guide_polylines(&mut self, guides: Vec<Vec<Vector3>>) {
        self.guide_polylines = guides;
    }

    /// Mirrors `setSharpPolylines` (see the C++ contract comment: explicit
    /// sharp/feature constraints for hard-surface props; default off).
    pub fn set_sharp_polylines(&mut self, sharps: Vec<Vec<Vector3>>) {
        self.sharp_polylines = sharps;
    }

    /// Mirrors `setFeaturePolylines` (the CLI-flag-spelled alias of
    /// `setSharpPolylines`).
    pub fn set_feature_polylines(&mut self, sharps: Vec<Vec<Vector3>>) {
        self.sharp_polylines = sharps;
    }

    /// Mirrors `remeshedVertices`.
    #[must_use]
    pub fn remeshed_vertices(&self) -> &[Vector3] {
        &self.remeshed_vertices
    }

    /// Mirrors `remeshedQuads`.
    #[must_use]
    pub fn remeshed_quads(&self) -> &[Vec<usize>] {
        &self.remeshed_quads
    }

    /// Mirrors `remeshedVertexUvs` (only populated when
    /// `set_compute_remeshed_uvs(true)` was called before `remesh`;
    /// otherwise empty; size always matches `remeshed_vertices` when
    /// populated).
    #[must_use]
    pub fn remeshed_vertex_uvs(&self) -> &[Vector2] {
        &self.remeshed_vertex_uvs
    }

    /// Mirrors `setComputeRemeshedUvs`.
    pub fn set_compute_remeshed_uvs(&mut self, compute: bool) {
        self.compute_remeshed_uvs = compute;
    }

    /// Mirrors `decimatedVertices`.
    #[must_use]
    pub fn decimated_vertices(&self) -> &[Vector3] {
        &self.decimated_vertices
    }

    /// Mirrors `decimatedTriangles`.
    #[must_use]
    pub fn decimated_triangles(&self) -> &[Vec<usize>] {
        &self.decimated_triangles
    }

    /// Mirrors `decimated`.
    #[must_use]
    pub fn decimated(&self) -> bool {
        self.decimated
    }

    /// Mirrors `isotropicVertices`.
    #[must_use]
    pub fn isotropic_vertices(&self) -> &[Vector3] {
        &self.isotropic_vertices
    }

    /// Mirrors `isotropicTriangles`.
    #[must_use]
    pub fn isotropic_triangles(&self) -> &[Vec<usize>] {
        &self.isotropic_triangles
    }

    /// Mirrors `isotropicExtractedConnectionMoved`.
    #[must_use]
    pub fn isotropic_extracted_connection_moved(&self) -> &[u8] {
        &self.isotropic_extracted_connection_moved
    }

    /// Mirrors `isotropicOriginalTriangleUvs`.
    #[must_use]
    pub fn isotropic_original_triangle_uvs(&self) -> &[Vec<Vector2>] {
        &self.isotropic_original_triangle_uvs
    }

    /// Mirrors `isotropicTriangleUvs`.
    #[must_use]
    pub fn isotropic_triangle_uvs(&self) -> &[Vec<Vector2>] {
        &self.isotropic_triangle_uvs
    }

    /// Mirrors `isotropicSingularVertices`.
    #[must_use]
    pub fn isotropic_singular_vertices(&self) -> &[Vector3] {
        &self.isotropic_singular_vertices
    }

    /// Mirrors `isotropicExtractedConnections`.
    #[must_use]
    pub fn isotropic_extracted_connections(&self) -> &[(Vector3, Vector3)] {
        &self.isotropic_extracted_connections
    }

    /// Mirrors `phaseReport`.
    #[must_use]
    pub fn phase_report(&self) -> &[String] {
        &self.phase_report
    }

    /// Mirrors `islandOutputQuadCounts` (per-island output accounting in
    /// island order; entry i is the number of output faces island i
    /// contributed, 0 when dropped; empty when `remesh` never ran or
    /// rejected the input).
    #[must_use]
    pub fn island_output_quad_counts(&self) -> &[usize] {
        &self.island_output_quad_counts
    }

    /// Per-island dipole flips applied by the last `remesh` (island
    /// order, alongside `island_output_quad_counts`; all zeros when
    /// dipoles are off, unmasked, or mild).
    #[must_use]
    pub fn island_dipole_counts(&self) -> &[usize] {
        &self.island_dipole_counts
    }

    /// Dipole-insertion config (no C++ counterpart). Default off: enable
    /// explicitly (CLI `--dipoles auto`) for density-boundary singularity
    /// rings on sharp masked steps.
    pub fn set_dipoles(&mut self, config: DipoleConfig) {
        self.dipoles = config;
    }

    /// The current dipole-insertion config.
    #[must_use]
    pub fn dipoles(&self) -> DipoleConfig {
        self.dipoles
    }

    /// Mirrors `calculateAverageEdgeLength` (dead code on both sides:
    /// defined but never called in the C++ either).
    #[allow(dead_code)]
    fn calculate_average_edge_length(vertices: &[Vector3], faces: &[Vec<usize>]) -> f64 {
        let mut sum_of_length = 0.0;
        let mut edge_count = 0usize;
        for face in faces {
            for i in 0..face.len() {
                let j = (i + 1) % face.len();
                sum_of_length += (vertices[face[i]] - vertices[face[j]]).length();
                edge_count += 1;
            }
        }
        if edge_count == 0 {
            return 0.0;
        }
        sum_of_length / edge_count as f64
    }

    fn initialize_voxel_size(&mut self) {
        let area = Self::calculate_mesh_area(&self.vertices, &self.triangles);
        let triangle_area = area / self.target_triangle_count as f64;
        self.voxel_size = (triangle_area / (0.86602540378 * 0.5)).sqrt();
        // (C++ AUTO_REMESHER_DEBUG stderr omitted: the flag is never
        // defined by the build; likewise at every site below.)
    }

    fn calculate_mesh_area(vertices: &[Vector3], triangles: &[Vec<usize>]) -> f64 {
        let mut area = 0.0;
        for it in triangles {
            area += Vector3::area(&vertices[it[0]], &vertices[it[1]], &vertices[it[2]]);
        }
        area
    }

    /// Mirrors `decimateIfTooDense`. The `stats` pointer is non-null at the
    /// single call site, so it transcribes as a plain reference; the FFI
    /// calls transcribe the C++ meshoptimizer calls argument by argument.
    fn decimate_if_too_dense(
        vertices: &mut Vec<Vector3>,
        triangles: &mut Vec<Vec<usize>>,
        voxel_size: f64,
        sharp_edge_degrees: f64,
        _island_index: usize,
        stats: &DecimationStats,
    ) -> bool {
        stats.islands_considered.fetch_add(1, Ordering::SeqCst);

        if vertices.is_empty() || triangles.is_empty() || voxel_size <= 0.0 {
            return false;
        }

        if vertices.len() > u32::MAX as usize {
            return false;
        }

        let target_triangle_area = voxel_size * voxel_size * 0.86602540378 * 0.5;
        if target_triangle_area <= 0.0 {
            return false;
        }
        let island_target_triangle_count =
            Self::calculate_mesh_area(vertices, triangles) / target_triangle_area;
        if island_target_triangle_count < 1.0 {
            return false;
        }

        if (triangles.len() as f64) < island_target_triangle_count * DECIMATE_TRIGGER_RATIO {
            return false;
        }

        let decimate_triangle_count =
            (island_target_triangle_count * DECIMATE_TARGET_RATIO) as usize;

        let mut indices = Vec::with_capacity(triangles.len() * 3);
        for triangle in triangles.iter() {
            if triangle.len() != 3 {
                return false;
            }
            for &corner in triangle {
                indices.push(corner as u32);
            }
        }

        let mut lower_bound = vertices[0];
        let mut upper_bound = vertices[0];
        for position in vertices.iter() {
            for i in 0..3 {
                lower_bound[i] = cxx_min(lower_bound[i], position[i]);
                upper_bound[i] = cxx_max(upper_bound[i], position[i]);
            }
        }
        let center = (lower_bound + upper_bound) * 0.5;

        let mut positions = Vec::with_capacity(vertices.len() * 3);
        for position in vertices.iter() {
            positions.push((position.x() - center.x()) as f32);
            positions.push((position.y() - center.y()) as f32);
            positions.push((position.z() - center.z()) as f32);
        }

        let mut remap = vec![0u32; vertices.len()];
        // SAFETY: FFI transcription of the C++ calls. All pointers come
        // from live same-length-or-longer buffers (`remap` has one entry
        // per vertex; `indices`/`positions` are non-empty here), sizes are
        // in the units each function documents (12-byte vertices), and
        // meshoptimizer touches only the ranges it is given. The calls are
        // pure functions of their inputs (no shared mutable state), so
        // concurrent calls from island workers are safe — the C++ already
        // calls them from TBB workers.
        let welded_vertex_count = unsafe {
            meshopt_generateVertexRemap(
                remap.as_mut_ptr(),
                indices.as_ptr(),
                indices.len(),
                positions.as_ptr() as *const c_void,
                vertices.len(),
                std::mem::size_of::<f32>() * 3,
            )
        };
        let mut welded_vertices = vec![Vector3::default(); welded_vertex_count];
        let mut welded_positions = vec![0.0f32; welded_vertex_count * 3];
        unsafe {
            meshopt_remapIndexBuffer(
                indices.as_mut_ptr(),
                indices.as_ptr(),
                indices.len(),
                remap.as_ptr(),
            );
            meshopt_remapVertexBuffer(
                welded_positions.as_mut_ptr() as *mut c_void,
                positions.as_ptr() as *const c_void,
                vertices.len(),
                std::mem::size_of::<f32>() * 3,
                remap.as_ptr(),
            );
        }
        for (i, &r) in remap.iter().enumerate() {
            if r != u32::MAX {
                welded_vertices[r as usize] = vertices[i];
            }
        }

        let mut vertex_lock = Vec::new();
        if sharp_edge_degrees > 0.0 {
            vertex_lock.resize(welded_vertex_count, 0);
            mark_sharp_edge_vertices(
                &welded_vertices,
                &indices,
                sharp_edge_degrees * (std::f64::consts::PI / 180.0),
                &mut vertex_lock,
            );
        }

        let mut decimated = vec![0u32; indices.len()];
        let mut result_error = 0.0f32;
        let decimated_len = unsafe {
            meshopt_simplifyWithAttributes(
                decimated.as_mut_ptr(),
                indices.as_ptr(),
                indices.len(),
                welded_positions.as_ptr(),
                welded_vertex_count,
                std::mem::size_of::<f32>() * 3,
                std::ptr::null(),
                0,
                std::ptr::null(),
                0,
                if vertex_lock.is_empty() {
                    std::ptr::null()
                } else {
                    vertex_lock.as_ptr()
                },
                decimate_triangle_count * 3,
                f32::MAX,
                MESHOPT_SIMPLIFY_REGULARIZE,
                &mut result_error,
            )
        };
        decimated.truncate(decimated_len);
        let _ = result_error;

        if decimated.len() < 3 || decimated.len() >= indices.len() {
            return false;
        }

        let mut output_index_of_welded = vec![usize::MAX; welded_vertex_count];
        let mut decimated_vertices = Vec::new();
        let mut decimated_triangles = Vec::with_capacity(decimated.len() / 3);
        let mut i = 0;
        while i + 2 < decimated.len() {
            let mut triangle = vec![0usize; 3];
            for j in 0..3 {
                let welded_index = decimated[i + j];
                if output_index_of_welded[welded_index as usize] == usize::MAX {
                    output_index_of_welded[welded_index as usize] = decimated_vertices.len();
                    decimated_vertices.push(welded_vertices[welded_index as usize]);
                }
                triangle[j] = output_index_of_welded[welded_index as usize];
            }
            decimated_triangles.push(triangle);
            i += 3;
        }

        stats.islands_decimated.fetch_add(1, Ordering::SeqCst);
        stats
            .triangles_before
            .fetch_add(triangles.len(), Ordering::SeqCst);
        stats
            .triangles_after
            .fetch_add(decimated_triangles.len(), Ordering::SeqCst);

        *vertices = decimated_vertices;
        *triangles = decimated_triangles;
        true
    }

    /// Mirrors `resample`. The stats/time pointers are non-null at the
    /// single call site (plain references); only the progress handler is
    /// genuinely optional (`None` in quiet mode). The decimated-output
    /// pointers are likewise always non-null (plain `&mut`). The handler
    /// moves in by value where the C++ passes a pointer to its local: the
    /// downstream remesher takes ownership either way (the C++ copies the
    /// `std::function` into its member), so exactly one live handler
    /// exists during the call on both sides.
    #[allow(clippy::too_many_arguments)]
    fn resample(
        vertices: &mut Vec<Vector3>,
        triangles: &mut Vec<Vec<usize>>,
        voxel_size: f64,
        adaptivity: f64,
        sharp_edge_degrees: f64,
        smooth_normal_degrees: f64,
        // (Used only by the C++ AUTO_REMESHER_DEBUG prints, which compile
        // out; kept for the 1:1 signature.)
        _island_index: usize,
        decimation_stats: &DecimationStats,
        adaptive_field_time_us: &AtomicI64,
        progress_handler: Option<ProgressHandler>,
        decimated_vertices_out: &mut Vec<Vector3>,
        decimated_triangles_out: &mut Vec<Vec<usize>>,
        density_in: &[f64],
        density_out: &mut Vec<f64>,
    ) {
        // Local density control, default off: every block below is guarded
        // on densityActive, so a run without a mask executes the exact same
        // statements (and floating-point ops) as before.
        let mut island_density = Vec::new();
        if !density_in.is_empty() && density_in.len() == vertices.len() {
            island_density = Density::normalize_field(density_in);
        }
        let mut density_active = !island_density.is_empty();
        let mut positions_before_decimate = Vec::new();
        if density_active {
            positions_before_decimate = vertices.clone();
        }

        let t_decimate_start = Instant::now();
        let decimated = Self::decimate_if_too_dense(
            vertices,
            triangles,
            voxel_size,
            sharp_edge_degrees,
            _island_index,
            decimation_stats,
        );
        if density_active && decimated {
            // Decimation retopologized the island: carry the mask across by
            // nearest position, then re-normalize (a degenerate map falls
            // back to uniform, which normalizes back to OFF).
            island_density = Density::normalize_field(&Density::resample_nearest(
                &positions_before_decimate,
                &island_density,
                vertices,
            ));
            density_active = !island_density.is_empty();
        }
        decimation_stats.time_us.fetch_add(
            t_decimate_start.elapsed().as_micros() as i64,
            Ordering::SeqCst,
        );

        *decimated_vertices_out = vertices.clone();
        *decimated_triangles_out = triangles.clone();

        let t_field_start = Instant::now();
        let mut vertex_target_lengths = Vec::new();
        let density_usable = density_active && island_density.len() == vertices.len();
        if (adaptivity > 0.0 || density_usable) && !vertices.is_empty() {
            // A target-length field redistributes the uniform triangle
            // budget. The field is deliberately computed on the input mesh:
            // IsotropicRemesher propagates it to vertices created by edge
            // splits.
            let min_ratio = 0.35;
            let max_ratio = 3.0;
            let epsilon = 1e-12;

            // Do not add face normals directly from parallel workers:
            // adjacent faces write to the same vertex. Compute faces in
            // parallel, then do the small accumulation pass serially.
            let mut face_normals = vec![Vector3::default(); triangles.len()];
            let mut face_areas = vec![0.0; triangles.len()];
            parallel_each_zip2(&mut face_normals, &mut face_areas, |i, normal, area| {
                let tri = &triangles[i];
                *area = Vector3::area(&vertices[tri[0]], &vertices[tri[1]], &vertices[tri[2]]);
                if *area > epsilon {
                    *normal =
                        Vector3::normal(&vertices[tri[0]], &vertices[tri[1]], &vertices[tri[2]]);
                }
            });

            let mut normals = vec![Vector3::default(); vertices.len()];
            let mut neighbors: Vec<Vec<usize>> = Vec::new();
            neighbors.resize_with(vertices.len(), Vec::new);
            for i in 0..triangles.len() {
                let tri = &triangles[i];
                if face_areas[i] <= epsilon {
                    continue;
                }
                let weighted_normal = face_normals[i] * face_areas[i];
                for j in 0..3 {
                    normals[tri[j]] += weighted_normal;
                    neighbors[tri[j]].push(tri[(j + 1) % 3]);
                    neighbors[tri[j]].push(tri[(j + 2) % 3]);
                }
            }
            parallel_each_zip2(&mut normals, &mut neighbors, |_, normal, ring| {
                normal.normalize();
                ring.sort_unstable();
                ring.dedup();
            });

            // Mean normal variation per unit length is less sensitive to a
            // single bad triangle than the previous maximum-one-ring
            // estimate.
            let mut vertex_curvature = vec![0.0; vertices.len()];
            parallel_each(&mut vertex_curvature, |v, curvature| {
                let ring = &neighbors[v];
                if ring.is_empty() || normals[v].length_squared() <= epsilon {
                    return;
                }
                let mut weighted_curvature = 0.0;
                let mut total_weight = 0.0;
                for &u in ring {
                    let length = (vertices[u] - vertices[v]).length();
                    if length <= epsilon || normals[u].length_squared() <= epsilon {
                        continue;
                    }
                    let mut cosine = Vector3::dot_product(&normals[v], &normals[u]);
                    cosine = cxx_min(1.0, cxx_max(-1.0, cosine));
                    weighted_curvature += cosine.acos();
                    total_weight += length;
                }
                if total_weight > epsilon {
                    *curvature = weighted_curvature / total_weight;
                }
            });

            // A percentile reference prevents a few very sharp/noisy
            // vertices from making the rest of the surface appear flat.
            let mut non_zero_curvatures = Vec::with_capacity(vertex_curvature.len());
            for &curvature in &vertex_curvature {
                if curvature > epsilon {
                    non_zero_curvatures.push(curvature);
                }
            }
            let have_curvature_reference = adaptivity > 0.0 && !non_zero_curvatures.is_empty();
            if have_curvature_reference || density_usable {
                vertex_target_lengths.resize(vertices.len(), 0.0);
                let mut importance = vec![1.0; vertices.len()];
                if have_curvature_reference {
                    let reference_index = (non_zero_curvatures.len() - 1) * 3 / 4;
                    // `std::nth_element` with `<`: both place the rank-`k`
                    // value at `k`, so the reference value is identical on
                    // NaN-free inputs (NaN curvatures make the comparator
                    // inconsistent on both sides — unspecified either way).
                    let (_, reference, _) = non_zero_curvatures
                        .select_nth_unstable_by(reference_index, |a, b| {
                            a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal)
                        });
                    let curvature_reference = *reference;
                    let strength = cxx_min(adaptivity, 2.0) * 7.0;
                    parallel_each(&mut importance, |v, slot| {
                        let normalized = cxx_min(
                            4.0,
                            vertex_curvature[v] / cxx_max(curvature_reference, epsilon),
                        );
                        // FMA audit (site A): Clang fuses the outer
                        // multiply-add (IR: plain fmul, then
                        // llvm.fmuladd(t, normalized, slot)); the first
                        // multiply stays unfused. Transcribed explicitly.
                        *slot = (strength * normalized).mul_add(normalized, *slot);
                    });
                    if density_usable {
                        // Fold the mask in multiplicatively: local triangle
                        // density scales by the mask while the
                        // average-importance normalization below keeps the
                        // island budget fixed.
                        for v in 0..vertices.len() {
                            importance[v] *= island_density[v];
                        }
                    }
                } else {
                    // Flat island under a mask (or adaptivity off): the mask
                    // alone drives the target-length field.
                    importance.clone_from(&island_density);
                }

                // Keep integral(area / h^2) equal to the uniform field,
                // which preserves the budget implied by voxelSize while
                // moving triangles from flat regions to detailed ones.
                let mut total_area = 0.0;
                let mut weighted_importance = 0.0;
                for i in 0..triangles.len() {
                    let tri = &triangles[i];
                    total_area += face_areas[i];
                    weighted_importance += face_areas[i]
                        * (importance[tri[0]] + importance[tri[1]] + importance[tri[2]])
                        / 3.0;
                }
                if total_area > epsilon {
                    let average_importance = weighted_importance / total_area;
                    for v in 0..vertices.len() {
                        let mut multiplier = (average_importance / importance[v]).sqrt();
                        multiplier = cxx_min(max_ratio, cxx_max(min_ratio, multiplier));
                        vertex_target_lengths[v] = voxel_size * multiplier;
                    }
                } else {
                    vertex_target_lengths.clear();
                }
            }
        }
        adaptive_field_time_us
            .fetch_add(t_field_start.elapsed().as_micros() as i64, Ordering::SeqCst);

        let mut positions_before_remesh = Vec::new();
        if density_usable {
            positions_before_remesh.clone_from(vertices);
        }
        let mut isotropic_remesher = IsotropicRemesher::new(vertices, triangles);
        // NOTE: the C++ re-checks `*progressHandler` (the std::function
        // null state); the only constructed handler is a live closure, so
        // the check is dead and has no counterpart.
        if let Some(handler) = progress_handler {
            isotropic_remesher.set_progress_handler(handler);
        }
        isotropic_remesher.set_target_edge_length(voxel_size);
        if !vertex_target_lengths.is_empty() {
            isotropic_remesher.set_vertex_target_edge_lengths(&vertex_target_lengths);
        }
        isotropic_remesher.set_sharp_edge_degrees(sharp_edge_degrees);
        isotropic_remesher.set_smooth_normal_degrees(smooth_normal_degrees);
        isotropic_remesher.remesh();
        *vertices = isotropic_remesher.remeshed_vertices().to_vec();
        *triangles = isotropic_remesher.remeshed_triangles().to_vec();
        density_out.clear();
        if density_usable {
            // Isotropic remeshing retopologized the island: carry the mask
            // to the new vertices so the parameterizer can modulate its
            // scaling field. A degenerate map normalizes back to OFF
            // downstream.
            *density_out = Density::normalize_field(&Density::resample_nearest(
                &positions_before_remesh,
                &island_density,
                vertices,
            ));
        }
    }

    /// Mirrors `updateProgress`: `progress` is how far island `thread_index`
    /// has got, 0..1; `status` names the step it is on, or `None` to keep
    /// the island's current one. Called from the island worker threads.
    pub fn update_progress(&self, thread_index: usize, progress: f32, status: Option<&str>) {
        let mut data = self.progress.lock_progress();
        ProgressState::update_locked(&mut data, thread_index, progress, status);
    }

    /// Mirrors `accumulateStageTime`: records how long a named pipeline
    /// step took, summed over the islands that ran it. `order` places the
    /// step in the phase report; it is the step's position along the
    /// pipeline, so the report reads in execution order no matter which
    /// island happened to reach the step first.
    pub fn accumulate_stage_time(&self, name: &str, order: f32, microseconds: i64) {
        self.progress.accumulate(name, order, microseconds);
    }

    /// Mirrors `makeStageProgress`: a handler for one stage of one island
    /// that maps the stage's own 0..1 fraction onto `[begin, end]` of that
    /// island's progress, and times each named step on the way through for
    /// the phase report. `stage_order` is where the stage sits along the
    /// pipeline, so the report reads in execution order. Only the island's
    /// own worker thread calls the result.
    pub fn make_stage_progress(
        &self,
        island_index: usize,
        begin: f32,
        end: f32,
        stage_order: f32,
    ) -> ProgressHandler {
        // The C++ lambda is `mutable`, but `ProgressHandler` is `Fn`: the
        // call state lives behind a small mutex instead. Only the island's
        // own worker thread calls the result, so the lock is uncontended
        // and the call sequence is identical.
        struct StageCapture {
            last_time: Instant,
            last_name: Option<String>,
            last_order: f32,
        }
        let progress = Arc::clone(&self.progress);
        let capture = Mutex::new(StageCapture {
            last_time: Instant::now(),
            last_name: None,
            last_order: 0.0,
        });
        Box::new(move |fraction: f32, name: &str| {
            let now = Instant::now();
            let mut capture = capture.lock().unwrap_or_else(|e| e.into_inner());
            if let Some(prev) = capture.last_name.take() {
                let us = now.duration_since(capture.last_time).as_micros() as i64;
                progress.accumulate(&prev, capture.last_order, us);
            }
            capture.last_time = now;
            capture.last_name = Some(name.to_string());
            capture.last_order = stage_order + fraction;
            drop(capture);
            let mut data = progress.lock_progress();
            // FMA audit (site D): Clang fuses the f32 map to
            // llvm.fmuladd(end - begin, fraction, begin). Transcribed
            // explicitly.
            let mapped = (end - begin).mul_add(fraction, begin);
            ProgressState::update_locked(&mut data, island_index, mapped, Some(name));
        })
    }

    /// Merges per-island vertex/triangle vectors into one mesh, offsetting
    /// triangle corners by each island's vertex base (the C++ `mergeIslands`
    /// lambda, which captures nothing).
    fn merge_islands(
        island_vertices: &[Vec<Vector3>],
        island_triangles: &[Vec<Vec<usize>>],
        merged_vertices: &mut Vec<Vector3>,
        merged_triangles: &mut Vec<Vec<usize>>,
    ) {
        for i in 0..island_vertices.len() {
            let vertex_offset = merged_vertices.len();
            merged_vertices.extend(island_vertices[i].iter().cloned());
            for triangle in &island_triangles[i] {
                let mut offset_triangle = Vec::with_capacity(triangle.len());
                for &index in triangle {
                    offset_triangle.push(index + vertex_offset);
                }
                merged_triangles.push(offset_triangle);
            }
        }
    }

    /// Formats a microsecond count like the C++ phase-report `milliseconds`
    /// closure (one decimal place: whole milliseconds hide the per-island
    /// steps on a mesh split into many small islands).
    fn format_ms(microseconds: i64) -> String {
        format!("{:.1} ms", microseconds as f64 / 1000.0)
    }

    /// Pushes one `"<name>: <ms>"` line onto the phase report (the C++
    /// `phase` lambda).
    fn push_phase_line(&mut self, name: &str, microseconds: i64) {
        self.phase_report
            .push(format!("{name}: {}", Self::format_ms(microseconds)));
    }

    /// Mirrors `remesh`: validates the input, resolves the symmetry plane,
    /// sizes voxels, splits into islands, resamples / parameterizes /
    /// extracts each island on worker threads, merges the outputs (plus
    /// the optional UV atlas and symmetry snap), and builds the phase
    /// report. Returns `false` (with no outputs populated) when the input
    /// is rejected.
    pub fn remesh(&mut self) -> bool {
        self.island_output_quad_counts.clear();
        self.island_dipole_counts.clear();
        // Validate inputs before any sizing math. In particular a zero
        // target triangle count would divide by zero in
        // initializeVoxelSize().
        let mut invalid_input_reason: Option<&str> = None;
        if self.vertices.is_empty() {
            invalid_input_reason = Some("input mesh has no vertices");
        } else if self.triangles.is_empty() {
            invalid_input_reason = Some("input mesh has no triangles");
        } else if self.target_triangle_count == 0 {
            invalid_input_reason = Some("target triangle count must be greater than zero");
        } else {
            // Last line of defense behind the loaders: the island build
            // below indexes m_vertices[face[i]] for i < 3, so a
            // non-triangle face or an out-of-range corner is an
            // out-of-bounds access (observed segfault via a corrupt face
            // line). Valid meshes never trip this.
            for face in &self.triangles {
                let mut face_valid = face.len() == 3;
                for &corner in face {
                    if !face_valid {
                        break;
                    }
                    face_valid = corner < self.vertices.len();
                }
                if !face_valid {
                    invalid_input_reason = Some("input mesh has invalid face indices");
                    break;
                }
            }
        }
        if let Some(reason) = invalid_input_reason {
            // (C++ `Invalid remesh input: ...` stderr omitted per the
            // stderr-gap memo; the handler call below carries it.)
            self.progress.report_direct(1.0, reason);
            return false;
        }

        // Resolve the symmetry plane once for the whole run. Islands share
        // the input's global coordinates, so the plane applies to every
        // island as-is. From here on, m_symmetryPlane.valid() alone gates
        // all symmetry work.
        self.symmetry_plane = SymmetryPlane::default();
        if self.symmetry_enabled {
            if self.symmetry_axis >= 0 && self.symmetry_axis < 3 {
                self.symmetry_plane = Symmetry::fixed_plane(&self.vertices, self.symmetry_axis);
            } else {
                self.symmetry_plane = Symmetry::detect_plane(&self.vertices);
            }
            if !self.symmetry_plane.valid() || self.symmetry_plane.score < MIN_SYMMETRY_SCORE {
                // (C++ `Symmetry skipped: ...` stderr omitted per the
                // stderr-gap memo; the getters report the fallback.)
                self.symmetry_plane = SymmetryPlane::default();
            }
        }

        let t_start = Instant::now();

        // Each label names the step that is about to run, not the one that
        // just finished, so the status line matches what the process is
        // actually doing.
        self.progress.report_direct(0.0, "Computing voxel size");
        let t_voxel_start = Instant::now();
        self.initialize_voxel_size();
        let t_voxel_end = Instant::now();

        self.progress
            .report_direct(0.01, "Splitting mesh into islands");
        let mut triangles_islands: Vec<Vec<Vec<usize>>> = Vec::new();
        let t_split_start = Instant::now();
        MeshSeparator::split_to_islands(&self.triangles, &mut triangles_islands);
        let t_after_split = Instant::now();

        if triangles_islands.is_empty() {
            // (C++ `Input mesh is empty` stderr omitted per the
            // stderr-gap memo; the handler call below carries it.)
            self.progress.report_direct(1.0, "Input mesh is empty");
            return false;
        }

        self.progress
            .report_direct(0.02, "Building island contexts");
        // Islands are compacted independently of each other, and writing
        // into a pre-sized vector by index keeps them in the original
        // order.
        let mut island_ctxs: Vec<IslandContext> = (0..triangles_islands.len())
            .map(|_| IslandContext::default())
            .collect();
        {
            let this = &*self;
            parallel_each(&mut island_ctxs, |island_index, context| {
                let island = &triangles_islands[island_index];
                context.triangles.reserve(island.len());
                // `BTreeMap` for the C++ `unordered_map` (insert/lookup
                // only, never iterated — deterministic either way).
                let mut old_to_new_vertex_map = BTreeMap::new();
                let use_density = !this.density_multipliers.is_empty();
                for face in island {
                    let mut triangle = Vec::with_capacity(3);
                    // (Island faces are validated triangles upstream, so
                    // the C++ `i < 3` loop reads the whole face; `take(3)`
                    // keeps the bound literal.)
                    for &corner in face.iter().take(3) {
                        let next = context.vertices.len();
                        let new_index = *old_to_new_vertex_map.entry(corner).or_insert(next);
                        if new_index == next {
                            context.vertices.push(this.vertices[corner]);
                            if use_density {
                                context.density.push(this.density_multipliers[corner]);
                            }
                        }
                        triangle.push(new_index);
                    }
                    context.triangles.push(triangle);
                }
                if use_density {
                    // Islands fully outside the mask normalize back to
                    // empty and run the unmodified pipeline.
                    context.density = Density::normalize_field(&context.density);
                }

                context.scaling = this.scaling;
                context.voxel_size = this.voxel_size;
                context.adaptivity = this.adaptivity;
                context.anisotropy = this.anisotropy;
                context.sharp_edge_degrees = this.sharp_edge_degrees;
                context.smooth_normal_degrees = this.smooth_normal_degrees;
                context.symmetry_plane = this.symmetry_plane;
            });
        }
        let t_build_end = Instant::now();
        self.progress
            .report_direct(PARALLEL_PHASE_BEGIN, "Remeshing uniformly");

        let resample_time_us = AtomicI64::new(0);
        let adaptive_field_time_us = AtomicI64::new(0);
        let decimation_stats = DecimationStats::default();

        {
            {
                let mut data = self.progress.lock_progress();
                data.thread_progress_weights = vec![1.0; island_ctxs.len()];
                for (i, ctx) in island_ctxs.iter().enumerate() {
                    if !self.triangles.is_empty() {
                        data.thread_progress_weights[i] =
                            (ctx.triangles.len() as f64 / self.triangles.len() as f64) as f32;
                    }
                }
                data.thread_progress = vec![0.0; island_ctxs.len()];
                data.thread_status = vec![None; island_ctxs.len()];
                data.progress_sum = 0.0;
            }

            self.isotropic_vertices.clear();
            self.isotropic_triangles.clear();
            self.decimated_vertices.clear();
            self.decimated_triangles.clear();
            let mut isotropic_island_vertices: Vec<Vec<Vector3>> =
                (0..island_ctxs.len()).map(|_| Vec::new()).collect();
            let mut isotropic_island_triangles: Vec<Vec<Vec<usize>>> =
                (0..island_ctxs.len()).map(|_| Vec::new()).collect();
            let mut decimated_island_vertices: Vec<Vec<Vector3>> =
                (0..island_ctxs.len()).map(|_| Vec::new()).collect();
            let mut decimated_island_triangles: Vec<Vec<Vec<usize>>> =
                (0..island_ctxs.len()).map(|_| Vec::new()).collect();
            // The C++ `IsotropicPhase` functor, as an inline five-way
            // zipped chunk loop (one worker per chunk, disjoint slices):
            // `parallel_each` covers one `&mut` slice, but the worker
            // needs the context plus four output slices at once.
            if !island_ctxs.is_empty() {
                let this = &*self;
                // Share by reference: the `move` worker closures would
                // otherwise try to move these into the first thread.
                let resample_time = &resample_time_us;
                let adaptive_field_time = &adaptive_field_time_us;
                let stats = &decimation_stats;
                let chunk_len = worker_chunk_len(island_ctxs.len());
                thread::scope(|s| {
                    let zipped = island_ctxs
                        .chunks_mut(chunk_len)
                        .zip(isotropic_island_vertices.chunks_mut(chunk_len))
                        .zip(isotropic_island_triangles.chunks_mut(chunk_len))
                        .zip(decimated_island_vertices.chunks_mut(chunk_len))
                        .zip(decimated_island_triangles.chunks_mut(chunk_len));
                    for (
                        chunk_index,
                        ((((ctx_chunk, iso_v_chunk), iso_t_chunk), dec_v_chunk), dec_t_chunk),
                    ) in zipped.enumerate()
                    {
                        let base = chunk_index * chunk_len;
                        s.spawn(move || {
                            let rows = ctx_chunk
                                .iter_mut()
                                .zip(iso_v_chunk.iter_mut())
                                .zip(iso_t_chunk.iter_mut())
                                .zip(dec_v_chunk.iter_mut())
                                .zip(dec_t_chunk.iter_mut());
                            for (k, ((((ctx, iso_v), iso_t), dec_v), dec_t)) in rows.enumerate() {
                                let i = base + k;
                                this.update_progress(i, 0.0, Some("Remeshing uniformly"));
                                // Quiet runs skip downstream progress
                                // installation: no per-stage timings, and
                                // downstream progress echoes (the
                                // extractor's stderr chatter) stay off with
                                // no handler.
                                let quiet = this.quiet();
                                let isotropic_progress = if quiet {
                                    None
                                } else {
                                    Some(this.make_stage_progress(
                                        i,
                                        0.0,
                                        ISLAND_RESAMPLE_END,
                                        -1.0,
                                    ))
                                };

                                let t0 = Instant::now();
                                Self::resample(
                                    &mut ctx.vertices,
                                    &mut ctx.triangles,
                                    ctx.voxel_size,
                                    ctx.adaptivity,
                                    ctx.sharp_edge_degrees,
                                    ctx.smooth_normal_degrees,
                                    i,
                                    stats,
                                    adaptive_field_time,
                                    isotropic_progress,
                                    dec_v,
                                    dec_t,
                                    &ctx.density,
                                    &mut ctx.resampled_density,
                                );
                                let t1 = Instant::now();
                                resample_time.fetch_add(
                                    t1.duration_since(t0).as_micros() as i64,
                                    Ordering::SeqCst,
                                );

                                *iso_v = ctx.vertices.clone();
                                *iso_t = ctx.triangles.clone();

                                this.update_progress(i, ISLAND_RESAMPLE_END, None);
                            }
                        });
                    }
                });
            }
            Self::merge_islands(
                &isotropic_island_vertices,
                &isotropic_island_triangles,
                &mut self.isotropic_vertices,
                &mut self.isotropic_triangles,
            );
            self.decimated = decimation_stats.islands_decimated.load(Ordering::SeqCst) > 0;
            if self.decimated {
                Self::merge_islands(
                    &decimated_island_vertices,
                    &decimated_island_triangles,
                    &mut self.decimated_vertices,
                    &mut self.decimated_triangles,
                );
            }
        }
        let t_isotropic_end = Instant::now();

        let mut parameterization_threads: Vec<ParameterizationThread<'_>> = island_ctxs
            .iter()
            .enumerate()
            .map(|(i, context)| ParameterizationThread {
                island_index: i,
                island: context,
                captured_uvs: Vec::new(),
                captured_original_uvs: Vec::new(),
                captured_extracted_connection_moved: Vec::new(),
                captured_singular_vertices: Vec::new(),
                captured_singular_vertex_indices: Vec::new(),
                captured_extracted_connections: Vec::new(),
                captured_vertex_uvs: Vec::new(),
                remeshed_vertices: Vec::new(),
                remeshed_quads: Vec::new(),
                dipoles_placed: 0,
                compute_vertex_uvs: self.compute_remeshed_uvs,
            })
            .collect();

        let parameterize_time_us = AtomicI64::new(0);
        let extract_time_us = AtomicI64::new(0);
        // The C++ `SurfaceParameterizer` functor.
        {
            let this = &*self;
            parallel_each(&mut parameterization_threads, |_, thread| {
                let t0 = Instant::now();

                let island = thread.island;
                let vertices = &island.vertices;
                let triangles = &island.triangles;

                if vertices.is_empty() || triangles.is_empty() {
                    // Still retire the island, otherwise its share of the
                    // bar is never filled in and the total stalls short of
                    // the end.
                    this.update_progress(thread.island_index, 1.0, None);
                    return;
                }

                this.update_progress(thread.island_index, ISLAND_RESAMPLE_END, None);
                let mut parameterizer = Parameterizer::new(vertices, triangles, None);
                if !this.quiet() {
                    parameterizer.set_progress_handler(this.make_stage_progress(
                        thread.island_index,
                        ISLAND_RESAMPLE_END,
                        ISLAND_PARAMETERIZE_END,
                        0.0,
                    ));
                }
                if island.scaling > 0.0 {
                    parameterizer.set_scaling(island.scaling);
                }
                parameterizer.set_gradient_adaptivity(island.adaptivity);
                parameterizer.set_anisotropy(island.anisotropy);
                parameterizer.set_sharp_edge_degrees(island.sharp_edge_degrees);
                parameterizer.set_symmetry_plane(island.symmetry_plane);
                // Always `Some` (possibly empty, never null — the C++
                // stores `&m_guidePolylines` unconditionally).
                parameterizer.set_guide_polylines(Some(&this.guide_polylines));
                parameterizer.set_sharp_polylines(Some(&this.sharp_polylines));
                if !island.resampled_density.is_empty() {
                    parameterizer.set_density_field(island.resampled_density.clone());
                }
                // Always set (the product default flows down even when it
                // is off: the leaf default must never shadow it).
                parameterizer.set_dipoles(this.dipoles);
                // (No try/catch counterpart: the port signals failure
                // through the `bool` return, so there is nothing to catch —
                // and the C++ `Island N: parameterization failed` stderr
                // falls under the stderr-gap memo besides.)
                let parameterize_succeeded = parameterizer.parameterize();

                let t1 = Instant::now();
                parameterize_time_us
                    .fetch_add(t1.duration_since(t0).as_micros() as i64, Ordering::SeqCst);

                if parameterize_succeeded {
                    thread.dipoles_placed = parameterizer.dipole_flips();
                    this.update_progress(thread.island_index, ISLAND_PARAMETERIZE_END, None);
                    // `take_triangle_uvs` is `Some` on every success path
                    // (the C++ `if (uvs)` null branch is unreachable after
                    // success: `parameterize` populates the UVs before its
                    // single `return true`).
                    if let Some(uvs) = parameterizer.take_triangle_uvs() {
                        // Save a copy of UVs for the [param] preview overlay
                        thread.captured_uvs = uvs.clone();
                        thread.captured_original_uvs =
                            parameterizer.original_triangle_uvs().to_vec();
                        // Capture singular vertex positions for the [param]
                        // preview
                        thread.captured_singular_vertices =
                            parameterizer.singular_vertex_positions().to_vec();
                        thread.captured_singular_vertex_indices =
                            parameterizer.singular_vertex_indices().to_vec();
                        let mut remesher = QuadExtractor::new(vertices, triangles, &uvs);
                        remesher.set_original_triangle_uvs(&thread.captured_original_uvs);
                        remesher.set_singular_vertices(&thread.captured_singular_vertex_indices);
                        // No handler in quiet mode: the extractor's stderr
                        // progress echoes key off handler presence.
                        if !this.quiet() {
                            remesher.set_progress_handler(this.make_stage_progress(
                                thread.island_index,
                                ISLAND_PARAMETERIZE_END,
                                1.0,
                                1.0,
                            ));
                        }
                        remesher.set_compute_vertex_uvs(thread.compute_vertex_uvs);
                        if remesher.extract() {
                            thread.captured_extracted_connections =
                                remesher.extracted_connections().to_vec();
                            thread.captured_extracted_connection_moved =
                                remesher.extracted_connection_moved().to_vec();
                            thread.captured_vertex_uvs = remesher.remeshed_vertex_uvs().to_vec();
                            thread.remeshed_vertices = remesher.remeshed_vertices().to_vec();
                            thread.remeshed_quads = remesher.remeshed_quads().to_vec();
                        }
                    }
                }
                this.update_progress(thread.island_index, 1.0, None);
                let t2 = Instant::now();
                extract_time_us
                    .fetch_add(t2.duration_since(t1).as_micros() as i64, Ordering::SeqCst);
            });
        }
        let t_parallel_end = Instant::now();

        self.progress
            .report_direct(PARALLEL_PHASE_END, "Merging mesh islands");

        // Merge isotropic UVs from all islands (for [param] preview)
        self.isotropic_triangle_uvs.clear();
        self.isotropic_original_triangle_uvs.clear();
        for thread in &parameterization_threads {
            if thread.captured_uvs.is_empty() {
                continue;
            }
            self.isotropic_triangle_uvs
                .extend(thread.captured_uvs.iter().cloned());
            self.isotropic_original_triangle_uvs
                .extend(thread.captured_original_uvs.iter().cloned());
        }

        // Merge singular vertex positions from all islands (for [param]
        // preview)
        self.isotropic_singular_vertices.clear();
        for thread in &parameterization_threads {
            if thread.captured_singular_vertices.is_empty() {
                continue;
            }
            self.isotropic_singular_vertices
                .extend(thread.captured_singular_vertices.iter().cloned());
        }

        // Merge the raw quad-extraction connections for the [param] preview.
        self.isotropic_extracted_connections.clear();
        self.isotropic_extracted_connection_moved.clear();
        for thread in &parameterization_threads {
            self.isotropic_extracted_connections
                .extend(thread.captured_extracted_connections.iter().cloned());
            let keep = self.isotropic_extracted_connections.len()
                - thread.captured_extracted_connections.len();
            self.isotropic_extracted_connection_moved.resize(keep, 0);
            self.isotropic_extracted_connection_moved
                .extend(thread.captured_extracted_connection_moved.iter().cloned());
            let total = self.isotropic_extracted_connections.len();
            self.isotropic_extracted_connection_moved.resize(total, 0);
        }
        self.remeshed_vertices.clear();
        self.remeshed_quads.clear();
        self.remeshed_vertex_uvs.clear();
        self.island_output_quad_counts = vec![0; parameterization_threads.len()];
        self.island_dipole_counts = vec![0; parameterization_threads.len()];
        let mut island_uv_spans: Vec<(usize, usize)> = Vec::new();
        for thread in &parameterization_threads {
            // Dipole counts merge for every island (placement happens in
            // parameterize(), even when extraction later yields nothing).
            self.island_dipole_counts[thread.island_index] = thread.dipoles_placed;
            // (The C++ null-remesher skip: `remeshed_quads` stays empty when
            // the island produced nothing, subsuming both C++ skip cases.)
            if thread.remeshed_quads.is_empty() {
                continue;
            }
            self.island_output_quad_counts[thread.island_index] = thread.remeshed_quads.len();
            let vertex_start_index = self.remeshed_vertices.len();
            self.remeshed_vertices
                .reserve(thread.remeshed_vertices.len());
            for it in &thread.remeshed_vertices {
                self.remeshed_vertices.push(*it);
            }
            for it in &thread.remeshed_quads {
                let mut quad = Vec::with_capacity(it.len());
                for &v in it {
                    quad.push(vertex_start_index + v);
                }
                self.remeshed_quads.push(quad);
            }
            if self.compute_remeshed_uvs {
                // The extractor guarantees one UV per vertex; pad
                // defensively so the merged accessor can never disagree
                // with the vertex count.
                island_uv_spans.push((vertex_start_index, thread.remeshed_vertices.len()));
                self.remeshed_vertex_uvs
                    .reserve(self.remeshed_vertices.len());
                for i in 0..thread.remeshed_vertices.len() {
                    self.remeshed_vertex_uvs.push(
                        thread
                            .captured_vertex_uvs
                            .get(i)
                            .cloned()
                            .unwrap_or(Vector2::new(0.5, 0.5)),
                    );
                }
            }
        }
        if self.compute_remeshed_uvs {
            self.remeshed_vertex_uvs
                .resize(self.remeshed_vertices.len(), Vector2::new(0.5, 0.5));
            pack_island_uvs_into_atlas(&mut self.remeshed_vertex_uvs, &island_uv_spans);
        } else {
            self.remeshed_vertex_uvs.clear();
        }

        // Mirror partners can live on different islands (two disconnected
        // halves), so the vertex constraint runs once on the merged output,
        // not per island.
        if self.symmetry_plane.valid() && !self.remeshed_vertices.is_empty() {
            Symmetry::symmetrize_vertices(&mut self.remeshed_vertices, &self.symmetry_plane);
        }

        let t_merge_end = Instant::now();

        let elapsed_us = |from: Instant, to: Instant| to.duration_since(from).as_micros() as i64;
        let t_voxel_us = elapsed_us(t_voxel_start, t_voxel_end);
        let t_split_us = elapsed_us(t_split_start, t_after_split);
        let t_build_us = elapsed_us(t_after_split, t_build_end);
        let t_isotropic_wall_us = elapsed_us(t_build_end, t_isotropic_end);
        let t_parameterize_wall_us = elapsed_us(t_isotropic_end, t_parallel_end);
        let t_parallel_wall_us = elapsed_us(t_build_end, t_parallel_end);
        let t_merge_us = elapsed_us(t_parallel_end, t_merge_end);
        let t_total_us = elapsed_us(t_start, t_merge_end);

        let t_decimate_us = decimation_stats.time_us.load(Ordering::SeqCst);
        let t_adaptive_field_us = adaptive_field_time_us.load(Ordering::SeqCst);
        let decimated_islands = decimation_stats.islands_decimated.load(Ordering::SeqCst);

        self.phase_report.clear();
        // (The C++ builds these lines through a `phase` lambda capturing
        // an ostringstream; direct pushes keep the borrow checker quiet
        // across the stage-timing lock below. Same lines, same order.)
        self.phase_report.push(format!(
            "Islands: {}, input triangles: {}",
            island_ctxs.len(),
            self.triangles.len()
        ));
        self.push_phase_line("Compute voxel size", t_voxel_us);
        self.push_phase_line("Split into islands", t_split_us);
        self.push_phase_line("Build island contexts", t_build_us);

        if decimated_islands > 0 {
            self.phase_report.push(format!(
                "Mesh simplifier: RAN on {decimated_islands} of {} islands, {} -> {} triangles, {}",
                decimation_stats.islands_considered.load(Ordering::SeqCst),
                decimation_stats.triangles_before.load(Ordering::SeqCst),
                decimation_stats.triangles_after.load(Ordering::SeqCst),
                Self::format_ms(t_decimate_us)
            ));
        } else {
            self.phase_report.push(format!(
                "Mesh simplifier: SKIPPED (no island above {}x target triangle count), {}",
                DECIMATE_TRIGGER_RATIO as i64,
                Self::format_ms(t_decimate_us)
            ));
        }

        // The accumulated figures sum the islands, so on a multi-island
        // mesh they add up to more than the wall clock next to them. That
        // gap is the point: accumulated / wall is how many cores the phase
        // actually kept busy.
        self.push_phase_line(
            "Adaptive target length field (accumulated)",
            t_adaptive_field_us,
        );
        self.push_phase_line(
            "Isotropic remesh (accumulated)",
            resample_time_us.load(Ordering::SeqCst) - t_decimate_us - t_adaptive_field_us,
        );
        self.push_phase_line(
            "Parameterize (accumulated)",
            parameterize_time_us.load(Ordering::SeqCst),
        );
        self.push_phase_line(
            "Quad extract (accumulated)",
            extract_time_us.load(Ordering::SeqCst),
        );

        {
            let mut stages = self.progress.lock_stages();
            // `std::sort` transcribes as `sort_unstable_by` (both unstable;
            // equal orders keep no defined relative order on either side).
            stages.sort_unstable_by(|a, b| {
                a.order
                    .partial_cmp(&b.order)
                    .unwrap_or(std::cmp::Ordering::Equal)
            });
            for it in stages.iter() {
                self.phase_report.push(format!(
                    "    {}: {}",
                    it.name,
                    Self::format_ms(it.microseconds)
                ));
            }
        }

        self.push_phase_line("Isotropic phase wall clock", t_isotropic_wall_us);
        self.push_phase_line("Parameterize phase wall clock", t_parameterize_wall_us);
        self.push_phase_line("Parallel phase wall clock", t_parallel_wall_us);

        {
            let accumulated = resample_time_us.load(Ordering::SeqCst)
                + parameterize_time_us.load(Ordering::SeqCst)
                + extract_time_us.load(Ordering::SeqCst);
            self.phase_report.push(format!(
                "Cores kept busy across the parallel phase: {:.2} (islands are the unit of parallelism)",
                if t_parallel_wall_us > 0 {
                    accumulated as f64 / t_parallel_wall_us as f64
                } else {
                    0.0
                }
            ));
        }

        self.push_phase_line("Merge islands", t_merge_us);
        self.push_phase_line("Total", t_total_us);

        // The report itself is always populated (phaseReport()); only the
        // stderr dump is quiet-gated. Warnings and errors above still
        // print. (The dump itself falls under the stderr-gap memo: omitted
        // on both paths; the main lane owns the stderr audit.)

        self.progress.report_direct(1.0, "Done");

        true
    }
}
