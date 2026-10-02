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
//! zero vendoring, zero third-party calls: pre-decimation
//! (`decimate_if_too_dense`) runs the native Rust port
//! ([`crate::decimator`], f64 internals for noise stability). The port
//! was proven bit-faithful against the C++ transcription seed-for-seed,
//! then flipped to the default once the coverage retry (item 6b)
//! robustified the downstream against its re-tiling (the beast@1000
//! knife-edge that blocked the flip now recovers 8/8); the vendored
//! meshoptimizer, its FFI, and the `build.rs` cc step were deleted with
//! the flip.
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
//! Layout: one phase per file (`normalize` input conditioning,
//! `decimate` sizing + pre-decimation, `island` splitting +
//! attempt state, `resample` isotropic resampling, `coverage`
//! check + retry, `pipeline` the `remesh` driver + merge, plus
//! `progress` reporting, `atlas` UV packing). Method order follows
//! the C++ within and across files.

mod atlas;
mod coverage;
mod decimate;
mod island;
mod normalize;
mod pipeline;
mod progress;
mod resample;
#[cfg(test)]
mod tests;

use self::progress::ProgressState;
use crate::density::Density;
use crate::progress::ProgressHandler;
use crate::quad_parameterizer::DipoleConfig;
use crate::symmetry::SymmetryPlane;
use crate::vector2::Vector2;
use crate::vector3::Vector3;
use std::ffi::c_void;
use std::sync::{Arc, Mutex};
use std::time::Instant;

pub use self::coverage::CoverageReport;
pub use self::decimate::DecimationStats;

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

const PARALLEL_PHASE_BEGIN: f32 = 0.03;
const PARALLEL_PHASE_END: f32 = 0.95;

const DECIMATE_TRIGGER_RATIO: f64 = 8.0;

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
    coverage_reports: Vec<CoverageReport>,
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
    verbose: bool,
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
            coverage_reports: Vec::new(),
            island_dipole_counts: Vec::new(),
            // Product default for dipole insertion (no C++ counterpart;
            // see set_dipoles): automatic. Validated end to end on the
            // finger fixtures (docs/dipole-production.md) — systematic
            // masked gains, unmasked/mild runs bit-identical via gating.
            dipoles: DipoleConfig::automatic(),
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
            verbose: false,
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

    /// Enables engine-owned stderr chatter (extractor progress dump).
    /// The CLI turns it on for `--verbose` only; default runs keep
    /// progress percentages without the dump.
    pub fn set_verbose(&mut self, verbose: bool) {
        self.verbose = verbose;
    }

    /// Mirrors `verbose`.
    #[must_use]
    pub fn verbose(&self) -> bool {
        self.verbose
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

    /// Coverage retry reports from the last `remesh` (island order; one
    /// entry per island whose first attempt failed coverage, recovered
    /// or not; empty when no retry fired).
    #[must_use]
    pub fn coverage_reports(&self) -> &[CoverageReport] {
        &self.coverage_reports
    }

    /// Per-island dipole flips applied by the last `remesh` (island
    /// order, alongside `island_output_quad_counts`; all zeros when
    /// dipoles are off, unmasked, or mild).
    #[must_use]
    pub fn island_dipole_counts(&self) -> &[usize] {
        &self.island_dipole_counts
    }

    /// Dipole-insertion config (no C++ counterpart). Default automatic:
    /// density-boundary singularity rings on sharp masked steps (asks
    /// above ~2.5x); unmasked, mild, and smooth masks gate to a no-op.
    /// Pass `DipoleConfig::off()` (CLI `--dipoles off`) for the legacy
    /// sizing-only behavior.
    pub fn set_dipoles(&mut self, config: DipoleConfig) {
        self.dipoles = config;
    }

    /// The current dipole-insertion config.
    #[must_use]
    pub fn dipoles(&self) -> DipoleConfig {
        self.dipoles
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
}
