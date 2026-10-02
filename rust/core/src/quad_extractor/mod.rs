//! Port of `core/quadextractor.*` (`retopo.core.quad_extractor`).
//!
//! Line-by-line mirror: same methods in the same order, same thresholds,
//! same constants. Public API is snake_cased 1:1 (`extractConnections` ->
//! [`QuadExtractor::extract_connections`], but private in both).
//!
//! Deliberate restructures (all behavior-preserving, each marked at the
//! site with `// Restructure:`):
//!
//! - The port once emulated libc++ `__hash_table` iteration order
//!   (`CxxSet`/`CxxMap`) because the greedy passes observe it (collapse
//!   survivors, face discovery, neighbor sums). The C++ reference is
//!   gone, so every container here is now [`BTreeMap`]/[`BTreeSet`]:
//!   deterministic sorted order, same one the `std::map`/`std::set`
//!   mirrors always used. Switching orders re-tiles meshes (re-baselined
//!   oracles), but order is arbitrary either way — the noise harness
//!   shows hash order was never load-bearing for quality.
//! - C++ erases from maps/sets while iterating (`simplifyGraph`,
//!   `smoothAroundVertices`) and default-inserts on `operator[]` reads;
//!   Rust collects keys first, then mutates, and reads through `get` with
//!   an explicit default. Each site notes why the insert/erase traffic is
//!   unobservable.
//! - C++ lambdas capturing `&mut self` become private associated functions
//!   and free helpers with explicit parameters (borrowck); the call graphs
//!   are unchanged.
//! - `tbb::parallel_for` becomes [`crate::par::parallel_each`]: every
//!   parallel site in the C++ reads one buffer and writes another (or
//!   writes disjoint slots), so the parallel result is identical by
//!   construction (noted at the site).
//! - `std::sort` on hole edge scores becomes `sort_unstable_by`: both are
//!   deterministic but order score ties differently (libc++ introsort vs
//!   pdqsort). The oracle measures the fallout; see the report.
//! - `std::max_element` (first maximum) is a manual strict-`<` loop:
//!   Rust's `max_by` returns the LAST maximum.
//! - `AUTO_REMESHER_DEV` obj-dump blocks are compiled out (the flag is not
//!   in the build definitions) and are not mirrored.
//!
//! FMA transcription (mandatory audit, redone 2026-10-01 on brew Clang 23
//! ARM64 `-O3` after the oracle rejected the first audit — see below):
//! BARE-SCALAR `p +/- q*r` shapes fuse (first product fused: `p*b + q*d`
//! -> `fma(p, b, q*d)`, `v + d*s` -> `fma(d, s, v)`), but Vector2/3
//! OPERATOR expressions never do: `a*(1-r)+b*r`, `v+d*s`, `v-d*s`,
//! `a+ab*v+ac*w` and `a-ab*v-ac*w` through the (inlined) `operator*` /
//! `operator+` / `operator-` temporaries all compile to plain fmul/fadd/
//! fsub chains (probe-verified per shape; the temporaries break the
//! fusion patterns). The first (wrong) audit assumed the vector shapes
//! fused like the scalar ones and transcribed them with explicit
//! `mul_add`; the differential oracle caught it (200/218 cases off by
//! 1ulp on connection endpoints). Rule used below: vector shapes go
//! through the crate operators untouched (Rust never fuses implicitly,
//! so the operator expression is already bitwise); bare-scalar mul-add
//! shapes use explicit [`f64::mul_add`] via `fma_first`/`fma_first_sub`
//! or inline. The vector-internal fusions (dot, length) are already
//! mirrored by [`crate::vector2`]/[`crate::vector3`].
//!
//! Former inline mirrors, deduped at join: `PositionKey` was a private
//! mirror of `retopo.core.position_key` and
//! `TpVector3`/`AxisAlignedBox`/`AxisAlignedBoxTree` a private mirror of
//! the thirdparty isotropicremesher bounding-box tree, both vendored while
//! the sibling lanes ran concurrently. The port now uses the joined
//! `crate::position_key` and `crate::iso_remesh_kernel` directly
//! (call sites adapted to the joined owned-box API).
//! Layout: one pipeline stage per file, in pipeline order
//! (`extract` -> `simplify` -> `mesh` -> `walk` -> `holes` ->
//! `project` -> `splits` -> `merge` -> `valence` -> `cleanup` ->
//! `finish`; shared bits in `types`/`geom`). Method order follows
//! the C++ within and across files.

mod cleanup;
mod extract;
mod finish;
mod geom;
mod holes;
mod merge;
mod mesh;
mod project;
mod simplify;
mod splits;
#[cfg(test)]
mod tests;
mod types;
mod valence;
mod walk;

use self::types::ConnectionInfo;
use crate::progress::ProgressHandler;
use crate::vector2::Vector2;
use crate::vector3::Vector3;
use std::collections::{BTreeMap, BTreeSet};

/// Quad extractor (mirrors `AutoRemesher::QuadExtractor`).
///
/// Holds borrowed inputs (the C++ holds pointers) and owned outputs.
/// Nullable inputs (null in C++) are `Option`s that stay `None` unless the
/// corresponding setter ran.
pub struct QuadExtractor<'a> {
    vertices: &'a [Vector3],
    triangles: &'a [Vec<usize>],
    triangle_uvs: &'a [Vec<Vector2>],
    remeshed_vertices: Vec<Vector3>,
    remeshed_polygons: Vec<Vec<usize>>,
    remeshed_vertex_uvs: Vec<Vector2>,
    compute_vertex_uvs: bool,
    extracted_connections: Vec<(Vector3, Vector3)>,
    extracted_connection_moved: Vec<u8>,
    original_triangle_uvs: Option<&'a [Vec<Vector2>]>,
    singular_vertices: Option<&'a [usize]>,
    progress_handler: Option<ProgressHandler>,
    verbose_dump: bool,
    connection_infos: BTreeMap<(usize, usize), ConnectionInfo>,
    added_connections: BTreeSet<(usize, usize)>,
    half_edges: BTreeSet<(usize, usize)>,
}

/// A walked cleanup-ladder route: the rungs, the dissolved faces, and the
/// sink face (`usize::MAX` for a border sink).
type CleanupRoute = (Vec<(usize, usize)>, BTreeSet<usize>, usize);

impl<'a> QuadExtractor<'a> {
    #[must_use]
    pub fn new(
        vertices: &'a [Vector3],
        triangles: &'a [Vec<usize>],
        triangle_uvs: &'a [Vec<Vector2>],
    ) -> Self {
        Self {
            vertices,
            triangles,
            triangle_uvs,
            remeshed_vertices: Vec::new(),
            remeshed_polygons: Vec::new(),
            remeshed_vertex_uvs: Vec::new(),
            compute_vertex_uvs: false,
            extracted_connections: Vec::new(),
            extracted_connection_moved: Vec::new(),
            original_triangle_uvs: None,
            singular_vertices: None,
            progress_handler: None,
            verbose_dump: false,
            connection_infos: BTreeMap::new(),
            added_connections: BTreeSet::new(),
            half_edges: BTreeSet::new(),
        }
    }

    #[must_use]
    pub fn remeshed_vertices(&self) -> &[Vector3] {
        &self.remeshed_vertices
    }

    #[must_use]
    pub fn remeshed_quads(&self) -> &[Vec<usize>] {
        &self.remeshed_polygons
    }

    /// Per-output-vertex UVs interpolated from the input parameterization,
    /// normalized to 0..1 over this island's UV bounding box. Only computed
    /// when [`Self::set_compute_vertex_uvs`] ran before [`Self::extract`];
    /// otherwise empty. Computing them never alters geometry: a pure
    /// post-pass over the final positions.
    #[must_use]
    pub fn remeshed_vertex_uvs(&self) -> &[Vector2] {
        &self.remeshed_vertex_uvs
    }

    pub fn set_compute_vertex_uvs(&mut self, compute: bool) {
        self.compute_vertex_uvs = compute;
    }

    /// The raw connections produced by `extract_connections`, before graph
    /// cleanup.
    #[must_use]
    pub fn extracted_connections(&self) -> &[(Vector3, Vector3)] {
        &self.extracted_connections
    }

    pub fn set_original_triangle_uvs(&mut self, original_triangle_uvs: &'a [Vec<Vector2>]) {
        self.original_triangle_uvs = Some(original_triangle_uvs);
    }

    pub fn set_singular_vertices(&mut self, singular_vertices: &'a [usize]) {
        self.singular_vertices = Some(singular_vertices);
    }

    pub fn set_progress_handler(&mut self, progress_handler: ProgressHandler) {
        self.progress_handler = Some(progress_handler);
    }

    /// Enables the stderr progress chatter (`diagnose` + merge lines).
    /// Off by default: the CLI turns it on for `--verbose` only, so
    /// default runs keep progress percentages without the dump.
    pub fn set_verbose_dump(&mut self, verbose: bool) {
        self.verbose_dump = verbose;
    }

    /// Per connection of [`Self::extracted_connections`]: 0 untouched, 1 on
    /// a triangle whose uv was repaired, 2 added by `hold_singular_lines`.
    #[must_use]
    pub fn extracted_connection_moved(&self) -> &[u8] {
        &self.extracted_connection_moved
    }

    fn report(&self, fraction: f32, name: &str) {
        if let Some(handler) = &self.progress_handler {
            handler(fraction, name);
        }
    }

    fn diagnose(&self, build: impl FnOnce() -> String) {
        // Every std::cerr diagnostic in extract() and its helpers prints
        // only with a progress subscriber (never on quiet runs) AND the
        // verbose flag (never on default runs): the CLI keeps progress
        // percentages in default mode but gates the dump behind
        // `--verbose`. Failures propagate via return values. The closure
        // keeps it zero-cost when unset, like the C++ `if`.
        if self.verbose_dump && self.progress_handler.is_some() {
            eprint!("{}", build());
        }
    }
}
