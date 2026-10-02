//! Selectable patch-layout back end (`--backend patch`).
//!
//! Clean-room quad patch layout in the QuadWild (Pietroni et al. 2021)
//! + Bi-MDF quantization (Heistermann et al. 2023) family shape,
//! written from the papers' architecture with no reference-code reuse.
//! It keeps the current front end exactly (weld, decimation, isotropic
//! remesh, cross field, singularity simplification, guides, density,
//! symmetry) and replaces only the back end: instead of the
//! least-squares seamless cover + one-shot rounding + isoline tracing
//! (which folds globally on cases like beast@1000-native, see
//! `docs/beast-knife-edge-bisection.md`), it
//!
//! 1. traces separatrix arcs from the cross-field singularities along
//!    mesh edges (motorcycle graph, [`trace`]) — every arc lies on
//!    vertices, so crossings are exact and no T-junctions form;
//! 2. partitions the working mesh into patches bounded by those arcs
//!    ([`layout`]);
//! 3. quantizes each shared arc once to an integer edge count and fills
//!    every patch with Coons grids that meet exactly on the shared
//!    sides ([`fill`]), projecting grid vertices onto their own
//!    patch's triangles.
//!
//! Coverage is structural, not retried: every working face belongs to
//! exactly one patch, and every patch emits faces covering its own
//! triangles (unfillable patches take a per-face subdivision), so no
//! region can silently vanish the way a folded uv region can. There is
//! no coverage retry in this back end by design.
//!
//! ## v1 scope and known limits
//!
//! - The front end runs through an internal default-engine pass at a
//!   finer working resolution (`RETOPO_PATCH_WORKING_MULT`, default 4)
//!   whose extraction output is discarded; a future frontend-only hook
//!   in `auto_remesher.rs` would halve the runtime.
//! - Sizing is isotropic (adaptivity + density); anisotropic elongation
//!   and dipole field surgery are future work (density masks still
//!   modulate the resample and the sizing).
//! - Joint loops and owner strokes as forced patch boundaries
//!   (`docs/direction.md`) are not wired yet; patches come from the
//!   automatic tracing only.
//! - Opposite-side conflicts (rare) fall back to diagonal triangle
//!   pairs, and unfillable patches to per-face subdivision; both are
//!   counted in the phase report.
//! - Islands run sequentially (the default back end threads them).

mod field;
mod fill;
mod layout;
mod trace;

use crate::auto_remesher::AutoRemesher;
use crate::auto_remesher::AutoRemesherProgressHandler;
use crate::auto_remesher::CoverageReport;
use crate::auto_remesher::ModelType;
use crate::density::Density;
use crate::mesh_separator::MeshSeparator;
use crate::quad_parameterizer::DipoleConfig;
use crate::surface_mesh::SurfaceMesh;
use crate::symmetry::Symmetry;
use crate::symmetry::SymmetryPlane;
use crate::vector2::Vector2;
use crate::vector3::Vector3;
use std::collections::BTreeMap;
use std::ffi::c_void;
use trace::NodeKind;

/// Env-tunable working-mesh refinement multiple over the requested
/// target (default 4): tracing and projection run on a finer mesh
/// than the quad budget, so patch boundaries and the projection
/// surface resolve below one quad width.
fn working_mult() -> usize {
    std::env::var("RETOPO_PATCH_WORKING_MULT")
        .ok()
        .and_then(|text| text.parse::<usize>().ok())
        .filter(|&value| value >= 1)
        .unwrap_or(4)
}

/// Mirror of the default back end's symmetry acceptance bar
/// (`MIN_SYMMETRY_SCORE` in `auto_remesher.rs`, private there).
const MIN_SYMMETRY_SCORE: f64 = 0.75;

/// Selectable patch-layout remesher.
///
/// Settings mirror the [`AutoRemesher`] surface used by the CLI; see
/// the module docs for v1 scope.
pub struct PatchRemesher {
    vertices: Vec<Vector3>,
    triangles: Vec<Vec<usize>>,
    remeshed_vertices: Vec<Vector3>,
    remeshed_quads: Vec<Vec<usize>>,
    remeshed_vertex_uvs: Vec<Vector2>,
    compute_remeshed_uvs: bool,
    phase_report: Vec<String>,
    island_output_quad_counts: Vec<usize>,
    target_triangle_count: usize,
    scaling: f64,
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
    tag: *mut c_void,
    progress_handler: Option<AutoRemesherProgressHandler>,
}

// SAFETY: same contract as `AutoRemesher` — the tag is an opaque
// caller pointer only dereferenced by the progress handler during
// `remesh()`.
unsafe impl Send for PatchRemesher {}
unsafe impl Sync for PatchRemesher {}

impl PatchRemesher {
    /// Copies its inputs like [`AutoRemesher::new`].
    #[must_use]
    pub fn new(vertices: &[Vector3], triangles: &[Vec<usize>]) -> Self {
        Self {
            vertices: vertices.to_vec(),
            triangles: triangles.to_vec(),
            remeshed_vertices: Vec::new(),
            remeshed_quads: Vec::new(),
            remeshed_vertex_uvs: Vec::new(),
            compute_remeshed_uvs: false,
            phase_report: Vec::new(),
            island_output_quad_counts: Vec::new(),
            target_triangle_count: 0,
            scaling: 0.0,
            adaptivity: 1.0,
            anisotropy: 1.0,
            sharp_edge_degrees: AutoRemesher::DEFAULT_SHARP_EDGE_DEGREES,
            smooth_normal_degrees: 0.0,
            model_type: ModelType::Organic,
            symmetry_enabled: false,
            symmetry_axis: -1,
            symmetry_plane: SymmetryPlane::default(),
            guide_polylines: Vec::new(),
            sharp_polylines: Vec::new(),
            density_multipliers: Vec::new(),
            quiet: false,
            tag: std::ptr::null_mut(),
            progress_handler: None,
        }
    }

    /// Mirrors `setTargetTriangleCount` (twice the target quads).
    pub fn set_target_triangle_count(&mut self, target_triangle_count: usize) {
        self.target_triangle_count = target_triangle_count;
    }

    /// Mirrors `setScaling` (edge scaling; <= 0 means 1.0).
    pub fn set_scaling(&mut self, scaling: f64) {
        self.scaling = scaling;
    }

    /// Mirrors `setProgressHandler`.
    pub fn set_progress_handler(&mut self, progress_handler: Option<AutoRemesherProgressHandler>) {
        self.progress_handler = progress_handler;
    }

    /// Mirrors `setTag`.
    pub fn set_tag(&mut self, tag: *mut c_void) {
        self.tag = tag;
    }

    /// Mirrors `setQuiet`.
    pub fn set_quiet(&mut self, quiet: bool) {
        self.quiet = quiet;
    }

    /// Mirrors `setModelType` (flows into the frontend pass).
    pub fn set_model_type(&mut self, model_type: ModelType) {
        self.model_type = model_type;
    }

    /// Mirrors `setGradientAdaptivity` (drives the sizing field).
    pub fn set_gradient_adaptivity(&mut self, adaptivity: f64) {
        self.adaptivity = adaptivity;
    }

    /// Mirrors `setAnisotropy`. v1 holds it for a future elongation
    /// pass: patch sizing is isotropic (stored, not yet applied).
    pub fn set_anisotropy(&mut self, anisotropy: f64) {
        self.anisotropy = anisotropy;
    }

    /// Mirrors `setSharpEdgeDegrees`.
    pub fn set_sharp_edge_degrees(&mut self, degrees: f64) {
        self.sharp_edge_degrees = degrees;
    }

    /// Mirrors `setSmoothNormalDegrees` (flows into the frontend pass).
    pub fn set_smooth_normal_degrees(&mut self, degrees: f64) {
        self.smooth_normal_degrees = degrees;
    }

    /// Mirrors `setSymmetryEnabled`.
    pub fn set_symmetry_enabled(&mut self, enabled: bool) {
        self.symmetry_enabled = enabled;
    }

    /// Mirrors `setSymmetryPlane` (axis selector).
    pub fn set_symmetry_plane(&mut self, axis: i32) {
        self.symmetry_axis = axis;
    }

    /// Mirrors `setDensityMultipliers`.
    pub fn set_density_multipliers(&mut self, multipliers: &[f64]) {
        self.density_multipliers = multipliers.to_vec();
    }

    /// Mirrors `setGuidePolylines`.
    pub fn set_guide_polylines(&mut self, guides: Vec<Vec<Vector3>>) {
        self.guide_polylines = guides;
    }

    /// Mirrors `setSharpPolylines`.
    pub fn set_sharp_polylines(&mut self, sharps: Vec<Vec<Vector3>>) {
        self.sharp_polylines = sharps;
    }

    /// Mirrors `set_compute_remeshed_uvs`.
    pub fn set_compute_remeshed_uvs(&mut self, compute: bool) {
        self.compute_remeshed_uvs = compute;
    }

    /// Mirrors `remeshedVertices`.
    pub fn remeshed_vertices(&self) -> &[Vector3] {
        &self.remeshed_vertices
    }

    /// Mirrors `remeshedQuads` (length-4 faces, plus length-3 residual
    /// triangles from odd patches and conflict diagonals).
    pub fn remeshed_quads(&self) -> &[Vec<usize>] {
        &self.remeshed_quads
    }

    /// Mirrors `remeshed_vertex_uvs` (patch-grid atlas, one cell per
    /// patch, when enabled).
    pub fn remeshed_vertex_uvs(&self) -> &[Vector2] {
        &self.remeshed_vertex_uvs
    }

    /// Mirrors `phaseReport`.
    pub fn phase_report(&self) -> &[String] {
        &self.phase_report
    }

    /// Mirrors `island_output_quad_counts` (per input island).
    pub fn island_output_quad_counts(&self) -> &[usize] {
        &self.island_output_quad_counts
    }

    /// Coverage reports: always empty — this back end covers by
    /// construction and never retries.
    pub fn coverage_reports(&self) -> &[CoverageReport] {
        &[]
    }

    /// Mirrors `symmetry_plane_axis`.
    pub fn symmetry_plane_axis(&self) -> i32 {
        self.symmetry_plane.axis
    }

    /// Mirrors `symmetry_plane_offset`.
    pub fn symmetry_plane_offset(&self) -> f64 {
        self.symmetry_plane.offset
    }

    /// Mirrors `symmetry_plane_score`.
    pub fn symmetry_plane_score(&self) -> f64 {
        self.symmetry_plane.score
    }

    fn report(&self, progress: f32, status: &str) {
        if let Some(handler) = self.progress_handler {
            handler(self.tag, progress, status);
        }
    }

    /// Runs the patch back end: frontend pass, then per-island field +
    /// trace + layout + fill. Returns `false` (with no outputs
    /// populated) when the input is rejected or the frontend fails.
    pub fn remesh(&mut self) -> bool {
        self.phase_report.clear();
        self.island_output_quad_counts.clear();
        self.remeshed_vertices.clear();
        self.remeshed_quads.clear();
        self.remeshed_vertex_uvs.clear();
        if self.vertices.is_empty() || self.triangles.is_empty() || self.target_triangle_count == 0
        {
            self.report(1.0, "invalid remesh input");
            return false;
        }
        for face in &self.triangles {
            if face.len() != 3 || face.iter().any(|&c| c >= self.vertices.len()) {
                self.report(1.0, "invalid remesh input");
                return false;
            }
        }
        // Symmetry plane: resolved once on the whole input, exactly
        // like the default back end.
        self.symmetry_plane = SymmetryPlane::default();
        if self.symmetry_enabled {
            if self.symmetry_axis >= 0 && self.symmetry_axis < 3 {
                self.symmetry_plane = Symmetry::fixed_plane(&self.vertices, self.symmetry_axis);
            } else {
                self.symmetry_plane = Symmetry::detect_plane(&self.vertices);
            }
            if !self.symmetry_plane.valid() || self.symmetry_plane.score < MIN_SYMMETRY_SCORE {
                self.symmetry_plane = SymmetryPlane::default();
            }
        }

        // Frontend: the default engine's own decimation + resample at a
        // finer working resolution. Symmetry stays off inside (the
        // working mesh is symmetry-independent; the plane above feeds
        // the patch field stage directly), dipoles stay off (their
        // flips feed the discarded default field), and progress stays
        // quiet (this back end reports its own coarse stages).
        self.report(0.0, "Patch backend: frontend resample");
        let mut frontend = AutoRemesher::new(&self.vertices, &self.triangles);
        frontend.set_target_triangle_count(
            self.target_triangle_count.saturating_mul(working_mult()),
        );
        frontend.set_scaling(self.scaling);
        frontend.set_model_type(self.model_type);
        frontend.set_gradient_adaptivity(self.adaptivity);
        frontend.set_anisotropy(self.anisotropy);
        frontend.set_sharp_edge_degrees(self.sharp_edge_degrees);
        frontend.set_smooth_normal_degrees(self.smooth_normal_degrees);
        frontend.set_symmetry_enabled(false);
        frontend.set_density_multipliers(&self.density_multipliers);
        frontend.set_guide_polylines(self.guide_polylines.clone());
        frontend.set_sharp_polylines(self.sharp_polylines.clone());
        frontend.set_dipoles(DipoleConfig::off());
        frontend.set_compute_remeshed_uvs(false);
        frontend.set_quiet(true);
        if !frontend.remesh() {
            self.report(1.0, "frontend produced no result");
            return false;
        }
        let working_vertices = frontend.isotropic_vertices().to_vec();
        let working_triangles = frontend.isotropic_triangles().to_vec();
        if working_vertices.is_empty() || working_triangles.is_empty() {
            self.report(1.0, "frontend produced no result");
            return false;
        }

        // Input islands (for density sources and output attribution).
        let mut input_islands: Vec<Vec<Vec<usize>>> = Vec::new();
        MeshSeparator::split_to_islands(&self.triangles, &mut input_islands);
        let mut input_island_of_vertex = vec![usize::MAX; self.vertices.len()];
        let mut input_island_verts: Vec<Vec<Vector3>> = Vec::new();
        let mut input_island_masks: Vec<Vec<f64>> = Vec::new();
        for island in &input_islands {
            let id = input_island_verts.len();
            let mut verts: Vec<Vector3> = Vec::new();
            let mut mask: Vec<f64> = Vec::new();
            let mut seen = BTreeMap::new();
            for face in island {
                for &corner in face.iter().take(3) {
                    input_island_of_vertex[corner] = id;
                    if seen.insert(corner, true).is_none() {
                        verts.push(self.vertices[corner]);
                        if self.density_multipliers.len() == self.vertices.len() {
                            mask.push(self.density_multipliers[corner]);
                        }
                    }
                }
            }
            if mask.len() != verts.len() {
                mask.clear();
            } else {
                mask = Density::normalize_field(&mask);
            }
            input_island_verts.push(verts);
            input_island_masks.push(mask);
        }
        // Working islands (compacted) mapped to input islands by
        // nearest-input-vertex majority vote.
        let mut working_islands: Vec<Vec<Vec<usize>>> = Vec::new();
        MeshSeparator::split_to_islands(&working_triangles, &mut working_islands);
        let island_ids: Vec<f64> = input_island_of_vertex
            .iter()
            .map(|&id| if id == usize::MAX { -1.0 } else { id as f64 })
            .collect();
        let working_votes =
            Density::resample_nearest(&self.vertices, &island_ids, &working_vertices);

        let total_area: f64 = working_triangles
            .iter()
            .map(|tri| {
                Vector3::area(
                    &working_vertices[tri[0]],
                    &working_vertices[tri[1]],
                    &working_vertices[tri[2]],
                )
            })
            .sum();
        let target_quads = (self.target_triangle_count / 2).max(1) as f64;
        let scaling = if self.scaling > 0.0 { self.scaling } else { 1.0 };
        let edge_scale = if total_area > 0.0 {
            scaling * (total_area / target_quads).sqrt()
        } else {
            1.0
        };

        let mut island_quad_counts = vec![0usize; input_islands.len()];
        let mut uv_global_patch: Vec<usize> = Vec::new();
        let mut merged_patch_base = 0usize;
        let mut total_singularities = 0usize;
        let mut total_crossings = 0usize;
        let mut total_dead_ends = 0usize;
        let mut total_patches = 0usize;
        let mut total_conflicts = 0usize;
        let mut total_triangles = 0usize;
        let mut total_clamped = 0usize;
        let mut total_fallback_faces = 0usize;
        let island_total = working_islands.len().max(1);
        for (working_index, island) in working_islands.iter().enumerate() {
            self.report(
                0.5 + 0.5 * working_index as f32 / island_total as f32,
                "Patch backend: patch fill",
            );
            // Compact the working island.
            let mut old_to_new = BTreeMap::new();
            let mut verts: Vec<Vector3> = Vec::new();
            let mut tris: Vec<Vec<usize>> = Vec::new();
            let mut votes: BTreeMap<usize, usize> = BTreeMap::new();
            for face in island {
                let mut tri = Vec::with_capacity(3);
                for &corner in face.iter().take(3) {
                    let next = verts.len();
                    let new_index = *old_to_new.entry(corner).or_insert(next);
                    if new_index == next {
                        verts.push(working_vertices[corner]);
                        let vote = working_votes[corner] as usize;
                        if working_votes[corner] >= 0.0 {
                            *votes.entry(vote).or_insert(0) += 1;
                        }
                    }
                    tri.push(new_index);
                }
                tris.push(tri);
            }
            let input_island = votes
                .iter()
                .max_by(|a, b| a.1.cmp(b.1).then(a.0.cmp(b.0).reverse()))
                .map(|(&id, _)| id)
                .unwrap_or(0)
                .min(input_islands.len().saturating_sub(1));
            let density = if input_island < input_island_masks.len()
                && !input_island_masks[input_island].is_empty()
            {
                Density::normalize_field(&Density::resample_nearest(
                    &input_island_verts[input_island],
                    &input_island_masks[input_island],
                    &verts,
                ))
            } else {
                Vec::new()
            };
            let Some(field) = field::compute_field(
                &verts,
                &tris,
                self.sharp_edge_degrees,
                &self.guide_polylines,
                &self.sharp_polylines,
                &self.symmetry_plane,
                self.adaptivity,
                &density,
                edge_scale,
            ) else {
                continue;
            };
            total_singularities += field.singularities.len();
            let topology = SurfaceMesh::new(&verts, &tris);
            let graph = trace::trace_separatrices(&topology, &verts, &field.field, &field.charges);
            for node in &graph.nodes {
                match node.kind {
                    NodeKind::Crossing => total_crossings += 1,
                    NodeKind::DeadEnd => total_dead_ends += 1,
                    _ => {}
                }
            }
            let layout = layout::build_layout(&topology, &graph);
            let degenerate_area = (1e-4 * edge_scale).powi(2);
            let filled = fill::fill_layout(
                &topology,
                &verts,
                &layout,
                &field.face_width,
                degenerate_area,
            );
            total_patches += filled.stats.patches;
            total_conflicts += filled.stats.conflicted_quads;
            total_triangles += filled.stats.triangles;
            total_clamped += filled.stats.clamped_arcs;
            total_fallback_faces += filled.stats.fallback_faces;
            let vertex_base = self.remeshed_vertices.len();
            self.remeshed_vertices.extend(filled.verts.iter().cloned());
            let mut quads_here = 0usize;
            for face in &filled.faces {
                let mut offset = Vec::with_capacity(face.len());
                for &corner in face {
                    offset.push(vertex_base + corner);
                }
                if face.len() == 4 {
                    quads_here += 1;
                }
                self.remeshed_quads.push(offset);
            }
            island_quad_counts[input_island] += quads_here;
            if self.compute_remeshed_uvs {
                for (index, _) in filled.verts.iter().enumerate() {
                    uv_global_patch.push(merged_patch_base + filled.uv_patch[index]);
                    let (u, v) = filled.uv_local[index];
                    self.remeshed_vertex_uvs.push(Vector2::new(u, v));
                }
            }
            merged_patch_base += filled.stats.patches;
        }
        if self.compute_remeshed_uvs {
            // One atlas cell per patch over the unit square.
            let cells = (merged_patch_base as f64).sqrt().ceil().max(1.0) as usize;
            for (index, uv) in self.remeshed_vertex_uvs.iter_mut().enumerate() {
                let patch = uv_global_patch[index].min(merged_patch_base.saturating_sub(1));
                let column = patch % cells;
                let row = patch / cells;
                let u = (column as f64 + uv.x().clamp(0.0, 1.0)) / cells as f64;
                let v = (row as f64 + uv.y().clamp(0.0, 1.0)) / cells as f64;
                *uv = Vector2::new(u, v);
            }
        }
        self.island_output_quad_counts = island_quad_counts;
        if self.symmetry_plane.valid() && !self.remeshed_vertices.is_empty() {
            Symmetry::symmetrize_vertices(&mut self.remeshed_vertices, &self.symmetry_plane);
        }
        self.phase_report.push(format!(
            "Islands: {}, input triangles: {}",
            input_islands.len(),
            self.triangles.len()
        ));
        self.phase_report.push(format!(
            "Patch backend: {} working islands, {} singularities, {} crossings, {} dead ends, {} patches, {} conflicts, {} triangles, {} clamped, {} fallback faces",
            working_islands.len(),
            total_singularities,
            total_crossings,
            total_dead_ends,
            total_patches,
            total_conflicts,
            total_triangles,
            total_clamped,
            total_fallback_faces
        ));
        self.report(1.0, "Patch backend: done");
        !self.remeshed_quads.is_empty()
    }
}
