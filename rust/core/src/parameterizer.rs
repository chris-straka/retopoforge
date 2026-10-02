//! Port of `core/parameterizer.*`: curvature-adaptive scaling field,
//! frame-field solve (+ symmetry + singularity cleanup), anisotropic
//! scaling, and the quad-cover solve that produces per-triangle UVs.
//!
//! Line-by-line mirror of the C++ implementation: same functions in the
//! same order, same thresholds, same solver call sequence. Deliberate
//! restructures, each noted at the site:
//! - the TBB data-parallel loops run sequentially (every one is a disjoint
//!   per-element write, so the C++ is already deterministic and the op order
//!   per element is unchanged);
//! - `computeConformalScaling` is omitted: its only call site is commented
//!   out in the C++, the function is eliminated from the Release binary,
//!   and nothing observes it (quad precedent: unobservable code omitted
//!   with a comment);
//! - the `AUTO_REMESHER_DEV` debug-obj dump is omitted (the macro is never
//!   defined by the build, so the C++ compiles it out too);
//! - the stderr diagnostics (`Topology rejected...`, `...solve failed`,
//!   the singularity summary) are omitted: library code reports status
//!   through the `bool` return, like the quad/frame mirrors;
//! - the progress-remap closure owns an `Arc` clone of the caller's
//!   handler instead of rebundling the `'static` box (which cannot capture
//!   the caller's handler reference); the fractions are bitwise-identical;
//! - the `report` lambda is a free function (a closure would borrow `self`
//!   across the output stores at the end of `parameterize`);
//! - `SingularitySimplifier` was vendored during the concurrent port and
//!   rewired to the joined `singularity_simplifier` at join (the vendored
//!   copy lacked the backend-fused `sincos` rebuild fix).
//!
//! FMA audit (Release IR, `llvm.fmuladd`, brew Clang, ARM64, project flags
//! `-O3 -funroll-loops`): 45 fused calls in the TU, of which all but 12
//! sit inside inlined `Vector3` methods already replicated by the FMA-exact
//! `vector2`/`vector3` ports. The 12 owned fusions, each mirrored with
//! explicit [`f64::mul_add`] (and [`f32::mul_add`]) below:
//! - `quadraticForm`: five per evaluation — the two line-198 adds fuse
//!   their LHS products (`fma(T0*tx, tx, T2*ty*ty)`,
//!   `fma(T5*tz, tz, s1)`), the two inner line-199 adds likewise
//!   (`fma(T1*tx, ty, T3*tx*tz)`, `fma(T4*ty, tz, s3)`), and the outer add
//!   fuses the `2.0` scaling (`fma(inner, 2.0, s2)`);
//! - the tensor smoothing line fuses as `fma(tensor, 0.5, (0.5*avg)/n)`;
//! - the f32 cover-progress remap fuses as
//!   `fma(fraction, 0.99-0.28, 0.28)`.
//!
//! Notably unfused (plain operators, verified absent from the IR): the
//! tensor accumulation, the neighbor averaging, the face-tensor gather,
//! the scaling-field curvature loop, and every `pow`/`sqrt`/`acos` site.
//! `computeConformalScaling` is absent from the IR entirely (dead).
//!
//! Vendored `SingularitySimplifier` (below, private): the sibling
//! rs-singularitysimplifier lane runs concurrently, so the exact queries
//! this module calls are mirrored privately with 1:1 names
//! (`SingularitySimplifier::new`, `set_maximum_pair_distance`,
//! `set_sharp_edge_degrees`, `simplify`, plus the private helpers they
//! need). The coordinator deletes the vendored section and rewires this
//! module to `crate::singularity_simplifier` at join. The vendored
//! smoother replicates the IR-verified fusions (`fma(-jump, PI/2, T)`,
//! `fma(w, X, sum)`, verified in this lane's own IR dump) and the
//! explicit-NaN `maxChange` comparison.

use crate::density::Density;
use crate::frame_field::FrameField;
use crate::guides::Guides;
use crate::progress::ProgressHandler;
use crate::quad_parameterizer::{DipoleConfig, QuadParameterizer};
use crate::singularity_simplifier::SingularitySimplifier;
use crate::surface_mesh::SurfaceMesh;
use crate::symmetry::{Symmetry, SymmetryPlane};
use crate::vector2::Vector2;
use crate::vector3::Vector3;
use std::sync::Arc;

fn report_progress(handler: Option<&Arc<ProgressHandler>>, fraction: f32, name: &str) {
    if let Some(h) = handler {
        h(fraction, name);
    }
}

fn snap_polylines_to_mesh(mesh: &SurfaceMesh, input: &[Vec<Vector3>]) -> Vec<Vec<Vector3>> {
    let mut snapped: Vec<Vec<Vector3>> = Vec::with_capacity(input.len());
    if mesh.vertex_count() == 0 {
        return snapped;
    }
    let radius = Guides::influence_radius(mesh);
    let radius_squared = radius * radius;
    for polyline in input {
        if polyline.len() < 2 {
            continue;
        }
        let mut out = Vec::with_capacity(polyline.len());
        for point in polyline {
            let mut best = 0;
            let mut best_distance_squared = (*mesh.position(0) - *point).length_squared();
            for v in 1..mesh.vertex_count() {
                let d2 = (*mesh.position(v) - *point).length_squared();
                if d2 < best_distance_squared {
                    best_distance_squared = d2;
                    best = v;
                }
            }
            if best_distance_squared > radius_squared {
                continue;
            }
            out.push(*mesh.position(best));
        }
        if out.len() >= 2 {
            snapped.push(out);
        }
    }
    snapped
}

// `computeConformalScaling` intentionally omitted: its only call site is
// commented out in parameterizer.cpp, the function is eliminated from the
// Release binary (verified absent from the IR), and nothing observes it.

/// Mirrors the anonymous-namespace `quadraticForm` (`T` is the gathered
/// face tensor as `[xx, xy, yy, xz, yz, zz]`).
fn quadratic_form(t: &[f64; 6], v: &Vector3) -> f64 {
    let (tx, ty, tz) = (v.x(), v.y(), v.z());
    // FMA audit: each left-nested add fuses its LHS product (verified
    // operand-by-operand in the Release IR, two inlined instances with
    // identical shapes). The transcription keeps the IR's exact operand
    // order (`ty * (t2 * ty)`); fmul commutes exactly either way.
    let b1 = ty * (t[2] * ty);
    let s1 = (t[0] * tx).mul_add(tx, b1);
    let s2 = (t[5] * tz).mul_add(tz, s1);
    let b2 = tz * (t[3] * tx);
    let s3 = (t[1] * tx).mul_add(ty, b2);
    let s4 = (t[4] * ty).mul_add(tz, s3);
    s4.mul_add(2.0, s2)
}

#[allow(clippy::too_many_arguments)]
fn compute_face_anisotropy_field(
    mesh: &SurfaceMesh,
    field: &[Vector3],
    anisotropy: f64,
    max_aspect_ratio: f64,
    scaling_u: &mut Vec<f64>,
    scaling_v: &mut Vec<f64>,
) {
    // `assign` semantics (clear + refill).
    scaling_u.clear();
    scaling_u.resize(mesh.face_count(), 1.0);
    scaling_v.clear();
    scaling_v.resize(mesh.face_count(), 1.0);
    if anisotropy <= 0.0 || max_aspect_ratio <= 1.0 || field.len() != mesh.face_count() {
        return;
    }
    let mut tensor = vec![0.0; mesh.vertex_count() * 6];
    let mut neighbors: Vec<Vec<usize>> = vec![Vec::new(); mesh.vertex_count()];
    for c in 0..mesh.corner_count() {
        let mate = mesh.opposite_corner(c);
        if mate == SurfaceMesh::NPOS || mate < c {
            continue;
        }
        let edge = mesh.edge_vector(c);
        let length = edge.length();
        if length <= 0.0 {
            continue;
        }
        let d = edge / length;
        let weight = length * mesh.normal_angle(c);
        // FMA audit: chained multiplies, no add — unfused in the IR.
        let contribution = [
            weight * d.x() * d.x(),
            weight * d.x() * d.y(),
            weight * d.y() * d.y(),
            weight * d.x() * d.z(),
            weight * d.y() * d.z(),
            weight * d.z() * d.z(),
        ];
        let a = mesh.corner_vertex(c);
        let b = mesh.corner_vertex(mesh.next_corner(c));
        for i in 0..6 {
            tensor[6 * a + i] += contribution[i];
            tensor[6 * b + i] += contribution[i];
        }
    }
    for c in 0..mesh.corner_count() {
        let a = mesh.corner_vertex(c);
        let b = mesh.corner_vertex(mesh.next_corner(c));
        neighbors[a].push(b);
        neighbors[b].push(a);
    }
    let mut smoothed = vec![0.0; tensor.len()];
    for _ in 0..12 {
        for v in 0..mesh.vertex_count() {
            if neighbors[v].is_empty() {
                for i in 0..6 {
                    smoothed[6 * v + i] = tensor[6 * v + i];
                }
                continue;
            }
            for i in 0..6 {
                let mut average = 0.0;
                for &n in &neighbors[v] {
                    average += tensor[6 * n + i];
                }
                // FMA audit: the C++ fuses this line (6x unrolled) as
                // `fma(tensor, 0.5, (0.5*average)/n)`.
                smoothed[6 * v + i] =
                    tensor[6 * v + i].mul_add(0.5, 0.5 * average / neighbors[v].len() as f64);
            }
        }
        std::mem::swap(&mut tensor, &mut smoothed);
    }
    let mut along_u = vec![0.0; mesh.face_count()];
    let mut along_v = vec![0.0; mesh.face_count()];
    let mut total = 0.0;
    for f in 0..mesh.face_count() {
        let mut t = [0.0; 6];
        for l in 0..3 {
            let v = mesh.corner_vertex(3 * f + l);
            for i in 0..6 {
                t[i] += tensor[6 * v + i];
            }
        }
        let n = mesh.face_normal(f);
        let b = field[f].normalized();
        let bt = Vector3::cross_product(&n, &b);
        along_u[f] = quadratic_form(&t, &bt).abs();
        along_v[f] = quadratic_form(&t, &b).abs();
        total += along_u[f] + along_v[f];
    }
    if total <= 0.0 {
        return;
    }
    let floor = 0.001 * total / (2.0 * mesh.face_count() as f64);
    let max_rho = max_aspect_ratio.sqrt();
    for f in 0..mesh.face_count() {
        let mut rho = ((along_v[f] + floor) / (along_u[f] + floor)).powf(0.25);
        rho = rho.powf(anisotropy);
        rho = cxx_max(1.0 / max_rho, cxx_min(max_rho, rho));
        scaling_u[f] = rho;
        scaling_v[f] = 1.0 / rho;
    }
}

/// `std::min<double>` semantics: `(b < a) ? b : a` — a NaN `a` stays NaN.
/// (`f64::min` returns the non-NaN operand instead.)
#[inline]
fn cxx_min(a: f64, b: f64) -> f64 {
    if b < a { b } else { a }
}

/// `std::max<double>` semantics: `(a < b) ? b : a` — a NaN `a` stays NaN.
/// (`f64::max` returns the non-NaN operand instead.)
#[inline]
fn cxx_max(a: f64, b: f64) -> f64 {
    if a < b { b } else { a }
}

/// Mixed-integer parameterizer (mirrors `AutoRemesher::Parameterizer`).
///
/// Borrows the input mesh (like the C++ raw pointers; the caller keeps the
/// slices alive through [`Self::parameterize`]) and owns the settings and
/// outputs. Nullable C++ pointers are `Option`s; the `bool` return mirrors
/// the C++ success/failure exactly.
pub struct Parameterizer<'a> {
    vertices: &'a [Vector3],
    triangles: &'a [Vec<usize>],
    triangle_field_vectors: Option<&'a [Vector3]>,
    triangle_uvs: Option<Vec<Vec<Vector2>>>,
    singular_vertex_positions: Vec<Vector3>,
    singular_vertex_indices: Vec<usize>,
    original_triangle_uvs: Vec<Vec<Vector2>>,
    scaling: f64,
    adaptivity: f64,
    sharp_edge_degrees: f64,
    anisotropy: f64,
    max_aspect_ratio: f64,
    singularity_simplification: bool,
    maximum_singularity_pair_distance: usize,
    symmetry_plane: SymmetryPlane,
    guide_polylines: Option<&'a [Vec<Vector3>]>,
    sharp_polylines: Option<&'a [Vec<Vector3>]>,
    density_field: Vec<f64>,
    // Dipole insertion has no C++ counterpart: the engine-leaf default
    // is off (differential oracles pin the no-dipole path without
    // touching this); AutoRemesher owns the product default and always
    // sets it before parameterize().
    dipoles: DipoleConfig,
    // Dipole flips applied by the last parameterize() (0 on failure).
    dipole_flips: usize,
    // Restructure: the C++ owns a `std::function` here; the wrap is an
    // `Arc` so the cover-progress remap below can own a clone (the
    // `'static` box cannot capture a borrow of this field). Unobservable:
    // every report call still reaches the same handler with the same
    // bitwise fractions.
    progress_handler: Option<Arc<ProgressHandler>>,
}

impl<'a> Parameterizer<'a> {
    /// Mirrors the C++ constructor (same defaults: scaling 1.0, adaptivity
    /// 0.5, sharp 90 degrees, anisotropy 1.0, simplification on, pair
    /// distance 6, no symmetry plane, no guides/sharps/density/progress).
    pub fn new(
        vertices: &'a [Vector3],
        triangles: &'a [Vec<usize>],
        triangle_field_vectors: Option<&'a [Vector3]>,
    ) -> Self {
        Self {
            vertices,
            triangles,
            triangle_field_vectors,
            triangle_uvs: None,
            singular_vertex_positions: Vec::new(),
            singular_vertex_indices: Vec::new(),
            original_triangle_uvs: Vec::new(),
            scaling: 1.0,
            adaptivity: 0.5,
            sharp_edge_degrees: 90.0,
            anisotropy: 1.0,
            max_aspect_ratio: 2.3,
            singularity_simplification: true,
            maximum_singularity_pair_distance: 6,
            symmetry_plane: SymmetryPlane::default(),
            guide_polylines: None,
            sharp_polylines: None,
            density_field: Vec::new(),
            dipoles: DipoleConfig::off(),
            dipole_flips: 0,
            progress_handler: None,
        }
    }

    /// Moves the solved UVs out (mirrors `takeTriangleUvs`, which moves the
    /// `unique_ptr` out; a second call returns `None` like the C++ null).
    pub fn take_triangle_uvs(&mut self) -> Option<Vec<Vec<Vector2>>> {
        self.triangle_uvs.take()
    }

    /// Mirrors `originalTriangleUvs` (kept for the extractor; identical to
    /// the taken UVs at solve time).
    pub fn original_triangle_uvs(&self) -> &[Vec<Vector2>] {
        &self.original_triangle_uvs
    }

    /// Mirrors `singularVertexPositions`.
    pub fn singular_vertex_positions(&self) -> &[Vector3] {
        &self.singular_vertex_positions
    }

    /// Mirrors `singularVertexIndices`.
    pub fn singular_vertex_indices(&self) -> &[usize] {
        &self.singular_vertex_indices
    }

    /// Mirrors `setScaling`.
    pub fn set_scaling(&mut self, scaling: f64) {
        self.scaling = scaling;
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

    /// Mirrors `setSingularitySimplification`.
    pub fn set_singularity_simplification(&mut self, simplify: bool) {
        self.singularity_simplification = simplify;
    }

    /// Mirrors `setMaximumSingularityPairDistance`.
    pub fn set_maximum_singularity_pair_distance(&mut self, face_hops: usize) {
        self.maximum_singularity_pair_distance = face_hops;
    }

    /// Mirrors `setSymmetryPlane`.
    pub fn set_symmetry_plane(&mut self, plane: SymmetryPlane) {
        self.symmetry_plane = plane;
    }

    /// Mirrors `setGuidePolylines` (`None` disables the guide pass).
    pub fn set_guide_polylines(&mut self, guides: Option<&'a [Vec<Vector3>]>) {
        self.guide_polylines = guides;
    }

    /// Mirrors `setSharpPolylines` (`None` disables the sharp pass).
    pub fn set_sharp_polylines(&mut self, sharps: Option<&'a [Vec<Vector3>]>) {
        self.sharp_polylines = sharps;
    }

    /// Mirrors `setFeaturePolylines` (the CLI-flag-spelled alias).
    pub fn set_feature_polylines(&mut self, sharps: Option<&'a [Vec<Vector3>]>) {
        self.sharp_polylines = sharps;
    }

    /// Mirrors `setDensityField`.
    pub fn set_density_field(&mut self, field: Vec<f64>) {
        self.density_field = field;
    }

    /// Dipole-insertion config (no C++ counterpart; engine-leaf default
    /// off — see the field comment).
    pub fn set_dipoles(&mut self, config: DipoleConfig) {
        self.dipoles = config;
    }

    /// Dipole flips applied by the last `parameterize` (0 when disabled,
    /// unmasked, mild, or on failure).
    #[must_use]
    pub fn dipole_flips(&self) -> usize {
        self.dipole_flips
    }

    /// Mirrors `setProgressHandler`.
    pub fn set_progress_handler(&mut self, progress_handler: ProgressHandler) {
        self.progress_handler = Some(Arc::new(progress_handler));
    }

    #[allow(clippy::manual_clamp)] // Port mirrors the C++ branch ladder; `clamp` differs on NaN.
    fn compute_face_scaling_field(
        &self,
        vertices: &[Vector3],
        triangles: &[Vec<usize>],
        vertex_normals: &[Vector3],
        face_around_vertex_map: &[Vec<usize>],
    ) -> Vec<f64> {
        let mut face_scaling = vec![1.0; triangles.len()];
        if self.adaptivity <= 0.0 || vertices.is_empty() {
            return face_scaling;
        }
        let mut vertex_curvature = vec![0.0; vertices.len()];
        // Sequential: the C++ TBB loop writes disjoint per-vertex slots.
        for v in 0..vertices.len() {
            let faces_around_vertex = &face_around_vertex_map[v];
            if faces_around_vertex.is_empty() {
                continue;
            }
            let normal_v = vertex_normals[v];
            let mut max_curvature = 0.0;
            for &face_index in faces_around_vertex {
                for &u in &triangles[face_index] {
                    if u == v {
                        continue;
                    }
                    let distance = (vertices[u] - vertices[v]).length();
                    if distance <= 0.0 {
                        continue;
                    }
                    let mut cos_angle = Vector3::dot_product(&normal_v, &vertex_normals[u]);
                    if cos_angle > 1.0 {
                        cos_angle = 1.0;
                    } else if cos_angle < -1.0 {
                        cos_angle = -1.0;
                    }
                    let curvature = cos_angle.acos() / distance;
                    if curvature > max_curvature {
                        max_curvature = curvature;
                    }
                }
            }
            vertex_curvature[v] = max_curvature;
        }
        let mut sum_curvature = 0.0;
        for curvature in &vertex_curvature {
            sum_curvature += *curvature;
        }
        let average_curvature = sum_curvature / vertex_curvature.len() as f64;
        if average_curvature <= 0.0 {
            return face_scaling;
        }
        const MIN_RATIO: f64 = 0.3;
        #[allow(clippy::manual_clamp)] // Port mirrors the C++ branch ladder; `clamp` differs on NaN.
        const MAX_RATIO: f64 = 3.0;
        // Sequential: the C++ TBB loop writes disjoint per-face slots.
        for (i, triangle) in triangles.iter().enumerate() {
            let mut face_curvature = 0.0;
            for &v in triangle {
                face_curvature += vertex_curvature[v];
            }
            face_curvature /= triangle.len() as f64;
            let mut normalized = face_curvature / average_curvature;
            if normalized < 1e-3 {
                normalized = 1e-3;
            }
            let mut multiplier = normalized.powf(-self.adaptivity);
            if multiplier < MIN_RATIO {
                multiplier = MIN_RATIO;
            } else if multiplier > MAX_RATIO {
                multiplier = MAX_RATIO;
            }
            face_scaling[i] = multiplier;
        }
        // Renormalize after clamping: rescale all m so SUM A_f/m_f^2 equals
        // the uniform-field value (total area), preserving the quad budget
        // implied by m_scaling while moving quads from flat regions to
        // detailed ones.
        let mut total_area = 0.0;
        let mut weighted_area = 0.0;
        for (i, triangle) in triangles.iter().enumerate() {
            let mut face_area = 0.0;
            if triangle.len() >= 3 {
                let e0 = vertices[triangle[1]] - vertices[triangle[0]];
                let e1 = vertices[triangle[2]] - vertices[triangle[0]];
                face_area = 0.5 * Vector3::cross_product(&e0, &e1).length();
            }
            total_area += face_area;
            let m = face_scaling[i];
            if m > 0.0 {
                weighted_area += face_area / (m * m);
            }
        }
        if total_area > 0.0 && weighted_area > 0.0 {
            let rescale = (weighted_area / total_area).sqrt();
            if rescale > 0.0 && rescale.is_finite() {
                for m in &mut face_scaling {
                    *m *= rescale;
                }
            }
        }
        face_scaling
    }

    /// Runs the full parameterization (mirrors `Parameterizer::parameterize`;
    /// `false` mirrors every C++ `false` return).
    pub fn parameterize(&mut self) -> bool {
        self.dipole_flips = 0;
        // The AUTO_REMESHER_DEV obj dump is compiled out of the C++ too
        // (the macro is never defined by the build).

        // The fractions below are the measured share of parameterization
        // each step costs; "Solving quad cover" dominates, so the bar has
        // to keep moving through it rather than sitting still until it
        // finishes.
        report_progress(
            self.progress_handler.as_ref(),
            0.0,
            "Computing vertex normals",
        );
        let mut vertex_normals = vec![Vector3::default(); self.vertices.len()];
        {
            let mut face_normals = vec![Vector3::default(); self.triangles.len()];
            // Sequential: the C++ TBB loop writes disjoint per-face slots.
            for (i, triangle) in self.triangles.iter().enumerate() {
                face_normals[i] = Vector3::normal(
                    &self.vertices[triangle[0]],
                    &self.vertices[triangle[1]],
                    &self.vertices[triangle[2]],
                );
            }
            for (i, triangle) in self.triangles.iter().enumerate() {
                vertex_normals[triangle[0]] += face_normals[i];
                vertex_normals[triangle[1]] += face_normals[i];
                vertex_normals[triangle[2]] += face_normals[i];
            }
            // Sequential: the C++ TBB loop writes disjoint per-vertex slots.
            for normal in &mut vertex_normals {
                normal.normalize();
            }
        }

        report_progress(
            self.progress_handler.as_ref(),
            0.01,
            "Computing scaling field",
        );
        let mut face_around_vertex_map: Vec<Vec<usize>> = vec![Vec::new(); self.vertices.len()];
        for (i, triangle) in self.triangles.iter().enumerate() {
            face_around_vertex_map[triangle[0]].push(i);
            face_around_vertex_map[triangle[1]].push(i);
            face_around_vertex_map[triangle[2]].push(i);
        }
        let mut face_scaling_field = self.compute_face_scaling_field(
            self.vertices,
            self.triangles,
            &vertex_normals,
            &face_around_vertex_map,
        );

        // Local density control, default off: an empty field skips
        // everything so the scaling field stays bit-identical to a run
        // without any mask.
        // EXPERIMENTAL SPIKE (lane/dipole-mechanism): hoisted so the raw
        // mask can feed the env-gated dipole insertion downstream.
        let density = if !self.density_field.is_empty() {
            Density::normalize_field(&self.density_field)
        } else {
            Vec::new()
        };
        if !density.is_empty() {
            Density::apply_to_scaling_field(
                self.vertices,
                self.triangles,
                &density,
                &mut face_scaling_field,
            );
        }

        report_progress(
            self.progress_handler.as_ref(),
            0.02,
            "Building surface topology",
        );
        // The parameterization pipeline uses the triangle/corner mesh;
        // no attribute-backed interchange mesh is constructed.
        let topology = SurfaceMesh::new(self.vertices, self.triangles);
        if topology.face_count() != self.triangles.len() {
            return false;
        }

        report_progress(self.progress_handler.as_ref(), 0.03, "Solving frame field");
        // Topology, field, and quad cover form the complete active path.
        // Explicit sharps arrive in input-mesh coordinates; the island mesh
        // was resampled upstream, so snap them onto it before constraining.
        // Null or empty input leaves snapped_sharps empty and every sharp
        // pass below is skipped, keeping the default run bit-identical.
        let mut snapped_sharps: Vec<Vec<Vector3>> = Vec::new();
        if let Some(sharps) = self.sharp_polylines
            && !sharps.is_empty()
        {
            snapped_sharps = snap_polylines_to_mesh(&topology, sharps);
        }
        let mut field: Vec<Vector3>;
        if let Some(triangle_field_vectors) = self.triangle_field_vectors {
            field = triangle_field_vectors.to_vec();
        } else if let Some(solved) = FrameField::create(
            &topology,
            self.sharp_edge_degrees,
            self.guide_polylines.unwrap_or(&[]),
            &snapped_sharps,
        ) {
            field = solved;
        } else {
            return false;
        }
        if field.len() != topology.face_count() {
            return false;
        }

        if self.symmetry_plane.valid() {
            report_progress(
                self.progress_handler.as_ref(),
                0.16,
                "Symmetrizing frame field",
            );
            Symmetry::symmetrize_frame_field(
                self.vertices,
                self.triangles,
                &mut field,
                &self.symmetry_plane,
            );
        }

        // The simplifier only mutates `field`; its before/after counts feed
        // nothing but the C++ stderr summary, so they are not read back
        // (and the borrow ends here, before the cover solve reborrows).
        let mut simplifier = SingularitySimplifier::new(&topology, &mut field);
        if self.singularity_simplification {
            report_progress(
                self.progress_handler.as_ref(),
                0.17,
                "Simplifying singularities",
            );
            simplifier.set_sharp_edge_degrees(self.sharp_edge_degrees);
            simplifier.set_maximum_pair_distance(self.maximum_singularity_pair_distance);
            simplifier.simplify();
        }
        drop(simplifier);

        // Research probe (RETOPO_DUMP_STAGES=dir): stage-2 per-face cross
        // field (post-simplification), parallel to the working triangles.
        // Research probe (kept for item-7 stage analysis); no state touched. The parameterizer
        // has no island index, so dumps are sequence-numbered in
        // completion order (exact for single-island runs; multi-island
        // runs join offline via the face-count header).
        if let Some(dir) = std::env::var_os("RETOPO_DUMP_STAGES") {
            static DUMP_SEQ: std::sync::atomic::AtomicUsize =
                std::sync::atomic::AtomicUsize::new(0);
            let n = DUMP_SEQ.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
            let path = std::path::Path::new(&dir).join(format!("stage2_field_{n}.txt"));
            let mut out = format!("# faces={}\n", field.len());
            for (i, f) in field.iter().enumerate() {
                out.push_str(&format!("{i} {} {} {}\n", f.x(), f.y(), f.z()));
            }
            let _ = std::fs::write(path, out);
        }

        //faceScalingField = computeConformalScaling(...); (commented out in
        // the C++; see the omission note above.)

        let mut face_scaling_u = vec![1.0; self.triangles.len()];
        let mut face_scaling_v = vec![1.0; self.triangles.len()];
        if self.anisotropy > 0.0 {
            report_progress(
                self.progress_handler.as_ref(),
                0.26,
                "Computing anisotropy field",
            );
            compute_face_anisotropy_field(
                &topology,
                &field,
                self.anisotropy,
                self.max_aspect_ratio,
                &mut face_scaling_u,
                &mut face_scaling_v,
            );
        }
        // The cover solve is the longest single step here, so it reports
        // its own sub-steps from 0.28 onwards rather than going quiet until
        // it finishes.
        //
        // Restructure note: the C++ re-wraps the handler in a second
        // `std::function`; the Rust `ProgressHandler` box is `'static` and
        // cannot capture the caller's handler reference, so the remap owns
        // an `Arc` clone of the stored handler instead. Same handler, same
        // bitwise fractions.
        let remap: Option<ProgressHandler> = self.progress_handler.clone().map(|shared| {
            Box::new(move |fraction: f32, name: &str| {
                // FMA audit: the C++ fuses this into one f32 fmuladd.
                shared((0.99f32 - 0.28f32).mul_add(fraction, 0.28f32), name);
            }) as ProgressHandler
        });
        let sharps_arg: Option<&[Vec<Vector3>]> = if snapped_sharps.is_empty() {
            None
        } else {
            Some(&snapped_sharps)
        };
        let cover = QuadParameterizer::parameterize(
            self.vertices,
            self.triangles,
            &field,
            self.scaling,
            self.sharp_edge_degrees,
            &face_scaling_field,
            &face_scaling_u,
            &face_scaling_v,
            remap.as_ref(),
            sharps_arg,
            // Raw mask for the dipole insertion (no-op unless configured).
            &density,
            self.dipoles,
        );
        let cover = match cover {
            Some(cover) => cover,
            None => {
                self.dipole_flips = 0;
                return false;
            }
        };
        self.dipole_flips = cover.dipole_flips;
        report_progress(
            self.progress_handler.as_ref(),
            0.99,
            "Collecting singularities",
        );
        self.original_triangle_uvs = cover.triangle_uvs.clone();
        self.triangle_uvs = Some(cover.triangle_uvs);
        self.singular_vertex_positions.clear();
        self.singular_vertex_indices.clear();
        for &v in &cover.singular_vertices {
            if v >= self.vertices.len() {
                continue;
            }
            self.singular_vertex_positions.push(self.vertices[v]);
            self.singular_vertex_indices.push(v);
        }
        // (The C++ stderr singularity summary is omitted: unobservable
        // through the API; the counts feed nothing else.)
        report_progress(self.progress_handler.as_ref(), 1.0, "");
        true
    }
}
