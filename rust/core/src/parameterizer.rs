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
//! - `SingularitySimplifier` is a vendored private mirror (see below).
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
use crate::quad_parameterizer::QuadParameterizer;
use crate::surface_mesh::SurfaceMesh;
use crate::symmetry::{Symmetry, SymmetryPlane};
use crate::vector2::Vector2;
use crate::vector3::Vector3;
use std::sync::Arc;
use vendored_singularity::SingularitySimplifier;

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

    /// Mirrors `setProgressHandler`.
    pub fn set_progress_handler(&mut self, progress_handler: ProgressHandler) {
        self.progress_handler = Some(Arc::new(progress_handler));
    }

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
        if !self.density_field.is_empty() {
            let density = Density::normalize_field(&self.density_field);
            if !density.is_empty() {
                Density::apply_to_scaling_field(
                    self.vertices,
                    self.triangles,
                    &density,
                    &mut face_scaling_field,
                );
            }
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
        if let Some(sharps) = self.sharp_polylines {
            if !sharps.is_empty() {
                snapped_sharps = snap_polylines_to_mesh(&topology, sharps);
            }
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
                shared(
                    (0.99f32 - 0.28f32).mul_add(fraction, 0.28f32),
                    name,
                );
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
        );
        let cover = match cover {
            Some(cover) => cover,
            None => return false,
        };
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

// =====================================================================
// VENDORED `SingularitySimplifier` mirror — pending coordinator dedup.
//
// The rs-singularitysimplifier lane runs concurrently, so this module
// privately mirrors the exact queries it calls, with 1:1 names per the
// lane contract so the coordinator can delete this section and rewire the
// two call sites (`SingularitySimplifier::new`, the setter/simplify block)
// to `crate::singularity_simplifier` mechanically:
//
// Vendored items (all names match the sibling lane): `unit`, `Frame`,
// `frame_for_face`, `angle_in_frame`, `quarter_turn`, `count_affected`,
// `Candidate`, `SingularitySimplifier::new`,
// `set_maximum_pair_distance`, `set_sharp_edge_degrees`, `simplify`
// (public here for the call sites above; private to the crate), and the
// private helpers `vertex_charges`, `build_frames_and_connection`,
// `update_mismatches`, `facets_around_vertex`, `corner_along_edge`,
// `path_between`, `cancel_pair`. Deliberately NOT vendored: the
// before/after/cancelled count getters (this module never reads them; they
// feed only the omitted stderr summary) and the sibling's TEMP-DEBUG.
//
// Transcription notes (all verified in this lane's own Release IR dump
// of singularitysimplifier.cpp): the smoother fuses two fmuladds per
// corner (`fma(-jump, PI/2, T)`, `fma(w, X, sum)`); `std::lround` lowers
// to `llvm.lround` (half away from zero, exactly [`f64::round`]);
// `std::max(maxChange, fabs)` lowers to `fcmp olt` + select (NaN keeps
// the accumulator, mirrored with an explicit comparison since
// [`f64::max`] instead drops NaN); the projections, quarter-turn
// quotients, and field rebuild stay unfused. The round-loop candidate
// sort is a stable [`slice::sort_by`]: the pinned libc++ `std::sort`
// sorts ranges under 24 elements with stable insertion sort / stable
// sort3-5 networks, so the orders agree exactly in that regime.
// =====================================================================
mod vendored_singularity {
    use crate::surface_mesh::SurfaceMesh;
    use crate::vector3::Vector3;
    use std::collections::{BTreeSet, VecDeque};
    use std::f64::consts::PI;

    /// Mirrors the anonymous-namespace `unit` (by value: `Vector3` is
    /// `Copy`, so this compiles to the same loads as the C++ const refs).
    #[inline]
    fn unit(value: Vector3, fallback: Vector3) -> Vector3 {
        if value.length() < 1e-12 {
            fallback.normalized()
        } else {
            value.normalized()
        }
    }

    #[derive(Clone, Copy)]
    struct Frame {
        u: Vector3,
        v: Vector3,
    }

    fn frame_for_face(mesh: &SurfaceMesh, face: usize) -> Frame {
        let normal = unit(mesh.face_normal(face), Vector3::new(0.0, 0.0, 1.0));
        let mut u = mesh.edge_vector(3 * face);
        // FMA audit: the C++ keeps these projections unfused (separate
        // fmul/fsub in IR), so the plain operators below are exact.
        u = u - Vector3::dot_product(&u, &normal) * normal;
        u = unit(
            u,
            if normal.x().abs() < 0.9 {
                Vector3::new(1.0, 0.0, 0.0)
            } else {
                Vector3::new(0.0, 1.0, 0.0)
            },
        );
        u = u - Vector3::dot_product(&u, &normal) * normal;
        u = unit(u, Vector3::new(1.0, 0.0, 0.0));
        Frame {
            u,
            v: Vector3::cross_product(&normal, &u),
        }
    }

    fn angle_in_frame(vector: &Vector3, frame: &Frame) -> f64 {
        Vector3::dot_product(vector, &frame.v).atan2(Vector3::dot_product(vector, &frame.u))
    }

    fn quarter_turn(
        mesh: &SurfaceMesh,
        corner: usize,
        frames: &[Frame],
        field_angles: &[f64],
    ) -> i32 {
        let opposite = mesh.opposite_corner(corner);
        if opposite == SurfaceMesh::NPOS {
            return 0;
        }
        let f = mesh.corner_face(corner);
        let g = mesh.corner_face(opposite);
        let edge = mesh.edge_vector(corner);
        let connection = angle_in_frame(&edge, &frames[g]) - angle_in_frame(&edge, &frames[f]);
        // `std::lround` is half-away-from-zero, exactly `f64::round`; the
        // values here are small, so the float->int cast never saturates.
        let turns = ((field_angles[g] - field_angles[f] - connection) / (PI / 2.0)).round() as i32;
        ((turns % 4) + 4) % 4
    }

    /// Counts nonzero charges over the affected set (mirrors the C++
    /// `countAffected` lambda; a free function because the closure form
    /// would hold `&self` across the angle mutations between its two
    /// calls).
    fn count_affected(charges: &[i32], affected: &[bool]) -> usize {
        let mut count = 0;
        for (v, &is_affected) in affected.iter().enumerate() {
            if is_affected && charges[v] != 0 {
                count += 1;
            }
        }
        count
    }

    /// Round-loop pairing candidate (mirrors the C++ block-local struct,
    /// lifted to module scope).
    struct Candidate {
        a: usize,
        b: usize,
        hops: usize,
    }

    /// Dipole canceller over a per-face cross field (mirrors
    /// `AutoRemesher::SingularitySimplifier`; vendored, see above).
    pub struct SingularitySimplifier<'a> {
        mesh: &'a SurfaceMesh,
        field: &'a mut [Vector3],
        maximum_pair_distance: usize,
        maximum_rounds: usize,
        region_margin: usize,
        sharp_edge_degrees: f64,
        angles: Vec<f64>,
        connection: Vec<f64>,
        mismatch: Vec<i32>,
        sharp_corner: Vec<bool>,
        frame_u: Vec<Vector3>,
        frame_v: Vec<Vector3>,
    }

    impl<'a> SingularitySimplifier<'a> {
        /// Mirrors the C++ constructor (stores the mesh and field
        /// references; the caches are built by [`Self::simplify`]).
        pub fn new(mesh: &'a SurfaceMesh, field: &'a mut [Vector3]) -> Self {
            Self {
                mesh,
                field,
                maximum_pair_distance: 6,
                maximum_rounds: 4,
                region_margin: 6,
                sharp_edge_degrees: 90.0,
                angles: Vec::new(),
                connection: Vec::new(),
                mismatch: Vec::new(),
                sharp_corner: Vec::new(),
                frame_u: Vec::new(),
                frame_v: Vec::new(),
            }
        }

        pub fn set_maximum_pair_distance(&mut self, hops: usize) {
            self.maximum_pair_distance = hops;
        }

        pub fn set_sharp_edge_degrees(&mut self, degrees: f64) {
            self.sharp_edge_degrees = degrees;
        }

        fn vertex_charges(&self) -> Vec<i32> {
            let mut result = vec![0i32; self.mesh.vertex_count()];
            if self.field.len() != self.mesh.face_count() {
                return result;
            }
            // simplify() is an inner loop that calls this several times per
            // cancelled pair, so only pay for the per-face frames when the
            // cached corner mismatches are missing and quarterTurn actually
            // needs them.
            let use_cached_mismatch = self.mismatch.len() == self.mesh.corner_count();
            let mut frames: Vec<Frame> = Vec::new();
            let mut field_angles: Vec<f64> = Vec::new();
            if !use_cached_mismatch {
                frames.reserve(self.mesh.face_count());
                field_angles.reserve(self.mesh.face_count());
                for f in 0..self.mesh.face_count() {
                    frames.push(frame_for_face(self.mesh, f));
                    let back = frames.len() - 1;
                    field_angles.push(angle_in_frame(&self.field[f], &frames[back]));
                }
            }
            for v in 0..self.mesh.vertex_count() {
                let corners = self.mesh.corners_around_vertex(v);
                if corners.is_empty() {
                    continue;
                }
                let mut c = corners[0];
                let start = c;
                let mut sum = 0i32;
                let mut steps = 0usize;
                loop {
                    steps += 1;
                    if steps > self.mesh.corner_count() {
                        sum = 0;
                        break;
                    }
                    sum += if use_cached_mismatch {
                        self.mismatch[c]
                    } else {
                        quarter_turn(self.mesh, c, &frames, &field_angles)
                    };
                    let opposite = self.mesh.opposite_corner(c);
                    if opposite == SurfaceMesh::NPOS {
                        sum = 0;
                        break;
                    }
                    c = self.mesh.next_corner(opposite);
                    if self.mesh.corner_vertex(c) != v {
                        sum = 0;
                        break;
                    }
                    if c == start {
                        break;
                    }
                }
                if c == start {
                    result[v] = ((sum % 4) + 4) % 4;
                }
            }
            result
        }

        fn build_frames_and_connection(&mut self) {
            let faces = self.mesh.face_count();
            let corners = self.mesh.corner_count();
            self.frame_u.resize(faces, Vector3::default());
            self.frame_v.resize(faces, Vector3::default());
            self.angles.resize(faces, 0.0);
            for f in 0..faces {
                let frame = frame_for_face(self.mesh, f);
                self.frame_u[f] = frame.u;
                self.frame_v[f] = frame.v;
                self.angles[f] = Vector3::dot_product(&self.field[f], &frame.v)
                    .atan2(Vector3::dot_product(&self.field[f], &frame.u));
            }
            // `assign` semantics (clear + refill): a second simplify() call
            // must not see stale corners.
            self.connection.clear();
            self.connection.resize(corners, 0.0);
            self.mismatch.clear();
            self.mismatch.resize(corners, 0);
            self.sharp_corner.clear();
            self.sharp_corner.resize(corners, false);
            for c in 0..corners {
                let other = self.mesh.opposite_corner(c);
                if other == SurfaceMesh::NPOS {
                    continue;
                }
                let f = self.mesh.corner_face(c);
                let g = self.mesh.corner_face(other);
                let edge = self.mesh.edge_vector(c);
                self.connection[c] = Vector3::dot_product(&edge, &self.frame_v[g])
                    .atan2(Vector3::dot_product(&edge, &self.frame_u[g]))
                    - Vector3::dot_product(&edge, &self.frame_v[f])
                        .atan2(Vector3::dot_product(&edge, &self.frame_u[f]));
                self.sharp_corner[c] =
                    self.mesh.normal_angle(c) >= self.sharp_edge_degrees * PI / 180.0;
            }
            let every: Vec<usize> = (0..faces).collect();
            self.update_mismatches(&every);
        }

        fn update_mismatches(&mut self, faces: &[usize]) {
            for &f in faces {
                for c in (3 * f)..(3 * f + 3) {
                    let other = self.mesh.opposite_corner(c);
                    if other == SurfaceMesh::NPOS {
                        continue;
                    }
                    let g = self.mesh.corner_face(other);
                    let turns =
                        ((self.angles[g] - (self.angles[f] + self.connection[c])) / (PI / 2.0))
                            .round() as i32;
                    self.mismatch[c] = ((turns % 4) + 4) % 4;
                    self.mismatch[other] = (4 - self.mismatch[c]) % 4;
                }
            }
        }

        fn facets_around_vertex(&self, vertex: usize) -> Vec<usize> {
            let incident = self.mesh.corners_around_vertex(vertex);
            if incident.is_empty() {
                return Vec::new();
            }
            let mut result = Vec::new();
            let mut c = incident[0];
            let start = c;
            loop {
                if result.len() > self.mesh.corner_count() {
                    return Vec::new();
                }
                result.push(self.mesh.corner_face(c));
                let other = self.mesh.opposite_corner(c);
                if other == SurfaceMesh::NPOS {
                    return Vec::new();
                }
                c = self.mesh.next_corner(other);
                if self.mesh.corner_vertex(c) != vertex {
                    return Vec::new();
                }
                if c == start {
                    break;
                }
            }
            result
        }

        fn corner_along_edge(&self, from: usize, to: usize) -> usize {
            for &c in self.mesh.corners_around_vertex(from) {
                if self.mesh.corner_vertex(self.mesh.next_corner(c)) == to
                    && self.mesh.opposite_corner(c) != SurfaceMesh::NPOS
                {
                    return c;
                }
            }
            SurfaceMesh::NPOS
        }

        fn path_between(
            &self,
            first: usize,
            second: usize,
            in_region: &[bool],
            free_faces: &[usize],
        ) -> Vec<usize> {
            let mut is_free = vec![false; self.mesh.face_count()];
            for &f in free_faces {
                is_free[f] = true;
            }
            let usable = |vertex: usize| {
                let fan = self.facets_around_vertex(vertex);
                if fan.is_empty() {
                    return false;
                }
                for &f in &fan {
                    if !in_region[f] {
                        return false;
                    }
                }
                true
            };
            let crossable = |corner: usize| {
                let other = self.mesh.opposite_corner(corner);
                other != SurfaceMesh::NPOS
                    && is_free[self.mesh.corner_face(corner)]
                    && is_free[self.mesh.corner_face(other)]
            };
            let mut previous = vec![SurfaceMesh::NPOS; self.mesh.vertex_count()];
            let mut queue = VecDeque::new();
            previous[first] = first;
            queue.push_back(first);
            while let Some(v) = queue.pop_front() {
                if v == second {
                    let mut path = Vec::new();
                    let mut at = second;
                    while at != first {
                        path.push(at);
                        at = previous[at];
                    }
                    path.push(first);
                    path.reverse();
                    return path;
                }
                for &c in self.mesh.corners_around_vertex(v) {
                    let next = self.mesh.corner_vertex(self.mesh.next_corner(c));
                    if previous[next] != SurfaceMesh::NPOS || !crossable(c) {
                        continue;
                    }
                    if next != second && !usable(next) {
                        continue;
                    }
                    previous[next] = v;
                    queue.push_back(next);
                }
            }
            Vec::new()
        }

        fn cancel_pair(&mut self, first: usize, second: usize, hops: usize) -> bool {
            let radius = hops + self.region_margin;
            // Restructure: the C++ `ball` lambda captures `this`; the copy
            // of `&SurfaceMesh` below keeps later `&mut` passes free of
            // reborrow dances.
            let mesh = self.mesh;
            let ball = |seeds: &[usize]| {
                let mut distance = vec![SurfaceMesh::NPOS; mesh.face_count()];
                let mut queue = VecDeque::new();
                for &f in seeds {
                    if distance[f] == SurfaceMesh::NPOS {
                        distance[f] = 0;
                        queue.push_back(f);
                    }
                }
                while let Some(f) = queue.pop_front() {
                    if distance[f] >= radius {
                        continue;
                    }
                    for c in (3 * f)..(3 * f + 3) {
                        let other = mesh.opposite_corner(c);
                        if other == SurfaceMesh::NPOS {
                            continue;
                        }
                        let g = mesh.corner_face(other);
                        if distance[g] == SurfaceMesh::NPOS {
                            distance[g] = distance[f] + 1;
                            queue.push_back(g);
                        }
                    }
                }
                distance
            };
            let first_fan = self.facets_around_vertex(first);
            let second_fan = self.facets_around_vertex(second);
            if first_fan.is_empty() || second_fan.is_empty() {
                return false;
            }
            let a = ball(&first_fan);
            let b = ball(&second_fan);
            let mut in_region = vec![false; self.mesh.face_count()];
            let mut region: Vec<usize> = Vec::new();
            let mut free_faces: Vec<usize> = Vec::new();
            for f in 0..self.mesh.face_count() {
                if a[f] != SurfaceMesh::NPOS && b[f] != SurfaceMesh::NPOS {
                    in_region[f] = true;
                    region.push(f);
                }
            }
            if region.is_empty() {
                return false;
            }
            for &f in &region {
                let mut frozen = false;
                for c in (3 * f)..(3 * f + 3) {
                    let other = self.mesh.opposite_corner(c);
                    if self.sharp_corner[c]
                        || other == SurfaceMesh::NPOS
                        || !in_region[self.mesh.corner_face(other)]
                    {
                        frozen = true;
                        break;
                    }
                }
                if !frozen {
                    free_faces.push(f);
                }
            }
            if free_faces.is_empty() {
                return false;
            }
            for v in [first, second] {
                for f in self.facets_around_vertex(v) {
                    if !in_region[f] {
                        return false;
                    }
                }
            }
            let path = self.path_between(first, second, &in_region, &free_faces);
            if path.len() < 2 {
                return false;
            }
            let mut affected = vec![false; self.mesh.vertex_count()];
            for &f in &region {
                for c in (3 * f)..(3 * f + 3) {
                    affected[self.mesh.corner_vertex(c)] = true;
                }
            }
            let before = count_affected(&self.vertex_charges(), &affected);
            let mut saved: Vec<f64> = Vec::with_capacity(free_faces.len());
            for &f in &free_faces {
                saved.push(self.angles[f]);
            }

            let mut jump = vec![0i32; self.mesh.corner_count()];
            for &f in &region {
                for c in (3 * f)..(3 * f + 3) {
                    let other = self.mesh.opposite_corner(c);
                    if other == SurfaceMesh::NPOS {
                        continue;
                    }
                    jump[c] = ((self.angles[self.mesh.corner_face(other)] - self.angles[f]
                        - self.connection[c])
                        / (PI / 2.0))
                        .round() as i32;
                }
            }
            let carried = (((self.vertex_charges()[first] + 1) % 4) + 4) % 4 - 1;
            for i in 0..path.len() - 1 {
                let corner = self.corner_along_edge(path[i], path[i + 1]);
                if corner == SurfaceMesh::NPOS {
                    return false;
                }
                let other = self.mesh.opposite_corner(corner);
                jump[corner] -= carried;
                jump[other] += carried;
            }

            for _ in 0..256usize.max(40 * radius * radius) {
                let mut max_change = 0.0;
                for &f in &free_faces {
                    let mut sum = 0.0;
                    let mut sw = 0.0;
                    for c in (3 * f)..(3 * f + 3) {
                        let other = self.mesh.opposite_corner(c);
                        if other == SurfaceMesh::NPOS {
                            continue;
                        }
                        let g = self.mesh.corner_face(other);
                        let w = self.mesh.edge_vector(c).length();
                        // FMA audit: the C++ fuses this line into two
                        // fmuladds per corner (verified in Release IR):
                        // `fma(-jump, PI/2, angles[g] - connection[c])` for
                        // the parenthesized value, then `fma(w, value, sum)`
                        // for the accumulation. `sw += w` stays a plain
                        // fadd.
                        let t = self.angles[g] - self.connection[c];
                        let x = (-(jump[c] as f64)).mul_add(PI / 2.0, t);
                        sum = w.mul_add(x, sum);
                        sw += w;
                    }
                    if sw > 0.0 {
                        let value = sum / sw;
                        // `std::max(maxChange, fabs)` is `fcmp olt` +
                        // select in IR (NaN keeps the accumulator);
                        // `f64::max` would drop NaN instead, so mirror the
                        // comparison explicitly.
                        let delta = (value - self.angles[f]).abs();
                        max_change = if max_change < delta { delta } else { max_change };
                        self.angles[f] = value;
                    }
                }
                if max_change < 1e-10 {
                    break;
                }
            }
            self.update_mismatches(&region);
            let after = count_affected(&self.vertex_charges(), &affected);
            let charges = self.vertex_charges();
            if charges[first] != 0 || charges[second] != 0 || after + 2 > before {
                for (i, &f) in free_faces.iter().enumerate() {
                    self.angles[f] = saved[i];
                }
                self.update_mismatches(&region);
                return false;
            }
            // FMA audit: the C++ keeps this rebuild unfused (no fmuladd at
            // its line in IR), so the plain operators below are exact.
            for &f in &free_faces {
                self.field[f] =
                    self.angles[f].cos() * self.frame_u[f] + self.angles[f].sin() * self.frame_v[f];
            }
            true
        }

        pub fn simplify(&mut self) {
            // The null-`m_field` case is unrepresentable (`field` is
            // `&mut [Vector3]`, never null): an empty slice early-returns
            // exactly like the C++ `nullptr || empty` branch.
            if self.field.is_empty() {
                return;
            }
            self.build_frames_and_connection();
            for _ in 0..self.maximum_rounds {
                let charges = self.vertex_charges();
                let mut singular: Vec<usize> = Vec::new();
                for (v, &charge) in charges.iter().enumerate() {
                    if charge != 0 {
                        singular.push(v);
                    }
                }
                if singular.len() < 2 {
                    break;
                }
                let mut owner = vec![SurfaceMesh::NPOS; self.mesh.face_count()];
                let mut distance = vec![0usize; self.mesh.face_count()];
                let mut queue = VecDeque::new();
                let mut encounters: Vec<(usize, usize)> = Vec::new();
                for (i, &s) in singular.iter().enumerate() {
                    for f in self.facets_around_vertex(s) {
                        if owner[f] == SurfaceMesh::NPOS {
                            owner[f] = i;
                            queue.push_back(f);
                        } else {
                            encounters.push((owner[f], i));
                        }
                    }
                }
                let mut candidates: Vec<Candidate> = Vec::new();
                let mut seen = BTreeSet::new();
                let maximum_pair_distance = self.maximum_pair_distance;
                let mut consider = |a: usize, b: usize, d: usize| {
                    if a == b
                        || d > maximum_pair_distance
                        || (charges[singular[a]] + charges[singular[b]]) % 4 != 0
                    {
                        return;
                    }
                    let key = if a < b { (a, b) } else { (b, a) };
                    if seen.insert(key) {
                        candidates.push(Candidate {
                            a: key.0,
                            b: key.1,
                            hops: d,
                        });
                    }
                };
                for &(first, second) in &encounters {
                    consider(first, second, 0);
                }
                while let Some(f) = queue.pop_front() {
                    for c in (3 * f)..(3 * f + 3) {
                        let other = self.mesh.opposite_corner(c);
                        if other == SurfaceMesh::NPOS {
                            continue;
                        }
                        let g = self.mesh.corner_face(other);
                        if owner[g] == SurfaceMesh::NPOS {
                            if distance[f] + 1 > maximum_pair_distance {
                                continue;
                            }
                            owner[g] = owner[f];
                            distance[g] = distance[f] + 1;
                            queue.push_back(g);
                        } else {
                            consider(owner[f], owner[g], distance[f] + distance[g]);
                        }
                    }
                }
                // Stable sort: identical to the pinned libc++ `std::sort`
                // for the < 24-candidate regime (see the section docs);
                // past that the C++ tie order is unspecified anyway.
                candidates.sort_by(|x, y| x.hops.cmp(&y.hops));
                // Restructure: the C++ `used` vector is `vector<char>`;
                // `Vec<bool>` has identical semantics here (no bitset
                // tricks: only indexed test + set).
                let mut used = vec![false; singular.len()];
                let mut cancelled = 0usize;
                for c in &candidates {
                    if !used[c.a] && !used[c.b] && self.cancel_pair(singular[c.a], singular[c.b], c.hops)
                    {
                        used[c.a] = true;
                        used[c.b] = true;
                        cancelled += 1;
                    }
                }
                if cancelled == 0 {
                    break;
                }
            }
        }
    }
}
