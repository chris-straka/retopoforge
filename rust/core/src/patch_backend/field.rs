//! Cross-field + sizing stage for the patch back end.
//!
//! Mirrors the default back end's field pipeline
//! (`Parameterizer::parameterize` up to the cover solve: snap sharps,
//! `FrameField::create`, symmetry, singularity simplification) and its
//! face-sizing field (adaptivity curvature term + density modulation),
//! using only the front-end public modules. The cover solve, rounding,
//! and isoline extraction of the default back end are NOT used here:
//! the field below feeds separatrix tracing ([`crate::patch_backend::trace`])
//! instead.

use crate::density::Density;
use crate::frame_field::FrameField;
use crate::guides::Guides;
use crate::singularity_simplifier::SingularitySimplifier;
use crate::surface_mesh::SurfaceMesh;
use crate::symmetry::{Symmetry, SymmetryPlane};
use crate::vector3::Vector3;

/// Field products for one working-mesh island.
pub(crate) struct IslandField {
    /// Post-simplification cross field, one unit tangent per face.
    pub field: Vec<Vector3>,
    /// Post-simplification vertex charges (0 = regular; 1/2/3 as in
    /// `SingularitySimplifier::vertex_charges`).
    pub charges: Vec<i32>,
    /// Per-face target quad width in mesh units (budget-normalized).
    pub face_width: Vec<f64>,
    /// Vertices with nonzero charge, ascending.
    pub singularities: Vec<usize>,
}

/// Nearest-vertex snap of sharp polylines onto the working mesh.
///
/// Equivalent of the default back end's polyline snap: points farther
/// than the guide influence radius from every vertex are dropped, and
/// polylines left with fewer than two points are dropped.
fn snap_sharps_to_mesh(mesh: &SurfaceMesh, input: &[Vec<Vector3>]) -> Vec<Vec<Vector3>> {
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
            let mut best_d2 = (*mesh.position(0) - *point).length_squared();
            for v in 1..mesh.vertex_count() {
                let d2 = (*mesh.position(v) - *point).length_squared();
                if d2 < best_d2 {
                    best_d2 = d2;
                    best = v;
                }
            }
            if best_d2 > radius_squared {
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

/// Curvature-driven per-face sizing multipliers (adaptivity term).
///
/// Same shape as the default back end's scaling field: per-vertex max
/// normal-variation curvature, normalized by the mesh mean, raised to
/// `-adaptivity`, clamped to [0.3, 3.0], then renormalized so
/// `SUM A_f/m_f^2` equals the total area (budget-preserving).
fn adaptivity_multipliers(
    vertices: &[Vector3],
    triangles: &[Vec<usize>],
    adaptivity: f64,
) -> Vec<f64> {
    let mut multipliers = vec![1.0; triangles.len()];
    if adaptivity <= 0.0 || vertices.is_empty() || triangles.is_empty() {
        return multipliers;
    }
    let mut vertex_normals = vec![Vector3::default(); vertices.len()];
    for triangle in triangles {
        let normal = Vector3::normal(
            &vertices[triangle[0]],
            &vertices[triangle[1]],
            &vertices[triangle[2]],
        );
        for &v in triangle {
            vertex_normals[v] += normal;
        }
    }
    for normal in &mut vertex_normals {
        normal.normalize();
    }
    let mut face_around: Vec<Vec<usize>> = vec![Vec::new(); vertices.len()];
    for (i, triangle) in triangles.iter().enumerate() {
        for &v in triangle {
            face_around[v].push(i);
        }
    }
    let mut vertex_curvature = vec![0.0; vertices.len()];
    for v in 0..vertices.len() {
        let normal_v = vertex_normals[v];
        let mut max_curvature = 0.0;
        for &face_index in &face_around[v] {
            for &u in &triangles[face_index] {
                if u == v {
                    continue;
                }
                let distance = (vertices[u] - vertices[v]).length();
                if distance <= 0.0 {
                    continue;
                }
                let mut cos_angle = Vector3::dot_product(&normal_v, &vertex_normals[u]);
                cos_angle = cos_angle.clamp(-1.0, 1.0);
                let curvature = cos_angle.acos() / distance;
                if curvature > max_curvature {
                    max_curvature = curvature;
                }
            }
        }
        vertex_curvature[v] = max_curvature;
    }
    let sum: f64 = vertex_curvature.iter().sum();
    let mean = sum / vertex_curvature.len() as f64;
    if mean <= 0.0 {
        return multipliers;
    }
    for (i, triangle) in triangles.iter().enumerate() {
        let mut face_curvature = 0.0;
        for &v in triangle {
            face_curvature += vertex_curvature[v];
        }
        face_curvature /= triangle.len() as f64;
        let normalized = (face_curvature / mean).max(1e-3);
        multipliers[i] = normalized.powf(-adaptivity).clamp(0.3, 3.0);
    }
    let mut total_area = 0.0;
    let mut weighted_area = 0.0;
    for (i, triangle) in triangles.iter().enumerate() {
        let e0 = vertices[triangle[1]] - vertices[triangle[0]];
        let e1 = vertices[triangle[2]] - vertices[triangle[0]];
        let face_area = 0.5 * Vector3::cross_product(&e0, &e1).length();
        total_area += face_area;
        let m = multipliers[i];
        if m > 0.0 {
            weighted_area += face_area / (m * m);
        }
    }
    if total_area > 0.0 && weighted_area > 0.0 {
        let rescale = (weighted_area / total_area).sqrt();
        if rescale > 0.0 && rescale.is_finite() {
            for m in &mut multipliers {
                *m *= rescale;
            }
        }
    }
    multipliers
}

/// Total mesh area.
fn mesh_area(vertices: &[Vector3], triangles: &[Vec<usize>]) -> f64 {
    let mut area = 0.0;
    for triangle in triangles {
        area += Vector3::area(
            &vertices[triangle[0]],
            &vertices[triangle[1]],
            &vertices[triangle[2]],
        );
    }
    area
}

/// Run the field stage on one working-mesh island.
///
/// `density_per_vertex` is the resampled+normalized mask for this
/// island (empty = off). `edge_scale` is the island's nominal quad
/// width for a uniform field; per-face widths scale it by the
/// adaptivity/density multipliers. Returns `None` when the field
/// solve fails (empty mesh).
pub(crate) fn compute_field(
    vertices: &[Vector3],
    triangles: &[Vec<usize>],
    sharp_edge_degrees: f64,
    guides: &[Vec<Vector3>],
    sharps: &[Vec<Vector3>],
    symmetry_plane: &SymmetryPlane,
    adaptivity: f64,
    density_per_vertex: &[f64],
    edge_scale: f64,
) -> Option<IslandField> {
    let topology = SurfaceMesh::new(vertices, triangles);
    if topology.face_count() != triangles.len() || topology.face_count() == 0 {
        return None;
    }
    let snapped_sharps = if sharps.is_empty() {
        Vec::new()
    } else {
        snap_sharps_to_mesh(&topology, sharps)
    };
    let mut field = FrameField::create(&topology, sharp_edge_degrees, guides, &snapped_sharps)?;
    if field.len() != topology.face_count() {
        return None;
    }
    if symmetry_plane.valid() {
        Symmetry::symmetrize_frame_field(vertices, triangles, &mut field, symmetry_plane);
    }
    let charges = {
        let mut simplifier = SingularitySimplifier::new(&topology, &mut field);
        simplifier.set_sharp_edge_degrees(sharp_edge_degrees);
        simplifier.simplify();
        simplifier.vertex_charges()
    };
    let mut multipliers = adaptivity_multipliers(vertices, triangles, adaptivity);
    if !density_per_vertex.is_empty() {
        Density::apply_to_scaling_field(
            vertices,
            triangles,
            density_per_vertex,
            &mut multipliers,
        );
    }
    // Budget: with renormalized multipliers SUM A_f/m_f^2 equals the
    // island area, so widths of edge_scale*m_f integrate to
    // area/edge_scale^2 quads.
    let face_width: Vec<f64> = multipliers.iter().map(|m| edge_scale * m.max(1e-6)).collect();
    let mut singularities = Vec::new();
    for (v, &charge) in charges.iter().enumerate() {
        if charge != 0 {
            singularities.push(v);
        }
    }
    let _ = mesh_area;
    Some(IslandField {
        field,
        charges,
        face_width,
        singularities,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Closed diamond (octahedron): 6 verts, 8 tris.
    fn diamond() -> (Vec<Vector3>, Vec<Vec<usize>>) {
        let vertices = vec![
            Vector3::new(0.0, 0.0, 1.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
            Vector3::new(-1.0, 0.0, 0.0),
            Vector3::new(0.0, -1.0, 0.0),
            Vector3::new(0.0, 0.0, -1.0),
        ];
        let triangles = vec![
            vec![0, 1, 2],
            vec![0, 2, 3],
            vec![0, 3, 4],
            vec![0, 4, 1],
            vec![5, 2, 1],
            vec![5, 3, 2],
            vec![5, 4, 3],
            vec![5, 1, 4],
        ];
        (vertices, triangles)
    }

    #[test]
    fn field_stage_runs_on_closed_mesh() {
        let (vertices, triangles) = diamond();
        let plane = SymmetryPlane::default();
        let island = compute_field(
            &vertices,
            &triangles,
            90.0,
            &[],
            &[],
            &plane,
            1.0,
            &[],
            0.5,
        )
        .expect("field must solve on a diamond");
        assert_eq!(island.field.len(), triangles.len());
        assert_eq!(island.charges.len(), vertices.len());
        assert_eq!(island.face_width.len(), triangles.len());
        for width in &island.face_width {
            assert!(width.is_finite() && *width > 0.0);
        }
        // Unit tangents.
        let topology = SurfaceMesh::new(&vertices, &triangles);
        for (f, direction) in island.field.iter().enumerate() {
            assert!((direction.length() - 1.0).abs() < 1e-9, "face {f}");
            let normal = topology.face_normal(f);
            assert!(
                Vector3::dot_product(direction, &normal).abs() < 1e-9,
                "face {f}"
            );
        }
    }

    #[test]
    fn empty_mesh_fails_cleanly() {
        let plane = SymmetryPlane::default();
        assert!(
            compute_field(&[], &[], 90.0, &[], &[], &plane, 1.0, &[], 0.5).is_none()
        );
    }
}
