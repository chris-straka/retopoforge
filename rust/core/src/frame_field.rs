//! Port of `core/framefield.*`: smooth cross (4-symmetric) field over the mesh
//! (sharp-edge locks, guide/sharp polyline locks, curvature-tensor seeding,
//! constrained smoothing solve).
//!
//! Line-by-line mirror of the C++ implementation: same functions in the
//! same order, same thresholds, same solver call sequence. Deliberate
//! restructures, each noted at the site:
//! - the TBB data-parallel loops run sequentially (every one is a disjoint
//!   per-element write, so the C++ is already deterministic and the op order
//!   per element is unchanged);
//! - `Guides::influenceRadius` / `Guides::tangentNear` were private
//!   minimal mirrors while the guides lane was in flight; both are
//!   deduped against the joined sibling port now (the dedup also fixed
//!   a latent NaN bug: the mirror used Rust `.min/.max`, the sibling
//!   uses C++-exact `cxx_min/cxx_max`);
//! - `Eigen::SelfAdjointEigenSolver<Matrix3d>` becomes a small self-contained
//!   cyclic Jacobi eigensolver (no Eigen on the Rust side); the eigenvalue
//!   index sort is a stable insertion sort, matching libc++/libstdc++
//!   `std::sort` on 3 elements (insertion sort there, hence stable);
//! - C++ out-params become return values (`None` mirrors every `false`
//!   return); the `normalizePeriodic` closure is an associated function.
//!
//! FMA audit (Release IR, `llvm.fmuladd`, per
//! `docs/rust-port-conventions.md`): every fusion inside `Vector3` methods
//! is already replicated by the FMA-exact `vector3` port, which this module
//! uses exclusively, and no `Vector3` operator `+`/`-`/scalar `*` site
//! fuses. Of this module's own expression sites, exactly the six
//! `accumulate_curvature_tensor` lines fuse (each `+=` becomes
//! `fma(w*u_i, u_j, t)`; the inner products stay plain `fmul`), all
//! mirrored with explicit [`f64::mul_add`] below — every other site
//! (chained multiplies, scalar-vector products, divisions, the
//! `-kSymmetry * (a - b)` transport) emits no fusion. The remaining IR
//! fusions sit inside Eigen's eigensolver internals, which the Jacobi
//! restructure replaces algorithmically (nothing to fuse against); the
//! oracle's 1e-6 tolerance covers its backend-class noise.
//!
//! Sincos audit (Release machine code, `___sincos_stret` relocs): all six
//! adjacent `cos`/`sin` pairs fuse (cpp:111-112, 145-146, 175-176 unrolled
//! x3, 223-224, 261, 294 unrolled x2), each mirrored with
//! [`crate::double_utils::joint_sin_cos`] below.

use crate::guides::Guides;
use crate::surface_mesh::SurfaceMesh;
use crate::vector3::Vector3;
use retopo_solvers::constrained::ConstrainedLeastSquares;
use std::f64::consts::PI;

const K_SYMMETRY: f64 = 4.0;

struct FacetTangentBasis {
    tangent: Vector3,
    perpendicular_tangent: Vector3,
    normal: Vector3,
}

fn normalized_or_fallback(vector: Vector3, fallback: Vector3) -> Vector3 {
    if vector.length() <= 1e-12 {
        fallback.normalized()
    } else {
        vector.normalized()
    }
}

fn create_facet_tangent_basis(mesh: &SurfaceMesh, face: usize) -> FacetTangentBasis {
    let normal = normalized_or_fallback(mesh.face_normal(face), Vector3::new(0.0, 0.0, 1.0));
    let mut tangent = mesh.edge_vector(3 * face);
    tangent = tangent - Vector3::dot_product(&tangent, &normal) * normal;
    if tangent.length() <= 1e-12 {
        tangent = if normal.x().abs() < 0.9 {
            Vector3::new(1.0, 0.0, 0.0)
        } else {
            Vector3::new(0.0, 1.0, 0.0)
        };
        tangent = tangent - Vector3::dot_product(&tangent, &normal) * normal;
    }
    tangent = normalized_or_fallback(tangent, Vector3::new(1.0, 0.0, 0.0));
    FacetTangentBasis {
        tangent,
        perpendicular_tangent: Vector3::cross_product(&normal, &tangent),
        normal,
    }
}

fn tangent_angle(vector: &Vector3, basis: &FacetTangentBasis) -> f64 {
    Vector3::dot_product(vector, &basis.perpendicular_tangent)
        .atan2(Vector3::dot_product(vector, &basis.tangent))
}

fn accumulate_curvature_tensor(tensor: &mut [f64; 6], edge: Vector3, dihedral: f64) {
    let unit_edge = normalized_or_fallback(edge, Vector3::new(1.0, 0.0, 0.0));
    let weighted_dihedral = edge.length() * dihedral;
    // FMA audit: Clang fuses each `+=` (but not the inner products, two of
    // which it CSEs across lines — bitwise-transparent since the inputs
    // are identical) into `fma(w*u_i, u_j, t)`. Mirrored exactly.
    tensor[0] = (weighted_dihedral * unit_edge.x()).mul_add(unit_edge.x(), tensor[0]);
    tensor[1] = (weighted_dihedral * unit_edge.x()).mul_add(unit_edge.y(), tensor[1]);
    tensor[2] = (weighted_dihedral * unit_edge.y()).mul_add(unit_edge.y(), tensor[2]);
    tensor[3] = (weighted_dihedral * unit_edge.x()).mul_add(unit_edge.z(), tensor[3]);
    tensor[4] = (weighted_dihedral * unit_edge.y()).mul_add(unit_edge.z(), tensor[4]);
    tensor[5] = (weighted_dihedral * unit_edge.z()).mul_add(unit_edge.z(), tensor[5]);
}

fn curvature_tensor_matrix(coefficients: &[f64; 6]) -> [[f64; 3]; 3] {
    [
        [coefficients[0], coefficients[1], coefficients[3]],
        [coefficients[1], coefficients[2], coefficients[4]],
        [coefficients[3], coefficients[4], coefficients[5]],
    ]
}

/// Symmetric 3x3 eigendecomposition via cyclic Jacobi rotations.
///
/// Deliberate restructure standing in for
/// `Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>`: returns the eigenvalues
/// in diagonal order (NOT sorted — the caller index-sorts exactly like the
/// C++) with the eigenvectors as columns. `None` mirrors
/// `eig.info() != Eigen::Success` (the caller skips the face); Jacobi has no
/// other failure mode, so only a non-finite result (NaN/Inf input tensor)
/// reports failure.
fn symmetric_eigen_3x3(matrix: &[[f64; 3]; 3]) -> Option<([f64; 3], [[f64; 3]; 3])> {
    let mut a = *matrix;
    let mut v = [[0.0; 3]; 3];
    v[0][0] = 1.0;
    v[1][1] = 1.0;
    v[2][2] = 1.0;
    for _ in 0..50 {
        let off = (a[0][1] * a[0][1] + a[0][2] * a[0][2] + a[1][2] * a[1][2]).sqrt();
        if !(off > 0.0) {
            break;
        }
        // Quadratic convergence reaches the ulp floor in a handful of
        // sweeps for 3x3; further sweeps only rotate by ~identity and
        // accumulate rounding noise. Exit once the off-diagonal norm is
        // at relative ulps (eigenvalues accurate to ~1e-15, the same
        // noise class as full convergence).
        let scale = a[0][0].abs() + a[1][1].abs() + a[2][2].abs();
        if off <= 1e-15 * scale {
            break;
        }
        for (p, q) in [(0, 1), (0, 2), (1, 2)] {
            if a[p][q] == 0.0 {
                continue;
            }
            let tau = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
            let t = (if tau >= 0.0 { 1.0 } else { -1.0 }) / (tau.abs() + (1.0 + tau * tau).sqrt());
            let c = 1.0 / (1.0 + t * t).sqrt();
            let s = t * c;
            a[p][p] -= t * a[p][q];
            a[q][q] += t * a[p][q];
            a[p][q] = 0.0;
            a[q][p] = 0.0;
            for r in 0..3 {
                if r != p && r != q {
                    let arp = a[r][p];
                    let arq = a[r][q];
                    a[r][p] = c * arp - s * arq;
                    a[p][r] = a[r][p];
                    a[r][q] = s * arp + c * arq;
                    a[q][r] = a[r][q];
                }
            }
            for r in 0..3 {
                let vrp = v[r][p];
                let vrq = v[r][q];
                v[r][p] = c * vrp - s * vrq;
                v[r][q] = s * vrp + c * vrq;
            }
        }
    }
    let values = [a[0][0], a[1][1], a[2][2]];
    if !values.iter().all(|x| x.is_finite()) || !v.iter().flatten().all(|x| x.is_finite()) {
        return None;
    }
    Some((values, v))
}

fn lock_guide_faces(
    mesh: &SurfaceMesh,
    facet_bases: &[FacetTangentBasis],
    guides: &[Vec<Vector3>],
    periodic: &mut [f64],
    locked: &mut [bool],
) {
    let radius = Guides::influence_radius(mesh);
    for face_index in 0..mesh.face_count() {
        if locked[face_index] {
            continue;
        }
        let triangle = mesh.triangle(face_index);
        let centroid = (*mesh.position(triangle[0])
            + *mesh.position(triangle[1])
            + *mesh.position(triangle[2]))
            / 3.0;
        let tangent =
            Guides::tangent_near(guides, &centroid, &facet_bases[face_index].normal, radius);
        if tangent.length() <= 1e-12 {
            continue;
        }
        let field_angle = K_SYMMETRY * tangent_angle(&tangent, &facet_bases[face_index]);
        // Fused `sincos` in C++ (cpp:111-112, reloc 0x688): joint call.
        let (sn, cs) = crate::double_utils::joint_sin_cos(field_angle);
        periodic[2 * face_index] = cs;
        periodic[2 * face_index + 1] = sn;
        locked[face_index] = true;
    }
}

fn sharp_influence_radius(mesh: &SurfaceMesh) -> f64 {
    2.0 * mesh.average_edge_length()
}

fn lock_sharp_faces(
    mesh: &SurfaceMesh,
    facet_bases: &[FacetTangentBasis],
    sharps: &[Vec<Vector3>],
    periodic: &mut [f64],
    locked: &mut [bool],
) {
    let radius = sharp_influence_radius(mesh);
    for face_index in 0..mesh.face_count() {
        if locked[face_index] {
            continue;
        }
        let triangle = mesh.triangle(face_index);
        let centroid = (*mesh.position(triangle[0])
            + *mesh.position(triangle[1])
            + *mesh.position(triangle[2]))
            / 3.0;
        let tangent =
            Guides::tangent_near(sharps, &centroid, &facet_bases[face_index].normal, radius);
        if tangent.length() <= 1e-12 {
            continue;
        }
        let field_angle = K_SYMMETRY * tangent_angle(&tangent, &facet_bases[face_index]);
        // Fused `sincos` in C++ (cpp:145-146, reloc 0x510): joint call.
        let (sn, cs) = crate::double_utils::joint_sin_cos(field_angle);
        periodic[2 * face_index] = cs;
        periodic[2 * face_index + 1] = sn;
        locked[face_index] = true;
    }
}

fn normalize_periodic(periodic: &mut [f64], faces: usize) {
    for face_index in 0..faces {
        let periodic_length = periodic[2 * face_index].hypot(periodic[2 * face_index + 1]);
        if periodic_length > 1e-30 {
            periodic[2 * face_index] /= periodic_length;
            periodic[2 * face_index + 1] /= periodic_length;
        }
    }
}

/// Smooth cross field (mirrors `AutoRemesher::FrameField`).
pub struct FrameField;

impl FrameField {
    /// Computes the 4-symmetric direction field, one unit tangent per face.
    ///
    /// Mirrors `FrameField::create`: `guides`/`sharps` are user polylines as
    /// ordered point chains in mesh coordinates (empty slices disable the
    /// respective pass, like the C++ defaults). `None` mirrors every `false`
    /// return (empty mesh, solver failure).
    pub fn create(
        mesh: &SurfaceMesh,
        sharp_edge_degrees: f64,
        guides: &[Vec<Vector3>],
        sharps: &[Vec<Vector3>],
    ) -> Option<Vec<Vector3>> {
        if mesh.face_count() == 0 {
            return None;
        }
        let faces = mesh.face_count();
        let mut facet_bases = Vec::with_capacity(faces);
        for face_index in 0..faces {
            facet_bases.push(create_facet_tangent_basis(mesh, face_index));
        }

        let mut periodic = vec![0.0; 2 * faces];
        let mut certainty = vec![0.0; faces];
        let mut locked = vec![false; faces];
        let sharp_radians = sharp_edge_degrees * PI / 180.0;
        for face_index in 0..faces {
            for corner_index in 3 * face_index..3 * face_index + 3 {
                let opposite_corner_index = mesh.opposite_corner(corner_index);
                if opposite_corner_index != SurfaceMesh::NPOS
                    && mesh.normal_angle(corner_index).abs() <= sharp_radians
                {
                    continue;
                }
                let field_angle = K_SYMMETRY
                    * tangent_angle(&mesh.edge_vector(corner_index), &facet_bases[face_index]);
                // Fused `sincos` in C++ (cpp:175-176, relocs 0x21c/0x2c8/0x35c): joint call.
                let (sn, cs) = crate::double_utils::joint_sin_cos(field_angle);
                periodic[2 * face_index] = cs;
                periodic[2 * face_index + 1] = sn;
                locked[face_index] = true;
            }
        }

        if !sharps.is_empty() {
            lock_sharp_faces(mesh, &facet_bases, sharps, &mut periodic, &mut locked);
        }

        if !guides.is_empty() {
            lock_guide_faces(mesh, &facet_bases, guides, &mut periodic, &mut locked);
        }

        let mut vertex_tensor = vec![[0.0; 6]; mesh.vertex_count()];
        for corner_index in 0..mesh.corner_count() {
            let opposite_corner_index = mesh.opposite_corner(corner_index);
            if opposite_corner_index == SurfaceMesh::NPOS || opposite_corner_index < corner_index {
                continue;
            }
            let edge = mesh.edge_vector(corner_index);
            let dihedral = mesh.normal_angle(corner_index);
            let head = mesh.corner_vertex(corner_index);
            let tail = mesh.corner_vertex(mesh.next_corner(corner_index));
            accumulate_curvature_tensor(&mut vertex_tensor[head], edge, dihedral);
            accumulate_curvature_tensor(&mut vertex_tensor[tail], edge, dihedral);
        }
        let mut maximum_certainty = 0.0;
        for face_index in 0..faces {
            if locked[face_index] {
                continue;
            }
            let mut total = [0.0; 6];
            for corner_index in 3 * face_index..3 * face_index + 3 {
                for coefficient_index in 0..6 {
                    total[coefficient_index] +=
                        vertex_tensor[mesh.corner_vertex(corner_index)][coefficient_index];
                }
            }
            let mut tensor = curvature_tensor_matrix(&total);
            let trace = tensor[0][0] + tensor[1][1] + tensor[2][2];
            let regularizer = if trace == 0.0 { 1e-6 } else { 1e-6 * trace };
            tensor[0][0] += regularizer;
            tensor[1][1] += regularizer;
            tensor[2][2] += regularizer;
            let Some((eigenvalues, eigenvectors)) = symmetric_eigen_3x3(&tensor) else {
                continue;
            };
            // Stable insertion sort of the indices by |eigenvalue|,
            // descending: mirrors `std::sort` on 3 elements (insertion sort
            // in both libc++ and libstdc++, hence stable for ties).
            let mut ordered = [0, 1, 2];
            for i in 1..3 {
                let mut j = i;
                while j > 0 && eigenvalues[ordered[j]].abs() > eigenvalues[ordered[j - 1]].abs() {
                    ordered.swap(j, j - 1);
                    j -= 1;
                }
            }
            let primary = ordered[0];
            let secondary = ordered[1];
            let principal_direction = Vector3::new(
                eigenvectors[0][primary],
                eigenvectors[1][primary],
                eigenvectors[2][primary],
            );
            let field_angle =
                K_SYMMETRY * tangent_angle(&principal_direction, &facet_bases[face_index]);
            // Fused `sincos` in C++ (cpp:223-224, reloc 0x51ac): joint call.
            let (sn, cs) = crate::double_utils::joint_sin_cos(field_angle);
            periodic[2 * face_index] = cs;
            periodic[2 * face_index + 1] = sn;
            certainty[face_index] = (eigenvalues[primary] - eigenvalues[secondary]).abs();
        }
        for certainty_value in &certainty {
            // Exact `std::max` semantics for the accumulator (NaN can never
            // stick: `certainty_value > maximum` is false for NaN, same as
            // the C++ `(max < v) ? v : max`).
            if *certainty_value > maximum_certainty {
                maximum_certainty = *certainty_value;
            }
        }
        if maximum_certainty > 0.0 {
            for certainty_value in &mut certainty {
                *certainty_value /= maximum_certainty;
            }
        }

        normalize_periodic(&mut periodic, faces);
        let mut system = ConstrainedLeastSquares::new(2 * faces);
        for f in 0..faces {
            if locked[f] {
                system.add_constraint(&[(2 * f, 1.0)], periodic[2 * f]);
                system.add_constraint(&[(2 * f + 1, 1.0)], periodic[2 * f + 1]);
            }
        }
        for c in 0..mesh.corner_count() {
            let other = mesh.opposite_corner(c);
            if other == SurfaceMesh::NPOS {
                continue;
            }
            let f = mesh.corner_face(c);
            let g = mesh.corner_face(other);
            if f < g {
                continue;
            }
            let edge = mesh.edge_vector(c);
            let transport = -K_SYMMETRY
                * (tangent_angle(&edge, &facet_bases[g]) - tangent_angle(&edge, &facet_bases[f]));
            // Fused `sincos` in C++ (cpp:261, reloc 0xc44): joint call.
            let (si, co) = crate::double_utils::joint_sin_cos(transport);
            system.add_energy(&[(2 * f, co), (2 * f + 1, si), (2 * g, -1.0)], 0.0, 1.0);
            system.add_energy(
                &[(2 * f, -si), (2 * f + 1, co), (2 * g + 1, -1.0)],
                0.0,
                1.0,
            );
        }
        let mut certainty_rows: Vec<(usize, usize)> = Vec::new();
        for f in 0..faces {
            if certainty[f] > 0.0 {
                let weight = certainty[f] * certainty[f];
                let row_u = system.add_energy(&[(2 * f, 1.0)], periodic[2 * f], weight);
                let row_v = system.add_energy(&[(2 * f + 1, 1.0)], periodic[2 * f + 1], weight);
                certainty_rows.push((row_u, row_v));
            }
        }

        for _ in 0..5 {
            let mut row_index = 0;
            for f in 0..faces {
                if certainty[f] > 0.0 {
                    let (row_u, row_v) = certainty_rows[row_index];
                    system.set_energy_right_hand_side(row_u, periodic[2 * f]);
                    system.set_energy_right_hand_side(row_v, periodic[2 * f + 1]);
                    row_index += 1;
                }
            }
            let Some(solved) = system.solve() else {
                return None;
            };
            periodic = solved;
            normalize_periodic(&mut periodic, faces);
        }
        let mut field = vec![Vector3::default(); faces];
        for face_index in 0..faces {
            let field_angle =
                periodic[2 * face_index + 1].atan2(periodic[2 * face_index]) / K_SYMMETRY;
            // Fused `sincos` in C++ (cpp:294, relocs 0x7934/0x7cac): joint call.
            let (sn, cs) = crate::double_utils::joint_sin_cos(field_angle);
            field[face_index] = cs * facet_bases[face_index].tangent
                + sn * facet_bases[face_index].perpendicular_tangent;
        }
        Some(field)
    }
}
