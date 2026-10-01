//! Port of `core/quadparameterizer.*`: quad cover parameterization over a
//! smoothed cross field (field smoothing, spanning-tree brush, corner
//! rotations, curl correction, seam placement, mixed-integer cover solve,
//! UV build).
//!
//! Line-by-line mirror of the C++ implementation: same functions in the
//! same order, same thresholds, same solver call sequence. Deliberate
//! restructures, each noted at the site:
//! - the TBB data-parallel loops run sequentially (every one is a disjoint
//!   per-element write, so the C++ is already deterministic and the op order
//!   per element is unchanged);
//! - `Guides::tangentNear` and `SurfaceMesh` were private minimal mirrors
//!   while their lanes were in flight; both are deduped against the
//!   joined sibling ports now (the guides dedup also fixed a latent NaN
//!   bug: the mirror used Rust `.min/.max`, the sibling uses C++-exact
//!   `cxx_min/cxx_max`);
//! - progress remap closures pass `&dyn Fn` instead of rebundling the
//!   `'static` [`ProgressHandler`] box, which cannot capture the caller's
//!   handler reference;
//! - C++ out-params become return values; every C++ copy is either kept as
//!   an explicit `clone` or fused into the constructed value (no borrowck
//!   clones of whole matrices).
//!
//! FMA audit (Release IR, `llvm.fmuladd`, per
//! `docs/rust-port-conventions.md`): every fusion inside `Vector3` methods
//! is already replicated by the FMA-exact `vector2`/`vector3` ports, which
//! this module uses exclusively, and no `Vector3` operator `+`/`-`/scalar
//! `*` site fuses. The C++ has exactly five fused instructions at four of
//! its own expression sites, all mirrored with explicit [`f64::mul_add`]
//! (and [`f32::mul_add`]) below: the two-line smoothing rotation fold, the
//! quarter-turn application, and the two f32 progress remaps. The
//! quarter-turn fusion is provably unobservable (it multiplies by exact
//! +1/-1/0 constants, where fused and unfused round identically), but it is
//! mirrored anyway.

use crate::guides::Guides;
use crate::progress::ProgressHandler;
use crate::surface_mesh::SurfaceMesh;
use crate::vector2::Vector2;
use crate::vector3::Vector3;
use retopo_solvers::constrained::ConstrainedLeastSquares;
use retopo_solvers::mixed_integer::MixedIntegerLeastSquares;
use std::collections::VecDeque;
use std::f64::consts::PI;

/// Mirrors `unit` (by value: `Vector3` is `Copy`, so this compiles to the
/// same loads as the C++ const refs).
#[inline]
fn unit(v: Vector3, fallback: Vector3) -> Vector3 {
    if v.length() < 1e-12 {
        fallback.normalized()
    } else {
        v.normalized()
    }
}

fn edge_quarter_turn(
    mesh: &SurfaceMesh,
    corner: usize,
    field: &[Vector3],
    normals: &[Vector3],
) -> i32 {
    let opposite = mesh.opposite_corner(corner);
    if opposite == SurfaceMesh::NPOS {
        return 0;
    }
    let f = mesh.corner_face(corner);
    let g = mesh.corner_face(opposite);
    if f > g {
        let r = edge_quarter_turn(mesh, opposite, field, normals);
        return (4 - r) % 4;
    }
    let (mut v0, mut v1) = (
        mesh.corner_vertex(corner),
        mesh.corner_vertex(mesh.next_corner(corner)),
    );
    if v1 < v0 {
        std::mem::swap(&mut v0, &mut v1);
    }
    let e = unit(
        *mesh.position(v1) - *mesh.position(v0),
        Vector3::new(1.0, 0.0, 0.0),
    );
    let y0 = unit(
        Vector3::cross_product(&normals[f], &e),
        Vector3::new(0.0, 1.0, 0.0),
    );
    let yg = unit(
        Vector3::cross_product(&normals[g], &e),
        Vector3::new(0.0, 1.0, 0.0),
    );
    let a0 = Vector3::dot_product(&field[f], &y0).atan2(Vector3::dot_product(&field[f], &e));
    let mut best = 0;
    let mut best_error = 1e100;
    let mut candidate = field[g];
    for r in 0..4 {
        let ag = Vector3::dot_product(&candidate, &yg).atan2(Vector3::dot_product(&candidate, &e));
        let mut d = (a0 - ag).abs();
        while d > PI {
            d = (d - 2.0 * PI).abs();
        }
        if d < best_error {
            best_error = d;
            best = r;
        }
        candidate = Vector3::cross_product(&normals[g], &candidate);
    }
    best
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum EdgeConstraint {
    ConstraintNone,
    ConstraintU,
    ConstraintV,
}

fn edge_constraint(
    mesh: &SurfaceMesh,
    c: usize,
    field: &[Vector3],
    normals: &[Vector3],
    hard_edge_degrees: f64,
) -> EdgeConstraint {
    if mesh.opposite_corner(c) != SurfaceMesh::NPOS
        && mesh.normal_angle(c).abs() * 180.0 / PI < hard_edge_degrees
    {
        return EdgeConstraint::ConstraintNone;
    }
    let f = mesh.corner_face(c);
    let edge = unit(mesh.edge_vector(c), Vector3::new(1.0, 0.0, 0.0));
    let b = unit(field[f], edge);
    let br = unit(
        Vector3::cross_product(&normals[f], &b),
        Vector3::new(0.0, 1.0, 0.0),
    );
    let along_b = Vector3::dot_product(&edge, &b)
        .abs()
        .min(1.0)
        .max(-1.0)
        .acos()
        < 10.0 * PI / 180.0;
    let along_br = Vector3::dot_product(&edge, &br)
        .abs()
        .min(1.0)
        .max(-1.0)
        .acos()
        < 10.0 * PI / 180.0;
    if along_b == along_br {
        return EdgeConstraint::ConstraintNone;
    }
    if along_b {
        EdgeConstraint::ConstraintV
    } else {
        EdgeConstraint::ConstraintU
    }
}

// Explicit-sharp twin of edgeConstraint: the dihedral gate is replaced
// by proximity to a user sharp polyline (edge midpoint within `radius`
// of a segment). The field-alignment check below is identical, so a
// marked edge constrains the same U/V coordinate an automatic sharp
// would, and anchors the curl correction through the same
// cornerConstraints channel. Returns None when no sharp passes nearby.
fn sharp_edge_constraint(
    mesh: &SurfaceMesh,
    c: usize,
    field: &[Vector3],
    normals: &[Vector3],
    sharps: &[Vec<Vector3>],
    radius: f64,
) -> EdgeConstraint {
    let f = mesh.corner_face(c);
    let v0 = mesh.corner_vertex(c);
    let v1 = mesh.corner_vertex(mesh.next_corner(c));
    let midpoint = (*mesh.position(v0) + *mesh.position(v1)) / 2.0;
    if Guides::tangent_near(sharps, &midpoint, &normals[f], radius).length() <= 1e-12 {
        return EdgeConstraint::ConstraintNone;
    }
    let edge = unit(mesh.edge_vector(c), Vector3::new(1.0, 0.0, 0.0));
    let b = unit(field[f], edge);
    let br = unit(
        Vector3::cross_product(&normals[f], &b),
        Vector3::new(0.0, 1.0, 0.0),
    );
    let along_b = Vector3::dot_product(&edge, &b)
        .abs()
        .min(1.0)
        .max(-1.0)
        .acos()
        < 10.0 * PI / 180.0;
    let along_br = Vector3::dot_product(&edge, &br)
        .abs()
        .min(1.0)
        .max(-1.0)
        .acos()
        < 10.0 * PI / 180.0;
    if along_b == along_br {
        return EdgeConstraint::ConstraintNone;
    }
    if along_b {
        EdgeConstraint::ConstraintV
    } else {
        EdgeConstraint::ConstraintU
    }
}

struct CoverContext<'a> {
    mesh: &'a SurfaceMesh,
    field: &'a [Vector3],
    normals: &'a [Vector3],
    rotation: &'a [i32],
    seam: &'a [bool],
    corner_constraints: &'a [EdgeConstraint],
    scaling_u: &'a [f64],
    scaling_v: &'a [f64],
    face_scaling: &'a [f64],
    scale: f64,
}

fn initialize_field_and_normals(
    mesh: &SurfaceMesh,
    guidance: &[Vector3],
    normals: &mut Vec<Vector3>,
    field: &mut Vec<Vector3>,
) {
    let has_guidance = guidance.len() == mesh.face_count();
    normals.clear();
    normals.resize(mesh.face_count(), Vector3::default());
    field.resize(mesh.face_count(), Vector3::default());
    // Sequential: the C++ TBB loop writes disjoint per-face slots.
    for face_index in 0..mesh.face_count() {
        normals[face_index] = unit(mesh.face_normal(face_index), Vector3::new(0.0, 0.0, 1.0));
        let mut tangent_axis = Vector3::new(1.0, 0.0, 0.0);
        if Vector3::dot_product(&tangent_axis, &normals[face_index]).abs() > 0.8 {
            tangent_axis = Vector3::new(0.0, 1.0, 0.0);
        }
        let mut field_direction = if has_guidance {
            guidance[face_index]
        } else {
            tangent_axis
        };
        if !has_guidance {
            let n = normals[face_index];
            field_direction = field_direction - n * Vector3::dot_product(&field_direction, &n);
        }
        field[face_index] = unit(field_direction, mesh.edge_vector(3 * face_index));
    }
}

fn smooth_cross_field(
    mesh: &SurfaceMesh,
    normals: &[Vector3],
    hard_edge_degrees: f64,
    field: &mut [Vector3],
) {
    let mut alpha = vec![0.0; 2 * mesh.face_count()];
    let mut locked = vec![false; mesh.face_count()];
    // Sequential: the C++ TBB loop writes disjoint per-face slots.
    for face_index in 0..mesh.face_count() {
        alpha[2 * face_index] = 1.0;
        for local_corner in 0..3 {
            let corner_index = 3 * face_index + local_corner;
            let opposite_corner_index = mesh.opposite_corner(corner_index);
            if opposite_corner_index != SurfaceMesh::NPOS
                && mesh.normal_angle(corner_index).abs() * 180.0 / PI < hard_edge_degrees
            {
                continue;
            }
            let edge = unit(mesh.edge_vector(corner_index), Vector3::new(1.0, 0.0, 0.0));
            let field_direction = field[face_index];
            let perpendicular = unit(
                Vector3::cross_product(&normals[face_index], &field_direction),
                Vector3::new(0.0, 1.0, 0.0),
            );
            let field_angle = Vector3::dot_product(&edge, &perpendicular)
                .atan2(Vector3::dot_product(&edge, &field_direction));
            alpha[2 * face_index] = (4.0 * field_angle).cos();
            alpha[2 * face_index + 1] = (4.0 * field_angle).sin();
            locked[face_index] = true;
        }
    }
    for _iteration in 0..40 {
        let mut next = alpha.clone();
        // Sequential: the C++ TBB loop reads the frozen `alpha` snapshot and
        // writes disjoint `next` slots.
        for f in 0..mesh.face_count() {
            if locked[f] {
                continue;
            }
            let mut x = alpha[2 * f];
            let mut y = alpha[2 * f + 1];
            let bf = field[f];
            let btf = unit(
                Vector3::cross_product(&normals[f], &bf),
                Vector3::new(0.0, 1.0, 0.0),
            );
            for l in 0..3 {
                let oc = mesh.opposite_corner(3 * f + l);
                if oc == SurfaceMesh::NPOS {
                    continue;
                }
                let g = mesh.corner_face(oc);
                let bg_raw = field[g];
                let bg = unit(
                    bg_raw - normals[f] * Vector3::dot_product(&bg_raw, &normals[f]),
                    bf,
                );
                let d = Vector3::dot_product(&bg, &btf).atan2(Vector3::dot_product(&bg, &bf));
                let cs = (4.0 * d).cos();
                let sn = (4.0 * d).sin();
                // FMA audit: the C++ fuses each line into one fmuladd with
                // the FIRST product fused: `fma(cs, ax, sn * -ay)` and
                // `fma(sn, ax, ay * cs)`; the `+=` stays a separate fadd.
                x += cs.mul_add(alpha[2 * g], sn * -alpha[2 * g + 1]);
                y += sn.mul_add(alpha[2 * g], alpha[2 * g + 1] * cs);
            }
            let length = x.hypot(y);
            if length > 1e-12 {
                next[2 * f] = x / length;
                next[2 * f + 1] = y / length;
            }
        }
        std::mem::swap(&mut alpha, &mut next);
    }
    // Sequential: the C++ TBB loop writes disjoint per-face slots.
    for face_index in 0..mesh.face_count() {
        let field_angle = 0.25 * alpha[2 * face_index + 1].atan2(alpha[2 * face_index]);
        let field_direction = field[face_index];
        let perpendicular = unit(
            Vector3::cross_product(&normals[face_index], &field_direction),
            Vector3::new(0.0, 1.0, 0.0),
        );
        field[face_index] = unit(
            field_direction * field_angle.cos() + perpendicular * field_angle.sin(),
            field_direction,
        );
    }
}

fn brush_field_along_spanning_tree(mesh: &SurfaceMesh, normals: &[Vector3], field: &mut [Vector3]) {
    let mut seen = vec![false; mesh.face_count()];
    if mesh.face_count() != 0 {
        let mut q = VecDeque::new();
        q.push_back(0usize);
        seen[0] = true;
        while let Some(f) = q.pop_front() {
            for l in 0..3 {
                let c = 3 * f + l;
                let oc = mesh.opposite_corner(c);
                if oc == SurfaceMesh::NPOS {
                    continue;
                }
                let g = mesh.corner_face(oc);
                if !seen[g] {
                    let turns = edge_quarter_turn(mesh, c, field, normals);
                    let mut brushed = field[g];
                    for _k in 0..turns {
                        brushed = Vector3::cross_product(&normals[g], &brushed);
                    }
                    field[g] = unit(brushed, mesh.edge_vector(3 * g));
                    seen[g] = true;
                    q.push_back(g);
                }
            }
        }
    }
}

fn compute_corner_rotations(
    mesh: &SurfaceMesh,
    field: &[Vector3],
    normals: &[Vector3],
) -> Vec<i32> {
    let corners = mesh.corner_count();
    let mut rotation = vec![0; corners];
    for c in 0..corners {
        let oc = mesh.opposite_corner(c);
        if oc == SurfaceMesh::NPOS {
            continue;
        }
        rotation[c] = edge_quarter_turn(mesh, c, field, normals);
    }
    rotation
}

fn compute_corner_constraints(
    mesh: &SurfaceMesh,
    field: &[Vector3],
    normals: &[Vector3],
    hard_edge_degrees: f64,
    sharps: Option<&[Vec<Vector3>]>,
) -> Vec<EdgeConstraint> {
    let corners = mesh.corner_count();
    let mut corner_constraints = vec![EdgeConstraint::ConstraintNone; corners];
    let sharp_set = sharps.filter(|s| !s.is_empty());
    // Corner marks pin integer coordinates, so the radius stays tight
    // (half an edge length): only edges ON the snapped feature line
    // qualify. Anything wider pins rings of edges around every feature
    // and collapses the quad budget on small hard-surface parts.
    let sharp_radius = if sharp_set.is_some() {
        0.5 * mesh.average_edge_length()
    } else {
        0.0
    };
    // Sequential: the C++ TBB loop writes disjoint per-corner slots.
    for c in 0..corners {
        let automatic = edge_constraint(mesh, c, field, normals, hard_edge_degrees);
        if automatic != EdgeConstraint::ConstraintNone {
            corner_constraints[c] = automatic;
            continue;
        }
        if let Some(sharp_polylines) = sharp_set {
            corner_constraints[c] =
                sharp_edge_constraint(mesh, c, field, normals, sharp_polylines, sharp_radius);
        }
    }
    corner_constraints
}

fn apply_directional_swaps(
    mesh: &SurfaceMesh,
    field_before_brush: &[Vector3],
    field: &[Vector3],
    normals: &[Vector3],
    scaling_u: &mut [f64],
    scaling_v: &mut [f64],
) {
    for f in 0..mesh.face_count() {
        let before = unit(field_before_brush[f], field[f]);
        let after = unit(field[f], before);
        let perpendicular = unit(Vector3::cross_product(&normals[f], &after), before);
        if Vector3::dot_product(&before, &after).abs()
            < Vector3::dot_product(&before, &perpendicular).abs()
        {
            std::mem::swap(&mut scaling_u[f], &mut scaling_v[f]);
            // The C++ also bumps a write-only swap counter here; omitted.
        }
    }
}

const QUARTER_TURN: [[[f64; 2]; 2]; 4] = [
    [[1.0, 0.0], [0.0, 1.0]],
    [[0.0, 1.0], [-1.0, 0.0]],
    [[-1.0, 0.0], [0.0, -1.0]],
    [[0.0, -1.0], [1.0, 0.0]],
];

/// One edge's curl-correction linear model: (constant, turn gradient, along
/// gradient, across gradient), each a UV pair. Mirrors the `request` lambda
/// (associated fn returning a tuple instead of writing through `double*`
/// out-params).
type CurlTerms = ([f64; 2], [f64; 2], [f64; 2], [f64; 2]);

#[allow(clippy::too_many_arguments)]
fn curl_request(
    mesh: &SurfaceMesh,
    corner: usize,
    face: usize,
    field: &[Vector3],
    perpendicular: &[Vector3],
    su: &[f64],
    sv: &[f64],
) -> CurlTerms {
    let e = mesh.edge_vector(corner);
    let along = Vector3::dot_product(&field[face], &e);
    let across = Vector3::dot_product(&perpendicular[face], &e);
    (
        [along / su[face], across / sv[face]],
        [across / su[face], -along / sv[face]],
        [-along / su[face], 0.0],
        [0.0, -across / sv[face]],
    )
}

#[allow(clippy::too_many_arguments)]
fn apply_curl_correction(
    mesh: &SurfaceMesh,
    normals: &[Vector3],
    rotation: &[i32],
    corner_constraints: &[EdgeConstraint],
    face_scaling: &[f64],
    scale: f64,
    regularization: f64,
    scaling_u: &mut [f64],
    scaling_v: &mut [f64],
    field: &mut [Vector3],
) {
    let face_count = mesh.face_count();
    let has_face_scaling = face_scaling.len() == face_count;
    let mut su = vec![0.0; face_count];
    let mut sv = vec![0.0; face_count];
    let mut perpendicular = vec![Vector3::default(); face_count];
    for f in 0..face_count {
        let face_scale = if has_face_scaling {
            face_scaling[f].max(1e-12)
        } else {
            1.0
        };
        su[f] = scale * face_scale * scaling_u[f].max(1e-12);
        sv[f] = scale * face_scale * scaling_v[f].max(1e-12);
        perpendicular[f] = unit(
            Vector3::cross_product(&normals[f], &field[f]),
            mesh.edge_vector(3 * f),
        );
    }

    // Anchored faces pin the curl-correction rotation to zero. Automatic
    // dihedral marks and explicit sharp marks arrive through the same
    // cornerConstraints channel, so both anchor identically.
    let mut anchored = vec![false; face_count];
    for c in 0..mesh.corner_count() {
        if corner_constraints[c] != EdgeConstraint::ConstraintNone {
            anchored[mesh.corner_face(c)] = true;
        }
    }

    let mut system = ConstrainedLeastSquares::new(3 * face_count);
    let mut edge_count = 0usize;
    for c in 0..mesh.corner_count() {
        let oc = mesh.opposite_corner(c);
        if oc == SurfaceMesh::NPOS || oc < c {
            continue;
        }
        let f = mesh.corner_face(c);
        let g = mesh.corner_face(oc);
        let r = (((rotation[c] % 4) + 4) % 4) as usize;
        let here = curl_request(mesh, c, f, field, &perpendicular, &su, &sv);
        let across = curl_request(mesh, oc, g, field, &perpendicular, &su, &sv);
        // FMA audit: the C++ fuses this into one fmuladd
        // (`fma(q[k][0], v[0], q[k][1] * v[1])`). Unobservable — the
        // quarter-turn entries are exact +1/-1/0, where fused and unfused
        // round identically — but mirrored exactly anyway.
        let turned = |value: &[f64; 2], k: usize| {
            QUARTER_TURN[r][k][0].mul_add(value[0], QUARTER_TURN[r][k][1] * value[1])
        };
        for k in 0..2 {
            let residual = here.0[k] + turned(&across.0, k);
            system.add_energy(
                &[
                    (3 * f, here.1[k]),
                    (3 * f + 1, here.2[k]),
                    (3 * f + 2, here.3[k]),
                    (3 * g, turned(&across.1, k)),
                    (3 * g + 1, turned(&across.2, k)),
                    (3 * g + 2, turned(&across.3, k)),
                ],
                -residual,
                1.0,
            );
        }
        edge_count += 1;
    }
    if edge_count == 0 {
        return;
    }
    for f in 0..face_count {
        if anchored[f] {
            system.add_constraint(&[(3 * f, 1.0)], 0.0);
        } else {
            system.add_energy(&[(3 * f, 1.0)], 0.0, regularization);
        }
        system.add_energy(&[(3 * f + 1, 1.0)], 0.0, regularization);
        system.add_energy(&[(3 * f + 2, 1.0)], 0.0, regularization);
    }

    let Some(correction) = system.solve() else {
        return;
    };
    if correction.len() != 3 * face_count {
        return;
    }

    let limit = 0.3;
    let scale_limit = 1.5f64.ln();
    for f in 0..face_count {
        let angle = correction[3 * f].min(limit).max(-limit);
        field[f] = unit(
            angle.cos() * field[f] + angle.sin() * perpendicular[f],
            field[f],
        );
        scaling_u[f] *= correction[3 * f + 1]
            .min(scale_limit)
            .max(-scale_limit)
            .exp();
        scaling_v[f] *= correction[3 * f + 2]
            .min(scale_limit)
            .max(-scale_limit)
            .exp();
    }
    let mut area_before = 0.0;
    let mut area_after = 0.0;
    for f in 0..face_count {
        let face_scale = if has_face_scaling {
            face_scaling[f].max(1e-12)
        } else {
            1.0
        };
        let e0 = mesh.edge_vector(3 * f);
        let e2 = mesh.edge_vector(3 * f + 2);
        let area = Vector3::cross_product(&e0, &(-e2)).length();
        area_before += area / (su[f] * sv[f]);
        su[f] = scale * face_scale * scaling_u[f].max(1e-12);
        sv[f] = scale * face_scale * scaling_v[f].max(1e-12);
        area_after += area / (su[f] * sv[f]);
    }
    if area_before > 0.0 && area_after > 0.0 {
        let factor = (area_after / area_before).sqrt();
        for f in 0..face_count {
            scaling_u[f] *= factor;
            scaling_v[f] *= factor;
        }
    }
}

fn compute_seam(mesh: &SurfaceMesh, rotation: &[i32]) -> Vec<bool> {
    let corners = mesh.corner_count();
    let mut inside_ball = vec![false; corners];
    let mut ball_seen = vec![false; mesh.face_count()];
    if mesh.face_count() != 0 {
        let mut q = VecDeque::new();
        q.push_back(0usize);
        ball_seen[0] = true;
        while let Some(f) = q.pop_front() {
            for l in 0..3 {
                let c = 3 * f + l;
                let oc = mesh.opposite_corner(c);
                if oc == SurfaceMesh::NPOS || rotation[c] != 0 {
                    continue;
                }
                let g = mesh.corner_face(oc);
                if !ball_seen[g] {
                    ball_seen[g] = true;
                    q.push_back(g);
                    inside_ball[c] = true;
                    inside_ball[oc] = true;
                }
            }
        }
    }
    let mut seam = vec![false; corners];
    for c in 0..corners {
        seam[c] = mesh.opposite_corner(c) == SurfaceMesh::NPOS || !inside_ball[c];
    }
    let mut border_degree = vec![0usize; mesh.vertex_count()];
    for c in 0..corners {
        if seam[c] {
            border_degree[mesh.corner_vertex(c)] += 1;
        }
    }
    let mut changed = true;
    while changed {
        changed = false;
        for c in 0..corners {
            let oc = mesh.opposite_corner(c);
            if oc == SurfaceMesh::NPOS || !seam[c] || rotation[c] != 0 {
                continue;
            }
            let v0 = mesh.corner_vertex(c);
            if border_degree[v0] != 1 {
                continue;
            }
            let v1 = mesh.corner_vertex(mesh.next_corner(c));
            seam[c] = false;
            seam[oc] = false;
            inside_ball[c] = true;
            inside_ball[oc] = true;
            if border_degree[v0] > 0 {
                border_degree[v0] -= 1;
            }
            if border_degree[v1] > 0 {
                border_degree[v1] -= 1;
            }
            changed = true;
        }
    }
    seam
}

fn add_rotation_constraints(
    s: &mut MixedIntegerLeastSquares,
    ax: usize,
    bx: usize,
    r: i32,
    sign: f64,
) {
    let r = ((r % 4) + 4) % 4;
    if r == 0 {
        s.add_constraint2(ax, 1.0, bx, sign);
        s.add_constraint2(ax + 1, 1.0, bx + 1, sign);
    } else if r == 1 {
        s.add_constraint2(ax, 1.0, bx + 1, sign);
        s.add_constraint2(ax + 1, 1.0, bx, -sign);
    } else if r == 2 {
        s.add_constraint2(ax, 1.0, bx, -sign);
        s.add_constraint2(ax + 1, 1.0, bx + 1, -sign);
    } else {
        s.add_constraint2(ax, 1.0, bx + 1, -sign);
        s.add_constraint2(ax + 1, 1.0, bx, sign);
    }
}

/// Mirrors `solveQuadCover`, returning the solved values (`None` when a
/// rounding iteration fails or the system never converges, exactly where
/// the C++ returns `false`).
fn solve_quad_cover(ctx: &CoverContext, progress: Option<&dyn Fn(f32, &str)>) -> Option<Vec<f64>> {
    let mesh = ctx.mesh;
    let field = ctx.field;
    let normals = ctx.normals;
    let rotation = ctx.rotation;
    let seam = ctx.seam;
    let corner_constraints = ctx.corner_constraints;
    let active_scaling_u = ctx.scaling_u;
    let active_scaling_v = ctx.scaling_v;
    let face_scaling = ctx.face_scaling;
    let scale = ctx.scale;
    let corners = mesh.corner_count();
    let uv_variables = 2 * corners;
    let variables = 2 * uv_variables;
    let report = |fraction: f32, name: &str| {
        if let Some(p) = progress {
            p(fraction, name);
        }
    };
    report(0.0, "Building cover system");
    let mut s = MixedIntegerLeastSquares::new(variables);
    for t in 0..2 * corners {
        s.set_variable_period(uv_variables + t, 2);
    }
    for f in 0..mesh.face_count() {
        let u = field[f];
        let v = unit(
            Vector3::cross_product(&normals[f], &u),
            mesh.edge_vector(3 * f),
        );
        let face_scale = if face_scaling.len() == mesh.face_count() {
            face_scaling[f].max(1e-12)
        } else {
            1.0
        };
        let directional_u = active_scaling_u[f].max(1e-12);
        let directional_v = active_scaling_v[f].max(1e-12);
        let su = scale * face_scale * directional_u;
        let sv = scale * face_scale * directional_v;
        for l in 0..3 {
            let c = 3 * f + l;
            let n = mesh.next_corner(c);
            let e = mesh.edge_vector(c);
            let weight = su * sv;
            s.add_energy2(
                2 * n,
                1.0,
                2 * c,
                -1.0,
                Vector3::dot_product(&u, &e) / su,
                weight,
            );
            s.add_energy2(
                2 * n + 1,
                1.0,
                2 * c + 1,
                -1.0,
                Vector3::dot_product(&v, &e) / sv,
                weight,
            );
        }
    }
    s.add_constraint1(0, 1.0);
    s.add_constraint1(1, 1.0);
    // The C++ keeps a write-only hard-coordinate counter across the loops
    // below; omitted.
    for c in 0..corners {
        let oc = mesh.opposite_corner(c);
        if oc == SurfaceMesh::NPOS {
            continue;
        }
        let tc = uv_variables + 2 * c;
        let toc = uv_variables + 2 * oc;
        let r = rotation[c];
        if seam[c] {
            add_rotation_constraints(&mut s, tc, toc, r, 1.0);
        } else {
            s.add_constraint1(tc, 1.0);
            s.add_constraint1(tc + 1, 1.0);
        }
    }
    for c in 0..corners {
        let oc = mesh.opposite_corner(c);
        if oc == SurfaceMesh::NPOS {
            continue;
        }
        let other = mesh.next_corner(oc);
        let tc = uv_variables + 2 * c;
        let r = ((rotation[c] % 4) + 4) % 4;
        if r == 0 {
            s.add_constraint3(2 * c, 1.0, 2 * other, -1.0, tc, -1.0);
            s.add_constraint3(2 * c + 1, 1.0, 2 * other + 1, -1.0, tc + 1, -1.0);
        } else if r == 1 {
            s.add_constraint3(2 * c, 1.0, 2 * other + 1, -1.0, tc, -1.0);
            s.add_constraint3(2 * c + 1, 1.0, 2 * other, 1.0, tc + 1, -1.0);
        } else if r == 2 {
            s.add_constraint3(2 * c, 1.0, 2 * other, 1.0, tc, -1.0);
            s.add_constraint3(2 * c + 1, 1.0, 2 * other + 1, 1.0, tc + 1, -1.0);
        } else {
            s.add_constraint3(2 * c, 1.0, 2 * other + 1, 1.0, tc, -1.0);
            s.add_constraint3(2 * c + 1, 1.0, 2 * other, -1.0, tc + 1, -1.0);
        }
    }
    for vertex in 0..mesh.vertex_count() {
        let incident = mesh.corners_around_vertex(vertex);
        if incident.is_empty() {
            continue;
        }
        let start = incident[0];
        let mut c = start;
        let mut accumulated = 0i32;
        let mut closed = true;
        let mut wheel: Vec<(usize, i32)> = Vec::new();
        loop {
            if mesh.opposite_corner(c) == SurfaceMesh::NPOS {
                closed = false;
                break;
            }
            wheel.push((c, accumulated));
            accumulated = (accumulated + rotation[c]) % 4;
            c = mesh.next_corner(mesh.opposite_corner(c));
            if !(c != start && wheel.len() <= incident.len() + 1) {
                break;
            }
        }
        if !closed || c != start || accumulated != 0 {
            continue;
        }
        for coord in 0..2 {
            let mut row: Vec<(usize, f64)> = Vec::new();
            for &(corner, r) in &wheel {
                let t = uv_variables + 2 * corner;
                if coord == 0 {
                    if r == 0 {
                        row.push((t, 1.0));
                    } else if r == 1 {
                        row.push((t + 1, 1.0));
                    } else if r == 2 {
                        row.push((t, -1.0));
                    } else {
                        row.push((t + 1, -1.0));
                    }
                } else if r == 0 {
                    row.push((t + 1, 1.0));
                } else if r == 1 {
                    row.push((t, -1.0));
                } else if r == 2 {
                    row.push((t + 1, -1.0));
                } else {
                    row.push((t, 1.0));
                }
            }
            s.add_constraint(&row);
        }
    }
    for c in 0..corners {
        let n = mesh.next_corner(c);
        match corner_constraints[c] {
            EdgeConstraint::ConstraintV => {
                s.set_variable_period(2 * c + 1, 1);
                s.set_variable_period(2 * n + 1, 1);
                s.add_constraint2(2 * c + 1, 1.0, 2 * n + 1, -1.0);
            }
            EdgeConstraint::ConstraintU => {
                s.set_variable_period(2 * c, 1);
                s.set_variable_period(2 * n, 1);
                s.add_constraint2(2 * c, 1.0, 2 * n, -1.0);
            }
            EdgeConstraint::ConstraintNone => {}
        }
    }
    report(0.2, "Eliminating cover constraints");
    s.finalize_constraints();
    // Rounding usually converges after a couple of passes, well short of the
    // cap, so spread the fraction over the passes it is expected to take and
    // clamp instead of pacing it against the cap and barely moving.
    const MAXIMUM_ITERATIONS: usize = 100;
    for iteration in 0..MAXIMUM_ITERATIONS {
        // FMA audit: the C++ fuses this into one f32 fmuladd
        // (`fma(0.65 * t, 0.3)`); the `min` bounds `iteration / 4` at 1.
        let t = (iteration as f32 / 4.0).min(1.0);
        report(0.65f32.mul_add(t, 0.3f32), "Rounding cover to integers");
        if !s.solve_iteration() {
            return None;
        }
        if s.converged() {
            break;
        }
    }
    report(0.98, "Rounding cover to integers");
    let mut values = Vec::with_capacity(variables);
    for i in 0..variables {
        values.push(s.value(i));
    }
    if s.converged() { Some(values) } else { None }
}

/// Mirrors `buildResultUv`, returning the UV table and singular vertices
/// instead of writing through the result out-param.
fn build_result_uv(
    mesh: &SurfaceMesh,
    rotation: &[i32],
    all_values: &[f64],
    uv_variables: usize,
) -> (Vec<Vec<Vector2>>, Vec<usize>) {
    let mut uv = all_values[..uv_variables].to_vec();
    for coordinate in uv.iter_mut() {
        let integer = coordinate.round();
        if (*coordinate - integer).abs() < 0.01 {
            *coordinate = integer;
        }
    }
    let mut singular_vertices = Vec::new();
    for vertex in 0..mesh.vertex_count() {
        let fan = mesh.corners_around_vertex(vertex);
        let mut sum = 0i32;
        let mut boundary = false;
        for &c in fan.iter() {
            sum = (sum + rotation[c]) % 4;
            if mesh.opposite_corner(c) == SurfaceMesh::NPOS {
                boundary = true;
            }
        }
        if !boundary && sum != 0 {
            singular_vertices.push(vertex);
        }
    }
    let mut triangle_uvs = vec![vec![Vector2::default(); 3]; mesh.face_count()];
    for f in 0..mesh.face_count() {
        for l in 0..3 {
            let c = 3 * f + l;
            triangle_uvs[f][l] = Vector2::new(uv[2 * c], uv[2 * c + 1]);
        }
    }
    (triangle_uvs, singular_vertices)
}

/// Solved quad parameterization (mirrors `QuadParameterizer::Result`).
#[derive(Clone, Debug)]
pub struct ParameterizeResult {
    /// Per-triangle corner UVs, one `[u, v]` triple per face.
    pub triangle_uvs: Vec<Vec<Vector2>>,
    /// Smoothed cross-field direction per face.
    pub field: Vec<Vector3>,
    /// Quarter-turn rotation per corner (0..3).
    pub corner_rotations: Vec<i32>,
    /// Interior vertices with a nonzero rotation sum.
    pub singular_vertices: Vec<usize>,
}

/// Quad cover parameterizer (mirrors `AutoRemesher::QuadParameterizer`).
pub struct QuadParameterizer;

impl QuadParameterizer {
    /// Mirrors `QuadParameterizer::parameterize`: `None` exactly where the
    /// C++ returns `false` (empty input, non-positive scaling, dropped
    /// triangles, or a cover solve that fails to converge).
    #[allow(clippy::too_many_arguments)]
    pub fn parameterize(
        vertices: &[Vector3],
        triangles: &[Vec<usize>],
        guidance: &[Vector3],
        scaling: f64,
        hard_edge_degrees: f64,
        face_scaling: &[f64],
        face_scaling_u: &[f64],
        face_scaling_v: &[f64],
        progress_handler: Option<&ProgressHandler>,
        sharps: Option<&[Vec<Vector3>]>,
    ) -> Option<ParameterizeResult> {
        let progress = |fraction: f32, name: &str| {
            if let Some(p) = progress_handler {
                p(fraction, name);
            }
        };

        if vertices.is_empty() || triangles.is_empty() || scaling <= 0.0 {
            return None;
        }
        progress(0.0, "Initializing cover field");
        let mesh = SurfaceMesh::new(vertices, triangles);
        if mesh.face_count() != triangles.len() {
            return None;
        }
        let corners = mesh.corner_count();
        let uv_variables = 2 * corners;
        let scale = (scaling * mesh.average_edge_length()).max(1e-12);

        let mut normals = Vec::new();
        let mut field = Vec::new();
        initialize_field_and_normals(&mesh, guidance, &mut normals, &mut field);
        let field_before_brush = field.clone();

        let mut active_scaling_u = vec![1.0; mesh.face_count()];
        let mut active_scaling_v = vec![1.0; mesh.face_count()];
        let track_directional_scale =
            face_scaling_u.len() == mesh.face_count() && face_scaling_v.len() == mesh.face_count();
        if track_directional_scale {
            active_scaling_u.copy_from_slice(face_scaling_u);
            active_scaling_v.copy_from_slice(face_scaling_v);
        }

        progress(0.04, "Smoothing cross field");
        if guidance.len() != mesh.face_count() {
            smooth_cross_field(&mesh, &normals, hard_edge_degrees, &mut field);
        }
        brush_field_along_spanning_tree(&mesh, &normals, &mut field);

        progress(0.14, "Computing corner rotations");
        let rotation = compute_corner_rotations(&mesh, &field, &normals);
        let corner_constraints =
            compute_corner_constraints(&mesh, &field, &normals, hard_edge_degrees, sharps);
        if track_directional_scale {
            apply_directional_swaps(
                &mesh,
                &field_before_brush,
                &field,
                &normals,
                &mut active_scaling_u,
                &mut active_scaling_v,
            );
        }
        progress(0.20, "Correcting field curl");
        apply_curl_correction(
            &mesh,
            &normals,
            &rotation,
            &corner_constraints,
            face_scaling,
            scale,
            1e-4,
            &mut active_scaling_u,
            &mut active_scaling_v,
            &mut field,
        );
        let seam = compute_seam(&mesh, &rotation);

        let ctx = CoverContext {
            mesh: &mesh,
            field: &field,
            normals: &normals,
            rotation: &rotation,
            seam: &seam,
            corner_constraints: &corner_constraints,
            scaling_u: &active_scaling_u,
            scaling_v: &active_scaling_v,
            face_scaling,
            scale,
        };

        // The cover solve reports on its own 0..1, so remap it into the tail of this
        // function's range and keep the fractions monotonic end to end.
        // Restructure note: the C++ re-wraps the handler in a second
        // `std::function`; the Rust `ProgressHandler` box is `'static` and
        // cannot capture the caller's handler reference, so the remap stays
        // a plain closure passed as `Option<&dyn Fn>`.
        let cover_progress = |fraction: f32, name: &str| {
            if let Some(p) = progress_handler {
                // FMA audit: the C++ fuses this into one f32 fmuladd.
                p(0.63f32.mul_add(fraction, 0.35f32), name);
            }
        };
        let cover_progress_ref: Option<&dyn Fn(f32, &str)> = if progress_handler.is_some() {
            Some(&cover_progress)
        } else {
            None
        };
        let all_values = solve_quad_cover(&ctx, cover_progress_ref)?;

        progress(0.99, "Building cover uvs");
        let (triangle_uvs, singular_vertices) =
            build_result_uv(&mesh, &rotation, &all_values, uv_variables);
        Some(ParameterizeResult {
            triangle_uvs,
            field,
            corner_rotations: rotation,
            singular_vertices,
        })
    }
}
