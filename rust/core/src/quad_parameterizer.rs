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
//!
//! Sincos audit (Release machine code, `___sincos_stret` relocs): all four
//! adjacent `cos`/`sin` pairs fuse (cpp:179-180, 201 unrolled x3, 219,
//! 402), each mirrored with [`crate::double_utils::joint_sin_cos`] below.
//! No other `sin`/`cos` pair exists in the TU (the `acos`/`exp` calls are
//! lone and unfused).

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
    /// Explicit-sharp (`--features`) marks: alignment only. The cover
    /// keeps the UV equality (the crisp line) but skips the integer
    /// period and the curl anchor. Full U/V pins on every marked edge
    /// over-constrain the cover: a 12-edge box cage pins 912 integer
    /// variables + 456 equalities + ~456 anchors, collapsing yield
    /// (561 -> 444 quads) and tripling irregulars (5.3 -> 15.2%).
    /// Alignment without positional locking recovers fully (see
    /// `docs/corner-marks.md`). Automatic dihedral marks keep U/V.
    AlignU,
    AlignV,
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
// of a segment). The field-alignment check below is identical, but the
// mark is alignment-only (`AlignU`/`AlignV`): the cover keeps the UV
// equality without the integer period or the curl anchor (see the
// `EdgeConstraint` docs). Returns None when no sharp passes nearby.
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
        EdgeConstraint::AlignV
    } else {
        EdgeConstraint::AlignU
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
            // Fused `sincos` in C++ (cpp:179-180, reloc 0x626c): joint call.
            let (sn, cs) = crate::double_utils::joint_sin_cos(4.0 * field_angle);
            alpha[2 * face_index] = cs;
            alpha[2 * face_index + 1] = sn;
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
                // Fused `sincos` in C++ (cpp:201, relocs 0x6ffc/0x7128/0x7254): joint call.
                let (sn, cs) = crate::double_utils::joint_sin_cos(4.0 * d);
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
        // Fused `sincos` in C++ (cpp:219, reloc 0x7dac): joint call.
        let (sn, cs) = crate::double_utils::joint_sin_cos(field_angle);
        field[face_index] = unit(field_direction * cs + perpendicular * sn, field_direction);
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
    // Full corner marks pin integer coordinates, so the explicit-mark
    // radius stays tight (half an edge length): only edges ON the
    // snapped feature line qualify. Anything wider pins rings of edges
    // around every feature and collapses the quad budget on small
    // hard-surface parts. (Explicit marks are alignment-only since the
    // cage fix — see the `EdgeConstraint` docs — but the tight radius
    // still keeps the alignment band on the line.)
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

    // Anchored faces pin the curl-correction rotation to zero. Only
    // full (automatic dihedral) marks anchor: explicit-sharp alignment
    // marks arrive through the same channel but must not freeze the
    // rotation (cage over-constraint — see the `EdgeConstraint` docs).
    let mut anchored = vec![false; face_count];
    for c in 0..mesh.corner_count() {
        if matches!(
            corner_constraints[c],
            EdgeConstraint::ConstraintU | EdgeConstraint::ConstraintV
        ) {
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
        // Fused `sincos` in C++ (cpp:402, reloc 0x1de4): joint call.
        let (sn, cs) = crate::double_utils::joint_sin_cos(angle);
        field[f] = unit(cs * field[f] + sn * perpendicular[f], field[f]);
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

// ===== Density-boundary dipole insertion (production) =====
// Mechanism (docs/density-poles-spike.md, validated in
// docs/dipole-mechanism-spike.md): solve_quad_cover skips the
// wheel-constraint rows at vertices with nonzero corner-rotation sum, so a
// ring of dipole (+1/-1) singularities along a sharp density step lifts the
// transition-flux conservation that makes sizing-gradient jumps infeasible
// in the continuous solution.
//
// Placement runs AFTER curl correction (field/sizing stay identical to a
// no-dipole run) and BEFORE seam computation + cover solve, so the only
// downstream change is cover topology. The flip op preserves matching
// antisymmetry mod 4 and moves exactly +1/-1 onto the two edge endpoints
// (unit-tested below). Keyed off the RAW per-vertex density multipliers
// (sharp mask steps only): disabled, uniform, or mild configs are silent
// no-ops, so unmasked runs stay bit-identical with dipoles enabled.
//
// `RETOPO_DIPOLE_DEBUG` (research only): stderr diagnostics — the
// placement summary here plus the per-iteration gradient probe below.
// Never touches state; unset = silent.
fn dipole_debug_enabled() -> bool {
    std::env::var_os("RETOPO_DIPOLE_DEBUG").is_some()
}

/// Density step sharpness (face-key ratio) above which automatic
/// placement fires. For a vertex step of ask A the sharpest adjacent
/// face pair is a 1-dense-vert straddle face against a pure coarse face:
/// max pair ratio = (A+2)/3, i.e. 4x -> 2.0, 3x -> 1.67, 2x -> 1.33.
/// The 1.5 gate therefore fires on asks >= ~3x and skips mild (<= 2x)
/// masks, which realize nearly fully without dipoles — and it keeps
/// the differential oracles' 2x density cases parity-clean.
pub const DIPOLE_AUTO_RATIO: f64 = 1.5;

/// Islands with fewer working-mesh faces are skipped: they span < 1 UV
/// cell under the global scaling, and dipoles there only explode the
/// sliver gradient (spike observation on 20-face pole fans).
pub const DIPOLE_MIN_ISLAND_FACES: usize = 64;

/// Rings with fewer step-crossing edges are specks (mask noise), not
/// boundaries: skipped.
pub const DIPOLE_MIN_RING_EDGES: usize = 4;

/// Where the flip pair sits relative to the density step.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DipolePlacement {
    /// Production rule (see `insert_dipoles`): offset rings everywhere
    /// (measured best on the finger fixtures; `OnBoundary` stays an
    /// explicit override).
    Auto,
    /// Flip the step-crossing edge itself (+1 on the denser-side
    /// endpoint): the validated isolated-boundary placement.
    OnBoundary,
    /// Flip a dense-side edge one ring in (+1 on the denser endpoint):
    /// for steps shared with a neighboring region, where the
    /// on-boundary -1 would refine both sides. Both poles land
    /// dense-side (the densest vertex of the crossing's denser face
    /// plus its densest above-mid neighbor). Offset-or-nothing (no
    /// on-boundary fallback): thin masks with no interior edge place
    /// nothing.
    Offset,
}

/// Dipole-insertion configuration (density-boundary singularity rings).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DipoleConfig {
    /// Master switch. Off is byte-identical to the no-dipole path (early
    /// return before any density work).
    pub enabled: bool,
    /// Dose stride over each ring's disjoint candidates: place every
    /// k-th. 0 = automatic (the per-ring line-ending estimate).
    pub every: usize,
    /// Step sharpness gate (max/min face-key ratio). <= 0.0 = automatic
    /// (`DIPOLE_AUTO_RATIO`).
    pub ratio: f64,
    /// Flip placement relative to the step.
    pub placement: DipolePlacement,
}

impl DipoleConfig {
    /// Dipoles off: `parameterize` is byte-identical to the no-dipole
    /// path (pins C++ parity in the differential oracles).
    #[must_use]
    pub const fn off() -> Self {
        Self {
            enabled: false,
            every: 0,
            ratio: 0.0,
            placement: DipolePlacement::Auto,
        }
    }

    /// Automatic placement: ring detection off raw density steps, dose
    /// from the per-ring line-ending estimate, production placement rule.
    #[must_use]
    pub const fn automatic() -> Self {
        Self {
            enabled: true,
            every: 0,
            ratio: 0.0,
            placement: DipolePlacement::Auto,
        }
    }
}

/// Per-vertex corner-rotation sums (same fold as build_result_uv).
fn dipole_vertex_sums(mesh: &SurfaceMesh, rotation: &[i32]) -> Vec<i32> {
    let mut sums = vec![0i32; mesh.vertex_count()];
    for v in 0..mesh.vertex_count() {
        let mut sum = 0i32;
        for &c in mesh.corners_around_vertex(v) {
            sum = (sum + rotation[c]) % 4;
        }
        sums[v] = sum;
    }
    sums
}

fn dipole_is_interior(mesh: &SurfaceMesh, v: usize) -> bool {
    mesh.corners_around_vertex(v)
        .iter()
        .all(|&c| mesh.opposite_corner(c) != SurfaceMesh::NPOS)
}

/// Flip the matching across one interior edge: +1 at corner_vertex(c),
/// -1 at corner_vertex(opposite(c)). Antisymmetry is preserved mod 4 and
/// values stay in 0..3 (several folds below use non-normalizing `% 4`).
/// Returns the (+1, -1) endpoints.
fn dipole_flip_edge(mesh: &SurfaceMesh, rotation: &mut [i32], c: usize) -> (usize, usize) {
    let oc = mesh.opposite_corner(c);
    debug_assert!(oc != SurfaceMesh::NPOS);
    rotation[c] = (rotation[c] + 1) % 4;
    rotation[oc] = (rotation[oc] + 3) % 4;
    (mesh.corner_vertex(c), mesh.corner_vertex(oc))
}

/// Insert dipole pairs along sharp density steps by flipping corner
/// rotations. Runs AFTER curl correction (field/sizing stay identical to
/// a no-dipole run) and BEFORE seam computation + cover solve, so the
/// only change downstream is cover topology (wheel skips, seams,
/// singularities).
///
/// Pipeline: raw per-vertex density multipliers -> per-face mean keys ->
/// sharp fans mark boundary vertices -> connected boundary components
/// are rings -> per ring, the greedy disjoint step-crossing edges (the
/// validated radial selection) strided down to the dose. Dose: explicit
/// `every` stride, else min(disjoint count, line-ending estimate
/// `n_cross * (sqrt(ring_ask) - 1)`). Offset placement flips one ring
/// into the dense side (see `DipolePlacement`); sub-threshold steps,
/// specks, and sliver islands place nothing. Returns the flip count (0
/// when disabled, unmasked, or mild).
fn insert_dipoles(
    mesh: &SurfaceMesh,
    rotation: &mut [i32],
    density_field: &[f64],
    config: DipoleConfig,
) -> usize {
    if !config.enabled {
        return 0;
    }
    // Density-only by design: no mask (or a uniform mask) => silent
    // no-op, so unmasked runs stay bit-identical with dipoles enabled.
    if density_field.len() != mesh.vertex_count() || !density_field.iter().any(|&d| d != 1.0) {
        return 0;
    }
    // Sliver-island guard (see DIPOLE_MIN_ISLAND_FACES).
    if mesh.face_count() < DIPOLE_MIN_ISLAND_FACES {
        return 0;
    }
    let ratio = if config.ratio > 0.0 {
        config.ratio
    } else {
        DIPOLE_AUTO_RATIO
    };
    // Boundary signal: per-face mean DENSITY multiplier. Dense ==
    // LARGER multiplier; only ratios matter below.
    let face_key: Vec<f64> = (0..mesh.face_count())
        .map(|f| {
            (density_field[mesh.corner_vertex(3 * f)]
                + density_field[mesh.corner_vertex(3 * f + 1)]
                + density_field[mesh.corner_vertex(3 * f + 2)])
                / 3.0
        })
        .collect();
    let mut glo = f64::INFINITY;
    let mut ghi = 0.0f64;
    for &m in &face_key {
        glo = glo.min(m);
        ghi = ghi.max(m);
    }
    // No sharp step anywhere: mild masks realize without dipoles.
    if !(glo > 0.0) || ghi / glo <= ratio {
        return 0;
    }
    let mid = (glo * ghi).sqrt();
    // Boundary vertices: interior verts whose incident face keys span
    // more than `ratio`. No mid-straddle requirement (the spike had
    // one): multi-step masks ring every step, and on single-step masks
    // every qualifying fan straddles the mid anyway.
    let mut boundary = vec![false; mesh.vertex_count()];
    for v in 0..mesh.vertex_count() {
        if !dipole_is_interior(mesh, v) {
            continue;
        }
        let mut lo = f64::INFINITY;
        let mut hi = 0.0f64;
        for &c in mesh.corners_around_vertex(v) {
            let m = face_key[mesh.corner_face(c)];
            lo = lo.min(m);
            hi = hi.max(m);
        }
        if lo > 0.0 && hi / lo > ratio {
            boundary[v] = true;
        }
    }
    let mut sums = dipole_vertex_sums(mesh, rotation);
    let n_sing_before = sums.iter().filter(|&&s| s != 0).count();
    // One corner sweep: boundary-boundary adjacency (for ring detection)
    // plus the step-crossing edges (flip candidates).
    let mut neighbors: Vec<Vec<usize>> = vec![Vec::new(); mesh.vertex_count()];
    let mut crossings: Vec<(usize, usize)> = Vec::new();
    for c in 0..mesh.corner_count() {
        let oc = mesh.opposite_corner(c);
        if oc == SurfaceMesh::NPOS || oc < c {
            continue;
        }
        let u = mesh.corner_vertex(c);
        let w = mesh.corner_vertex(oc);
        if boundary[u] && boundary[w] {
            neighbors[u].push(w);
            neighbors[w].push(u);
        }
        let mf = face_key[mesh.corner_face(c)];
        let mg = face_key[mesh.corner_face(oc)];
        if mf <= 0.0 || mg <= 0.0 || mf.max(mg) / mf.min(mg) <= ratio {
            continue;
        }
        crossings.push((c, oc));
    }
    for n in neighbors.iter_mut() {
        n.sort_unstable();
        n.dedup();
    }
    // Rings: connected boundary-vertex components (no leaf pruning:
    // spurs don't hurt radial placement; each component doses
    // independently).
    let mut comp = vec![usize::MAX; mesh.vertex_count()];
    let mut n_rings = 0usize;
    for v in 0..mesh.vertex_count() {
        if !boundary[v] || comp[v] != usize::MAX {
            continue;
        }
        let mut stack = vec![v];
        comp[v] = n_rings;
        while let Some(x) = stack.pop() {
            for &n in &neighbors[x] {
                if boundary[n] && comp[n] == usize::MAX {
                    comp[n] = n_rings;
                    stack.push(n);
                }
            }
        }
        n_rings += 1;
    }
    // Assign crossings to rings by endpoint membership; crossings with
    // no boundary endpoint are border specks (open-mesh rims whose
    // endpoints fail the interior test): dropped.
    let mut ring_crossings: Vec<Vec<(usize, usize)>> = vec![Vec::new(); n_rings];
    for &(c, oc) in &crossings {
        let u = mesh.corner_vertex(c);
        let w = mesh.corner_vertex(oc);
        let id = if comp[u] != usize::MAX {
            comp[u]
        } else if comp[w] != usize::MAX {
            comp[w]
        } else {
            continue;
        };
        ring_crossings[id].push((c, oc));
    }
    // Per-ring ask (dense-face mean / coarse-face mean over faces
    // incident to ring boundary verts) for the line-ending estimate.
    // Faces are counted once per incident ring vert; means are unaffected.
    let mut ring_ask = vec![1.0f64; n_rings];
    {
        let mut dense_sum = vec![0.0f64; n_rings];
        let mut dense_n = vec![0usize; n_rings];
        let mut coarse_sum = vec![0.0f64; n_rings];
        let mut coarse_n = vec![0usize; n_rings];
        for v in 0..mesh.vertex_count() {
            if comp[v] == usize::MAX {
                continue;
            }
            for &k in mesh.corners_around_vertex(v) {
                let m = face_key[mesh.corner_face(k)];
                if m > mid {
                    dense_sum[comp[v]] += m;
                    dense_n[comp[v]] += 1;
                } else {
                    coarse_sum[comp[v]] += m;
                    coarse_n[comp[v]] += 1;
                }
            }
        }
        for id in 0..n_rings {
            if dense_n[id] > 0 && coarse_n[id] > 0 {
                ring_ask[id] =
                    (dense_sum[id] / dense_n[id] as f64) / (coarse_sum[id] / coarse_n[id] as f64);
            }
        }
    }
    let fan_mean = |v: usize| {
        let fan = mesh.corners_around_vertex(v);
        fan.iter()
            .map(|&k| face_key[mesh.corner_face(k)])
            .sum::<f64>()
            / fan.len().max(1) as f64
    };
    // Neighbor vertices of v (sorted, deduped, deterministic).
    let neighbors_of = |v: usize| {
        let mut out = Vec::new();
        for &k in mesh.corners_around_vertex(v) {
            out.push(mesh.corner_vertex(mesh.next_corner(k)));
            out.push(mesh.corner_vertex(mesh.next_corner(mesh.next_corner(k))));
        }
        out.sort_unstable();
        out.dedup();
        out.retain(|&y| y != v);
        out
    };
    // Production placement rule: offset everywhere. Measured on the
    // finger fixtures (docs/dipole-production.md): offset beats or ties
    // on-boundary on 5 of 6 sharp rows and fixes the one harm case
    // (fused 3x: 1.02 -> 1.39 faceAbs); both poles land dense-side, so
    // shared steps cannot leak refinement across.
    let auto_offset = true;
    let mut used = vec![false; mesh.vertex_count()];
    let mut flips: Vec<(usize, usize)> = Vec::new();
    for (id, edges) in ring_crossings.iter().enumerate() {
        // Speck guard: a real ring has many crossings.
        if edges.len() < DIPOLE_MIN_RING_EDGES {
            continue;
        }
        // Greedy maximal disjoint candidates in corner-index order (the
        // validated radial selection), scratched against a `used` copy:
        // striding a disjoint list keeps it disjoint, and the copy is
        // committed ring by ring through the real flips below.
        let mut probe_used = used.clone();
        // (flip corner, +1 endpoint, -1 endpoint)
        let mut maximal: Vec<(usize, usize, usize)> = Vec::new();
        for &(c, oc) in edges {
            let u = mesh.corner_vertex(c);
            let w = mesh.corner_vertex(oc);
            let place_offset = match config.placement {
                DipolePlacement::OnBoundary => false,
                DipolePlacement::Offset => true,
                DipolePlacement::Auto => auto_offset,
            };
            if !place_offset {
                if probe_used[u] || probe_used[w] || sums[u] != 0 || sums[w] != 0 {
                    continue;
                }
                // +1 on the denser-side endpoint (larger mean multiplier).
                let (plus, minus) = if (fan_mean(u), u) >= (fan_mean(w), w) {
                    (u, w)
                } else {
                    (w, u)
                };
                let cc = if plus == u { c } else { oc };
                probe_used[plus] = true;
                probe_used[minus] = true;
                maximal.push((cc, plus, minus));
                continue;
            }
            // Offset: flip a dense-side edge one ring in. D is the
            // densest vertex of the crossing pair's denser face; x2 is
            // D's densest neighbor with fan-mean above mid. Both poles
            // land dense-side, so shared (septum) steps can't leak
            // refinement across. Offset-or-nothing: no on-boundary
            // fallback (the crossing endpoints stay untouched here).
            let mf = face_key[mesh.corner_face(c)];
            let mg = face_key[mesh.corner_face(oc)];
            let dense_face = if mf > mg {
                mesh.corner_face(c)
            } else {
                mesh.corner_face(oc)
            };
            let mut d_best: Option<(f64, usize)> = None;
            for l in 0..3 {
                let y = mesh.corner_vertex(3 * dense_face + l);
                let d = density_field[y];
                if d_best.map_or(true, |(bd, bi)| (d, y) > (bd, bi)) {
                    d_best = Some((d, y));
                }
            }
            let (_, d_vert) = d_best.expect("faces always have 3 vertices");
            if probe_used[d_vert] || sums[d_vert] != 0 {
                continue;
            }
            let mut x_best: Option<(f64, usize)> = None;
            for y in neighbors_of(d_vert) {
                if probe_used[y] || sums[y] != 0 || fan_mean(y) <= mid {
                    continue;
                }
                let m = fan_mean(y);
                if x_best.map_or(true, |(bm, bi)| (m, y) > (bm, bi)) {
                    x_best = Some((m, y));
                }
            }
            let Some((_, x2)) = x_best else {
                continue;
            };
            let (plus, minus) = if (fan_mean(d_vert), d_vert) >= (fan_mean(x2), x2) {
                (d_vert, x2)
            } else {
                (x2, d_vert)
            };
            // Flip corner on edge (d_vert, x2) with corner_vertex == plus.
            let mut cc = SurfaceMesh::NPOS;
            for &k in mesh.corners_around_vertex(plus) {
                let ko = mesh.opposite_corner(k);
                if ko != SurfaceMesh::NPOS && mesh.corner_vertex(ko) == minus {
                    cc = k;
                    break;
                }
            }
            if cc == SurfaceMesh::NPOS {
                continue;
            }
            probe_used[plus] = true;
            probe_used[minus] = true;
            maximal.push((cc, plus, minus));
        }
        if maximal.is_empty() {
            continue;
        }
        // Dose: explicit stride (every k-th), else the line-ending
        // estimate capped at the disjoint count, spread evenly
        // (deterministic). Strided subsets of a disjoint list stay
        // disjoint.
        let picks: Vec<usize> = if config.every > 0 {
            (0..maximal.len()).step_by(config.every).collect()
        } else {
            let est = (edges.len() as f64 * (ring_ask[id].sqrt() - 1.0)).round() as usize;
            let dose = est.clamp(1, maximal.len());
            (0..dose).map(|i| (i * maximal.len()) / dose).collect()
        };
        for i in picks {
            let (cc, a, b) = maximal[i];
            debug_assert!(!used[a] && !used[b]);
            let (pa, pb) = dipole_flip_edge(mesh, rotation, cc);
            debug_assert!(pa == a && pb == b);
            sums[pa] = (sums[pa] + 1) % 4;
            sums[pb] = (sums[pb] + 3) % 4;
            used[pa] = true;
            used[pb] = true;
            flips.push((pa, pb));
        }
    }
    if dipole_debug_enabled() {
        // Flip-y extent + histogram over the mesh height (placement check).
        let n_boundary = boundary.iter().filter(|&&b| b).count();
        let (mut ylo, mut yhi) = (f64::INFINITY, f64::NEG_INFINITY);
        let (mut mylo, mut myhi) = (f64::INFINITY, f64::NEG_INFINITY);
        for v in 0..mesh.vertex_count() {
            mylo = mylo.min(mesh.position(v).y());
            myhi = myhi.max(mesh.position(v).y());
        }
        let mut hist = [0usize; 8];
        for &(a, b) in &flips {
            for &v in &[a, b] {
                let y = mesh.position(v).y();
                ylo = ylo.min(y);
                yhi = yhi.max(y);
                let bin = (((y - mylo) / (myhi - mylo).max(1e-12) * 8.0) as usize).min(7);
                hist[bin] += 1;
            }
        }
        eprintln!(
            "DIPOLEDBG rings={n_rings} boundary_verts={n_boundary} flips={} sing_before={n_sing_before} sing_after={} flip_y=[{ylo:.3},{yhi:.3}] yhist={hist:?}",
            flips.len(),
            sums.iter().filter(|&&s| s != 0).count(),
        );
    }
    flips.len()
}

/// Per-iteration UV-gradient probe (research readout): mean uv-lines per
/// world unit over dense faces (fs <= geometric mid) vs coarse faces.
/// Iteration 0 solves purely continuous (nothing fixed yet); iteration 1
/// rounds + fixes all integers. Stderr only, no state touched.
/// RETOPO_DIPOLE_DEBUG=y<val> additionally splits faces by centroid
/// height (mask-region bands comparable across plain/masked runs).
fn dipole_log_cover_gradients(
    s: &MixedIntegerLeastSquares,
    mesh: &SurfaceMesh,
    face_scaling: &[f64],
    iteration: usize,
) {
    if !dipole_debug_enabled() || face_scaling.len() != mesh.face_count() {
        return;
    }
    let mut lo = f64::INFINITY;
    let mut hi = 0.0f64;
    for &m in face_scaling {
        lo = lo.min(m);
        hi = hi.max(m);
    }
    if !(lo > 0.0 && hi > lo) {
        return;
    }
    let mid = (lo * hi).sqrt();
    let dbgvar = std::env::var("RETOPO_DIPOLE_DEBUG").unwrap_or_default();
    // y-part is the text between 'y' and 'x' (if an x-cut follows).
    let ypart = dbgvar.split('x').next().unwrap_or("");
    let ycut: Option<f64> = ypart
        .split('y')
        .nth(1)
        .map(|t| t.parse().unwrap_or(f64::NAN))
        .filter(|t| t.is_finite());
    // Optional x-cut (RETOPO_DIPOLE_DEBUG=y2.2x0): splits the yhi band
    // into masked/control lobes for split/fused-style masks.
    let xcut: Option<f64> = dbgvar
        .split('x')
        .nth(1)
        .map(|t| t.parse().unwrap_or(f64::NAN))
        .filter(|t| t.is_finite());
    let (mut dg, mut dn, mut cg, mut cn) = (0.0, 0usize, 0.0, 0usize);
    let (mut yg, mut yn, mut ng, mut nn) = (0.0, 0usize, 0.0, 0usize);
    let (mut xg, mut xn, mut zg, mut zn) = (0.0, 0usize, 0.0, 0usize);
    for f in 0..mesh.face_count() {
        let mut g = 0.0;
        for l in 0..3 {
            let c = 3 * f + l;
            let n = mesh.next_corner(c);
            let du = s.value(2 * n) - s.value(2 * c);
            let dv = s.value(2 * n + 1) - s.value(2 * c + 1);
            let len = mesh.edge_vector(c).length().max(1e-12);
            g += du.hypot(dv) / len;
        }
        g /= 3.0;
        if face_scaling[f] <= mid {
            dg += g;
            dn += 1;
        } else {
            cg += g;
            cn += 1;
        }
        if let Some(cut) = ycut {
            let (mut cx, mut cy) = (0.0, 0.0);
            for l in 0..3 {
                let p = mesh.position(mesh.corner_vertex(3 * f + l));
                cx += p.x();
                cy += p.y();
            }
            if cy / 3.0 > cut {
                yg += g;
                yn += 1;
                if let Some(xc) = xcut {
                    if cx / 3.0 > xc {
                        xg += g;
                        xn += 1;
                    } else {
                        zg += g;
                        zn += 1;
                    }
                }
            } else {
                ng += g;
                nn += 1;
            }
        }
    }
    eprintln!(
        "DIPOLEDBG cover-iter={iteration} dense-lpu={:.3} (n={dn}) coarse-lpu={:.3} (n={cn})",
        dg / dn.max(1) as f64,
        cg / cn.max(1) as f64,
    );
    if ycut.is_some() {
        eprintln!(
            "DIPOLEDBG cover-iter={iteration} yhi-lpu={:.3} (n={yn}) ylo-lpu={:.3} (n={nn})",
            yg / yn.max(1) as f64,
            ng / nn.max(1) as f64,
        );
    }
    if ycut.is_some() && xcut.is_some() {
        eprintln!(
            "DIPOLEDBG cover-iter={iteration} masked-lpu={:.3} (n={xn}) control-lpu={:.3} (n={zn})",
            xg / xn.max(1) as f64,
            zg / zn.max(1) as f64,
        );
    }
}
// ===== END dipole insertion =====

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
            // Alignment-only explicit marks: the equality without the
            // integer periods (cage over-constraint — see the
            // `EdgeConstraint` docs).
            EdgeConstraint::AlignV => {
                s.add_constraint2(2 * c + 1, 1.0, 2 * n + 1, -1.0);
            }
            EdgeConstraint::AlignU => {
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
        // Research-only stderr probe (RETOPO_DIPOLE_DEBUG), no state touched.
        dipole_log_cover_gradients(&s, mesh, face_scaling, iteration);
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
    /// Dipole edge flips applied along density steps (0 unless the
    /// `dipoles` config enabled placement and a sharp step was found).
    pub dipole_flips: usize,
}

/// Quad cover parameterizer (mirrors `AutoRemesher::QuadParameterizer`).
pub struct QuadParameterizer;

impl QuadParameterizer {
    /// Mirrors `QuadParameterizer::parameterize`: `None` exactly where the
    /// C++ returns `false` (empty input, non-positive scaling, dropped
    /// triangles, or a cover solve that fails to converge).
    /// Trailing `density_field` (per-vertex multipliers, empty when
    /// unmasked) plus `dipoles` feed only the dipole insertion; the C++
    /// has neither param. `DipoleConfig::off()` (or an empty/uniform
    /// field) is byte-identical to the no-dipole path.
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
        density_field: &[f64],
        dipoles: DipoleConfig,
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
        // `mut` only for the dipole insertion below (no-op unless configured).
        let mut rotation = compute_corner_rotations(&mesh, &field, &normals);
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
        // Dipole insertion after curl correction, before seam + cover
        // solve (no-op unless configured and a sharp density step exists).
        let dipole_flips = insert_dipoles(&mesh, &mut rotation, density_field, dipoles);
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
            dipole_flips,
        })
    }
}

// Dipole insertion tests: flip-op lattice consistency (antisymmetry +
// charge conservation) plus placement gating (off/uniform/mild/sliver
// no-ops, sharp-step placement, stride dose, disjointness).
#[cfg(test)]
mod dipole_tests {
    use super::*;

    fn tetra_mesh() -> SurfaceMesh {
        let vertices = vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
            Vector3::new(0.0, 0.0, 1.0),
        ];
        // Consistently oriented (outward) closed tetrahedron.
        let triangles = vec![vec![0, 2, 1], vec![0, 1, 3], vec![0, 3, 2], vec![1, 2, 3]];
        let mesh = SurfaceMesh::new(&vertices, &triangles);
        assert_eq!(mesh.face_count(), 4);
        for c in 0..mesh.corner_count() {
            assert_ne!(
                mesh.opposite_corner(c),
                SurfaceMesh::NPOS,
                "tetra must be closed"
            );
        }
        mesh
    }

    fn assert_antisymmetric(mesh: &SurfaceMesh, rotation: &[i32]) {
        for c in 0..mesh.corner_count() {
            let oc = mesh.opposite_corner(c);
            assert!((0..4).contains(&rotation[c]), "rotation in 0..3");
            if oc == SurfaceMesh::NPOS {
                continue; // open-mesh rim: no opposite corner
            }
            assert_eq!(
                (rotation[c] + rotation[oc]).rem_euclid(4),
                0,
                "matching antisymmetric mod 4 across edge {c}/{oc}"
            );
        }
    }

    #[test]
    fn dipole_flip_preserves_lattice() {
        let mesh = tetra_mesh();
        // Start from the flat (all-zero) matching: valid and antisymmetric.
        let mut rotation = vec![0; mesh.corner_count()];
        assert_antisymmetric(&mesh, &rotation);
        let c = 0;
        let oc = mesh.opposite_corner(c);
        let (u, w) = dipole_flip_edge(&mesh, &mut rotation, c);
        assert_ne!(u, w, "flip endpoints are the two edge vertices");
        assert_eq!(u, mesh.corner_vertex(c));
        assert_eq!(w, mesh.corner_vertex(oc));
        assert_antisymmetric(&mesh, &rotation);
        // Exactly the two endpoints change, by +1/-1 (mod 4).
        let sums = dipole_vertex_sums(&mesh, &rotation);
        for v in 0..mesh.vertex_count() {
            let expected = if v == u {
                1
            } else if v == w {
                3
            } else {
                0
            };
            assert_eq!(sums[v], expected, "vertex {v} sum");
        }
        let total: i32 = sums.iter().sum();
        assert_eq!(total.rem_euclid(4), 0, "total charge conserved mod 4");
    }

    #[test]
    fn dipole_flip_on_computed_rotations() {
        let mesh = tetra_mesh();
        let normals = vec![Vector3::new(0.0, 0.0, 1.0); mesh.face_count()];
        let field = vec![Vector3::new(1.0, 0.0, 0.0); mesh.face_count()];
        let mut rotation = compute_corner_rotations(&mesh, &field, &normals);
        assert_antisymmetric(&mesh, &rotation);
        let before = dipole_vertex_sums(&mesh, &rotation);
        let c = 5;
        let oc = mesh.opposite_corner(c);
        let (u, w) = dipole_flip_edge(&mesh, &mut rotation, c);
        assert_antisymmetric(&mesh, &rotation);
        let after = dipole_vertex_sums(&mesh, &rotation);
        for v in 0..mesh.vertex_count() {
            let expected = if v == u {
                (before[v] + 1).rem_euclid(4)
            } else if v == w {
                (before[v] + 3).rem_euclid(4)
            } else {
                before[v]
            };
            assert_eq!(after[v], expected, "vertex {v} sum");
        }
        assert_eq!(mesh.corner_vertex(oc), w);
    }

    // 10x10 flat grid (162 faces, consistent winding), all rotations flat.
    fn grid_mesh() -> SurfaceMesh {
        let mut vertices = Vec::new();
        for j in 0..10 {
            for i in 0..10 {
                vertices.push(Vector3::new(i as f64, j as f64, 0.0));
            }
        }
        let mut triangles = Vec::new();
        for j in 0..9 {
            for i in 0..9 {
                let a = j * 10 + i;
                triangles.push(vec![a, a + 1, a + 11]);
                triangles.push(vec![a, a + 11, a + 10]);
            }
        }
        let mesh = SurfaceMesh::new(&vertices, &triangles);
        assert_eq!(mesh.face_count(), 162);
        mesh
    }

    // Sharp step at x = 4.5 (verts with i < 5 dense).
    fn step_density(mesh: &SurfaceMesh, ask: f64) -> Vec<f64> {
        (0..mesh.vertex_count())
            .map(|v| if mesh.position(v).x() < 4.5 { ask } else { 1.0 })
            .collect()
    }

    // Vertices whose corner-rotation sums changed (the flip endpoints).
    fn changed_verts(mesh: &SurfaceMesh, before: &[i32], after: &[i32]) -> Vec<usize> {
        let sb = dipole_vertex_sums(mesh, before);
        let sa = dipole_vertex_sums(mesh, after);
        (0..mesh.vertex_count())
            .filter(|&v| sb[v] != sa[v])
            .collect()
    }

    #[test]
    fn dipole_off_is_noop() {
        let mesh = grid_mesh();
        let density = step_density(&mesh, 4.0);
        let mut rotation = vec![0; mesh.corner_count()];
        let before = rotation.clone();
        assert_eq!(
            insert_dipoles(&mesh, &mut rotation, &density, DipoleConfig::off()),
            0
        );
        assert_eq!(rotation, before);
    }

    #[test]
    fn dipole_skips_uniform_and_mild() {
        let mesh = grid_mesh();
        let mut rotation = vec![0; mesh.corner_count()];
        let before = rotation.clone();
        // Uniform mask.
        let uniform = vec![1.0; mesh.vertex_count()];
        assert_eq!(
            insert_dipoles(&mesh, &mut rotation, &uniform, DipoleConfig::automatic()),
            0
        );
        // Mild 2x step: below the auto sharpness gate.
        let mild = step_density(&mesh, 2.0);
        assert_eq!(
            insert_dipoles(&mesh, &mut rotation, &mild, DipoleConfig::automatic()),
            0
        );
        assert_eq!(rotation, before, "no-op runs must not touch rotation");
    }

    #[test]
    fn dipole_onboundary_places_disjoint_ring() {
        let mesh = grid_mesh();
        let density = step_density(&mesh, 4.0);
        let cfg = DipoleConfig {
            placement: DipolePlacement::OnBoundary,
            ..DipoleConfig::automatic()
        };
        let mut rotation = vec![0; mesh.corner_count()];
        let flips = insert_dipoles(&mesh, &mut rotation, &density, cfg);
        assert!(flips > 1, "sharp step must place a ring, got {flips}");
        assert_antisymmetric(&mesh, &rotation);
        // Disjoint endpoints: every changed vert in exactly one flip.
        let changed = changed_verts(&mesh, &vec![0; mesh.corner_count()], &rotation);
        assert_eq!(changed.len(), 2 * flips, "endpoints must be disjoint");
        // On-boundary placement straddles the step.
        assert!(
            changed.iter().any(|&v| mesh.position(v).x() >= 4.5),
            "on-boundary ring must touch the coarse side"
        );
    }

    #[test]
    fn dipole_auto_resolves_to_offset() {
        // Pins the production rule: Auto must behave exactly like
        // explicit Offset (change this test consciously with the rule).
        let mesh = grid_mesh();
        let density = step_density(&mesh, 4.0);
        let off = DipoleConfig {
            placement: DipolePlacement::Offset,
            ..DipoleConfig::automatic()
        };
        let mut ra = vec![0; mesh.corner_count()];
        let mut ro = vec![0; mesh.corner_count()];
        let fa = insert_dipoles(&mesh, &mut ra, &density, DipoleConfig::automatic());
        let fo = insert_dipoles(&mesh, &mut ro, &density, off);
        assert!(fa > 0);
        assert_eq!(fa, fo);
        assert_eq!(ra, ro);
    }

    #[test]
    fn dipole_explicit_stride_thins_dose() {
        let mesh = grid_mesh();
        let density = step_density(&mesh, 4.0);
        let base = DipoleConfig {
            placement: DipolePlacement::OnBoundary,
            ..DipoleConfig::automatic()
        };
        let mut full = vec![0; mesh.corner_count()];
        let auto = insert_dipoles(&mesh, &mut full, &density, base);
        assert!(auto > 1);
        let one = DipoleConfig { every: 1, ..base };
        let mut r1 = vec![0; mesh.corner_count()];
        assert_eq!(
            insert_dipoles(&mesh, &mut r1, &density, one),
            auto,
            "every=1 places the full disjoint set"
        );
        let thin = DipoleConfig {
            every: 1000,
            ..base
        };
        let mut r2 = vec![0; mesh.corner_count()];
        assert_eq!(
            insert_dipoles(&mesh, &mut r2, &density, thin),
            1,
            "huge stride places exactly one flip"
        );
        // Sharpness override: an absurd ratio gates everything off.
        let strict = DipoleConfig {
            ratio: 100.0,
            ..base
        };
        let mut r3 = vec![0; mesh.corner_count()];
        assert_eq!(insert_dipoles(&mesh, &mut r3, &density, strict), 0);
    }

    #[test]
    fn dipole_offset_stays_dense_side() {
        let mesh = grid_mesh();
        let density = step_density(&mesh, 4.0);
        let off = DipoleConfig {
            placement: DipolePlacement::Offset,
            ..DipoleConfig::automatic()
        };
        let mut rotation = vec![0; mesh.corner_count()];
        let flips = insert_dipoles(&mesh, &mut rotation, &density, off);
        assert!(flips > 0, "offset must place interior flips");
        assert_antisymmetric(&mesh, &rotation);
        let changed = changed_verts(&mesh, &vec![0; mesh.corner_count()], &rotation);
        assert_eq!(changed.len(), 2 * flips);
        assert!(
            changed.iter().all(|&v| mesh.position(v).x() < 4.5),
            "offset endpoints must all sit on the dense side"
        );
    }

    #[test]
    fn dipole_skips_sliver_islands() {
        // Tetrahedron (4 faces): the sliver guard zeros it even with a
        // ratio override that passes the sharpness gate.
        let mesh = tetra_mesh();
        let density = vec![4.0, 4.0, 1.0, 1.0];
        let loose = DipoleConfig {
            ratio: 1.2,
            ..DipoleConfig::automatic()
        };
        let mut rotation = vec![0; mesh.corner_count()];
        assert_eq!(insert_dipoles(&mesh, &mut rotation, &density, loose), 0);
        // Same override on a real island places (the guard is what zeros
        // the tetra, not the override).
        let grid = grid_mesh();
        let gden = step_density(&grid, 4.0);
        let mut gr = vec![0; grid.corner_count()];
        assert!(insert_dipoles(&grid, &mut gr, &gden, loose) > 0);
    }
}
