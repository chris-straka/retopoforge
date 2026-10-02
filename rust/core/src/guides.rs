//! Port of `core/guides.*` (`retopo.core.guides`).
//!
//! Line-by-line mirror of `AutoRemesher::Guides`: same items in the same
//! order, same thresholds (`1e-12` degenerate segments, `0.5` projected
//! tangent length, `6.0` influence factor).
//!
//! FMA audit (Release IR, `llvm.fmuladd`, per
//! `docs/rust-port-conventions.md`): `Guides::tangentNear` contains ten
//! fused instructions, all inside inlined `Vector3::length` /
//! `lengthSquared` / `dotProduct`, whose fusion the FMA-exact
//! [`crate::vector3`] port already transcribes. The two expression sites
//! of its own — the projection `polyline[i] + direction * along` and the
//! tangent `bestDirection - dot * normal` — lower to separate
//! `fmul`+`fadd`/`fsub` pairs, so plain operators match bitwise and this
//! module needs no `mul_add`.
//!
//! The `std::min`/`std::max` clamp uses [`cxx_min`]/[`cxx_max`], which
//! replicate the `(b < a) ? b : a` / `(a < b) ? b : a` NaN propagation
//! (the IR lowers the clamp to the same two `select`s;
//! `f64::min`/`f64::max` return the non-NaN operand instead, which
//! diverges when `length` is NaN).

use crate::surface_mesh::SurfaceMesh;
use crate::vector3::Vector3;

/// Guide-curve queries (mirrors `AutoRemesher::Guides`: a stateless
/// namespace class, so every item is associated).
pub struct Guides;

impl Guides {
    /// Influence radius for guide constraints on this mesh: faces and edges
    /// within this distance of a guide polyline follow the guide tangent.
    #[must_use]
    pub fn influence_radius(mesh: &SurfaceMesh) -> f64 {
        // Six edge lengths: wide enough that the quad cover realizes the guided
        // flow inside the region instead of compromising it away, narrow enough
        // to stay local to the drawn curve.
        6.0 * mesh.average_edge_length()
    }

    /// Nearest guide segment tangent at `point`, projected onto the tangent
    /// plane of `normal` and normalized. Returns the zero vector when no
    /// guide segment passes within `radius` of `point`, when every nearby
    /// segment is degenerate, or when the nearest tangent runs into the
    /// surface (more than 60 degrees out of the tangent plane) and so
    /// carries no flow direction for it.
    #[must_use]
    #[allow(clippy::neg_cmp_op_on_partial_ord)] // Port mirrors the C++ negated comparison; `!(a<b)` differs from `a>=b` on NaN.
    pub fn tangent_near(
        guides: &[Vec<Vector3>],
        point: &Vector3,
        normal: &Vector3,
        radius: f64,
    ) -> Vector3 {
        if !(radius > 0.0) {
            return Vector3::default();
        }
        let radius_squared = radius * radius;
        let mut best_direction = Vector3::default();
        let mut best_distance_squared = radius_squared;
        let mut found = false;
        for polyline in guides {
            // Mirrors `for (size_t i = 0; i + 1 < polyline.size(); ++i)`.
            for i in 0..polyline.len().saturating_sub(1) {
                let delta = polyline[i + 1] - polyline[i];
                let length = delta.length();
                if length <= 1e-12 {
                    continue;
                }
                let direction = delta / length;
                let along = cxx_max(
                    0.0,
                    cxx_min(
                        length,
                        Vector3::dot_product(&(*point - polyline[i]), &direction),
                    ),
                );
                let distance_squared =
                    (*point - (polyline[i] + direction * along)).length_squared();
                if distance_squared < best_distance_squared {
                    best_distance_squared = distance_squared;
                    best_direction = direction;
                    found = true;
                }
            }
        }
        if !found {
            return Vector3::default();
        }
        let tangent = best_direction - Vector3::dot_product(&best_direction, normal) * *normal;
        if tangent.length() <= 0.5 {
            return Vector3::default();
        }
        tangent.normalized()
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
