use crate::vector2::Vector2;
use crate::vector3::Vector3;

// ---------------------------------------------------------------------------
// Caller-level vector-expression helpers: each mirrors one C++ expression
// shape through the crate operators (see the module FMA note: the vector
// shapes are UNFUSED on both sides, so the operator expression is already
// bitwise). Scalar mul-add shapes use `fma_first`/`fma_first_sub`/inline
// `mul_add` (those DO fuse in the C++).
// ---------------------------------------------------------------------------

/// `a * (1 - ratio) + b * ratio` via the crate operators (unfused, like
/// the C++; `1 - ratio` with the C++ int literal equals `1.0 - ratio`).
#[inline]
pub(crate) fn lerp_vec3(a: Vector3, b: Vector3, ratio: f64) -> Vector3 {
    a * (1.0 - ratio) + b * ratio
}

/// `Vector2` form of [`lerp_vec3`].
#[inline]
pub(crate) fn lerp_vec2(a: Vector2, b: Vector2, ratio: f64) -> Vector2 {
    a * (1.0 - ratio) + b * ratio
}

/// `v + d * s` via the crate operators (unfused, like the C++).
#[inline]
pub(crate) fn add_scaled_vec3(v: Vector3, d: Vector3, s: f64) -> Vector3 {
    v + d * s
}

/// `v - d * s` via the crate operators (unfused, like the C++).
#[inline]
pub(crate) fn sub_scaled_vec3(v: Vector3, d: Vector3, s: f64) -> Vector3 {
    v - d * s
}

/// `a + ab * v + ac * w` (left-nested, as written) via the crate
/// operators (unfused, like the C++).
#[inline]
pub(crate) fn add_two_scaled_vec3(a: Vector3, ab: Vector3, v: f64, ac: Vector3, w: f64) -> Vector3 {
    a + ab * v + ac * w
}

/// `p * b + q * d` with the FIRST product fused: `fma(p, b, q * d)`.
#[inline]
pub(crate) fn fma_first(p: f64, b: f64, q: f64, d: f64) -> f64 {
    p.mul_add(b, q * d)
}

/// `p * b - q * d` as the IR fuses it: `fma(p, b, q * -d)`.
#[inline]
pub(crate) fn fma_first_sub(p: f64, b: f64, q: f64, d: f64) -> f64 {
    p.mul_add(b, q * -d)
}
