//! Port of `core/vector2.*` (`retopo.core.vector2`).
//!
//! Line-by-line mirror: same methods in the same order, same `is_zero`
//! epsilon semantics from [`crate::double_utils`].
//!
//! FMA transcription: Clang fuses every `p*q+r*s` site in this header
//! (verified in `-O3` IR, brew Clang 23, ARM64 — see the FMA audit in the
//! wave-2 notes), always as `fma(p, q, r*s)` with the FIRST product fused
//! and the second computed separately. Every such site below uses explicit
//! [`f64::mul_add`] so the port is bit-identical, including inside the
//! Eigen 4x4 determinant replicated for [`Vector2::is_in_circle`].

use crate::double_utils::{is_equal, is_zero};
use std::ops::{Add, Index, IndexMut, Mul, Sub};

/// 2D double vector (mirrors `AutoRemesher::Vector2`).
#[derive(Clone, Copy, Debug, Default)]
pub struct Vector2 {
    data: [f64; 2],
}

impl Vector2 {
    #[inline]
    #[must_use]
    pub fn new(x: f64, y: f64) -> Self {
        Self { data: [x, y] }
    }

    #[inline]
    #[must_use]
    pub fn x(&self) -> f64 {
        self.data[0]
    }

    #[inline]
    #[must_use]
    pub fn y(&self) -> f64 {
        self.data[1]
    }

    #[inline]
    pub fn set_x(&mut self, x: f64) {
        self.data[0] = x;
    }

    #[inline]
    pub fn set_y(&mut self, y: f64) {
        self.data[1] = y;
    }

    /// Backing array (mirrors `data()`, which hands out a raw pointer;
    /// no downstream port needs pointer arithmetic).
    #[inline]
    #[must_use]
    pub fn as_array(&self) -> &[f64; 2] {
        &self.data
    }

    #[inline]
    pub fn set_data(&mut self, x: f64, y: f64) {
        self.data[0] = x;
        self.data[1] = y;
    }

    #[inline]
    #[must_use]
    pub fn length_squared(&self) -> f64 {
        self.data[0].mul_add(self.data[0], self.data[1] * self.data[1])
    }

    #[inline]
    #[must_use]
    pub fn length(&self) -> f64 {
        self.length_squared().sqrt()
    }

    #[inline]
    #[must_use]
    pub fn normalized(&self) -> Self {
        let length = self.length_squared().sqrt();
        if is_zero(length) {
            return Self::default();
        }
        Self::new(self.data[0] / length, self.data[1] / length)
    }

    #[inline]
    pub fn normalize(&mut self) {
        let length = self.length_squared().sqrt();
        if is_zero(length) {
            return;
        }
        self.data[0] /= length;
        self.data[1] /= length;
    }

    /// In-circle test via a 4x4 determinant (mirrors `isInCircle`, which
    /// builds an `Eigen::Matrix4d` and tests `determinant() > 0`).
    ///
    /// Transcription notes: Eigen's comma initializer fills COLUMN-major,
    /// so column j holds point j as `(x, y, |p|^2, 1)`; the determinant
    /// below replicates `determinant_impl<Derived, 4>::run` operation for
    /// operation, with [`f64::mul_add`] at every `pmadd` and fused
    /// `|p|^2`, exactly as the `-O3` IR fuses them. The `1.0` constants
    /// in the last column are kept inside the FMAs: `fma(-1, d, p)`
    /// rounds once where `-d + p` rounds twice.
    #[inline]
    #[must_use]
    pub fn is_in_circle(&self, a: &Self, b: &Self, c: &Self) -> bool {
        // Columns: col0 = a, col1 = b, col2 = c, col3 = self.
        let a2 = a.data[0].mul_add(a.data[0], a.data[1] * a.data[1]);
        let b2 = b.data[0].mul_add(b.data[0], b.data[1] * b.data[1]);
        let c2 = c.data[0].mul_add(c.data[0], c.data[1] * c.data[1]);
        let p2 = self.data[0].mul_add(self.data[0], self.data[1] * self.data[1]);
        // m[(row, col)]: rows are (x, y, |p|^2, 1) down each point column.
        let m = [
            [a.data[0], b.data[0], c.data[0], self.data[0]],
            [a.data[1], b.data[1], c.data[1], self.data[1]],
            [a2, b2, c2, p2],
            [1.0, 1.0, 1.0, 1.0],
        ];
        // det2(m, i0, i1) = m(i0,0)*m(i1,1) - m(i1,0)*m(i0,1).
        let det2 = |i0: usize, i1: usize| m[i0][0].mul_add(m[i1][1], m[i1][0] * -m[i0][1]);
        let d2_01 = det2(0, 1);
        let d2_02 = det2(0, 2);
        let d2_03 = det2(0, 3);
        let d2_12 = det2(1, 2);
        let d2_13 = det2(1, 3);
        let d2_23 = det2(2, 3);
        // det3 = pmadd(m(i0,2), d0, pmadd(-m(i1,2), d1, m(i2,2)*d2)).
        let det3 = |i0: usize, d0: f64, i1: usize, d1: f64, i2: usize, d2: f64| {
            m[i0][2].mul_add(d0, (-m[i1][2]).mul_add(d1, m[i2][2] * d2))
        };
        let d3_0 = det3(1, d2_23, 2, d2_13, 3, d2_12);
        let d3_1 = det3(0, d2_23, 2, d2_03, 3, d2_02);
        let d3_2 = det3(0, d2_13, 1, d2_03, 3, d2_01);
        let d3_3 = det3(0, d2_12, 1, d2_02, 2, d2_01);
        let det =
            (-m[0][3]).mul_add(d3_0, m[1][3] * d3_1) + (-m[2][3]).mul_add(d3_2, m[3][3] * d3_3);
        det > 0.0
    }

    #[inline]
    #[must_use]
    pub fn is_on_left(&self, a: &Self, b: &Self) -> bool {
        (b.data[0] - a.data[0]).mul_add(
            self.data[1] - a.data[1],
            (b.data[1] - a.data[1]) * -(self.data[0] - a.data[0]),
        ) > 0.0
    }

    #[inline]
    #[must_use]
    pub fn dot_product(a: &Self, b: &Self) -> f64 {
        a.data[0].mul_add(b.data[0], a.data[1] * b.data[1])
    }

    /// Barycentric `(alpha, beta)` with `v0 = c - a` (mirrors the C++
    /// argument order exactly: alpha weights toward c, beta toward b).
    #[inline]
    #[must_use]
    pub fn barycentric_coordinates(a: &Self, b: &Self, c: &Self, point: &Self) -> Self {
        let v0 = *c - *a;
        let v1 = *b - *a;
        let v2 = *point - *a;
        let dot00 = Self::dot_product(&v0, &v0);
        let dot01 = Self::dot_product(&v0, &v1);
        let dot02 = Self::dot_product(&v0, &v2);
        let dot11 = Self::dot_product(&v1, &v1);
        let dot12 = Self::dot_product(&v1, &v2);
        let inv_denom = 1.0 / dot00.mul_add(dot11, dot01 * -dot01);
        let alpha = dot11.mul_add(dot02, dot01 * -dot12) * inv_denom;
        let beta = dot00.mul_add(dot12, dot01 * -dot02) * inv_denom;
        Self::new(alpha, beta)
    }

    #[inline]
    #[must_use]
    pub fn is_in_triangle(a: &Self, b: &Self, c: &Self, point: &Self) -> bool {
        let ab = Self::barycentric_coordinates(a, b, c, point);
        if ab.data[0] < 0.0 {
            return false;
        }
        if ab.data[1] < 0.0 {
            return false;
        }
        if 1.0 - (ab.data[0] + ab.data[1]) < 0.0 {
            return false;
        }
        true
    }
}

impl Index<usize> for Vector2 {
    type Output = f64;
    #[inline]
    fn index(&self, index: usize) -> &f64 {
        &self.data[index]
    }
}

impl IndexMut<usize> for Vector2 {
    #[inline]
    fn index_mut(&mut self, index: usize) -> &mut f64 {
        &mut self.data[index]
    }
}

impl Add for Vector2 {
    type Output = Self;
    #[inline]
    fn add(self, other: Self) -> Self {
        Self::new(self.data[0] + other.data[0], self.data[1] + other.data[1])
    }
}

impl Sub for Vector2 {
    type Output = Self;
    #[inline]
    fn sub(self, other: Self) -> Self {
        Self::new(self.data[0] - other.data[0], self.data[1] - other.data[1])
    }
}

impl Mul<f64> for Vector2 {
    type Output = Self;
    #[inline]
    fn mul(self, number: f64) -> Self {
        // Note the C++ order: `number * v.x()` even for `v * number`;
        // multiplication commutes, so this is bit-identical either way.
        Self::new(number * self.data[0], number * self.data[1])
    }
}

impl Mul<Vector2> for f64 {
    type Output = Vector2;
    #[inline]
    fn mul(self, v: Vector2) -> Vector2 {
        Vector2::new(self * v.data[0], self * v.data[1])
    }
}

/// Epsilon equality (mirrors `operator==`/`operator!=`, which compare
/// through `Double::isEqual`).
impl PartialEq for Vector2 {
    #[inline]
    fn eq(&self, other: &Self) -> bool {
        is_equal(self.data[0], other.data[0]) && is_equal(self.data[1], other.data[1])
    }
}

/// `std::to_string(double)` formatting: fixed 6 decimals (`%f`), with
/// signless lowercase `nan` (Rust prints `NaN`/`-NaN`; macOS libc
/// prints every NaN as `nan`).
#[must_use]
pub fn to_string(v: &Vector2) -> String {
    fn coord(x: f64) -> String {
        // macOS libc prints every NaN signless ("nan"), unlike glibc's
        // "-nan": probed, not assumed.
        if x.is_nan() {
            return "nan".to_string();
        }
        format!("{x:.6}")
    }
    format!("{},{}", coord(v.x()), coord(v.y()))
}

impl std::fmt::Display for Vector2 {
    /// Shortest-round-trip `x,y`.
    ///
    /// Known difference: C++ `operator<<` uses ostream defaults (`%g`
    /// with 6 significant digits). Nothing downstream formats vectors,
    /// so this keeps Rust's exact default instead of emulating `%g`.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{},{}", self.data[0], self.data[1])
    }
}
