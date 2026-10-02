//! Port of `core/vector3.*` (`retopo.core.vector3`).
//!
//! Line-by-line mirror: same methods in the same order, same `is_zero`
//! epsilon semantics from [`crate::double_utils`].
//!
//! FMA transcription: Clang fuses every `p*q+r*s` site in this header
//! (verified in `-O3` IR, brew Clang 23, ARM64 — see the FMA audit in the
//! wave-2 notes), always as `fma(p, q, r*s)` with the FIRST product fused
//! and the second computed separately. Every such site below uses explicit
//! [`f64::mul_add`] so the port is bit-identical.

use crate::double_utils::is_zero;
use crate::vector2::Vector2;
use std::cmp::Ordering;
use std::ops::{Add, AddAssign, Div, DivAssign, Index, IndexMut, Mul, MulAssign, Neg, Sub};

/// 3D double vector (mirrors `AutoRemesher::Vector3`).
#[derive(Clone, Copy, Debug, Default)]
pub struct Vector3 {
    data: [f64; 3],
}

impl Vector3 {
    #[inline]
    #[must_use]
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { data: [x, y, z] }
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
    #[must_use]
    pub fn z(&self) -> f64 {
        self.data[2]
    }

    #[inline]
    pub fn set_x(&mut self, x: f64) {
        self.data[0] = x;
    }

    #[inline]
    pub fn set_y(&mut self, y: f64) {
        self.data[1] = y;
    }

    #[inline]
    pub fn set_z(&mut self, z: f64) {
        self.data[2] = z;
    }

    /// Backing array (mirrors `constData()`, which hands out a raw
    /// pointer; no downstream port needs pointer arithmetic).
    #[inline]
    #[must_use]
    pub fn as_array(&self) -> &[f64; 3] {
        &self.data
    }

    #[inline]
    pub fn set_data(&mut self, x: f64, y: f64, z: f64) {
        self.data[0] = x;
        self.data[1] = y;
        self.data[2] = z;
    }

    /// Left-nested FMA chain, exactly as the IR fuses
    /// `x*x + y*y + z*z`: `fma(z, z, fma(x, x, y*y))`.
    #[inline]
    #[must_use]
    pub fn length_squared(&self) -> f64 {
        self.data[2].mul_add(
            self.data[2],
            self.data[0].mul_add(self.data[0], self.data[1] * self.data[1]),
        )
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
        Self::new(
            self.data[0] / length,
            self.data[1] / length,
            self.data[2] / length,
        )
    }

    #[inline]
    pub fn normalize(&mut self) {
        let length = self.length_squared().sqrt();
        if is_zero(length) {
            return;
        }
        self.data[0] /= length;
        self.data[1] /= length;
        self.data[2] /= length;
    }

    #[inline]
    #[must_use]
    pub fn cross_product(a: &Self, b: &Self) -> Self {
        Self::new(
            a.data[1].mul_add(b.data[2], a.data[2] * -b.data[1]),
            a.data[2].mul_add(b.data[0], a.data[0] * -b.data[2]),
            a.data[0].mul_add(b.data[1], a.data[1] * -b.data[0]),
        )
    }

    #[inline]
    #[must_use]
    pub fn dot_product(a: &Self, b: &Self) -> f64 {
        a.data[2].mul_add(
            b.data[2],
            a.data[0].mul_add(b.data[0], a.data[1] * b.data[1]),
        )
    }

    #[inline]
    #[must_use]
    pub fn normal(a: &Self, b: &Self, c: &Self) -> Self {
        let ba_x = b.data[0] - a.data[0];
        let ba_y = b.data[1] - a.data[1];
        let ba_z = b.data[2] - a.data[2];
        let ca_x = c.data[0] - a.data[0];
        let ca_y = c.data[1] - a.data[1];
        let ca_z = c.data[2] - a.data[2];
        let cross_x = ba_y.mul_add(ca_z, ba_z * -ca_y);
        let cross_y = ba_z.mul_add(ca_x, ba_x * -ca_z);
        let cross_z = ba_x.mul_add(ca_y, ba_y * -ca_x);
        let length = cross_z
            .mul_add(cross_z, cross_x.mul_add(cross_x, cross_y * cross_y))
            .sqrt();
        if is_zero(length) {
            return Self::default();
        }
        Self::new(cross_x / length, cross_y / length, cross_z / length)
    }

    #[inline]
    #[must_use]
    pub fn angle(a: &Self, b: &Self) -> f64 {
        let dot = Self::dot_product(&a.normalized(), &b.normalized());
        if dot <= -1.0 {
            std::f64::consts::PI
        } else if dot >= 1.0 {
            0.0
        } else {
            dot.acos()
        }
    }

    #[inline]
    #[must_use]
    pub fn is_zero(&self) -> bool {
        is_zero(self.data[0]) && is_zero(self.data[1]) && is_zero(self.data[2])
    }

    /// Lexicographic `<` (mirrors `operator<` exactly, including the
    /// fall-through on NaN: `NaN < x` and `NaN > x` are both false, so a
    /// NaN component moves the comparison to the next axis).
    #[inline]
    #[must_use]
    pub fn less_than(&self, other: &Self) -> bool {
        if self.data[0] < other.data[0] {
            return true;
        }
        if self.data[0] > other.data[0] {
            return false;
        }
        if self.data[1] < other.data[1] {
            return true;
        }
        if self.data[1] > other.data[1] {
            return false;
        }
        if self.data[2] < other.data[2] {
            return true;
        }
        if self.data[2] > other.data[2] {
            return false;
        }
        false
    }

    #[inline]
    #[must_use]
    pub fn area(a: &Self, b: &Self, c: &Self) -> f64 {
        let ab = *b - *a;
        let ac = *c - *a;
        0.5 * Self::cross_product(&ab, &ac).length()
    }

    /// Projection onto `(axis, normal x axis)` as 2D points (mirrors the
    /// `vector<Vector2>` overload of `project`; `origin` has no default
    /// in Rust, pass `Vector3::default()` explicitly).
    pub fn project_to_2d(
        points_in_3d: &[Self],
        points_in_2d: &mut Vec<Vector2>,
        normal: &Self,
        axis: &Self,
        origin: &Self,
    ) {
        let perpendicular_axis = Self::cross_product(normal, axis);
        for it in points_in_3d {
            let direction = *it - *origin;
            points_in_2d.push(Vector2::new(
                Self::dot_product(&direction, axis),
                Self::dot_product(&direction, &perpendicular_axis),
            ));
        }
    }

    /// Projection as 3D points with `z = 0` (mirrors the
    /// `vector<Vector3>` overload).
    pub fn project_to_3d(
        points_in_3d: &[Self],
        points_out: &mut Vec<Self>,
        normal: &Self,
        axis: &Self,
        origin: &Self,
    ) {
        let perpendicular_axis = Self::cross_product(normal, axis);
        for it in points_in_3d {
            let direction = *it - *origin;
            points_out.push(Self::new(
                Self::dot_product(&direction, axis),
                Self::dot_product(&direction, &perpendicular_axis),
                0.0,
            ));
        }
    }

    #[inline]
    #[must_use]
    pub fn barycentric_coordinates(a: &Self, b: &Self, c: &Self, point: &Self) -> Self {
        let inverted_area_of_abc = 1.0 / Self::area(a, b, c);
        let area_of_pbc = Self::area(point, b, c);
        let area_of_pca = Self::area(point, c, a);
        let alpha = area_of_pbc * inverted_area_of_abc;
        let beta = area_of_pca * inverted_area_of_abc;
        Self::new(alpha, beta, 1.0 - alpha - beta)
    }
}

/// Bitwise equality. There is no C++ counterpart (`operator==` does not
/// exist for Vector3); this exists only because Rust's `PartialOrd`
/// requires `PartialEq`, and it deliberately invents no epsilon.
impl PartialEq for Vector3 {
    #[inline]
    fn eq(&self, other: &Self) -> bool {
        self.data[0] == other.data[0]
            && self.data[1] == other.data[1]
            && self.data[2] == other.data[2]
    }
}

/// Lexicographic order with C++ `<` semantics for the `<` operator.
/// (C++ defines only `<`, no `==`: NaN components fall through to the
/// next axis, so `partial_cmp` is derived from [`Vector3::less_than`]
/// rather than per-axis `partial_cmp`, which would stop at NaN.)
impl PartialOrd for Vector3 {
    #[inline]
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        if self.less_than(other) {
            Some(Ordering::Less)
        } else if other.less_than(self) {
            Some(Ordering::Greater)
        } else {
            Some(Ordering::Equal)
        }
    }
}

impl Index<usize> for Vector3 {
    type Output = f64;
    #[inline]
    fn index(&self, index: usize) -> &f64 {
        &self.data[index]
    }
}

impl IndexMut<usize> for Vector3 {
    #[inline]
    fn index_mut(&mut self, index: usize) -> &mut f64 {
        &mut self.data[index]
    }
}

impl Add for Vector3 {
    type Output = Self;
    #[inline]
    fn add(self, other: Self) -> Self {
        Self::new(
            self.data[0] + other.data[0],
            self.data[1] + other.data[1],
            self.data[2] + other.data[2],
        )
    }
}

impl AddAssign for Vector3 {
    #[inline]
    fn add_assign(&mut self, other: Self) {
        self.data[0] += other.data[0];
        self.data[1] += other.data[1];
        self.data[2] += other.data[2];
    }
}

impl Sub for Vector3 {
    type Output = Self;
    #[inline]
    fn sub(self, other: Self) -> Self {
        Self::new(
            self.data[0] - other.data[0],
            self.data[1] - other.data[1],
            self.data[2] - other.data[2],
        )
    }
}

impl Neg for Vector3 {
    type Output = Self;
    #[inline]
    fn neg(self) -> Self {
        Self::new(-self.data[0], -self.data[1], -self.data[2])
    }
}

impl Mul<f64> for Vector3 {
    type Output = Self;
    #[inline]
    fn mul(self, number: f64) -> Self {
        Self::new(
            number * self.data[0],
            number * self.data[1],
            number * self.data[2],
        )
    }
}

impl Mul<Vector3> for f64 {
    type Output = Vector3;
    #[inline]
    fn mul(self, v: Vector3) -> Vector3 {
        Vector3::new(self * v.data[0], self * v.data[1], self * v.data[2])
    }
}

impl MulAssign<f64> for Vector3 {
    #[inline]
    fn mul_assign(&mut self, number: f64) {
        self.data[0] *= number;
        self.data[1] *= number;
        self.data[2] *= number;
    }
}

impl Div<f64> for Vector3 {
    type Output = Self;
    #[inline]
    fn div(self, number: f64) -> Self {
        Self::new(
            self.data[0] / number,
            self.data[1] / number,
            self.data[2] / number,
        )
    }
}

impl DivAssign<f64> for Vector3 {
    #[inline]
    fn div_assign(&mut self, number: f64) {
        self.data[0] /= number;
        self.data[1] /= number;
        self.data[2] /= number;
    }
}

/// Elementwise product (mirrors `operator*(Vector3, Vector3)`).
impl Mul for Vector3 {
    type Output = Self;
    #[inline]
    fn mul(self, other: Self) -> Self {
        Self::new(
            self.data[0] * other.data[0],
            self.data[1] * other.data[1],
            self.data[2] * other.data[2],
        )
    }
}

/// Elementwise quotient (mirrors `operator/(Vector3, Vector3)`).
impl Div for Vector3 {
    type Output = Self;
    #[inline]
    fn div(self, other: Self) -> Self {
        Self::new(
            self.data[0] / other.data[0],
            self.data[1] / other.data[1],
            self.data[2] / other.data[2],
        )
    }
}

/// `std::to_string(double)` formatting: fixed 6 decimals (`%f`), with
/// signless lowercase `nan` (Rust prints `NaN`/`-NaN`; macOS libc
/// prints every NaN as `nan`).
#[must_use]
pub fn to_string(v: &Vector3) -> String {
    fn coord(x: f64) -> String {
        // macOS libc prints every NaN signless ("nan"), unlike glibc's
        // "-nan": probed, not assumed.
        if x.is_nan() {
            return "nan".to_string();
        }
        format!("{x:.6}")
    }
    format!("{},{},{}", coord(v.x()), coord(v.y()), coord(v.z()))
}

impl std::fmt::Display for Vector3 {
    /// Shortest-round-trip `x,y,z`.
    ///
    /// Known difference: C++ `operator<<` uses ostream defaults (`%g`
    /// with 6 significant digits). Nothing downstream formats vectors,
    /// so this keeps Rust's exact default instead of emulating `%g`.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{},{},{}", self.data[0], self.data[1], self.data[2])
    }
}
