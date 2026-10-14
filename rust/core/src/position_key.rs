//! Port of `core/positionkey.*` (`retopo.core.position_key`).
//!
//! Line-by-line mirror: quantized-truncation constructors, exact
//! `position()` round-trip, lexicographic `<` and `==` on the quantized
//! integers, same `100000` factor.
//!
//! Type mapping: C++ `long` is 64-bit on every target this tree builds
//! (LP64 macOS/Linux), so the quantized coordinates are [`i64`].
//!
//! Deliberate divergence on C++-UB inputs: `static_cast<long>(x)` is
//! undefined for NaN, infinite, or out-of-range `x` (in practice x86/ARM
//! `cvttsd2si` yields `i64::MIN`), while Rust's `as i64` saturates
//! (NaN -> 0, infinities/out-of-range -> `MIN`/`MAX`). The two agree
//! bitwise on all finite in-range inputs, which is the entire downstream
//! domain (mesh coordinates; weld drops NaN before keys are built).
//! The differential oracle therefore stays in the finite in-range
//! domain; NaN/inf are excluded, not asserted.
//!
//! FMA audit: the only FP op is a lone `x * 100000.0` multiply per axis
//! (100000 is exactly representable) — no multiply-add site exists, so
//! there is nothing for Clang to contract and no `mul_add` to transcribe.

use crate::vector3::Vector3;
use std::cmp::Ordering;

/// Quantization factor (mirrors `PositionKey::m_toIntFactor`, a file-static
/// `100000` that nothing ever mutates, hence a plain constant here).
const TO_INT_FACTOR: f64 = 100_000.0;

/// Vertex-weld key: exact position plus truncation-quantized integers
/// (mirrors `AutoRemesher::PositionKey`).
///
/// Downstream (`core/quadextractor.cpp`) uses this as a `std::map` key, so
/// the Rust mirror implements total [`Ord`] with the same lexicographic
/// order for the future `BTreeMap` port.
#[derive(Clone, Copy, Debug, Default)]
pub struct PositionKey {
    int_x: i64,
    int_y: i64,
    int_z: i64,
    position: Vector3,
}

impl PositionKey {
    /// Mirrors `PositionKey(const Vector3&)`, which delegates to the
    /// `(x, y, z)` constructor.
    #[inline]
    #[must_use]
    pub fn from_vector(v: &Vector3) -> Self {
        Self::new(v.x(), v.y(), v.z())
    }

    /// Mirrors `PositionKey(double x, double y, double z)`: stores the
    /// exact coordinates and the truncated `coord * 100000` integers.
    /// `as i64` truncates toward zero exactly like C++'s `static_cast`
    /// on all finite in-range inputs (see the module docs for the
    /// out-of-domain divergence).
    #[inline]
    #[must_use]
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        let mut position = Vector3::default();
        position.set_x(x);
        position.set_y(y);
        position.set_z(z);
        Self {
            int_x: (x * TO_INT_FACTOR) as i64,
            int_y: (y * TO_INT_FACTOR) as i64,
            int_z: (z * TO_INT_FACTOR) as i64,
            position,
        }
    }

    /// Mirrors `position()`: the exact input coordinates.
    #[inline]
    #[must_use]
    pub fn position(&self) -> &Vector3 {
        &self.position
    }

    /// Lexicographic `<` on the quantized integers (mirrors `operator<`
    /// exactly, if-chain for if-chain).
    #[inline]
    #[must_use]
    pub fn less_than(&self, other: &Self) -> bool {
        if self.int_x < other.int_x {
            return true;
        }
        if self.int_x > other.int_x {
            return false;
        }
        if self.int_y < other.int_y {
            return true;
        }
        if self.int_y > other.int_y {
            return false;
        }
        if self.int_z < other.int_z {
            return true;
        }
        if self.int_z > other.int_z {
            return false;
        }
        false
    }
}

/// Quantized equality (mirrors `operator==`: the exact positions play no
/// role, only the truncated integers).
impl PartialEq for PositionKey {
    #[inline]
    fn eq(&self, other: &Self) -> bool {
        self.int_x == other.int_x && self.int_y == other.int_y && self.int_z == other.int_z
    }
}

impl Eq for PositionKey {}

impl PartialOrd for PositionKey {
    #[inline]
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Total lexicographic order on the quantized integers, consistent with
/// [`PositionKey::less_than`] (integers cannot be NaN, so `<` never falls
/// through and the order is total).
impl Ord for PositionKey {
    #[inline]
    fn cmp(&self, other: &Self) -> Ordering {
        (self.int_x, self.int_y, self.int_z).cmp(&(other.int_x, other.int_y, other.int_z))
    }
}
