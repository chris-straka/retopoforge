//! Line-by-line mirror of `core/double.cppm` (`retopo.core.double_utils`).
//!
//! Same functions, same order, same threshold: `|x| <= f64::EPSILON`
//! mirrors `std::abs(number) <= std::numeric_limits<double>::epsilon()`
//! exactly (`f64::EPSILON` is 2^-52, the same double epsilon).

/// Returns true when `number` is within one double epsilon of zero.
#[inline]
#[must_use]
pub fn is_zero(number: f64) -> bool {
    number.abs() <= f64::EPSILON
}

/// Returns true when `a` and `b` differ by at most one double epsilon.
#[inline]
#[must_use]
pub fn is_equal(a: f64, b: f64) -> bool {
    is_zero(a - b)
}
