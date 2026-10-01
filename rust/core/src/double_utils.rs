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

// ---------------------------------------------------------------------------
// Port infrastructure (no counterpart in `core/double.cppm`).
// ---------------------------------------------------------------------------

// The C++ backend fuses adjacent `std::cos`/`std::sin` calls into one
// `sincos` libm call (see call sites); `f64::sin_cos` does not lower to it
// on this toolchain (verified: separate `sin`/`cos` calls in the binary),
// so the port binds the entry point directly. The out-param symbol is
// `__sincos` on Apple (libSystem) and `sincos` on glibc (libm), both in
// the default link; targets without it (e.g. MSVC, where the C++ likewise
// cannot fuse) fall back to separate calls in `joint_sin_cos`.
#[cfg(any(target_vendor = "apple", target_os = "linux"))]
unsafe extern "C" {
    #[cfg_attr(target_vendor = "apple", link_name = "__sincos")]
    fn sincos(x: f64, sin_out: *mut f64, cos_out: *mut f64);
}

/// Joint sine/cosine through the single `sincos` libm call (mirrors a
/// backend-fused C++ evaluation, whose sine differs by 1 ulp from
/// standalone `sin` on some inputs). Shared so the workspace audits this
/// `unsafe` once; first use: `singularity_simplifier` case 11 face 15.
#[inline]
#[must_use]
pub fn joint_sin_cos(x: f64) -> (f64, f64) {
    #[cfg(any(target_vendor = "apple", target_os = "linux"))]
    {
        let mut s = 0.0;
        let mut c = 0.0;
        // SAFETY: `sincos` unconditionally writes both out-params; they
        // point at live stack locals, and the call has no other effects.
        unsafe {
            sincos(x, &mut s, &mut c);
        }
        (s, c)
    }
    #[cfg(not(any(target_vendor = "apple", target_os = "linux")))]
    {
        (x.sin(), x.cos())
    }
}

#[cfg(test)]
mod tests {
    use super::joint_sin_cos;

    #[test]
    fn joint_matches_separate_within_two_ulp() {
        for &x in &[0.0, 0.5, -1.25, 2.0, 100.0, -1000.75] {
            let (s, c) = joint_sin_cos(x);
            let (es, ec) = (x.sin(), x.cos());
            let ds = (s.to_bits() as i64 - es.to_bits() as i64).abs();
            let dc = (c.to_bits() as i64 - ec.to_bits() as i64).abs();
            assert!(ds <= 2, "sin drift at {x}: {ds} ulp");
            assert!(dc <= 2, "cos drift at {x}: {dc} ulp");
        }
    }
}
