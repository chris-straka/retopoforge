//! Replicated goldens: 1:1 transcription of `tests/test_double.cpp` at the
//! same exact-boolean tolerance, plus contract tests for the `progress`
//! alias (no C++ test exists: it is a bare `std::function` alias, so the
//! goldens pin the mirrored call shape instead).

use retopo_core::double_utils::{is_equal, is_zero};
use retopo_core::progress::ProgressHandler;
use std::sync::{Arc, Mutex};

const EPS: f64 = f64::EPSILON;

#[test]
fn is_zero_inside_and_boundary() {
    // isZero: |x| <= epsilon.
    assert!(is_zero(0.0));
    assert!(is_zero(-0.0));
    assert!(is_zero(EPS)); // boundary: exactly epsilon counts as zero
    assert!(is_zero(-EPS));
    assert!(is_zero(0.5 * EPS));
    // Just inside the boundary: predecessor of eps. f64 has no nextafter
    // in core, so step one ulp via the bit pattern (eps is a power of two,
    // its predecessor shares the exponent's lower half: exact).
    let just_inside = f64::from_bits(EPS.to_bits() - 1);
    assert!(just_inside < EPS);
    assert!(is_zero(just_inside));
}

#[test]
fn is_zero_outside() {
    // Just outside the boundary: successor of eps.
    let just_outside = f64::from_bits(EPS.to_bits() + 1);
    assert!(just_outside > EPS);
    assert!(!is_zero(just_outside));
    assert!(!is_zero(2.0 * EPS));
    assert!(!is_zero(-2.0 * EPS));
    assert!(!is_zero(1e-10));
    assert!(!is_zero(-1e-10));
    assert!(!is_zero(1.0));
}

#[test]
fn is_equal_true_cases() {
    // isEqual: isZero(a - b).
    assert!(is_equal(0.0, 0.0));
    assert!(is_equal(1.0, 1.0));
    assert!(is_equal(-5.0, -5.0));
    assert!(is_equal(0.1 + 0.2, 0.3)); // classic fp rounding, diff ~5.6e-17 < eps
    assert!(is_equal(0.0, EPS)); // boundary
    assert!(is_equal(0.0, -EPS));
}

#[test]
fn is_equal_false_cases() {
    assert!(!is_equal(0.0, 2.0 * EPS));
    assert!(!is_equal(1.0, 1.0 + 1e-9));
    assert!(!is_equal(1.0, 2.0));
    assert!(!is_equal(1.0, -1.0));
}

#[test]
fn progress_handler_delivers_fraction_and_name() {
    // The alias must be constructible from a closure and deliver both
    // arguments unchanged, mirroring std::function<void(float, const char*)>.
    let seen = Arc::new(Mutex::new(Vec::new()));
    let push = Arc::clone(&seen);
    let handler: ProgressHandler = Box::new(move |fraction, name| {
        push.lock().unwrap().push((fraction, name.to_string()));
    });
    handler(0.0, "remesh");
    handler(0.5, "solve");
    handler(1.0, "weld");
    assert_eq!(
        *seen.lock().unwrap(),
        vec![
            (0.0f32, "remesh".to_string()),
            (0.5f32, "solve".to_string()),
            (1.0f32, "weld".to_string()),
        ]
    );
}

#[test]
fn progress_handler_is_send_sync() {
    // Static proof of the C++ "safe to call from several threads at once"
    // contract: the alias carries Send + Sync.
    fn assert_send_sync<T: Send + Sync>() {}
    assert_send_sync::<ProgressHandler>();
}
