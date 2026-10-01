// Replica goldens mirroring tests/test_positionkey.cpp 1:1 — same groups,
// same values, exact `==`/`<` where C++ uses them.
use retopo_core::position_key::PositionKey;
use retopo_core::vector3::Vector3;

#[test]
fn equality_on_identical_points() {
    assert!(PositionKey::new(1.0, 2.0, 3.0) == PositionKey::new(1.0, 2.0, 3.0));
    assert!(
        PositionKey::from_vector(&Vector3::new(1.0, 2.0, 3.0)) == PositionKey::new(1.0, 2.0, 3.0)
    );
    assert!(PositionKey::new(0.0, 0.0, 0.0) == PositionKey::new(0.0, 0.0, 0.0));
}

#[test]
fn equality_is_quantized() {
    // Points within one quantum compare equal ...
    assert!(PositionKey::new(1.000001, 0.0, 0.0) == PositionKey::new(1.000009, 0.0, 0.0));
    // ... while points a quantum apart compare unequal.
    assert!(PositionKey::new(1.0, 0.0, 0.0) != PositionKey::new(1.0001, 0.0, 0.0));
    assert!(PositionKey::new(0.0, 1.0, 0.0) != PositionKey::new(0.0, 2.0, 0.0));
    assert!(PositionKey::new(0.0, 0.0, 1.0) != PositionKey::new(0.0, 0.0, 2.0));
    assert!(PositionKey::new(1.0, 2.0, 3.0) != PositionKey::new(3.0, 2.0, 1.0));
}

#[test]
fn position_round_trips_exact_input() {
    let pos = PositionKey::new(1.5, -2.25, 3.125).position().clone();
    assert_eq!(pos.x(), 1.5);
    assert_eq!(pos.y(), -2.25);
    assert_eq!(pos.z(), 3.125);
}

#[test]
fn ordering_is_lexicographic_on_quantized_coordinates() {
    assert!(PositionKey::new(0.0, 0.0, 0.0) < PositionKey::new(1.0, 0.0, 0.0));
    assert!(PositionKey::new(1.0, 0.0, 0.0) < PositionKey::new(1.0, 1.0, 0.0));
    assert!(PositionKey::new(1.0, 1.0, 0.0) < PositionKey::new(1.0, 1.0, 1.0));
    assert!(!(PositionKey::new(1.0, 0.0, 0.0) < PositionKey::new(0.0, 0.0, 0.0)));
    assert!(!(PositionKey::new(1.0, 2.0, 3.0) < PositionKey::new(1.0, 2.0, 3.0)));
    // Ordering follows the quantized values.
    assert!(PositionKey::new(1.000001, 0.0, 0.0) < PositionKey::new(1.00002, 0.0, 0.0));
    // Equal keys are equivalent under ordering.
    assert!(!(PositionKey::new(1.000001, 0.0, 0.0) < PositionKey::new(1.000009, 0.0, 0.0)));
    assert!(!(PositionKey::new(1.000009, 0.0, 0.0) < PositionKey::new(1.000001, 0.0, 0.0)));
    // Negative coordinates order numerically.
    assert!(PositionKey::new(-2.0, 0.0, 0.0) < PositionKey::new(-1.0, 0.0, 0.0));
    assert!(PositionKey::new(-1.0, 0.0, 0.0) < PositionKey::new(0.0, 0.0, 0.0));
}
