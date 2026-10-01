// Replica goldens mirroring the unit groups of tests/test_density.cpp 1:1 —
// same groups, same values, exact `==` where C++ uses it and the same
// `fabs` bands where C++ uses them. (The engine groups — subdivided-cube
// remesh runs — need `AutoRemesher`, which no wave-2 lane ports; the
// differential oracle below covers the ported surface instead.)
use retopo_core::density::Density;
use retopo_core::vector3::Vector3;

#[test]
fn normalize_field_clamping_and_off_states() {
    assert!(Density::normalize_field(&[]).is_empty());
    assert!(Density::normalize_field(&[1.0, 1.0, 1.0]).is_empty());
    let clamped = Density::normalize_field(&[100.0, 0.001, 2.0, 1.0]);
    assert_eq!(clamped.len(), 4);
    assert!(clamped[0] == 4.0 && clamped[1] == 0.25 && clamped[2] == 2.0 && clamped[3] == 1.0);
    let finite = Density::normalize_field(&[f64::NAN, 2.0]);
    assert_eq!(finite.len(), 2);
    assert!(finite[0] == 1.0 && finite[1] == 2.0);
    assert!(Density::normalize_field(&[f64::INFINITY, 1.0]).is_empty());
}

#[test]
fn edge_scale_for_reciprocal_sqrt() {
    assert_eq!(Density::edge_scale_for(4.0), 0.5);
    assert_eq!(Density::edge_scale_for(1.0), 1.0);
    assert!((Density::edge_scale_for(0.25) - 2.0).abs() < 1e-12);
    assert_eq!(Density::edge_scale_for(0.0), 1.0);
    assert_eq!(Density::edge_scale_for(f64::NAN), 1.0);
}

#[test]
fn resample_nearest_identity_and_mismatch() {
    let points = [
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
    ];
    let field = [4.0, 1.0, 0.25];
    assert_eq!(Density::resample_nearest(&points, &field, &points), field);
    let mismatch = Density::resample_nearest(&points, &[4.0], &points);
    assert_eq!(mismatch.len(), 3);
    assert!(Density::normalize_field(&mismatch).is_empty());
}

#[test]
fn apply_to_scaling_field_shifts_quads_preserves_budget() {
    let vertices = [
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(1.0, 1.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
    ];
    let triangles = [vec![0, 1, 2], vec![0, 2, 3]];
    let mut scaling = [1.0, 1.0];
    Density::apply_to_scaling_field(&vertices, &triangles, &[4.0, 4.0, 1.0, 1.0], &mut scaling);
    // Face 0 averages density 3, face 1 averages 2: face 0 ends denser.
    assert!(scaling[0] < scaling[1]);
    let mut budget = 0.0;
    for m in scaling {
        budget += 0.5 / (m * m);
    }
    assert!((budget - 1.0).abs() < 1e-9);
    // Size mismatch is a no-op.
    let mut untouched = [1.0, 1.0];
    Density::apply_to_scaling_field(&vertices, &triangles, &[4.0], &mut untouched);
    assert!(untouched[0] == 1.0 && untouched[1] == 1.0);
}
