// Replica goldens mirroring tests/test_vector2.cpp and tests/test_vector3.cpp
// 1:1 — same groups, same values, exact `==` where C++ uses it and
// `is_equal` where C++ uses it.
use retopo_core::double_utils::is_equal;
use retopo_core::vector2::Vector2;
use retopo_core::vector3::Vector3;

#[test]
fn vector2_default_is_zero() {
    let zero = Vector2::default();
    assert_eq!(zero.x(), 0.0);
    assert_eq!(zero.y(), 0.0);
    assert_eq!(zero[0], 0.0);
    assert_eq!(zero[1], 0.0);
}

#[test]
fn vector2_setters_and_access() {
    let mut v = Vector2::default();
    v.set_x(1.5);
    v.set_y(-2.5);
    assert_eq!(v.x(), 1.5);
    assert_eq!(v.y(), -2.5);
    v[0] = 3.0;
    assert_eq!(v.x(), 3.0);
    v.set_data(4.0, 5.0);
    assert_eq!(v.x(), 4.0);
    assert_eq!(v.y(), 5.0);
}

#[test]
fn vector2_length_345() {
    let v = Vector2::new(3.0, 4.0);
    assert_eq!(v.length_squared(), 25.0);
    assert_eq!(v.length(), 5.0);
    assert_eq!(Vector2::new(1.0, 0.0).length(), 1.0);
    assert!(is_equal(Vector2::new(1.0, 1.0).length_squared(), 2.0));
}

#[test]
fn vector2_normalized() {
    let n = Vector2::new(3.0, 4.0).normalized();
    assert_eq!(n.x(), 0.6);
    assert_eq!(n.y(), 0.8);
    assert!(is_equal(n.length(), 1.0));
    let mut m = Vector2::new(3.0, 4.0);
    m.normalize();
    assert_eq!(m.x(), 0.6);
    assert_eq!(m.y(), 0.8);
}

#[test]
fn vector2_zero_normalize_noop() {
    let zero = Vector2::default();
    let zn = zero.normalized();
    assert_eq!(zn.x(), 0.0);
    assert_eq!(zn.y(), 0.0);
    assert_eq!(zero.length(), 0.0);
    assert_eq!(zero.length_squared(), 0.0);
    let mut mz = Vector2::default();
    mz.normalize();
    assert_eq!(mz.x(), 0.0);
    assert_eq!(mz.y(), 0.0);
}

#[test]
fn vector2_dot_product() {
    assert_eq!(
        Vector2::dot_product(&Vector2::new(1.0, 2.0), &Vector2::new(3.0, 4.0)),
        11.0
    );
    assert_eq!(
        Vector2::dot_product(&Vector2::new(1.0, 0.0), &Vector2::new(0.0, 1.0)),
        0.0
    );
    assert_eq!(
        Vector2::dot_product(&Vector2::new(2.0, -3.0), &Vector2::new(-4.0, 5.0)),
        -23.0
    );
    assert_eq!(
        Vector2::dot_product(&Vector2::default(), &Vector2::new(3.0, 4.0)),
        0.0
    );
}

#[test]
fn vector2_arithmetic() {
    assert_eq!(
        Vector2::new(1.0, 2.0) + Vector2::new(3.0, 4.0),
        Vector2::new(4.0, 6.0)
    );
    assert_eq!(
        Vector2::new(3.0, 4.0) - Vector2::new(1.0, 2.0),
        Vector2::new(2.0, 2.0)
    );
    assert_eq!(2.0 * Vector2::new(1.0, 2.0), Vector2::new(2.0, 4.0));
    assert_eq!(Vector2::new(1.0, 2.0) * 2.0, Vector2::new(2.0, 4.0));
}

#[test]
fn vector2_epsilon_equality() {
    assert_eq!(Vector2::new(1.0, 2.0), Vector2::new(1.0, 2.0));
    assert_ne!(Vector2::new(1.0, 2.0), Vector2::new(1.0, 3.0));
}

#[test]
fn vector3_default_is_zero() {
    let zero = Vector3::default();
    assert_eq!(zero.x(), 0.0);
    assert_eq!(zero.y(), 0.0);
    assert_eq!(zero.z(), 0.0);
    assert!(zero.is_zero());
    assert!(!Vector3::new(1.0, 0.0, 0.0).is_zero());
}

#[test]
fn vector3_length_122() {
    let v = Vector3::new(1.0, 2.0, 2.0);
    assert_eq!(v.length_squared(), 9.0);
    assert_eq!(v.length(), 3.0);
}

#[test]
fn vector3_normalized() {
    let n = Vector3::new(1.0, 2.0, 2.0).normalized();
    assert_eq!(n.x(), 1.0 / 3.0);
    assert_eq!(n.y(), 2.0 / 3.0);
    assert_eq!(n.z(), 2.0 / 3.0);
    assert!(is_equal(n.length(), 1.0));
    let mut m = Vector3::new(1.0, 2.0, 2.0);
    m.normalize();
    assert_eq!(m.x(), 1.0 / 3.0);
    assert_eq!(m.y(), 2.0 / 3.0);
    assert_eq!(m.z(), 2.0 / 3.0);
}

#[test]
fn vector3_zero_normalize_noop() {
    let zero = Vector3::default();
    let zn = zero.normalized();
    assert_eq!(zn.x(), 0.0);
    assert_eq!(zn.y(), 0.0);
    assert_eq!(zn.z(), 0.0);
    assert_eq!(zero.length(), 0.0);
    let mut mz = Vector3::default();
    mz.normalize();
    assert_eq!(mz.x(), 0.0);
    assert_eq!(mz.y(), 0.0);
    assert_eq!(mz.z(), 0.0);
}

#[test]
fn vector3_dot_product() {
    assert_eq!(
        Vector3::dot_product(&Vector3::new(1.0, 2.0, 3.0), &Vector3::new(4.0, -5.0, 6.0)),
        12.0
    );
    assert_eq!(
        Vector3::dot_product(&Vector3::new(1.0, 0.0, 0.0), &Vector3::new(0.0, 1.0, 0.0)),
        0.0
    );
    assert_eq!(
        Vector3::dot_product(&Vector3::default(), &Vector3::new(1.0, 2.0, 2.0)),
        0.0
    );
}

#[test]
fn vector3_cross_product() {
    let axis = Vector3::cross_product(&Vector3::new(1.0, 0.0, 0.0), &Vector3::new(0.0, 1.0, 0.0));
    assert_eq!((axis.x(), axis.y(), axis.z()), (0.0, 0.0, 1.0));
    let cp = Vector3::cross_product(&Vector3::new(1.0, 2.0, 3.0), &Vector3::new(4.0, 5.0, 6.0));
    assert_eq!((cp.x(), cp.y(), cp.z()), (-3.0, 6.0, -3.0));
    let par = Vector3::cross_product(&Vector3::new(1.0, 2.0, 3.0), &Vector3::new(2.0, 4.0, 6.0));
    assert_eq!((par.x(), par.y(), par.z()), (0.0, 0.0, 0.0));
}

#[test]
fn vector3_triangle_normal() {
    let tn = Vector3::normal(
        &Vector3::new(0.0, 0.0, 0.0),
        &Vector3::new(1.0, 0.0, 0.0),
        &Vector3::new(0.0, 1.0, 0.0),
    );
    assert_eq!((tn.x(), tn.y(), tn.z()), (0.0, 0.0, 1.0));
    let dn = Vector3::normal(
        &Vector3::new(0.0, 0.0, 0.0),
        &Vector3::new(1.0, 0.0, 0.0),
        &Vector3::new(2.0, 0.0, 0.0),
    );
    assert_eq!((dn.x(), dn.y(), dn.z()), (0.0, 0.0, 0.0));
}

#[test]
fn vector3_angle() {
    assert!(is_equal(
        Vector3::angle(&Vector3::new(1.0, 0.0, 0.0), &Vector3::new(0.0, 1.0, 0.0)),
        std::f64::consts::FRAC_PI_2
    ));
    assert_eq!(
        Vector3::angle(&Vector3::new(1.0, 0.0, 0.0), &Vector3::new(1.0, 0.0, 0.0)),
        0.0
    );
}

#[test]
#[allow(clippy::neg_cmp_op_on_partial_ord)] // Port mirrors the C++ negated comparison; `!(a<b)` differs from `a>=b` on NaN.
fn vector3_lexicographic_order() {
    assert!(Vector3::new(0.0, 0.0, 0.0) < Vector3::new(1.0, 0.0, 0.0));
    assert!(Vector3::new(1.0, 0.0, 0.0) < Vector3::new(1.0, 1.0, 0.0));
    assert!(!(Vector3::new(1.0, 0.0, 0.0) < Vector3::new(1.0, 0.0, 0.0)));
}
