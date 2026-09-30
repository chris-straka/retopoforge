//! Differential oracle for the MILS port: 1:1 replication of the 5 golden
//! groups in `tests/test_mixedintegerleastsquares.cpp` at 1e-6.

use retopo_solvers::mixed_integer::MixedIntegerLeastSquares;

fn near(a: f64, b: f64) -> bool {
    (a - b).abs() <= 1e-6
}

#[test]
fn unconstrained_energies_pass_through() {
    let mut s = MixedIntegerLeastSquares::new(2);
    s.add_energy(&[(0, 1.0)], 3.0, 1.0);
    s.add_energy(&[(1, 1.0)], -1.0, 1.0);
    s.finalize_constraints();
    assert_eq!(s.kernel_size(), 2);
    assert!(s.solve_iteration());
    assert!(s.converged());
    assert!(near(s.value(0), 3.0));
    assert!(near(s.value(1), -1.0));
}

#[test]
fn equality_constraint_merges_kernel() {
    let mut s = MixedIntegerLeastSquares::new(2);
    s.add_constraint2(0, 1.0, 1, -1.0);
    s.add_energy2(0, 1.0, 1, 1.0, 6.0, 1.0);
    s.finalize_constraints();
    assert_eq!(s.kernel_size(), 1);
    assert!(s.solve_iteration());
    assert!(s.converged());
    assert!(near(s.value(0), 3.0));
    assert!(near(s.value(1), 3.0));
}

#[test]
fn single_term_constraint_anchors() {
    let mut s = MixedIntegerLeastSquares::new(2);
    s.add_constraint1(0, 1.0);
    s.add_energy(&[(0, 1.0)], 5.0, 1.0);
    s.add_energy(&[(1, 1.0)], 7.0, 1.0);
    s.finalize_constraints();
    assert_eq!(s.kernel_size(), 1);
    assert!(s.solve_iteration());
    assert!(s.converged());
    assert!(near(s.value(0), 0.0));
    assert!(near(s.value(1), 7.0));
}

#[test]
fn periodic_variable_rounds_to_multiple() {
    let mut s = MixedIntegerLeastSquares::new(1);
    s.set_variable_period(0, 1);
    s.add_energy(&[(0, 1.0)], 2.3, 1.0);
    s.finalize_constraints();
    assert_eq!(s.kernel_size(), 1);
    assert_eq!(s.original_integer_variable_count(), 1);
    assert_eq!(s.integer_kernel_variable_count(), 1);
    let mut solved = false;
    for _ in 0..100 {
        solved = s.solve_iteration();
        if !solved || s.converged() {
            break;
        }
    }
    assert!(solved);
    assert!(s.converged());
    assert!(near(s.value(0), 2.0));
}

#[test]
fn degenerate_empty_system() {
    let mut s = MixedIntegerLeastSquares::new(0);
    s.finalize_constraints();
    assert_eq!(s.kernel_size(), 0);
    assert!(!s.solve_iteration());
    let mut t = MixedIntegerLeastSquares::new(1);
    t.add_energy(&[(0, 1.0)], 4.0, 1.0);
    t.finalize_constraints();
    assert!(t.solve_iteration());
    assert_eq!(t.value(99), 0.0);
}
