//! Differential oracle for the CLS port: 1:1 transcription of the 9 golden
//! groups in `tests/test_constrainedleastsquares.cpp` at the same 1e-6
//! tolerance. Transcribed line by line from the C++ file (same systems,
//! same values, same comments) — any divergence is a port defect or a
//! real Eigen-vs-faer behavioral difference to record.

use retopo_solvers::constrained::{ConstrainedLeastSquares, NO_INDEX};

fn near(a: f64, b: f64) -> bool {
    (a - b).abs() <= 1e-6
}

#[test]
fn unconstrained_diagonal_system() {
    // Unconstrained diagonal system: each energy pins one variable.
    let mut s = ConstrainedLeastSquares::new(2);
    s.add_energy(&[(0, 1.0)], 3.0, 1.0);
    s.add_energy(&[(1, 1.0)], -1.0, 1.0);
    let x = s.solve().expect("solve");
    assert_eq!(x.len(), 2);
    assert!(near(x[0], 3.0));
    assert!(near(x[1], -1.0));
}

#[test]
fn weighted_single_variable() {
    // Weighted single variable: (0*1 + 10*3) / (1+3) = 7.5.
    let mut s = ConstrainedLeastSquares::new(1);
    s.add_energy(&[(0, 1.0)], 0.0, 1.0);
    s.add_energy(&[(0, 1.0)], 10.0, 3.0);
    let x = s.solve().expect("solve");
    assert_eq!(x.len(), 1);
    assert!(near(x[0], 7.5));
}

#[test]
fn overdetermined_average_and_pin() {
    // Overdetermined: two energies on x0 average, x1 pinned directly.
    let mut s = ConstrainedLeastSquares::new(2);
    s.add_energy(&[(0, 1.0)], 1.0, 1.0);
    s.add_energy(&[(0, 1.0)], 3.0, 1.0);
    s.add_energy(&[(1, 1.0)], 5.0, 1.0);
    let x = s.solve().expect("solve");
    assert!(near(x[0], 2.0));
    assert!(near(x[1], 5.0));
}

#[test]
fn two_variable_hard_constraint() {
    // Two-variable hard constraint (substitution path): energies pull both
    // variables to 0, x0 + x1 = 4 splits the difference.
    let mut s = ConstrainedLeastSquares::new(2);
    s.add_energy(&[(0, 1.0)], 0.0, 1.0);
    s.add_energy(&[(1, 1.0)], 0.0, 1.0);
    s.add_constraint(&[(0, 1.0), (1, 1.0)], 4.0);
    let x = s.solve().expect("solve");
    assert!(near(x[0], 2.0));
    assert!(near(x[1], 2.0));
    assert!(near(x[0] + x[1], 4.0));
}

#[test]
fn single_variable_pin() {
    // Single-variable hard constraint pins x0; x1 stays at its energy.
    let mut s = ConstrainedLeastSquares::new(2);
    s.add_energy(&[(0, 1.0)], 0.0, 1.0);
    s.add_energy(&[(1, 1.0)], 0.0, 1.0);
    s.add_constraint(&[(0, 1.0)], 7.0);
    let x = s.solve().expect("solve");
    assert!(near(x[0], 7.0));
    assert!(near(x[1], 0.0));
}

#[test]
fn three_variable_lagrange_path() {
    // Three-variable hard constraint (Lagrange path): equal energies at 0
    // with x0 + x1 + x2 = 6 spread evenly.
    let mut s = ConstrainedLeastSquares::new(3);
    s.add_energy(&[(0, 1.0)], 0.0, 1.0);
    s.add_energy(&[(1, 1.0)], 0.0, 1.0);
    s.add_energy(&[(2, 1.0)], 0.0, 1.0);
    s.add_constraint(&[(0, 1.0), (1, 1.0), (2, 1.0)], 6.0);
    let x = s.solve().expect("solve");
    assert!(near(x[0], 2.0));
    assert!(near(x[1], 2.0));
    assert!(near(x[2], 2.0));
}

#[test]
fn energy_rhs_update_re_solves() {
    // setEnergyRightHandSide re-solves the same system with a new target.
    let mut s = ConstrainedLeastSquares::new(1);
    let index = s.add_energy(&[(0, 1.0)], 1.0, 1.0);
    assert_eq!(index, 0);
    let x = s.solve().expect("solve");
    assert!(near(x[0], 1.0));
    s.set_energy_right_hand_side(index, 5.0);
    let x = s.solve().expect("solve");
    assert!(near(x[0], 5.0));
}

#[test]
fn clear_constraints_restores_energies() {
    // clearConstraints drops the hard constraint; energies alone remain.
    let mut s = ConstrainedLeastSquares::new(2);
    s.add_energy(&[(0, 1.0)], 0.0, 1.0);
    s.add_energy(&[(1, 1.0)], 0.0, 1.0);
    s.add_constraint(&[(0, 1.0), (1, 1.0)], 4.0);
    let x = s.solve().expect("solve");
    assert!(near(x[0], 2.0));
    s.clear_constraints();
    let x = s.solve().expect("solve");
    assert!(near(x[0], 0.0));
    assert!(near(x[1], 0.0));
}

#[test]
fn degenerate_inputs_refused() {
    // Degenerate inputs: empty coefficients and non-positive weights are
    // refused with noIndex, solving zero variables fails.
    let mut s = ConstrainedLeastSquares::new(1);
    assert_eq!(s.add_energy(&[], 0.0, 1.0), NO_INDEX);
    assert_eq!(s.add_energy(&[(0, 1.0)], 0.0, 0.0), NO_INDEX);
    assert_eq!(s.add_energy(&[(0, 1.0)], 0.0, -2.0), NO_INDEX);
    let mut empty = ConstrainedLeastSquares::new(0);
    assert!(empty.solve().is_none());
}
