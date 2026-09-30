//! Port of `core/constrainedleastsquares.*` (Eigen sparse -> faer).
//!
//! Line-by-line structural mirror: substitution reduction, then normal
//! equations (faer sparse LLT instead of Eigen SimplicialLDLT), with a
//! Lagrange/KKT fallback (faer sparse LU) when constraints need 3+ roots.
//! Thresholds (1e-12 drop, 1e-9 consistency) and the mean-diagonal
//! relative ridge (1e-10) are preserved exactly.
//!
//! Deliberate simplifications (same answers, less machinery):
//! - No factorization cache: the C++ `Cache` (pattern analysis reuse +
//!   Accelerate fast path) is skipped; every solve rebuilds. The
//!   experiment measures solve quality, not cache behavior.
//!
//! One deliberate restructure: faer's sparse QR is not rank-revealing,
//! so independent-constraint selection runs dense `ColPivQr` on the
//! TRANSPOSE over the constraint support (constraints-as-columns, exactly
//! like Eigen's `constraintQR.compute(constraintMatrix.transpose())`).

use faer::linalg::solvers::Solve;
use faer::sparse::{SparseColMat, Triplet};
use faer::{Mat, MatRef, Side};
use std::collections::BTreeMap;

pub const NO_INDEX: usize = usize::MAX;
const RELATIVE_RIDGE: f64 = 1e-10;

#[derive(Clone, Copy, Debug)]
struct Substitution {
    root: usize,
    scale: f64,
    offset: f64,
    fixed: bool,
}

#[derive(Clone, Debug)]
struct Equation {
    coefficients: Vec<(usize, f64)>,
    rhs: f64,
    weight: f64,
}

pub struct ConstrainedLeastSquares {
    variable_count: usize,
    energy: Vec<Equation>,
    constraints: Vec<Equation>,
    substitutions: Vec<Substitution>,
    free_index_of_root: Vec<usize>,
    free_count: usize,
}

fn ridge_for(mean_diagonal: f64) -> f64 {
    if mean_diagonal > 0.0 {
        RELATIVE_RIDGE * mean_diagonal
    } else {
        RELATIVE_RIDGE
    }
}

fn rank_from_r(r: MatRef<f64>) -> usize {
    let m = r.nrows();
    let n = r.ncols();
    let d = m.min(n);
    if d == 0 {
        return 0;
    }
    let mut max_diag = 0.0f64;
    for i in 0..d {
        max_diag = max_diag.max(r[(i, i)].abs());
    }
    if max_diag == 0.0 {
        return 0;
    }
    // Eigen ColPivHouseholderQR::rank() default threshold semantics:
    //   |R_ii| > max(m, n) * eps * max|diag|
    let threshold = (m.max(n) as f64) * f64::EPSILON * max_diag;
    (0..d).filter(|&i| r[(i, i)].abs() > threshold).count()
}

fn sparse_from_entries(
    n: usize,
    entries: &BTreeMap<(usize, usize), f64>,
) -> Option<SparseColMat<usize, f64>> {
    let triplets: Vec<Triplet<usize, usize, f64>> = entries
        .iter()
        .filter(|(_, v)| **v != 0.0)
        .map(|(&(r, c), &v)| Triplet::new(r, c, v))
        .collect();
    SparseColMat::try_new_from_triplets(n, n, &triplets).ok()
}

impl ConstrainedLeastSquares {
    pub fn new(variable_count: usize) -> Self {
        Self {
            variable_count,
            energy: Vec::new(),
            constraints: Vec::new(),
            substitutions: Vec::new(),
            free_index_of_root: Vec::new(),
            free_count: 0,
        }
    }

    pub fn add_energy(&mut self, coefficients: &[(usize, f64)], rhs: f64, weight: f64) -> usize {
        if coefficients.is_empty() || weight <= 0.0 {
            return NO_INDEX;
        }
        self.energy.push(Equation {
            coefficients: coefficients.to_vec(),
            rhs,
            weight,
        });
        self.energy.len() - 1
    }

    pub fn set_energy_right_hand_side(&mut self, index: usize, rhs: f64) {
        if let Some(row) = self.energy.get_mut(index) {
            row.rhs = rhs;
        }
    }

    pub fn add_constraint(&mut self, coefficients: &[(usize, f64)], rhs: f64) {
        if coefficients.is_empty() {
            return;
        }
        self.constraints.push(Equation {
            coefficients: coefficients.to_vec(),
            rhs,
            weight: 1.0,
        });
    }

    pub fn clear_constraints(&mut self) {
        self.constraints.clear();
    }

    pub fn solve(&mut self) -> Option<Vec<f64>> {
        if self.variable_count == 0 {
            return None;
        }
        if !self.build_substitutions() {
            return self.solve_with_lagrange_multipliers();
        }
        self.solve_reduced()
    }

    fn resolve(&mut self, variable: usize) -> Substitution {
        let mut scale = 1.0;
        let mut offset = 0.0;
        let mut cursor = variable;
        while !self.substitutions[cursor].fixed && self.substitutions[cursor].root != cursor {
            let link = self.substitutions[cursor];
            offset += scale * link.offset;
            scale *= link.scale;
            cursor = link.root;
        }
        if self.substitutions[cursor].fixed {
            let base = self.substitutions[cursor].offset;
            self.substitutions[variable] = Substitution {
                root: variable,
                scale: 1.0,
                offset: offset + scale * base,
                fixed: true,
            };
        } else {
            self.substitutions[variable] = Substitution {
                root: cursor,
                scale,
                offset,
                fixed: false,
            };
        }
        self.substitutions[variable]
    }

    fn build_substitutions(&mut self) -> bool {
        self.substitutions = (0..self.variable_count)
            .map(|i| Substitution {
                root: i,
                scale: 1.0,
                offset: 0.0,
                fixed: false,
            })
            .collect();
        for c in 0..self.constraints.len() {
            let constraint = self.constraints[c].clone();
            let mut rhs = constraint.rhs;
            let mut roots: Vec<(usize, f64)> = Vec::new();
            for &(variable, value) in &constraint.coefficients {
                if variable >= self.variable_count {
                    return false;
                }
                let s = self.resolve(variable);
                rhs -= value * s.offset;
                if s.fixed {
                    continue;
                }
                let scaled = value * s.scale;
                let mut merged = false;
                for entry in roots.iter_mut() {
                    if entry.0 == s.root {
                        entry.1 += scaled;
                        merged = true;
                        break;
                    }
                }
                if !merged {
                    roots.push((s.root, scaled));
                }
            }
            roots.retain(|&(_, v)| v.abs() >= 1e-12);
            if roots.is_empty() {
                if rhs.abs() > 1e-9 * (1.0 + constraint.rhs.abs()) {
                    return false;
                }
                continue;
            }
            if roots.len() == 1 {
                let (root, a) = roots[0];
                self.substitutions[root] = Substitution {
                    root,
                    scale: 1.0,
                    offset: rhs / a,
                    fixed: true,
                };
            } else if roots.len() == 2 {
                // Eliminate roots[0] in favor of roots[1] (no magnitude
                // pivot, exactly like C++): x_a = -(v1/v0) x_b + rhs/v0.
                let (elim, elim_value) = roots[0];
                let (keep, keep_value) = roots[1];
                self.substitutions[elim] = Substitution {
                    root: keep,
                    scale: -keep_value / elim_value,
                    offset: rhs / elim_value,
                    fixed: false,
                };
            } else {
                return false;
            }
        }
        self.free_index_of_root = vec![NO_INDEX; self.variable_count];
        self.free_count = 0;
        for i in 0..self.variable_count {
            let s = self.resolve(i);
            if s.fixed {
                continue;
            }
            if self.free_index_of_root[s.root] == NO_INDEX {
                self.free_index_of_root[s.root] = self.free_count;
                self.free_count += 1;
            }
        }
        true
    }

    #[allow(clippy::type_complexity)]
    fn build_reduced_system(
        &self,
    ) -> Option<(
        SparseColMat<usize, f64>,
        Vec<f64>,
        Vec<f64>,
        Vec<Vec<(usize, f64)>>,
    )> {
        let n = self.free_count;
        let mut row_shift = vec![0.0; self.energy.len()];
        let mut row_weight_root = vec![0.0; self.energy.len()];
        // Scaled energy matrix entries: (row, column) -> value, summed.
        let mut scaled: BTreeMap<(usize, usize), f64> = BTreeMap::new();
        for (row, equation) in self.energy.iter().enumerate() {
            let weight_root = equation.weight.sqrt();
            row_weight_root[row] = weight_root;
            let mut shift = 0.0;
            for &(variable, value) in &equation.coefficients {
                if variable >= self.variable_count {
                    return None;
                }
                let s = self.substitutions[variable];
                shift += value * s.offset;
                if s.fixed {
                    continue;
                }
                let column = self.free_index_of_root[s.root];
                if column == NO_INDEX {
                    continue;
                }
                *scaled.entry((row, column)).or_insert(0.0) += weight_root * value * s.scale;
            }
            row_shift[row] = shift;
        }
        // Normal equations: A^T A over the scaled rows, then ridge * I.
        // Group scaled entries by row for the outer product.
        let mut rows: Vec<Vec<(usize, f64)>> = vec![Vec::new(); self.energy.len()];
        for (&(r, c), &v) in scaled.iter() {
            rows[r].push((c, v));
        }
        let mut normal_entries: BTreeMap<(usize, usize), f64> = BTreeMap::new();
        // Each unordered pair once: ordered pairs would double-count every
        // off-diagonal when folded into the upper triangle.
        for row in rows.iter() {
            for (a_idx, &(i, a)) in row.iter().enumerate() {
                for &(j, b) in &row[a_idx..] {
                    let (r, c) = if i <= j { (i, j) } else { (j, i) };
                    *normal_entries.entry((r, c)).or_insert(0.0) += a * b;
                }
            }
        }
        let mut diag_sum = 0.0;
        for i in 0..n {
            diag_sum += normal_entries.get(&(i, i)).copied().unwrap_or(0.0);
        }
        let mean = if n > 0 { diag_sum / n as f64 } else { 0.0 };
        let ridge = ridge_for(mean);
        for i in 0..n {
            *normal_entries.entry((i, i)).or_insert(0.0) += ridge;
        }
        // Symmetrize (upper triangle was accumulated; mirror it).
        let mut full = normal_entries.clone();
        for (&(r, c), &v) in normal_entries.iter() {
            if r != c {
                full.insert((c, r), v);
            }
        }
        let normal = sparse_from_entries(n, &full)?;
        Some((normal, row_shift, row_weight_root, rows))
    }

    fn solve_reduced(&mut self) -> Option<Vec<f64>> {
        if self.free_count == 0 {
            let mut solution = vec![0.0; self.variable_count];
            for i in 0..self.variable_count {
                solution[i] = self.substitutions[i].offset;
            }
            return Some(solution);
        }
        let (normal, row_shift, row_weight_root, scaled_rows) = self.build_reduced_system()?;
        let llt = normal.as_ref().sp_cholesky(Side::Lower).ok()?;
        // A^T b over the scaled rows: b_r = w_r (rhs_r - shift_r).
        let mut at_b = vec![0.0; self.free_count];
        for (r, row) in scaled_rows.iter().enumerate() {
            let b = row_weight_root[r] * (self.energy[r].rhs - row_shift[r]);
            for &(c, v) in row {
                at_b[c] += v * b;
            }
        }
        let b = Mat::from_fn(self.free_count, 1, |i, _| at_b[i]);
        let reduced = llt.solve(&b);
        for j in 0..reduced.ncols() {
            for i in 0..reduced.nrows() {
                if !reduced[(i, j)].is_finite() {
                    return None;
                }
            }
        }
        let mut solution = vec![0.0; self.variable_count];
        for i in 0..self.variable_count {
            let s = self.substitutions[i];
            if s.fixed {
                solution[i] = s.offset;
                continue;
            }
            let column = self.free_index_of_root[s.root];
            solution[i] = if column == NO_INDEX {
                s.offset
            } else {
                s.offset + s.scale * reduced[(column, 0)]
            };
        }
        Some(solution)
    }

    fn solve_with_lagrange_multipliers(&mut self) -> Option<Vec<f64>> {
        let m = self.constraints.len();
        let n = self.variable_count;
        // Support-submatrix restructure: QR over (support x m) transposed
        // layout, i.e. constraints-as-columns exactly like Eigen's
        // compute(constraintMatrix.transpose()).
        let mut support: Vec<usize> = Vec::new();
        for row in &self.constraints {
            for &(variable, _) in &row.coefficients {
                if variable < n && !support.contains(&variable) {
                    support.push(variable);
                }
            }
        }
        support.sort_unstable();
        for row in &self.constraints {
            for &(variable, _) in &row.coefficients {
                if variable >= n {
                    return None;
                }
            }
        }
        let dense: Mat<f64> = Mat::from_fn(support.len(), m, |i, j| {
            let v = support[i];
            self.constraints[j]
                .coefficients
                .iter()
                .filter(|c| c.0 == v)
                .map(|c| c.1)
                .sum()
        });
        let qr = dense.as_ref().col_piv_qr();
        let rank = rank_from_r(qr.R());
        let (fwd, _) = qr.P().arrays();
        let mut selected: Vec<usize> = (0..rank.min(m)).map(|k| fwd[k]).collect();
        selected.sort_unstable();

        let total = n + m;
        let mut entries: BTreeMap<(usize, usize), f64> = BTreeMap::new();
        let mut kkt_rhs = vec![0.0; total];
        let mut diagonal_sum = 0.0;
        for equation in &self.energy {
            for &(variable, value) in &equation.coefficients {
                if variable >= n {
                    return None;
                }
                kkt_rhs[variable] += equation.weight * value * equation.rhs;
                for &(other, other_value) in &equation.coefficients {
                    if other >= n {
                        return None;
                    }
                    let entry = equation.weight * value * other_value;
                    if variable == other {
                        diagonal_sum += entry;
                    }
                    *entries.entry((variable, other)).or_insert(0.0) += entry;
                }
            }
        }
        for (position, &constraint_index) in selected.iter().enumerate() {
            let equation = self.constraints[constraint_index].clone();
            kkt_rhs[n + position] = equation.rhs;
            for &(variable, value) in &equation.coefficients {
                *entries.entry((variable, n + position)).or_insert(0.0) += value;
                *entries.entry((n + position, variable)).or_insert(0.0) += value;
            }
        }
        let mean = if n > 0 { diagonal_sum / n as f64 } else { 0.0 };
        let ridge = ridge_for(mean);
        for i in 0..n {
            *entries.entry((i, i)).or_insert(0.0) += ridge;
        }
        // Mirror C++: the KKT system is n + k with multipliers at
        // n + position for position in 0..k (entries above already address
        // them so; the rank-deficient tail is dropped, not zero-padded).
        let k = selected.len();
        let total = n + k;
        let triplets: Vec<Triplet<usize, usize, f64>> = entries
            .iter()
            .filter(|(_, v)| **v != 0.0)
            .map(|(&(r, c), &v)| Triplet::new(r, c, v))
            .collect();
        let kkt = SparseColMat::try_new_from_triplets(total, total, &triplets).ok()?;
        let lu = kkt.as_ref().sp_lu().ok()?;
        let rhs = Mat::from_fn(total, 1, |i, _| kkt_rhs[i]);
        let solution = lu.solve(&rhs);
        for j in 0..solution.ncols() {
            for i in 0..solution.nrows() {
                if !solution[(i, j)].is_finite() {
                    return None;
                }
            }
        }
        let mut values = vec![0.0; n];
        for v in 0..n {
            values[v] = solution[(v, 0)];
        }
        Some(values)
    }
}
