//! Port of `core/mixedintegerleastsquares.*` (pure std logic over CLS).
//!
//! Direct structural port: kernel reduction (M0/M1/M2 passes), iterative
//! rounding over `ConstrainedLeastSquares`, same tolerances (drop 1e-10,
//! axpy floor 1e-14) and the same fixing-loop quirk as the C++.

use crate::constrained::ConstrainedLeastSquares;

const NO_INDEX: usize = usize::MAX;
const DROP_TOLERANCE: f64 = 1e-10;

#[derive(Clone, Copy, Debug)]
struct Coeff {
    index: usize,
    a: f64,
}

#[derive(Clone, Debug)]
struct Row {
    coefficients: Vec<Coeff>,
    rhs: f64,
    weight: f64,
}

#[derive(Clone, Debug, Default)]
struct SparseMatrix {
    rows: Vec<Vec<Coeff>>,
}

fn normalize(row: &mut Vec<Coeff>) {
    row.sort_by(|a, b| a.index.cmp(&b.index));
    let mut out = 0usize;
    let mut i = 0usize;
    while i < row.len() {
        let mut j = i + 1;
        let mut a = row[i].a;
        while j < row.len() && row[j].index == row[i].index {
            a += row[j].a;
            j += 1;
        }
        if a.abs() > DROP_TOLERANCE {
            row[out] = Coeff {
                index: row[i].index,
                a,
            };
            out += 1;
        }
        i = j;
    }
    row.truncate(out);
}

fn add(row: &mut Vec<Coeff>, index: usize, a: f64) {
    if a.abs() < 1e-14 {
        return;
    }
    let pos = row.iter().position(|c| c.index >= index);
    match pos {
        Some(p) if row[p].index == index => {
            row[p].a += a;
            if row[p].a.abs() < DROP_TOLERANCE {
                row.remove(p);
            }
        }
        Some(p) => row.insert(p, Coeff { index, a }),
        None => row.push(Coeff { index, a }),
    }
}

pub struct MixedIntegerLeastSquares {
    size: usize,
    pass: usize,
    kernel_size: usize,
    m0: Vec<usize>,
    m1: Vec<Coeff>,
    m2: SparseMatrix,
    m2t: SparseMatrix,
    period: Vec<i32>,
    initial_period: Vec<i32>,
    pending_constraint: Vec<Coeff>,
    constraint_data: Vec<Coeff>,
    constraint_ranges: Vec<(usize, usize)>,
    constraint_scratch: Vec<Coeff>,
    single_term_constraints: Vec<usize>,
    multi_term_constraints: Vec<usize>,
    energy: Vec<Row>,
    reduced_energy: Vec<Row>,
    values: Vec<f64>,
    fixed: Vec<bool>,
    kernel: SparseMatrix,
    kernel_built: bool,
    scatter: Vec<f64>,
    touched: Vec<usize>,
    merged: Vec<Coeff>,
    system: Option<ConstrainedLeastSquares>,
    // TEMP EXPERIMENT: progressive rounding schedule (RETOPO_ROUND_SCHEDULE).
    round_schedule: Vec<f64>,
    round_step: usize,
}

impl MixedIntegerLeastSquares {
    pub fn new(size: usize) -> Self {
        Self {
            size,
            pass: 0,
            kernel_size: 0,
            m0: (0..size).collect(),
            m1: Vec::new(),
            m2: SparseMatrix::default(),
            m2t: SparseMatrix::default(),
            period: vec![0; size],
            initial_period: vec![0; size],
            pending_constraint: Vec::new(),
            constraint_data: Vec::new(),
            constraint_ranges: Vec::new(),
            constraint_scratch: Vec::new(),
            single_term_constraints: Vec::new(),
            multi_term_constraints: Vec::new(),
            energy: Vec::new(),
            reduced_energy: Vec::new(),
            values: Vec::new(),
            fixed: Vec::new(),
            kernel: SparseMatrix::default(),
            kernel_built: false,
            scatter: Vec::new(),
            touched: Vec::new(),
            merged: Vec::new(),
            system: None,
            round_schedule: std::env::var("RETOPO_ROUND_SCHEDULE")
                .ok()
                .map(|v| v.split(',').filter_map(|t| t.trim().parse().ok()).collect())
                .unwrap_or_default(),
            round_step: 0,
        }
    }

    fn axpy(&mut self, row: &mut Vec<Coeff>, source: &[Coeff], factor: f64) {
        if source.is_empty() || factor == 0.0 {
            return;
        }
        self.merged.clear();
        self.merged.reserve(row.len() + source.len());
        let mut i = 0usize;
        let mut j = 0usize;
        while i < row.len() && j < source.len() {
            if row[i].index < source[j].index {
                self.merged.push(row[i]);
                i += 1;
            } else if source[j].index < row[i].index {
                let a = factor * source[j].a;
                if a.abs() >= 1e-14 {
                    self.merged.push(Coeff {
                        index: source[j].index,
                        a,
                    });
                }
                j += 1;
            } else {
                let contribution = factor * source[j].a;
                let a = row[i].a + contribution;
                if contribution.abs() < 1e-14 {
                    self.merged.push(row[i]);
                } else if a.abs() >= DROP_TOLERANCE {
                    self.merged.push(Coeff {
                        index: row[i].index,
                        a,
                    });
                }
                i += 1;
                j += 1;
            }
        }
        while i < row.len() {
            self.merged.push(row[i]);
            i += 1;
        }
        while j < source.len() {
            let a = factor * source[j].a;
            if a.abs() >= 1e-14 {
                self.merged.push(Coeff {
                    index: source[j].index,
                    a,
                });
            }
            j += 1;
        }
        std::mem::swap(row, &mut self.merged);
    }

    // Associated function with explicit scratch: callers pass disjoint field
    // borrows, so the matrix argument can alias `self` without cloning the
    // whole matrix per call (that clone cost 22x in finalize).
    fn multiply(
        scatter: &mut Vec<f64>,
        touched: &mut Vec<usize>,
        row: &[Coeff],
        m: &SparseMatrix,
    ) -> Vec<Coeff> {
        if scatter.len() < m.rows.len() {
            scatter.assign(m.rows.len(), 0.0);
        }
        touched.clear();
        for c in row {
            if c.index >= m.rows.len() {
                continue;
            }
            for b in &m.rows[c.index] {
                if scatter[b.index] == 0.0 {
                    touched.push(b.index);
                }
                scatter[b.index] += c.a * b.a;
            }
        }
        touched.sort_unstable();
        let mut out = Vec::with_capacity(touched.len());
        for &index in touched.iter() {
            let a = scatter[index];
            scatter[index] = 0.0;
            if a.abs() > DROP_TOLERANCE {
                out.push(Coeff { index, a });
            }
        }
        out
    }

    pub fn set_variable_period(&mut self, variable: usize, period: i32) {
        if variable < self.period.len() {
            self.period[variable] = period.max(0);
            self.initial_period[variable] = period.max(0);
        }
    }

    fn record_constraint(&mut self) {
        normalize(&mut self.constraint_scratch);
        if self.constraint_scratch.is_empty() {
            return;
        }
        if self.constraint_scratch.len() == 1 {
            self.single_term_constraints
                .push(self.constraint_ranges.len());
        } else {
            self.multi_term_constraints
                .push(self.constraint_ranges.len());
        }
        self.constraint_ranges
            .push((self.constraint_data.len(), self.constraint_scratch.len()));
        self.constraint_data
            .extend_from_slice(&self.constraint_scratch);
    }

    pub fn add_constraint(&mut self, row: &[(usize, f64)]) {
        self.constraint_scratch.clear();
        for &(index, a) in row {
            if index < self.size && a != 0.0 {
                self.constraint_scratch.push(Coeff { index, a });
            }
        }
        self.record_constraint();
    }

    pub fn add_constraint1(&mut self, a: usize, ca: f64) {
        self.constraint_scratch.clear();
        if a < self.size && ca != 0.0 {
            self.constraint_scratch.push(Coeff { index: a, a: ca });
        }
        self.record_constraint();
    }

    pub fn add_constraint2(&mut self, a: usize, ca: f64, b: usize, cb: f64) {
        self.constraint_scratch.clear();
        if a < self.size && ca != 0.0 {
            self.constraint_scratch.push(Coeff { index: a, a: ca });
        }
        if b < self.size && cb != 0.0 {
            self.constraint_scratch.push(Coeff { index: b, a: cb });
        }
        self.record_constraint();
    }

    #[allow(clippy::too_many_arguments)]
    pub fn add_constraint3(&mut self, a: usize, ca: f64, b: usize, cb: f64, c: usize, cc: f64) {
        self.constraint_scratch.clear();
        if a < self.size && ca != 0.0 {
            self.constraint_scratch.push(Coeff { index: a, a: ca });
        }
        if b < self.size && cb != 0.0 {
            self.constraint_scratch.push(Coeff { index: b, a: cb });
        }
        if c < self.size && cc != 0.0 {
            self.constraint_scratch.push(Coeff { index: c, a: cc });
        }
        self.record_constraint();
    }

    fn recorded_constraint(&self, index: usize) -> Vec<Coeff> {
        let (offset, count) = self.constraint_ranges[index];
        self.constraint_data[offset..offset + count].to_vec()
    }

    pub fn finalize_constraints(&mut self) {
        if self.pass != 0 {
            return;
        }
        let singles = self.single_term_constraints.clone();
        for index in singles {
            let row = self.recorded_constraint(index);
            self.process_constraint(&row);
        }
        self.end_pass(0);
        let multis = self.multi_term_constraints.clone();
        for index in multis {
            let row = self.recorded_constraint(index);
            self.process_constraint(&row);
        }
        self.end_pass(1);
        for index in 0..self.constraint_ranges.len() {
            let row = self.recorded_constraint(index);
            self.process_constraint(&row);
        }
        self.end_pass(2);
        for index in 0..self.constraint_ranges.len() {
            let row = self.recorded_constraint(index);
            self.process_constraint(&row);
        }
        self.end_pass(3);

        self.build_kernel();

        self.reduced_energy.clear();
        self.reduced_energy.reserve(self.energy.len());
        let energy = std::mem::take(&mut self.energy);
        for e in &energy {
            let mut reduced = Row {
                coefficients: Vec::new(),
                rhs: e.rhs,
                weight: e.weight,
            };
            for coefficient in &e.coefficients {
                let line = self.kernel_line_owned(coefficient.index);
                let mut tmp = std::mem::take(&mut reduced.coefficients);
                self.axpy(&mut tmp, &line, coefficient.a);
                reduced.coefficients = tmp;
            }
            if !reduced.coefficients.is_empty() {
                self.reduced_energy.push(reduced);
            }
        }
    }

    fn build_kernel(&mut self) {
        self.kernel.rows.resize(self.size, Vec::new());
        for i in 0..self.size {
            let line = self.line(i);
            self.kernel.rows[i] = line;
        }
        self.kernel_built = true;
    }

    fn kernel_line(&self, i: usize) -> &[Coeff] {
        if !self.kernel_built || i >= self.kernel.rows.len() {
            return &[];
        }
        &self.kernel.rows[i]
    }

    fn kernel_line_owned(&self, i: usize) -> Vec<Coeff> {
        self.kernel_line(i).to_vec()
    }

    pub fn begin_constraint(&mut self) {
        self.pending_constraint.clear();
    }

    pub fn add_constraint_coefficient(&mut self, variable: usize, coefficient: f64) {
        if variable < self.size && coefficient != 0.0 {
            self.pending_constraint.push(Coeff {
                index: variable,
                a: coefficient,
            });
        }
    }

    pub fn end_constraint(&mut self) -> bool {
        normalize(&mut self.pending_constraint);
        let row = self.pending_constraint.clone();
        self.process_constraint(&row)
    }

    fn through_m0(&self, r: &[Coeff]) -> Vec<Coeff> {
        let mut out = Vec::with_capacity(r.len());
        for c in r {
            if self.m0[c.index] != NO_INDEX {
                out.push(Coeff {
                    index: self.m0[c.index],
                    a: c.a,
                });
            }
        }
        out
    }

    fn through_m1(&self, r: &[Coeff]) -> Vec<Coeff> {
        let mut out = Vec::with_capacity(r.len());
        for c in r {
            out.push(Coeff {
                index: self.m1[c.index].index,
                a: c.a * self.m1[c.index].a,
            });
        }
        normalize(&mut out);
        out
    }

    fn finalize_m0(&mut self) {
        let mut n = 0usize;
        for i in 0..self.m0.len() {
            if self.m0[i] != NO_INDEX {
                self.period[n] = self.period[i];
                self.m0[i] = n;
                n += 1;
            }
        }
        self.period.resize(n, 0);
        self.m1.resize(n, Coeff { index: 0, a: 1.0 });
        for i in 0..n {
            self.m1[i] = Coeff { index: i, a: 1.0 };
        }
    }

    fn finalize_m1(&mut self) {
        for i in 0..self.m1.len() {
            while self.m1[i].index != self.m1[self.m1[i].index].index {
                let parent = self.m1[i].index;
                self.m1[i].a *= self.m1[parent].a;
                self.m1[i].index = self.m1[parent].index;
            }
        }
        let mut n = 0usize;
        let mut group = vec![NO_INDEX; self.m1.len()];
        for i in 0..self.m1.len() {
            if self.m1[i].index == i {
                group[i] = n;
                n += 1;
            }
        }
        let mut period = vec![0i32; n];
        for i in 0..self.m1.len() {
            let root = self.m1[i].index;
            let g = group[root];
            period[g] = period[g].max(self.period[root]);
            self.m1[i].index = g;
        }
        self.period = period;
        self.m2.rows.resize(n, Vec::new());
        self.m2t.rows.resize(n, Vec::new());
        for i in 0..n {
            self.m2.rows[i].push(Coeff { index: i, a: 1.0 });
            self.m2t.rows[i].push(Coeff { index: i, a: 1.0 });
        }
    }

    fn finalize_m2(&mut self) {
        self.kernel_size = self.m2.rows.len();
        self.period.resize(self.kernel_size, 0);
    }

    fn apply_remove_column(&mut self, row: &[Coeff], remove: usize) {
        let pivot = row[remove];
        let mut replacement = Vec::with_capacity(row.len());
        for (i, c) in row.iter().enumerate() {
            if i != remove {
                replacement.push(Coeff {
                    index: c.index,
                    a: -c.a / pivot.a,
                });
            }
        }
        let zero = pivot.index;
        let mut difference = replacement;
        add(&mut difference, zero, -1.0);
        let column = self.m2t.rows[zero].clone();
        for d in &difference {
            let col = column.clone();
            let mut target = std::mem::take(&mut self.m2t.rows[d.index]);
            self.axpy(&mut target, &col, d.a);
            self.m2t.rows[d.index] = target;
        }
        for source in &column {
            let diff = difference.clone();
            let mut target = std::mem::take(&mut self.m2.rows[source.index]);
            self.axpy(&mut target, &diff, source.a);
            self.m2.rows[source.index] = target;
        }
    }

    fn process_constraint(&mut self, row: &[Coeff]) -> bool {
        if self.pass == 0 {
            if row.len() == 1 && row[0].a != 0.0 {
                self.m0[row[0].index] = NO_INDEX;
            }
            return true;
        }
        let r0 = self.through_m0(row);
        let r1 = self.through_m1(&r0);
        if r1.is_empty() {
            return true;
        }
        if self.pass == 1 {
            if r1.len() == 2
                && (r1[0].a.abs() - 1.0).abs() < 1e-10
                && (r1[1].a.abs() - 1.0).abs() < 1e-10
            {
                let mut roots = [
                    Coeff {
                        index: r1[0].index,
                        a: 1.0,
                    },
                    Coeff {
                        index: r1[1].index,
                        a: -r1[0].a / r1[1].a,
                    },
                ];
                for x in roots.iter_mut() {
                    while x.index != self.m1[x.index].index {
                        x.a *= self.m1[x.index].a;
                        x.index = self.m1[x.index].index;
                    }
                }
                if self.period[roots[0].index] < self.period[roots[1].index] {
                    self.m1[roots[0].index] = Coeff {
                        index: roots[1].index,
                        a: roots[1].a * roots[0].a,
                    };
                } else {
                    self.m1[roots[1].index] = Coeff {
                        index: roots[0].index,
                        a: roots[1].a * roots[0].a,
                    };
                }
            }
            return true;
        }
        let r2 = Self::multiply(&mut self.scatter, &mut self.touched, &r1, &self.m2);
        if r2.is_empty() {
            return true;
        }
        if self.pass == 2 {
            let mut remove = 0usize;
            for i in 1..r2.len() {
                if (self.period[r2[i].index] as f64) * r2[i].a.abs()
                    < (self.period[r2[remove].index] as f64) * r2[remove].a.abs()
                {
                    remove = i;
                }
            }
            self.apply_remove_column(&r2, remove);
            return true;
        }
        false
    }

    fn end_pass(&mut self, pass: usize) {
        if pass != self.pass {
            return;
        }
        if self.pass == 0 {
            self.finalize_m0();
        } else if self.pass == 1 {
            self.finalize_m1();
        } else if self.pass == 2 {
            self.finalize_m2();
        }
        self.pass += 1;
    }

    pub fn clear_energy(&mut self) {
        self.energy.clear();
    }

    pub fn add_energy(&mut self, coefficients: &[(usize, f64)], rhs: f64, weight: f64) {
        if weight <= 0.0 {
            return;
        }
        let mut row = Row {
            coefficients: Vec::with_capacity(coefficients.len()),
            rhs,
            weight,
        };
        for &(index, a) in coefficients {
            if index < self.size && a != 0.0 {
                row.coefficients.push(Coeff { index, a });
            }
        }
        normalize(&mut row.coefficients);
        if !row.coefficients.is_empty() {
            self.energy.push(row);
        }
    }

    pub fn add_energy2(&mut self, a: usize, ca: f64, b: usize, cb: f64, rhs: f64, weight: f64) {
        if weight <= 0.0 {
            return;
        }
        let mut row = Row {
            coefficients: Vec::with_capacity(2),
            rhs,
            weight,
        };
        if a < self.size && ca != 0.0 {
            row.coefficients.push(Coeff { index: a, a: ca });
        }
        if b < self.size && cb != 0.0 {
            row.coefficients.push(Coeff { index: b, a: cb });
        }
        normalize(&mut row.coefficients);
        if !row.coefficients.is_empty() {
            self.energy.push(row);
        }
    }

    pub fn solve_iteration(&mut self) -> bool {
        if self.pass < 3 || self.kernel_size == 0 {
            return false;
        }
        if self.values.is_empty() {
            self.values.assign(self.kernel_size, 0.0);
            self.fixed.assign(self.kernel_size, false);
        } else {
            let mut threshold = 1e20f64;
            for i in 0..self.kernel_size {
                if !self.fixed[i] && self.period[i] > 0 {
                    let d = (self.values[i] / self.period[i] as f64
                        - (self.values[i] / self.period[i] as f64).round())
                    .abs();
                    threshold = (d + 0.001f64).max(1.0);
                    break;
                }
            }
            if self.round_step < self.round_schedule.len() {
                threshold = self.round_schedule[self.round_step];
            }
            self.round_step += 1;
            for i in 0..self.kernel_size {
                if !self.fixed[i] && self.period[i] > 0 {
                    let d = (self.values[i] / self.period[i] as f64
                        - (self.values[i] / self.period[i] as f64).round())
                    .abs();
                    if d < threshold {
                        self.fixed[i] = true;
                    }
                }
            }
        }
        if self.system.is_none() {
            let mut system = ConstrainedLeastSquares::new(self.kernel_size);
            for e in &self.reduced_energy {
                let coefficients: Vec<(usize, f64)> =
                    e.coefficients.iter().map(|c| (c.index, c.a)).collect();
                system.add_energy(&coefficients, e.rhs, e.weight);
            }
            self.system = Some(system);
        }
        let system = self.system.as_mut().unwrap();
        system.clear_constraints();
        for i in 0..self.kernel_size {
            if self.fixed[i] {
                let target =
                    self.period[i] as f64 * (self.values[i] / self.period[i] as f64).round();
                system.add_constraint(&[(i, 1.0)], target);
            }
        }
        match self.system.as_mut().unwrap().solve() {
            Some(values) => {
                self.values = values;
                true
            }
            None => false,
        }
    }

    pub fn converged(&self) -> bool {
        if self.values.is_empty() {
            return false;
        }
        // TEMP EXPERIMENT: hand the continuous solution straight through.
        if std::env::var_os("RETOPO_NO_ROUND").is_some() {
            return true;
        }
        for i in 0..self.kernel_size {
            if self.period[i] > 0 && !self.fixed[i] {
                return false;
            }
        }
        true
    }

    fn line(&mut self, i: usize) -> Vec<Coeff> {
        let mut r = vec![Coeff { index: i, a: 1.0 }];
        r = self.through_m0(&r);
        if self.m1.is_empty() {
            return r;
        }
        r = self.through_m1(&r);
        if self.m2.rows.is_empty() {
            return r;
        }
        Self::multiply(&mut self.scatter, &mut self.touched, &r, &self.m2)
    }

    pub fn value(&self, i: usize) -> f64 {
        let mut v = 0.0;
        if i >= self.size {
            return v;
        }
        for x in self.kernel_line(i) {
            if x.index < self.values.len() {
                v += x.a * self.values[x.index];
            }
        }
        v
    }

    pub fn kernel_size(&self) -> usize {
        self.kernel_size
    }

    /// Full variable `i` as a sparse combination of kernel variables:
    /// `value(i) == sum(a * kernel_values()[k])` over the returned pairs.
    pub fn kernel_expansion(&self, i: usize) -> Vec<(usize, f64)> {
        if i >= self.size {
            return Vec::new();
        }
        self.kernel_line(i).iter().map(|c| (c.index, c.a)).collect()
    }

    /// Current kernel solution (empty before the first solve).
    pub fn kernel_values(&self) -> &[f64] {
        &self.values
    }

    /// True for kernel variables with an integer period (rounded and held
    /// fixed once `converged()`); false for continuous ones.
    pub fn kernel_is_integer(&self, k: usize) -> bool {
        k < self.kernel_size && self.period[k] > 0
    }

    /// Replaces the kernel solution (same length), e.g. after a
    /// post-rounding optimization of the continuous variables.
    pub fn set_kernel_values(&mut self, values: Vec<f64>) {
        assert_eq!(values.len(), self.values.len());
        self.values = values;
    }

    pub fn integer_kernel_variable_count(&self) -> usize {
        (0..self.kernel_size)
            .filter(|&i| self.period[i] > 0)
            .count()
    }

    pub fn original_integer_variable_count(&self) -> usize {
        self.initial_period.iter().filter(|&&p| p > 0).count()
    }
}

trait VecAssign<T: Clone> {
    fn assign(&mut self, n: usize, v: T);
}

impl<T: Clone> VecAssign<T> for Vec<T> {
    fn assign(&mut self, n: usize, v: T) {
        self.clear();
        self.resize(n, v);
    }
}
