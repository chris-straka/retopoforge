//! Foldover-free untangling of the rounded quad cover (EXPERIMENTAL).
//!
//! The cover solve is plain least squares toward the cross field, so its
//! map carries inverted (flipped) and collapsed triangles before and after
//! integer rounding; the extractor's repair passes exist to patch them.
//! This stage keeps every integer kernel variable (seam translations,
//! cone positions: the quad layout) fixed and re-optimizes the continuous
//! ones with the untangling energy of Garanzha et al., "Foldover-free maps
//! in 50 lines of code" (SIGGRAPH 2021):
//!
//!   f(J) = (1 - theta) tr(J^T J) / chi(det J, eps)
//!        + theta (det^2 J + 1) / chi(det J, eps),
//!   chi(D, eps) = (D + sqrt(eps^2 + D^2)) / 2,
//!
//! where J maps each face's ideal frame-aligned, sizing-scaled reference
//! triangle (the target the cover's least squares pulls toward) onto its
//! current uv triangle. Minimized by L-BFGS under the paper's eps
//! continuation. Serial, fixed iteration order: bit-deterministic.

/// One face: the six uv scalars (u0 v0 u1 v1 u2 v2) as `constant +
/// sum(coef * x[free])`, plus the inverse of its reference edge matrix.
pub struct UntangleFace {
    pub terms: [Vec<(usize, f64)>; 6],
    pub constant: [f64; 6],
    /// Inverse of [A B] (reference edges as columns), row-major.
    pub reference_inverse: [f64; 4],
    pub weight: f64,
}

pub struct UntangleReport {
    pub min_det_before: f64,
    pub min_det_after: f64,
    pub flipped_before: usize,
    pub flipped_after: usize,
    pub outer_iterations: usize,
}

const THETA: f64 = 1.0 / 128.0;
const MAX_OUTER: usize = 60;
const MAX_INNER: usize = 300;
const LBFGS_HISTORY: usize = 8;

fn chi(d: f64, eps: f64) -> f64 {
    if d > 0.0 {
        (d + (eps * eps + d * d).sqrt()) * 0.5
    } else {
        // Same value, stable for large negative d.
        0.5 * eps * eps / ((eps * eps + d * d).sqrt() - d)
    }
}

fn chi_deriv(d: f64, eps: f64) -> f64 {
    0.5 + d / (2.0 * (eps * eps + d * d).sqrt())
}

fn jacobian(face: &UntangleFace, x: &[f64]) -> ([f64; 4], [f64; 6]) {
    let mut uv = face.constant;
    for (s, terms) in face.terms.iter().enumerate() {
        for &(k, a) in terms {
            uv[s] += a * x[k];
        }
    }
    // M = [a b], a = p1 - p0, b = p2 - p0 (columns).
    let m = [uv[2] - uv[0], uv[4] - uv[0], uv[3] - uv[1], uv[5] - uv[1]];
    let r = face.reference_inverse;
    let j = [
        m[0] * r[0] + m[1] * r[2],
        m[0] * r[1] + m[1] * r[3],
        m[2] * r[0] + m[3] * r[2],
        m[2] * r[1] + m[3] * r[3],
    ];
    (j, uv)
}

fn det(j: &[f64; 4]) -> f64 {
    j[0] * j[3] - j[1] * j[2]
}

/// Energy and (optionally) gradient over the free variables.
fn evaluate(faces: &[UntangleFace], x: &[f64], eps: f64, grad: Option<&mut [f64]>) -> f64 {
    let mut energy = 0.0;
    let mut grad = grad;
    if let Some(g) = grad.as_deref_mut() {
        g.iter_mut().for_each(|v| *v = 0.0);
    }
    for face in faces {
        let (j, _) = jacobian(face, x);
        let d = det(&j);
        let c = chi(d, eps);
        let cd = chi_deriv(d, eps);
        let t = j.iter().map(|v| v * v).sum::<f64>();
        let f = (1.0 - THETA) * t / c + THETA * (d * d + 1.0) / c;
        energy += face.weight * f;
        let Some(g) = grad.as_deref_mut() else {
            continue;
        };
        // dD/dJ = cofactor(J).
        let cof = [j[3], -j[2], -j[1], j[0]];
        let k_d = -(1.0 - THETA) * t * cd / (c * c)
            + THETA * (2.0 * d / c - (d * d + 1.0) * cd / (c * c));
        let mut gj = [0.0; 4];
        for i in 0..4 {
            gj[i] = face.weight * ((1.0 - THETA) * 2.0 * j[i] / c + k_d * cof[i]);
        }
        // dF/dM = dF/dJ * R^-T.
        let r = face.reference_inverse;
        let gm = [
            gj[0] * r[0] + gj[1] * r[1],
            gj[0] * r[2] + gj[1] * r[3],
            gj[2] * r[0] + gj[3] * r[1],
            gj[2] * r[2] + gj[3] * r[3],
        ];
        // M = [[u1-u0, u2-u0], [v1-v0, v2-v0]] -> per uv scalar.
        let guv = [
            -(gm[0] + gm[1]),
            -(gm[2] + gm[3]),
            gm[0],
            gm[2],
            gm[1],
            gm[3],
        ];
        for (s, terms) in face.terms.iter().enumerate() {
            for &(k, a) in terms {
                g[k] += a * guv[s];
            }
        }
    }
    energy
}

fn min_det_and_flips(faces: &[UntangleFace], x: &[f64]) -> (f64, usize) {
    let mut min_det = f64::INFINITY;
    let mut flips = 0;
    for face in faces {
        let d = det(&jacobian(face, x).0);
        min_det = min_det.min(d);
        if d <= 0.0 {
            flips += 1;
        }
    }
    (min_det, flips)
}

fn dot(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b).map(|(x, y)| x * y).sum()
}

/// L-BFGS with Armijo backtracking at fixed eps. Returns the final energy.
fn minimize(faces: &[UntangleFace], x: &mut [f64], eps: f64) -> f64 {
    let n = x.len();
    let mut g = vec![0.0; n];
    let mut e = evaluate(faces, x, eps, Some(&mut g));
    let mut s_hist: Vec<Vec<f64>> = Vec::new();
    let mut y_hist: Vec<Vec<f64>> = Vec::new();
    let mut x_new = vec![0.0; n];
    let mut g_new = vec![0.0; n];
    for _ in 0..MAX_INNER {
        // Two-loop recursion.
        let mut q = g.clone();
        let m = s_hist.len();
        let mut alpha = vec![0.0; m];
        for i in (0..m).rev() {
            let rho = 1.0 / dot(&y_hist[i], &s_hist[i]);
            alpha[i] = rho * dot(&s_hist[i], &q);
            for k in 0..n {
                q[k] -= alpha[i] * y_hist[i][k];
            }
        }
        let gamma = if m > 0 {
            dot(&s_hist[m - 1], &y_hist[m - 1]) / dot(&y_hist[m - 1], &y_hist[m - 1])
        } else {
            1.0 / dot(&g, &g).sqrt().max(1e-12)
        };
        q.iter_mut().for_each(|v| *v *= gamma);
        for i in 0..m {
            let rho = 1.0 / dot(&y_hist[i], &s_hist[i]);
            let beta = rho * dot(&y_hist[i], &q);
            for k in 0..n {
                q[k] += s_hist[i][k] * (alpha[i] - beta);
            }
        }
        let mut slope = -dot(&g, &q);
        if slope >= 0.0 {
            // Not a descent direction: reset to steepest descent.
            s_hist.clear();
            y_hist.clear();
            let scale = 1.0 / dot(&g, &g).sqrt().max(1e-12);
            for k in 0..n {
                q[k] = g[k] * scale;
            }
            slope = -dot(&g, &q);
        }
        let mut step = 1.0;
        let mut accepted = false;
        for _ in 0..40 {
            for k in 0..n {
                x_new[k] = x[k] - step * q[k];
            }
            let e_new = evaluate(faces, &x_new, eps, Some(&mut g_new));
            if e_new.is_finite() && e_new <= e + 1e-4 * step * slope {
                let s: Vec<f64> = (0..n).map(|k| x_new[k] - x[k]).collect();
                let y: Vec<f64> = (0..n).map(|k| g_new[k] - g[k]).collect();
                if dot(&s, &y) > 1e-16 {
                    if s_hist.len() == LBFGS_HISTORY {
                        s_hist.remove(0);
                        y_hist.remove(0);
                    }
                    s_hist.push(s);
                    y_hist.push(y);
                }
                let converged = (e - e_new).abs() <= 1e-10 * e.abs().max(1.0);
                x.copy_from_slice(&x_new);
                g.copy_from_slice(&g_new);
                e = e_new;
                accepted = true;
                if converged {
                    return e;
                }
                break;
            }
            step *= 0.5;
        }
        if !accepted {
            break;
        }
    }
    e
}

/// Untangles `x` (free kernel variables) in place.
pub fn untangle(faces: &[UntangleFace], x: &mut [f64]) -> UntangleReport {
    let (min_det_before, flipped_before) = min_det_and_flips(faces, x);
    let mut report = UntangleReport {
        min_det_before,
        flipped_before,
        min_det_after: min_det_before,
        flipped_after: flipped_before,
        outer_iterations: 0,
    };
    if faces.is_empty() || x.is_empty() {
        return report;
    }
    let mut min_det = min_det_before;
    let mut eps = if min_det < 0.0 {
        2.0 * (1e-3_f64 + min_det * min_det).sqrt()
    } else {
        1e-3
    };
    let mut e_prev = evaluate(faces, x, eps, None);
    let mut clean_rounds = 0;
    for outer in 0..MAX_OUTER {
        report.outer_iterations = outer + 1;
        let e = minimize(faces, x, eps);
        let (d, flips) = min_det_and_flips(faces, x);
        min_det = d;
        if flips == 0 {
            clean_rounds += 1;
            if clean_rounds >= 2 {
                break;
            }
        }
        // Paper's continuation: sigma = max(1 - E/E_prev, 0.1),
        // mu = (1 - sigma) chi(Dmin, eps), eps = 2 sqrt(mu (mu - Dmin^-)).
        let sigma = (1.0 - e / e_prev).max(0.1);
        let d_minus = min_det.min(0.0);
        let mu = (1.0 - sigma) * chi(min_det, eps);
        if min_det < mu {
            eps = 2.0 * (mu * (mu - d_minus)).max(0.0).sqrt();
        } else {
            eps = 1e-8;
        }
        eps = eps.max(1e-12);
        e_prev = evaluate(faces, x, eps, None);
    }
    let (d, flips) = min_det_and_flips(faces, x);
    report.min_det_after = d;
    report.flipped_after = flips;
    report
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Two triangles sharing an edge, the middle vertex folded across it:
    /// untangling must restore positive orientation for both.
    #[test]
    fn unfolds_a_flipped_fan() {
        // Free vars: x0, x1 = uv of the folded vertex; the rest constant.
        let reference_inverse = [1.0, 0.0, 0.0, 1.0];
        let mk = |p: [(f64, f64); 3], free: [bool; 3]| {
            let mut terms: [Vec<(usize, f64)>; 6] = Default::default();
            let mut constant = [0.0; 6];
            for l in 0..3 {
                if free[l] {
                    terms[2 * l] = vec![(0, 1.0)];
                    terms[2 * l + 1] = vec![(1, 1.0)];
                } else {
                    constant[2 * l] = p[l].0;
                    constant[2 * l + 1] = p[l].1;
                }
            }
            UntangleFace { terms, constant, reference_inverse, weight: 0.5 }
        };
        // Square (0,0) (1,0) (1,1) (0,1) with center vertex c, fan of 4.
        let corners = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)];
        let faces: Vec<UntangleFace> = (0..4)
            .map(|i| mk([corners[i], corners[(i + 1) % 4], (0.0, 0.0)], [false, false, true]))
            .collect();
        // Center pushed outside the square: some fan triangles flip.
        let mut x = vec![1.6, -0.4];
        let report = untangle(&faces, &mut x);
        assert!(report.flipped_before > 0);
        assert_eq!(report.flipped_after, 0, "x = {x:?}");
        assert!(x[0] > 0.0 && x[0] < 1.0 && x[1] > 0.0 && x[1] < 1.0);
    }

    #[test]
    fn gradient_matches_finite_differences() {
        let face = UntangleFace {
            terms: [
                vec![(0, 1.0)],
                vec![(1, 0.5), (2, 1.0)],
                vec![(2, -1.0)],
                vec![],
                vec![(0, 0.3), (1, 1.0)],
                vec![(2, 2.0)],
            ],
            constant: [0.1, -0.2, 1.0, 0.0, 0.2, 1.1],
            reference_inverse: [0.9, 0.1, -0.2, 1.2],
            weight: 0.7,
        };
        let faces = [face];
        for &eps in &[1e-3, 0.5] {
            let x = [0.3, -0.7, 0.25];
            let mut g = [0.0; 3];
            evaluate(&faces, &x, eps, Some(&mut g));
            for k in 0..3 {
                let h = 1e-6;
                let mut xp = x;
                let mut xm = x;
                xp[k] += h;
                xm[k] -= h;
                let fd = (evaluate(&faces, &xp, eps, None) - evaluate(&faces, &xm, eps, None)) / (2.0 * h);
                assert!((fd - g[k]).abs() <= 1e-5 * fd.abs().max(1.0), "k={k} eps={eps} fd={fd} g={}", g[k]);
            }
        }
    }
}
