//! Settle an island's output onto its input surface.
//!
//! The extractor places output vertices on the coarse working mesh, and a
//! vertex whose uv folded can land far off the body: a spike (review
//! 2026-11-13, findings 11-12; on the owner's creatures single vertices
//! sat 4-10% of the body's size away). This pass, run on each island's
//! extracted quads:
//!
//! 1. finds spikes: vertices farther than 1.5 median edges from the input
//!    surface (or not finite), and moves each to the mean of its settled
//!    neighbours, working inward until none is left with one;
//! 2. moves every vertex to the nearest point of the input surface, one
//!    vertex at a time, keeping a move only if none of its faces turns
//!    over (normal past 90 degrees from before the pass); three rounds.
//!
//! Topology is untouched.

use crate::vector3::Vector3;
use std::collections::HashMap;

struct TriangleGrid<'a> {
    vertices: &'a [Vector3],
    triangles: &'a [Vec<usize>],
    origin: Vector3,
    cell: f64,
    cells: HashMap<(i64, i64, i64), Vec<usize>>,
    /// Triangles whose box covers too many cells to list in each (long
    /// thin ones: hair strands); every query checks them directly.
    oversize: Vec<usize>,
}

fn closest_on_triangle(p: Vector3, a: Vector3, b: Vector3, c: Vector3) -> Vector3 {
    // Ericson, Real-Time Collision Detection 5.1.5
    let ab = b - a;
    let ac = c - a;
    let ap = p - a;
    let d1 = Vector3::dot_product(&ab, &ap);
    let d2 = Vector3::dot_product(&ac, &ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return a;
    }
    let bp = p - b;
    let d3 = Vector3::dot_product(&ab, &bp);
    let d4 = Vector3::dot_product(&ac, &bp);
    if d3 >= 0.0 && d4 <= d3 {
        return b;
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        return a + ab * (d1 / (d1 - d3));
    }
    let cp = p - c;
    let d5 = Vector3::dot_product(&ab, &cp);
    let d6 = Vector3::dot_product(&ac, &cp);
    if d6 >= 0.0 && d5 <= d6 {
        return c;
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        return a + ac * (d2 / (d2 - d6));
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
    }
    let denom = 1.0 / (va + vb + vc);
    a + ab * (vb * denom) + ac * (vc * denom)
}

impl<'a> TriangleGrid<'a> {
    fn new(vertices: &'a [Vector3], triangles: &'a [Vec<usize>], cell: f64) -> Self {
        let mut origin = Vector3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
        for v in vertices {
            for a in 0..3 {
                origin[a] = origin[a].min(v[a]);
            }
        }
        let mut grid = Self { vertices, triangles, origin, cell, cells: HashMap::new(), oversize: Vec::new() };
        for (t, tri) in triangles.iter().enumerate() {
            if tri.len() < 3 {
                continue;
            }
            let (mut lo, mut hi) = (vertices[tri[0]], vertices[tri[0]]);
            for &v in &tri[1..3] {
                for a in 0..3 {
                    lo[a] = lo[a].min(vertices[v][a]);
                    hi[a] = hi[a].max(vertices[v][a]);
                }
            }
            let (l, h) = (grid.key(&lo), grid.key(&hi));
            let span = (h.0 - l.0 + 1) as i128 * (h.1 - l.1 + 1) as i128 * (h.2 - l.2 + 1) as i128;
            if span > 64 {
                grid.oversize.push(t);
                continue;
            }
            for x in l.0..=h.0 {
                for y in l.1..=h.1 {
                    for z in l.2..=h.2 {
                        grid.cells.entry((x, y, z)).or_default().push(t);
                    }
                }
            }
        }
        grid
    }

    fn key(&self, p: &Vector3) -> (i64, i64, i64) {
        let k = |a: usize| ((p[a] - self.origin[a]) / self.cell).floor() as i64;
        (k(0), k(1), k(2))
    }

    /// The nearest point on the surface within `max_distance`, if any.
    fn nearest(&self, p: &Vector3, max_distance: f64) -> Option<(Vector3, f64)> {
        let c = self.key(p);
        let rings = (max_distance / self.cell).ceil() as i64 + 1;
        let mut best: Option<(Vector3, f64)> = None;
        for &t in &self.oversize {
            let tri = &self.triangles[t];
            let q = closest_on_triangle(*p, self.vertices[tri[0]], self.vertices[tri[1]], self.vertices[tri[2]]);
            let d = (q - *p).length();
            if d <= max_distance && best.is_none_or(|(_, bd)| d < bd) {
                best = Some((q, d));
            }
        }
        for r in 0..=rings {
            if let Some((_, d)) = best
                && (r - 1) as f64 * self.cell > d
            {
                break;
            }
            for x in -r..=r {
                for y in -r..=r {
                    for z in -r..=r {
                        if x.abs().max(y.abs()).max(z.abs()) != r {
                            continue;
                        }
                        let Some(list) = self.cells.get(&(c.0 + x, c.1 + y, c.2 + z)) else { continue };
                        for &t in list {
                            let tri = &self.triangles[t];
                            let q = closest_on_triangle(
                                *p,
                                self.vertices[tri[0]],
                                self.vertices[tri[1]],
                                self.vertices[tri[2]],
                            );
                            let d = (q - *p).length();
                            if d <= max_distance && best.is_none_or(|(_, bd)| d < bd) {
                                best = Some((q, d));
                            }
                        }
                    }
                }
            }
        }
        best
    }
}

fn polygon_normal(v: &[Vector3], face: &[usize]) -> Vector3 {
    let mut n = Vector3::default();
    for i in 0..face.len() {
        let a = v[face[i]];
        let b = v[face[(i + 1) % face.len()]];
        n = n + Vector3::new(
            (a.y() - b.y()) * (a.z() + b.z()),
            (a.z() - b.z()) * (a.x() + b.x()),
            (a.x() - b.x()) * (a.y() + b.y()),
        );
    }
    n
}

/// See the module docs. Returns (spikes found, vertices settled).
pub fn settle_onto_input(
    vertices: &mut [Vector3],
    faces: &[Vec<usize>],
    input_vertices: &[Vector3],
    input_triangles: &[Vec<usize>],
) -> (usize, usize) {
    if vertices.is_empty() || faces.is_empty() || input_triangles.is_empty() {
        return (0, 0);
    }
    let mut neighbours: Vec<Vec<usize>> = vec![Vec::new(); vertices.len()];
    let mut incident: Vec<Vec<usize>> = vec![Vec::new(); vertices.len()];
    let mut lengths = Vec::new();
    for (f, face) in faces.iter().enumerate() {
        for i in 0..face.len() {
            let (a, b) = (face[i], face[(i + 1) % face.len()]);
            if !neighbours[a].contains(&b) {
                neighbours[a].push(b);
                neighbours[b].push(a);
            }
            incident[a].push(f);
            let l = (vertices[a] - vertices[b]).length();
            if l.is_finite() {
                lengths.push(l);
            }
        }
    }
    if lengths.is_empty() {
        return (0, 0);
    }
    // the median edge: spikes are long edges and must not set the scale
    lengths.sort_by(|a, b| a.total_cmp(b));
    let edge = lengths[lengths.len() / 2];
    if !(edge > 0.0) {
        return (0, 0);
    }
    let grid = TriangleGrid::new(input_vertices, input_triangles, 2.0 * edge);
    let reach = 4.0 * edge;

    // 1. spikes
    let finite = |p: &Vector3| p.x().is_finite() && p.y().is_finite() && p.z().is_finite();
    let mut spike: Vec<bool> = vertices
        .iter()
        .map(|p| !finite(p) || grid.nearest(p, 1.5 * edge).is_none())
        .collect();
    let spikes = spike.iter().filter(|&&s| s).count();
    for _ in 0..32 {
        let mut fixed = Vec::new();
        for v in 0..vertices.len() {
            if !spike[v] {
                continue;
            }
            let good: Vec<usize> = neighbours[v].iter().copied().filter(|&u| !spike[u]).collect();
            if good.is_empty() {
                continue;
            }
            let mut p = Vector3::default();
            for &u in &good {
                p = p + vertices[u];
            }
            fixed.push((v, p / good.len() as f64));
        }
        if fixed.is_empty() {
            break;
        }
        for (v, p) in fixed {
            vertices[v] = p;
            spike[v] = false;
        }
    }

    // 2. onto the input, never turning a face over
    let before: Vec<Vector3> = faces.iter().map(|f| polygon_normal(vertices, f)).collect();
    let mut moved = vec![false; vertices.len()];
    for _ in 0..3 {
        for v in 0..vertices.len() {
            let Some((q, _)) = grid.nearest(&vertices[v], reach) else { continue };
            let old = vertices[v];
            vertices[v] = q;
            let ok = incident[v]
                .iter()
                .all(|&f| Vector3::dot_product(&polygon_normal(vertices, &faces[f]), &before[f]) > 0.0);
            if ok {
                moved[v] = true;
            } else {
                vertices[v] = old;
            }
        }
    }
    (spikes, moved.iter().filter(|&&m| m).count())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A triangle a thousand cells long goes to the overflow list (the
    /// grid stays small) and is still found.
    #[test]
    fn long_thin_triangle_is_not_spread_over_the_grid() {
        let v = vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1000.0, 1000.0, 1000.0),
            Vector3::new(0.0, 0.01, 0.0),
        ];
        let t = vec![vec![0, 1, 2]];
        let grid = TriangleGrid::new(&v, &t, 1.0);
        assert!(grid.cells.is_empty());
        assert_eq!(grid.oversize.len(), 1);
        let (q, d) = grid.nearest(&Vector3::new(500.0, 500.0, 500.5), 2.0).unwrap();
        assert!(d < 0.6 && (q.x() - 500.0).abs() < 1.0);
    }

    /// A flat input square; an output grid floating above it with one
    /// vertex thrown far off: the spike comes back between its
    /// neighbours and everything lands on the plane, no face turned.
    #[test]
    fn spike_comes_home_and_grid_lands() {
        let input = vec![
            Vector3::new(-1.0, -1.0, 0.0),
            Vector3::new(4.0, -1.0, 0.0),
            Vector3::new(4.0, 4.0, 0.0),
            Vector3::new(-1.0, 4.0, 0.0),
        ];
        let tris = vec![vec![0, 1, 2], vec![0, 2, 3]];
        let mut v = Vec::new();
        for y in 0..4 {
            for x in 0..4 {
                v.push(Vector3::new(x as f64, y as f64, 0.1));
            }
        }
        v[5] = Vector3::new(1.0, 1.0, 40.0);
        let mut q = Vec::new();
        for y in 0..3 {
            for x in 0..3 {
                let a = y * 4 + x;
                q.push(vec![a, a + 1, a + 5, a + 4]);
            }
        }
        let (spikes, moved) = settle_onto_input(&mut v, &q, &input, &tris);
        assert_eq!(spikes, 1);
        assert!(moved >= 15);
        for p in &v {
            assert!(p.z().abs() < 1e-9, "{p:?}");
        }
        assert!((v[5].x() - 1.0).abs() < 1e-9 && (v[5].y() - 1.0).abs() < 1e-9);
    }
}
