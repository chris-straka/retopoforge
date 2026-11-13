//! Joint rings: closed lines the output must carry as quad edge loops.
//!
//! A ring arrives as a polyline (the surface cut by a plane across a limb
//! at a joint, e.g. `bench/creature/rig_guides.py`). Steering the cross
//! field toward it (guides) or marking working-mesh edges near it
//! (features) cannot make a loop on a smooth limb: the working mesh has no
//! edges along the ring, and alignment without an integer leaves the line
//! between two quad rows. So the ring is cut into the working mesh here
//! (before the isotropic stage, which keeps tagged vertices and edges; see
//! `iso_remesh_kernel::IsoVertex::ring`), and the parameterizer pins one
//! integer per ring (`quad_parameterizer`), which puts the whole ring on
//! an integer isoline: an edge loop.

use crate::vector3::Vector3;
use std::collections::HashMap;

fn newell(points: &[Vector3]) -> Vector3 {
    let mut n = Vector3::default();
    for i in 0..points.len() {
        let a = points[i];
        let b = points[(i + 1) % points.len()];
        n = n + Vector3::new(
            (a.y() - b.y()) * (a.z() + b.z()),
            (a.z() - b.z()) * (a.x() + b.x()),
            (a.x() - b.x()) * (a.y() + b.y()),
        );
    }
    n
}

fn distance_to_polyline(p: &Vector3, points: &[Vector3]) -> f64 {
    let mut best = f64::INFINITY;
    for i in 0..points.len() {
        let a = points[i];
        let b = points[(i + 1) % points.len()];
        let ab = b - a;
        let len2 = ab.length_squared();
        let t = if len2 > 0.0 {
            (Vector3::dot_product(&(*p - a), &ab) / len2).clamp(0.0, 1.0)
        } else {
            0.0
        };
        best = best.min((*p - (a + ab * t)).length());
    }
    best
}

/// Cut every ring into the mesh and return a tag per vertex: `k + 1` for
/// a vertex on ring `k`, 0 elsewhere. `edge` is the mesh's edge length:
/// vertices within 0.3 edge of a ring's plane (and near the ring) are
/// moved onto it, so the cut leaves no slivers; only triangles within 1.5
/// edges of the ring are cut, and any neighbour sharing a cut edge is
/// split too, so the mesh stays conforming.
pub fn embed_rings(
    vertices: &mut Vec<Vector3>,
    triangles: &mut Vec<Vec<usize>>,
    rings: &[Vec<Vector3>],
    edge: f64,
) -> Vec<u32> {
    let mut tags = vec![0u32; vertices.len()];
    if !(edge > 0.0) {
        return tags;
    }
    for (k, ring) in rings.iter().enumerate() {
        let mut points = ring.clone();
        if points.len() > 1 && (points[0] - points[points.len() - 1]).length() <= 1e-12 * edge {
            points.pop();
        }
        if points.len() < 3 {
            continue;
        }
        let mut normal = newell(&points);
        if normal.length() <= 0.0 {
            continue;
        }
        normal = normal.normalized();
        let mut centre = Vector3::default();
        for p in &points {
            centre = centre + *p;
        }
        centre = centre / points.len() as f64;
        let (mut lo, mut hi) = (points[0], points[0]);
        for p in &points {
            for a in 0..3 {
                lo[a] = lo[a].min(p[a]);
                hi[a] = hi[a].max(p[a]);
            }
        }
        let reach = 1.5 * edge;
        let near: Vec<bool> = vertices
            .iter()
            .map(|v| {
                (0..3).all(|a| v[a] >= lo[a] - reach && v[a] <= hi[a] + reach)
                    && distance_to_polyline(v, &points) <= reach
            })
            .collect();
        if !near.iter().any(|&b| b) {
            continue;
        }
        let tag = k as u32 + 1;
        let mut side: Vec<f64> = vertices
            .iter()
            .map(|v| Vector3::dot_product(&(*v - centre), &normal))
            .collect();
        for v in 0..vertices.len() {
            if near[v] && tags[v] == 0 && side[v].abs() < 0.3 * edge {
                vertices[v] = vertices[v] - normal * side[v];
                side[v] = 0.0;
                tags[v] = tag;
            }
        }
        // the edges the ring's plane crosses, next to the ring
        let mut cut: HashMap<(usize, usize), usize> = HashMap::new();
        for t in triangles.iter() {
            if t.len() != 3 || !t.iter().any(|&v| near[v]) {
                continue;
            }
            for i in 0..3 {
                let (a, b) = (t[i], t[(i + 1) % 3]);
                if side[a] * side[b] < 0.0 && (near[a] || near[b]) {
                    let key = (a.min(b), a.max(b));
                    cut.entry(key).or_insert_with(|| {
                        let s = side[a] / (side[a] - side[b]);
                        let p = vertices[a] + (vertices[b] - vertices[a]) * s;
                        vertices.push(p);
                        side.push(0.0);
                        tags.push(tag);
                        vertices.len() - 1
                    });
                }
            }
        }
        if cut.is_empty() {
            continue;
        }
        let mut out: Vec<Vec<usize>> = Vec::with_capacity(triangles.len() + 2 * cut.len());
        for t in triangles.iter() {
            if t.len() != 3 {
                out.push(t.clone());
                continue;
            }
            // the triangle as a polygon with its cut points in place
            let mut poly: Vec<(usize, bool)> = Vec::with_capacity(5);
            for i in 0..3 {
                let (a, b) = (t[i], t[(i + 1) % 3]);
                poly.push((a, false));
                if let Some(&x) = cut.get(&(a.min(b), a.max(b))) {
                    poly.push((x, true));
                }
            }
            let xs: Vec<usize> = (0..poly.len()).filter(|&i| poly[i].1).collect();
            match xs.len() {
                0 => out.push(t.clone()),
                1 => {
                    // fan from the cut point (to the opposite corner)
                    let n = poly.len();
                    let i = xs[0];
                    for j in 1..n - 1 {
                        out.push(vec![poly[i].0, poly[(i + j) % n].0, poly[(i + j + 1) % n].0]);
                    }
                }
                _ => {
                    // split along the two cut points, fan each side
                    let n = poly.len();
                    let (i, j) = (xs[0], xs[1]);
                    for (from, to) in [(i, j), (j, i)] {
                        let mut side_poly = vec![poly[from].0];
                        let mut m = (from + 1) % n;
                        while m != to {
                            side_poly.push(poly[m].0);
                            m = (m + 1) % n;
                        }
                        side_poly.push(poly[to].0);
                        for q in 1..side_poly.len() - 1 {
                            out.push(vec![side_poly[0], side_poly[q], side_poly[q + 1]]);
                        }
                    }
                }
            }
        }
        *triangles = out;
    }
    tags
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A tube along z, cut by a ring at z = 0.37: every triangle that
    /// straddles the plane is split, the ring vertices lie on the plane,
    /// and the mesh keeps its area (no hole, no overlap).
    #[test]
    fn ring_cut_into_a_tube() {
        let (segs, rows) = (16usize, 8usize);
        let mut v = Vec::new();
        for r in 0..rows {
            for s in 0..segs {
                let a = s as f64 / segs as f64 * std::f64::consts::TAU;
                v.push(Vector3::new(a.cos(), a.sin(), r as f64 * 0.25 - 1.0));
            }
        }
        let mut t = Vec::new();
        for r in 0..rows - 1 {
            for s in 0..segs {
                let (a, b) = (r * segs + s, r * segs + (s + 1) % segs);
                let (c, d) = (a + segs, b + segs);
                t.push(vec![a, b, d]);
                t.push(vec![a, d, c]);
            }
        }
        let area = |v: &[Vector3], t: &[Vec<usize>]| -> f64 {
            t.iter()
                .map(|f| Vector3::cross_product(&(v[f[1]] - v[f[0]]), &(v[f[2]] - v[f[0]])).length() / 2.0)
                .sum()
        };
        let before = area(&v, &t);
        let ring: Vec<Vector3> = (0..32)
            .map(|s| {
                let a = s as f64 / 32.0 * std::f64::consts::TAU;
                Vector3::new(a.cos(), a.sin(), -0.13)
            })
            .collect();
        let tags = embed_rings(&mut v, &mut t, &[ring], 0.3);
        assert_eq!(tags.len(), v.len());
        let on: Vec<usize> = (0..v.len()).filter(|&i| tags[i] == 1).collect();
        assert!(on.len() >= segs, "ring has {} vertices", on.len());
        for &i in &on {
            assert!((v[i].z() + 0.13).abs() < 1e-9);
        }
        // every edge between two ring vertices: no triangle crosses the plane
        for f in &t {
            let s: Vec<f64> = f.iter().map(|&i| v[i].z() + 0.13).collect();
            assert!(!(s.iter().any(|&x| x > 1e-9) && s.iter().any(|&x| x < -1e-9)));
        }
        assert!((area(&v, &t) - before).abs() < 1e-6 * before.max(1.0) + 0.05 * before);
    }
}
