//! Pair up the odd faces an extraction leaves (triangles and pentagons)
//! and re-cut each pair, with the strip of quads between them, into quads
//! only.
//!
//! An odd face can't be fixed alone: a polygon with an odd number of
//! corners has no all-quad split. Two odd faces joined by a path of quads
//! can: n faces sharing one edge each have `a + b + 4k - 2(n - 1)`
//! corners on their joint outline (a, b the odd faces' sides, k quads),
//! which is even when a + b is even (3+3, 3+5, 5+5). That outline is cut
//! into quads by repeatedly taking the best four consecutive corners as a
//! quad (closest to square, never folding against the region's normal,
//! never adding an edge the mesh already has); the region is left as it
//! was if no clean cut exists. On the artist bench the extractor left
//! about five odd faces a model, nine in ten with another odd face within
//! eight faces.

use crate::vector3::Vector3;
use std::collections::{BTreeMap, HashMap, HashSet, VecDeque};

fn edge(a: usize, b: usize) -> (usize, usize) {
    if a < b { (a, b) } else { (b, a) }
}

fn newell(v: &[Vector3], poly: &[usize]) -> Vector3 {
    let mut n = Vector3::default();
    for i in 0..poly.len() {
        let a = v[poly[i]];
        let b = v[poly[(i + 1) % poly.len()]];
        n += Vector3::new(
            (a.y() - b.y()) * (a.z() + b.z()),
            (a.z() - b.z()) * (a.x() + b.x()),
            (a.x() - b.x()) * (a.y() + b.y()),
        );
    }
    n
}

/// Corner-angle error of a quad (sum of |angle - 90|, degrees), or None if
/// it folds against the surface (its normal against the corners' own
/// normals), has a corner past 179.5 degrees, or turns backwards at a
/// corner that isn't straight. Corners past 160 degrees cost extra.
fn quad_error(v: &[Vector3], q: [usize; 4], normal: &HashMap<usize, Vector3>) -> Option<f64> {
    let n = newell(v, &q);
    let mut up = Vector3::default();
    for c in q {
        up += *normal.get(&c)?;
    }
    if Vector3::dot_product(&n, &up) <= 0.0 {
        return None;
    }
    let mut err = 0.0;
    for i in 0..4 {
        let p = v[q[i]];
        let a = v[q[(i + 3) % 4]] - p;
        let b = v[q[(i + 1) % 4]] - p;
        let (la, lb) = (a.length(), b.length());
        if la == 0.0 || lb == 0.0 {
            return None;
        }
        let c = (Vector3::dot_product(&a, &b) / (la * lb)).clamp(-1.0, 1.0);
        let ang = c.acos().to_degrees();
        if ang > 179.5 {
            return None;
        }
        // a straight corner (an extra point on an edge) has no turn to
        // check; anything else must turn the surface's way
        let turn = Vector3::cross_product(&b, &a);
        if ang < 170.0 && Vector3::dot_product(&turn, &normal[&q[i]]) < 0.0 {
            return None;
        }
        // flat corners cost extra: a quad with one is the last resort
        err += (ang - 90.0).abs()
            + if ang > 160.0 {
                4.0 * (ang - 160.0)
            } else {
                0.0
            };
    }
    Some(err)
}

/// Cut a simple outline with an even number of corners into quads: a
/// small best-first search over which four consecutive corners to take
/// next (the best three choices at each step, at most 400 steps).
fn quadrangulate(
    v: &[Vector3],
    outline: &[usize],
    existing: &HashSet<(usize, usize)>,
    normal: &HashMap<usize, Vector3>,
) -> Option<Vec<Vec<usize>>> {
    fn go(
        v: &[Vector3],
        poly: Vec<usize>,
        existing: &HashSet<(usize, usize)>,
        added: &mut Vec<(usize, usize)>,
        normal: &HashMap<usize, Vector3>,
        budget: &mut usize,
    ) -> Option<Vec<Vec<usize>>> {
        if *budget == 0 {
            return None;
        }
        *budget -= 1;
        if poly.len() == 4 {
            quad_error(v, [poly[0], poly[1], poly[2], poly[3]], normal)?;
            return Some(vec![poly]);
        }
        let n = poly.len();
        let mut options: Vec<(f64, usize)> = Vec::new();
        for i in 0..n {
            let q = [
                poly[i],
                poly[(i + 1) % n],
                poly[(i + 2) % n],
                poly[(i + 3) % n],
            ];
            let cut = edge(q[0], q[3]);
            if existing.contains(&cut) || added.contains(&cut) {
                continue;
            }
            if let Some(e) = quad_error(v, q, normal) {
                options.push((e, i));
            }
        }
        options.sort_by(|a, b| a.0.total_cmp(&b.0));
        for &(_, i) in options.iter().take(6) {
            let q = vec![
                poly[i],
                poly[(i + 1) % n],
                poly[(i + 2) % n],
                poly[(i + 3) % n],
            ];
            let (r1, r2) = ((i + 1) % n, (i + 2) % n);
            let rest: Vec<usize> = (0..n)
                .filter(|&k| k != r1 && k != r2)
                .map(|k| poly[k])
                .collect();
            added.push(edge(q[0], q[3]));
            if let Some(mut more) = go(v, rest, existing, added, normal, budget) {
                more.push(q);
                return Some(more);
            }
            added.pop();
        }
        None
    }
    let mut budget = 5000usize;
    go(
        v,
        outline.to_vec(),
        existing,
        &mut Vec::new(),
        normal,
        &mut budget,
    )
}

/// The outline of a set of faces when it is one simple loop holding every
/// corner of the set (no corner inside), as an ordered corner list.
fn outline(faces: &[Vec<usize>], set: &[usize]) -> Option<Vec<usize>> {
    let mut count: HashMap<(usize, usize), usize> = HashMap::new();
    let mut directed: Vec<(usize, usize)> = Vec::new();
    for &f in set {
        let p = &faces[f];
        for i in 0..p.len() {
            let (a, b) = (p[i], p[(i + 1) % p.len()]);
            *count.entry(edge(a, b)).or_default() += 1;
            directed.push((a, b));
        }
    }
    let mut next: BTreeMap<usize, usize> = BTreeMap::new();
    for &(a, b) in &directed {
        if count[&edge(a, b)] == 1 && next.insert(a, b).is_some() {
            return None;
        }
    }
    let corners: HashSet<usize> = set.iter().flat_map(|&f| faces[f].iter().copied()).collect();
    if next.len() != corners.len() {
        return None;
    }
    let start = *next.keys().next()?;
    let mut loop_ = vec![start];
    let mut at = next[&start];
    while at != start {
        loop_.push(at);
        if loop_.len() > next.len() {
            return None;
        }
        at = *next.get(&at)?;
    }
    if loop_.len() != next.len() {
        return None;
    }
    Some(loop_)
}

/// See the module docs. `faces` is rewritten in place; returns how many
/// odd faces were made quads.
pub fn pair_odd_faces(
    vertices: &mut [Vector3],
    faces: &mut Vec<Vec<usize>>,
    max_path: usize,
) -> usize {
    let before: HashSet<Vec<usize>> = faces.iter().cloned().collect();
    let mut total = 0;
    for _ in 0..4 {
        let fixed = pair_round(vertices, faces, max_path);
        total += fixed;
        if fixed == 0 {
            break;
        }
    }
    if total > 0 {
        ease_flat_corners(vertices, faces, &before);
    }
    total
}

/// A new quad may keep a straight corner (a pentagon's extra point on an
/// edge). Each such point moves halfway to the mean of its neighbours,
/// along the surface (the move's component along the point's normal is
/// dropped), once; nothing else moves.
fn ease_flat_corners(v: &mut [Vector3], faces: &[Vec<usize>], before: &HashSet<Vec<usize>>) {
    let mut flat: HashSet<usize> = HashSet::new();
    for f in faces
        .iter()
        .filter(|f| f.len() == 4 && !before.contains(*f))
    {
        for i in 0..4 {
            let p = v[f[i]];
            let a = v[f[(i + 3) % 4]] - p;
            let b = v[f[(i + 1) % 4]] - p;
            let (la, lb) = (a.length(), b.length());
            if la > 0.0
                && lb > 0.0
                && (Vector3::dot_product(&a, &b) / (la * lb)) < (160.0f64).to_radians().cos()
            {
                flat.insert(f[i]);
            }
        }
    }
    let mut nb: HashMap<usize, HashSet<usize>> = HashMap::new();
    let mut nn: HashMap<usize, Vector3> = HashMap::new();
    for f in faces {
        let n = newell(v, f);
        for i in 0..f.len() {
            if flat.contains(&f[i]) {
                nb.entry(f[i]).or_default().insert(f[(i + 1) % f.len()]);
                nb.entry(f[i])
                    .or_default()
                    .insert(f[(i + f.len() - 1) % f.len()]);
                let e = nn.entry(f[i]).or_default();
                *e += n;
            }
        }
    }
    let moves: Vec<(usize, Vector3)> = flat
        .iter()
        .filter_map(|&p| {
            let ns = nb.get(&p)?;
            let mut m = Vector3::default();
            for &q in ns {
                m += v[q];
            }
            let m = m / ns.len() as f64;
            let n = nn[&p].normalized();
            let d = (m - v[p]) * 0.5;
            Some((p, v[p] + d - n * Vector3::dot_product(&d, &n)))
        })
        .collect();
    for (p, x) in moves {
        v[p] = x;
    }
}

fn pair_round(vertices: &[Vector3], faces: &mut Vec<Vec<usize>>, max_path: usize) -> usize {
    let mut edge_faces: HashMap<(usize, usize), Vec<usize>> = HashMap::new();
    for (k, f) in faces.iter().enumerate() {
        for i in 0..f.len() {
            edge_faces
                .entry(edge(f[i], f[(i + 1) % f.len()]))
                .or_default()
                .push(k);
        }
    }
    let neighbours = |k: usize, faces: &Vec<Vec<usize>>| -> Vec<usize> {
        let f = &faces[k];
        let mut out = Vec::new();
        for i in 0..f.len() {
            if let Some(fs) = edge_faces.get(&edge(f[i], f[(i + 1) % f.len()])) {
                for &g in fs {
                    if g != k && fs.len() == 2 {
                        out.push(g);
                    }
                }
            }
        }
        out
    };
    let mut existing: HashSet<(usize, usize)> = edge_faces.keys().copied().collect();
    let mut used = vec![false; faces.len()];
    let mut replace: Vec<(Vec<usize>, Vec<Vec<usize>>)> = Vec::new();
    let odd: Vec<usize> = (0..faces.len())
        .filter(|&k| faces[k].len() % 2 == 1)
        .collect();
    let mut fixed = 0usize;
    let (mut no_partner, mut bad_outline, mut odd_outline, mut no_cut) = (0, 0, 0, 0);
    for &t in &odd {
        if used[t] {
            continue;
        }
        // shortest face paths through unused quads to the nearest few
        // unused odd faces; the first that cuts cleanly wins
        let mut prev: HashMap<usize, usize> = HashMap::new();
        let mut depth: HashMap<usize, usize> = HashMap::new();
        depth.insert(t, 0);
        let mut q = VecDeque::from([t]);
        let mut ends = Vec::new();
        while let Some(x) = q.pop_front() {
            if x != t && faces[x].len() % 2 == 1 {
                ends.push(x);
                if ends.len() == 4 {
                    break;
                }
                continue;
            }
            if depth[&x] >= max_path {
                continue;
            }
            for y in neighbours(x, faces) {
                if used[y]
                    || depth.contains_key(&y)
                    || (faces[y].len() != 4 && faces[y].len().is_multiple_of(2))
                {
                    continue;
                }
                depth.insert(y, depth[&x] + 1);
                prev.insert(y, x);
                q.push_back(y);
            }
        }
        if ends.is_empty() {
            no_partner += 1;
            continue;
        }
        let mut done = None;
        for &end in &ends {
            let mut path = vec![end];
            let mut at = end;
            while let Some(&p) = prev.get(&at) {
                path.push(p);
                at = p;
            }
            let Some(loop_) = outline(faces, &path) else {
                bad_outline += 1;
                continue;
            };
            if loop_.len() % 2 == 1 {
                odd_outline += 1;
                continue;
            }
            // the region's own inner edges are free to remove
            let mut free = existing.clone();
            for &f in &path {
                let p = &faces[f];
                for i in 0..p.len() {
                    free.remove(&edge(p[i], p[(i + 1) % p.len()]));
                }
            }
            for i in 0..loop_.len() {
                free.insert(edge(loop_[i], loop_[(i + 1) % loop_.len()]));
            }
            // each corner's own surface normal, from the faces around it
            let mut normal: HashMap<usize, Vector3> = HashMap::new();
            for &c in &loop_ {
                let mut nn = Vector3::default();
                for &f in &path {
                    if faces[f].contains(&c) {
                        nn += newell(vertices, &faces[f]).normalized();
                    }
                }
                normal.insert(c, nn);
            }
            match quadrangulate(vertices, &loop_, &free, &normal) {
                Some(quads) => {
                    done = Some((path, quads));
                    break;
                }
                None => {
                    no_cut += 1;
                    if std::env::var_os("RETOPO_FILL_DEBUG").is_some() {
                        let n = loop_.len();
                        let angs: Vec<String> = (0..n)
                            .map(|i| {
                                let p = vertices[loop_[i]];
                                let a = vertices[loop_[(i + n - 1) % n]] - p;
                                let b = vertices[loop_[(i + 1) % n]] - p;
                                let c = (Vector3::dot_product(&a, &b) / (a.length() * b.length()))
                                    .clamp(-1.0, 1.0);
                                let turn = Vector3::dot_product(
                                    &Vector3::cross_product(&b, &a),
                                    &normal[&loop_[i]],
                                );
                                format!(
                                    "{:.0}{}",
                                    c.acos().to_degrees(),
                                    if turn < 0.0 { "R" } else { "" }
                                )
                            })
                            .collect();
                        let sizes: Vec<usize> = path.iter().map(|&f| faces[f].len()).collect();
                        eprintln!(
                            "fill: no cut, faces {sizes:?}, outline {n}: {}",
                            angs.join(" ")
                        );
                    }
                }
            }
        }
        let Some((path, quads)) = done else { continue };
        for &f in &path {
            used[f] = true;
        }
        for qd in &quads {
            for i in 0..4 {
                existing.insert(edge(qd[i], qd[(i + 1) % 4]));
            }
        }
        fixed += 2;
        replace.push((path, quads));
    }
    if std::env::var_os("RETOPO_FILL_DEBUG").is_some() {
        eprintln!(
            "fill: {} odd faces, {fixed} made quads; no partner {no_partner}, outline not simple {bad_outline}, odd outline {odd_outline}, no clean cut {no_cut}",
            odd.len()
        );
    }
    if replace.is_empty() {
        return 0;
    }
    let drop: HashSet<usize> = replace
        .iter()
        .flat_map(|(p, _)| p.iter().copied())
        .collect();
    let mut out: Vec<Vec<usize>> = faces
        .iter()
        .enumerate()
        .filter(|(k, _)| !drop.contains(k))
        .map(|(_, f)| f.clone())
        .collect();
    for (_, quads) in replace {
        out.extend(quads);
    }
    *faces = out;
    fixed
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A 3x1 strip of quads with a triangle at one end and a pentagon at
    /// the other (the pentagon's extra corner on the far edge): the pass
    /// turns all of it into quads with the same outline.
    #[test]
    fn triangle_and_pentagon_become_quads() {
        let v = vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(2.0, 0.0, 0.0),
            Vector3::new(3.0, 0.0, 0.0),
            Vector3::new(4.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
            Vector3::new(1.0, 1.0, 0.0),
            Vector3::new(2.0, 1.0, 0.0),
            Vector3::new(3.0, 1.0, 0.0),
            Vector3::new(4.0, 1.0, 0.0),
            Vector3::new(-0.8, 0.5, 0.0),
            Vector3::new(4.6, 0.5, 0.0),
        ];
        let mut f = vec![
            vec![10, 0, 5],
            vec![0, 1, 6, 5],
            vec![1, 2, 7, 6],
            vec![2, 3, 8, 7],
            vec![3, 4, 11, 9, 8],
        ];
        let mut v = v;
        let fixed = pair_odd_faces(&mut v, &mut f, 8);
        assert_eq!(fixed, 2);
        assert!(f.iter().all(|p| p.len() == 4), "{f:?}");
        let corners: usize = f.iter().map(|p| p.len()).sum();
        assert_eq!(corners, 4 * f.len());
    }
}
