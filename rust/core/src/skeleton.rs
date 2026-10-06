//! Deformation-aware density: size the edges where a skeleton's limbs
//! split.
//!
//! A rigged character tears at a joint when its skin weights blend from
//! one bone to the next over too short a distance: the edges across the
//! crease stretch past the deformation gate. Both the ML rigger
//! (SkinTokens) and weightforge's repairs blend over a roughly fixed
//! number of edge rings, so the blend's width in space is that ring
//! count times the local edge length. Measured on real characters
//! (README "Skeleton density"): more rings at the groin made the stretch
//! worse; coarser, even rings there removed it.
//!
//! The crease that needs this is the one between sibling limbs (two legs
//! under one pelvis, four legs under one body): when one limb swings, the
//! strip between it and its still sibling takes the whole sweep, and the
//! deeper that strip runs down the limb, the longer the arc. For each
//! bone that has a sibling, the crease depth is measured on the mesh:
//! slice the surface perpendicular to the bone, stepping from its pivot
//! toward its tail, until the loop around the bone holds no surface that
//! belongs (nearest bone) to a sibling limb. The crease then asks for a
//! minimum edge length:
//!
//! ```text
//! crease edge = bend (rad) * crease depth / ((GATE_STRETCH - 1) * BLEND_RINGS)
//! ```
//!
//! (spreading the crease's sweep over `BLEND_RINGS` edges keeps each edge
//! under `GATE_STRETCH`). Where the mesh's nominal edge is shorter, the
//! zone around the pivot gets the density multiplier
//! `(nominal / crease edge)^2`, floored at [`MIN_MULTIPLIER`]; creases the
//! nominal mesh already spans ask for nothing. The [`Density`] pipeline
//! moves those quads elsewhere without changing the total budget.
//!
//! Every input is skeleton-relative (bones, their hierarchy and bend
//! ranges) or measured on the mesh, so the rule works for any body plan
//! at any scale. Limb-to-trunk creases (armpits) are left alone: there
//! the blend can spread into the trunk, and coarsening them traded the
//! stretch for collapsed volume in the same measurements.

use crate::density::Density;
use crate::vector3::Vector3;
use std::collections::HashMap;

/// Per-edge stretch the deformation gate allows (weightforge D_STRETCH).
pub const GATE_STRETCH: f64 = 2.5;

/// Edge rings a skin-weight blend spans across a joint (SkinTokens'
/// transition and weightforge's leg band are both about this wide).
pub const BLEND_RINGS: f64 = 4.0;

/// Strongest coarsening a crease may ask for (the engine's own floor).
pub const MIN_MULTIPLIER: f64 = Density::MIN_MULTIPLIER;

/// Zone radius as a multiple of the crease depth (the inner half is the
/// full-strength plateau that contains the crease).
pub const ZONE_PER_CREASE: f64 = 2.0;

/// Slices per bone length when searching for the crease.
pub const CREASE_STEPS: usize = 32;

/// One bone of the skeleton, in the mesh's frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Bone {
    /// Pivot (where the bone joins its parent).
    pub head: Vector3,
    pub tail: Vector3,
    /// Index of the parent bone, `None` for a root.
    pub parent: Option<usize>,
    /// How far the joint at `head` bends, in degrees.
    pub bend_degrees: f64,
}

impl Bone {
    fn length(&self) -> f64 {
        (self.tail - self.head).length()
    }

    fn valid(&self) -> bool {
        self.head
            .as_array()
            .iter()
            .chain(self.tail.as_array())
            .all(|c| c.is_finite())
            && self.bend_degrees.is_finite()
            && self.length() > 0.0
    }
}

/// A crease the skeleton asks to size: where it is and how deep it runs.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Crease {
    /// The bone whose pivot owns the crease.
    pub bone: usize,
    /// Distance from the pivot down the bone to where the limb separates.
    pub depth: f64,
    pub bend_degrees: f64,
}

impl Crease {
    /// Shortest edge across the crease that keeps the blend under the
    /// gate when the joint bends its full range.
    #[must_use]
    pub fn edge_length(&self) -> f64 {
        self.bend_degrees.abs().to_radians() * self.depth / ((GATE_STRETCH - 1.0) * BLEND_RINGS)
    }

    /// Density multiplier for a mesh whose nominal quad edge is
    /// `nominal_edge`: 1.0 when the nominal edge already spans the
    /// crease, down to [`MIN_MULTIPLIER`].
    #[must_use]
    pub fn multiplier(&self, nominal_edge: f64) -> f64 {
        let needed = self.edge_length();
        if !nominal_edge.is_finite()
            || nominal_edge <= 0.0
            || !needed.is_finite()
            || needed <= nominal_edge
        {
            return 1.0;
        }
        let ratio = nominal_edge / needed;
        (ratio * ratio).max(MIN_MULTIPLIER)
    }
}

fn smoothstep(t: f64) -> f64 {
    let t = t.clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

fn segment_distance_squared(p: &Vector3, a: &Vector3, b: &Vector3) -> f64 {
    let ab = *b - *a;
    let len2 = ab.length_squared();
    let t = if len2 > 0.0 {
        (Vector3::dot_product(&(*p - *a), &ab) / len2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    (*p - (*a + ab * t)).length_squared()
}

/// Nominal quad edge length for `target_quads` quads over the surface:
/// sqrt(area / quads). 0 when either is degenerate.
#[must_use]
pub fn nominal_edge_length(
    vertices: &[Vector3],
    triangles: &[Vec<usize>],
    target_quads: usize,
) -> f64 {
    if target_quads == 0 {
        return 0.0;
    }
    let mut area = 0.0;
    for tri in triangles {
        if tri.len() != 3 || tri.iter().any(|&i| i >= vertices.len()) {
            continue;
        }
        area += Vector3::area(&vertices[tri[0]], &vertices[tri[1]], &vertices[tri[2]]);
    }
    if !area.is_finite() || area <= 0.0 {
        return 0.0;
    }
    (area / target_quads as f64).sqrt()
}

fn find(parent: &mut [usize], mut x: usize) -> usize {
    while parent[x] != x {
        parent[x] = parent[parent[x]];
        x = parent[x];
    }
    x
}

/// Plane/mesh cut: the connected loop nearest `point`, as crossing points.
/// Crossings live on mesh edges and two crossings in one triangle are
/// joined, so loops follow mesh connectivity exactly (no welding by
/// distance).
fn nearest_slice_loop(
    vertices: &[Vector3],
    triangles: &[Vec<usize>],
    point: &Vector3,
    normal: &Vector3,
) -> Vec<Vector3> {
    let side: Vec<f64> = vertices
        .iter()
        .map(|v| Vector3::dot_product(&(*v - *point), normal))
        .collect();
    let mut edge_index: HashMap<(usize, usize), usize> = HashMap::new();
    let mut crossings: Vec<Vector3> = Vec::new();
    let mut parent: Vec<usize> = Vec::new();
    for tri in triangles {
        if tri.len() != 3 || tri.iter().any(|&i| i >= vertices.len()) {
            continue;
        }
        let mut hits = [0usize; 2];
        let mut count = 0;
        for k in 0..3 {
            let (a, b) = (tri[k], tri[(k + 1) % 3]);
            let (da, db) = (side[a], side[b]);
            if (da < 0.0) == (db < 0.0) {
                continue;
            }
            let key = if a < b { (a, b) } else { (b, a) };
            let id = *edge_index.entry(key).or_insert_with(|| {
                let t = da / (da - db);
                crossings.push(vertices[a] + (vertices[b] - vertices[a]) * t);
                parent.push(parent.len());
                parent.len() - 1
            });
            if count < 2 {
                hits[count] = id;
            }
            count += 1;
        }
        if count == 2 {
            let (ra, rb) = (find(&mut parent, hits[0]), find(&mut parent, hits[1]));
            if ra != rb {
                parent[ra] = rb;
            }
        }
    }
    let Some(nearest) = (0..crossings.len()).min_by(|&a, &b| {
        (crossings[a] - *point)
            .length_squared()
            .total_cmp(&(crossings[b] - *point).length_squared())
    }) else {
        return Vec::new();
    };
    let root = find(&mut parent, nearest);
    (0..crossings.len())
        .filter(|&i| find(&mut parent, i) == root)
        .map(|i| crossings[i])
        .collect()
}

/// The creases between sibling limbs, measured on the mesh. A bone
/// qualifies when its parent has another child; its crease depth is the
/// first slice (stepping from the pivot toward the tail) whose loop
/// around the bone holds no surface nearer to a sibling limb than to
/// this one. Bones that separate within their own radius of the pivot
/// (no crease past the joint ring) or never within their length (buried
/// in the trunk) ask for nothing.
#[must_use]
pub fn sibling_creases(
    vertices: &[Vector3],
    triangles: &[Vec<usize>],
    bones: &[Bone],
) -> Vec<Crease> {
    let valid: Vec<bool> = bones.iter().map(Bone::valid).collect();
    let parent_of = |i: usize| bones[i].parent.filter(|&p| p < bones.len() && p != i);
    let mut children: Vec<Vec<usize>> = vec![Vec::new(); bones.len()];
    for i in 0..bones.len() {
        if let Some(p) = parent_of(i) {
            children[p].push(i);
        }
    }
    let mut creases = Vec::new();
    for (i, bone) in bones.iter().enumerate() {
        let Some(p) = parent_of(i) else { continue };
        if !valid[i]
            || bone.bend_degrees == 0.0
            || children[p].iter().filter(|&&c| valid[c]).count() < 2
        {
            continue;
        }
        // Bone -> 1 for this limb (the bone and everything below it), 2 for
        // a sibling limb, 0 for the trunk (parent chain), which is left out
        // of the territory test: near the pivot the trunk's own bones sit
        // closer to the limb's inner surface than the limb axis does.
        let mut limb = vec![0u8; bones.len()];
        for &root in &children[p] {
            let tag = if root == i { 1 } else { 2 };
            let mut stack = vec![root];
            while let Some(b) = stack.pop() {
                if limb[b] == 0 {
                    limb[b] = tag;
                    stack.extend(children[b].iter().copied());
                }
            }
        }
        let nearest_is_own = |q: &Vector3| {
            let mut best = (f64::INFINITY, 0u8);
            for (j, other) in bones.iter().enumerate() {
                if !valid[j] || limb[j] == 0 {
                    continue;
                }
                let d = segment_distance_squared(q, &other.head, &other.tail);
                if d < best.0 {
                    best = (d, limb[j]);
                }
            }
            best.1 == 1
        };
        let length = bone.length();
        let axis = (bone.tail - bone.head).normalized();
        for step in 0..CREASE_STEPS {
            let s = step as f64 / CREASE_STEPS as f64;
            let point = bone.head + axis * (length * s);
            let ring = nearest_slice_loop(vertices, triangles, &point, &axis);
            if ring.is_empty() || !ring.iter().all(&nearest_is_own) {
                continue;
            }
            // A limb that separates within its own radius of the pivot has
            // no crease beyond the joint's own ring.
            let radius =
                ring.iter().map(|q| (*q - point).length()).sum::<f64>() / ring.len() as f64;
            let depth = length * s;
            if depth > radius {
                creases.push(Crease {
                    bone: i,
                    depth,
                    bend_degrees: bone.bend_degrees,
                });
            }
            break;
        }
    }
    creases
}

/// Per-vertex density multipliers (1.0 = unchanged) from the skeleton's
/// sibling creases, for a mesh whose nominal quad edge is
/// `nominal_edge`, plus each crease and its ask. Overlapping zones take
/// the strongest ask.
#[must_use]
pub fn skeleton_density_field(
    vertices: &[Vector3],
    triangles: &[Vec<usize>],
    bones: &[Bone],
    nominal_edge: f64,
) -> (Vec<f64>, Vec<(Crease, f64)>) {
    let asks: Vec<(Crease, f64)> = sibling_creases(vertices, triangles, bones)
        .into_iter()
        .map(|crease| (crease, crease.multiplier(nominal_edge)))
        .collect();
    let zones: Vec<(Vector3, f64, f64)> = asks
        .iter()
        .filter(|(_, ask)| *ask < 1.0)
        .map(|(crease, ask)| {
            (
                bones[crease.bone].head,
                ZONE_PER_CREASE * crease.depth,
                *ask,
            )
        })
        .collect();
    let plateau = 1.0 / ZONE_PER_CREASE;
    let field = vertices
        .iter()
        .map(|vertex| {
            let mut best = 1.0f64;
            for &(center, radius, ask) in &zones {
                let distance_squared = (*vertex - center).length_squared();
                if distance_squared >= radius * radius {
                    continue;
                }
                let falloff =
                    smoothstep((1.0 - distance_squared.sqrt() / radius) / (1.0 - plateau));
                best = best.min(1.0 + (ask - 1.0) * falloff);
            }
            best
        })
        .collect();
    (field, asks)
}

/// Combine a skeleton field with a user `--density` mask: multiply where
/// both exist (the engine clamps the product to the supported range).
/// An empty side means "absent".
#[must_use]
pub fn combine_fields(user: &[f64], skeleton: &[f64]) -> Vec<f64> {
    match (user.is_empty(), skeleton.is_empty()) {
        (true, _) => skeleton.to_vec(),
        (false, true) => user.to_vec(),
        (false, false) => user.iter().zip(skeleton).map(|(a, b)| a * b).collect(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Open tube along z: `rings` x `segments`.
    #[allow(clippy::too_many_arguments)]
    fn tube(
        verts: &mut Vec<Vector3>,
        tris: &mut Vec<Vec<usize>>,
        center: Vector3,
        radius: f64,
        z0: f64,
        z1: f64,
        rings: usize,
        segments: usize,
    ) {
        let base = verts.len();
        for r in 0..=rings {
            let z = z0 + (z1 - z0) * r as f64 / rings as f64;
            for k in 0..segments {
                let a = std::f64::consts::TAU * k as f64 / segments as f64;
                verts.push(Vector3::new(
                    center.x() + radius * a.cos(),
                    center.y() + radius * a.sin(),
                    z,
                ));
            }
        }
        for r in 0..rings {
            for k in 0..segments {
                let a = base + r * segments + k;
                let b = base + r * segments + (k + 1) % segments;
                let c = a + segments;
                let d = b + segments;
                tris.push(vec![a, b, d]);
                tris.push(vec![a, d, c]);
            }
        }
    }

    fn bone(
        head: (f64, f64, f64),
        tail: (f64, f64, f64),
        parent: Option<usize>,
        bend: f64,
    ) -> Bone {
        Bone {
            head: Vector3::new(head.0, head.1, head.2),
            tail: Vector3::new(tail.0, tail.1, tail.2),
            parent,
            bend_degrees: bend,
        }
    }

    /// A pelvis (one wide tube from the crotch up to 1.4) over two legs
    /// (z 0 up to the crotch); hip pivots at z 1.0.
    fn pants(crotch: f64) -> (Vec<Vector3>, Vec<Vec<usize>>, Vec<Bone>) {
        let mut v = Vec::new();
        let mut t = Vec::new();
        tube(
            &mut v,
            &mut t,
            Vector3::new(0.0, 0.0, 0.0),
            0.3,
            crotch,
            1.4,
            24,
            48,
        );
        for x in [-0.15, 0.15] {
            tube(
                &mut v,
                &mut t,
                Vector3::new(x, 0.0, 0.0),
                0.1,
                0.0,
                crotch,
                24,
                24,
            );
        }
        let bones = vec![
            bone((0.0, 0.0, 1.0), (0.0, 0.0, 1.1), None, 0.0),
            bone((-0.15, 0.0, 1.0), (-0.15, 0.0, 0.5), Some(0), 90.0),
            bone((0.15, 0.0, 1.0), (0.15, 0.0, 0.5), Some(0), 90.0),
            bone((0.0, 0.0, 1.1), (0.0, 0.0, 1.4), Some(0), 45.0),
        ];
        (v, t, bones)
    }

    #[test]
    fn crease_depth_follows_the_crotch() {
        let (v, t, bones) = pants(0.8);
        let creases = sibling_creases(&v, &t, &bones);
        let legs: Vec<&Crease> = creases
            .iter()
            .filter(|c| c.bone == 1 || c.bone == 2)
            .collect();
        assert_eq!(legs.len(), 2, "{creases:?}");
        for c in legs {
            // The legs separate just below the crotch (0.2 below the pivot).
            assert!(c.depth > 0.19 && c.depth < 0.24, "{c:?}");
        }
        // A deeper crotch, a deeper crease.
        let (v2, t2, bones2) = pants(0.6);
        let deep = sibling_creases(&v2, &t2, &bones2);
        let d = deep.iter().find(|c| c.bone == 1).unwrap().depth;
        assert!(d > 0.39 && d < 0.44, "{deep:?}");
    }

    #[test]
    fn free_limbs_and_lone_children_ask_nothing() {
        // Legs that leave the body at the pivot: no crease.
        let (v, t, bones) = pants(1.0);
        let creases = sibling_creases(&v, &t, &bones);
        assert!(
            creases.iter().all(|c| c.bone != 1 && c.bone != 2),
            "{creases:?}"
        );
        // A chain without siblings: nothing to measure.
        let chain = vec![
            bone((-0.15, 0.0, 1.0), (-0.15, 0.0, 0.5), None, 0.0),
            bone((-0.15, 0.0, 0.5), (-0.15, 0.0, 0.0), Some(0), 140.0),
        ];
        let (v2, t2, _) = pants(0.8);
        assert!(sibling_creases(&v2, &t2, &chain).is_empty());
    }

    #[test]
    fn ask_only_coarsens_and_is_floored() {
        let c = Crease {
            bone: 0,
            depth: 0.5,
            bend_degrees: 90.0,
        }; // edge ~0.131
        assert_eq!(c.multiplier(0.2), 1.0);
        let m = c.multiplier(0.1);
        assert!(m < 1.0 && m > MIN_MULTIPLIER);
        assert!((m - (0.1 / c.edge_length()).powi(2)).abs() < 1e-12);
        assert_eq!(c.multiplier(0.01), MIN_MULTIPLIER);
        let still = Crease {
            bone: 0,
            depth: 0.5,
            bend_degrees: 0.0,
        };
        assert_eq!(still.multiplier(0.01), 1.0);
        assert_eq!(c.multiplier(0.0), 1.0);
    }

    #[test]
    fn field_is_scale_free() {
        let (v, t, bones) = pants(0.7);
        let scale = |k: f64| -> (Vec<Vector3>, Vec<Bone>) {
            (
                v.iter().map(|p| *p * k).collect(),
                bones
                    .iter()
                    .map(|b| Bone {
                        head: b.head * k,
                        tail: b.tail * k,
                        ..*b
                    })
                    .collect(),
            )
        };
        let (v1, b1) = scale(1.0);
        let (v2, b2) = scale(37.0);
        let n1 = nominal_edge_length(&v1, &t, 4000);
        let n2 = nominal_edge_length(&v2, &t, 4000);
        let (f1, a1) = skeleton_density_field(&v1, &t, &b1, n1);
        let (f2, a2) = skeleton_density_field(&v2, &t, &b2, n2);
        assert_eq!(a1.len(), a2.len());
        assert!(
            f1.iter().any(|&m| m < 1.0),
            "the deep crotch coarsens: {a1:?}"
        );
        for (x, y) in f1.iter().zip(&f2) {
            assert!((x - y).abs() < 1e-6);
        }
    }

    #[test]
    fn nominal_edge_from_area() {
        let v = vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
            Vector3::new(1.0, 1.0, 0.0),
        ];
        let t = vec![vec![0, 1, 2], vec![1, 3, 2]];
        assert!((nominal_edge_length(&v, &t, 100) - 0.1).abs() < 1e-12);
        assert_eq!(nominal_edge_length(&v, &t, 0), 0.0);
    }

    #[test]
    fn combine_multiplies_and_keeps_absent_sides() {
        assert_eq!(combine_fields(&[], &[0.5]), vec![0.5]);
        assert_eq!(combine_fields(&[2.0], &[]), vec![2.0]);
        assert_eq!(combine_fields(&[0.5, 2.0], &[2.0, 0.5]), vec![1.0, 1.0]);
    }
}
