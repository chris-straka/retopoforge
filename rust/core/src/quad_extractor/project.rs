use super::QuadExtractor;
use super::geom::{add_scaled_vec3, add_two_scaled_vec3, fma_first, fma_first_sub};
use crate::iso_remesh_kernel::{AxisAlignedBoundingBox, AxisAlignedBoundingBoxTree};
use crate::par::parallel_each;
use crate::vector2::Vector2;
use crate::vector3::Vector3;
use std::collections::{BTreeMap, BTreeSet};

impl<'a> QuadExtractor<'a> {
    pub(crate) fn smooth_and_project(
        &mut self,
        iterations: usize,
        // Membership-only use (`contains`, `is_empty`).
        movable_vertices: Option<&BTreeSet<usize>>,
    ) {
        if 0 == iterations
            || self.remeshed_vertices.is_empty()
            || self.remeshed_polygons.is_empty()
            || self.triangles.is_empty()
        {
            return;
        }

        let mut neighbors: Vec<BTreeSet<usize>> =
            vec![BTreeSet::new(); self.remeshed_vertices.len()];
        let mut edge_use_count: BTreeMap<(usize, usize), usize> = BTreeMap::new();
        for face in &self.remeshed_polygons {
            for i in 0..face.len() {
                let j = (i + 1) % face.len();
                if face[i] >= neighbors.len() || face[j] >= neighbors.len() {
                    continue;
                }
                neighbors[face[i]].insert(face[j]);
                neighbors[face[j]].insert(face[i]);
                *edge_use_count
                    .entry((face[i].min(face[j]), face[i].max(face[j])))
                    .or_default() += 1;
            }
        }
        let mut locked = vec![false; self.remeshed_vertices.len()];
        if let Some(movable) = movable_vertices {
            for (index, lock) in locked.iter_mut().enumerate() {
                *lock = !movable.contains(&index);
            }
        }
        for (edge, use_count) in &edge_use_count {
            if 1 == *use_count {
                locked[edge.0] = true;
                locked[edge.1] = true;
            }
        }

        let mut triangle_boxes = vec![AxisAlignedBoundingBox::default(); self.triangles.len()];
        let mut triangle_indices = vec![0; self.triangles.len()];
        let mut group_box = AxisAlignedBoundingBox::default();
        for (i, triangle) in self.triangles.iter().enumerate() {
            for k in 0..3 {
                triangle_boxes[i].update(&self.vertices[triangle[k]]);
                group_box.update(&self.vertices[triangle[k]]);
            }
            triangle_boxes[i].update_center();
            triangle_indices[i] = i;
        }
        group_box.update_center();
        let tree = AxisAlignedBoundingBoxTree::new(triangle_boxes, triangle_indices, group_box);

        // Average quad edge length drives the initial search radius
        let mut total_edge_length = 0.0;
        let mut edge_num = 0;
        for edge in edge_use_count.keys() {
            total_edge_length +=
                (self.remeshed_vertices[edge.0] - self.remeshed_vertices[edge.1]).length();
            edge_num += 1;
        }
        if 0 == edge_num {
            return;
        }
        let average_edge_length = total_edge_length / edge_num as f64;
        if average_edge_length <= 0.0 {
            return;
        }

        // Both passes below already read one buffer and write another, and
        // the projection only reads the bounding box tree, so each vertex
        // is independent and the parallel result is the same as the serial
        // one. (Restructure: `parallel_each` for `tbb::parallel_for`.)
        const SMOOTH_FACTOR: f64 = 0.5;
        for _ in 0..iterations {
            let mut smoothed_vertices = self.remeshed_vertices.clone();
            parallel_each(&mut smoothed_vertices, |i, smoothed| {
                if locked[i] || neighbors[i].is_empty() {
                    return;
                }
                let mut center = Vector3::default();
                for neighbor in &neighbors[i] {
                    center += self.remeshed_vertices[*neighbor];
                }
                center /= neighbors[i].len() as f64;
                // Unfused `v + smoothFactor * (center - v)` (see module FMA note).
                *smoothed = add_scaled_vec3(
                    self.remeshed_vertices[i],
                    center - self.remeshed_vertices[i],
                    SMOOTH_FACTOR,
                );
            });
            parallel_each(&mut smoothed_vertices, |i, smoothed| {
                if locked[i] || neighbors[i].is_empty() {
                    return;
                }
                if let Some(projected) = Self::project_to_target_mesh(
                    self.vertices,
                    self.triangles,
                    &tree,
                    average_edge_length,
                    *smoothed,
                ) {
                    *smoothed = projected;
                }
            });
            self.remeshed_vertices = smoothed_vertices;
        }
    }

    fn project_to_target_mesh(
        vertices: &[Vector3],
        triangles: &[Vec<usize>],
        tree: &AxisAlignedBoundingBoxTree,
        average_edge_length: f64,
        position: Vector3,
    ) -> Option<Vector3> {
        let mut radius = average_edge_length;
        while radius <= average_edge_length * 8.0 {
            let mut query_boxes = vec![AxisAlignedBoundingBox::default()];
            query_boxes[0].update(&Vector3::new(
                position.x() - radius,
                position.y() - radius,
                position.z() - radius,
            ));
            query_boxes[0].update(&Vector3::new(
                position.x() + radius,
                position.y() + radius,
                position.z() + radius,
            ));
            query_boxes[0].update_center();
            let outer = query_boxes[0].clone();
            let query_tree = AxisAlignedBoundingBoxTree::new(query_boxes, vec![0], outer);
            let mut pairs = Vec::new();
            tree.test(tree.root(), &query_tree, query_tree.root(), &mut pairs);
            let mut min_distance2 = f64::MAX;
            let mut projected = None;
            for (key, _) in &pairs {
                let triangle = &triangles[*key];
                let candidate = closest_point_on_triangle(
                    position,
                    vertices[triangle[0]],
                    vertices[triangle[1]],
                    vertices[triangle[2]],
                );
                let distance2 = (candidate - position).length_squared();
                if distance2 < min_distance2 {
                    min_distance2 = distance2;
                    projected = Some(candidate);
                }
            }
            if min_distance2 < f64::MAX {
                return projected;
            }
            radius *= 2.0;
        }
        None
    }

    pub(crate) fn compute_remeshed_vertex_uvs(&mut self) {
        self.remeshed_vertex_uvs.clear();
        if self.remeshed_vertices.is_empty()
            || self.triangles.is_empty()
            || self.triangle_uvs.len() != self.triangles.len()
        {
            return;
        }

        // Bounding-box tree over the source triangles, same construction as
        // smoothAndProject: output vertices lie on the source surface, so
        // an expanding-radius query finds the home triangle in a few
        // probes.
        let mut triangle_boxes = vec![AxisAlignedBoundingBox::default(); self.triangles.len()];
        let mut triangle_indices = vec![0; self.triangles.len()];
        let mut group_box = AxisAlignedBoundingBox::default();
        for (i, triangle) in self.triangles.iter().enumerate() {
            for k in 0..3 {
                triangle_boxes[i].update(&self.vertices[triangle[k]]);
                group_box.update(&self.vertices[triangle[k]]);
            }
            triangle_boxes[i].update_center();
            triangle_indices[i] = i;
        }
        group_box.update_center();
        let tree = AxisAlignedBoundingBoxTree::new(triangle_boxes, triangle_indices, group_box);

        let mut total_edge_length = 0.0;
        let mut edge_num = 0;
        for face in &self.remeshed_polygons {
            for i in 0..face.len() {
                let j = (i + 1) % face.len();
                if face[i] >= self.remeshed_vertices.len()
                    || face[j] >= self.remeshed_vertices.len()
                {
                    continue;
                }
                total_edge_length +=
                    (self.remeshed_vertices[face[i]] - self.remeshed_vertices[face[j]]).length();
                edge_num += 1;
            }
        }
        if 0 == edge_num {
            return;
        }
        let average_edge_length = total_edge_length / edge_num as f64;
        if average_edge_length <= 0.0 {
            return;
        }

        // Each vertex is independent (reads the tree and the source mesh,
        // writes its own slot), so the parallel result matches the serial
        // one exactly. (Restructure: `parallel_each` for `tbb::parallel_for`.)
        let mut uvs = vec![Vector2::default(); self.remeshed_vertices.len()];
        parallel_each(&mut uvs, |i, uv| {
            let mut projected = self.remeshed_vertices[i];
            let home = Self::find_home_triangle(
                self.vertices,
                self.triangles,
                &tree,
                average_edge_length,
                self.remeshed_vertices[i],
                &mut projected,
            );
            let triangle = &self.triangles[home];
            let corner_uvs = &self.triangle_uvs[home];
            if corner_uvs.len() < 3 {
                *uv = if corner_uvs.is_empty() {
                    Vector2::default()
                } else {
                    corner_uvs[0]
                };
                return;
            }
            let area = Vector3::area(
                &self.vertices[triangle[0]],
                &self.vertices[triangle[1]],
                &self.vertices[triangle[2]],
            );
            // `!(area > ...)` like the C++ (NaN takes this arm on both sides).
            #[allow(clippy::neg_cmp_op_on_partial_ord)]
            if !(area > 1e-18) {
                *uv = Vector2::new(
                    (corner_uvs[0].x() + corner_uvs[1].x() + corner_uvs[2].x()) / 3.0,
                    (corner_uvs[0].y() + corner_uvs[1].y() + corner_uvs[2].y()) / 3.0,
                );
                return;
            }
            let bary = Vector3::barycentric_coordinates(
                &self.vertices[triangle[0]],
                &self.vertices[triangle[1]],
                &self.vertices[triangle[2]],
                &projected,
            );
            // FMA: three left-nested products fuse outside-in, `fma(bz,
            // uz, fma(bx, ux, by * uy))` per the probe (same shape as the
            // dot-product chain).
            *uv = Vector2::new(
                bary.z().mul_add(
                    corner_uvs[2].x(),
                    fma_first(bary.x(), corner_uvs[0].x(), bary.y(), corner_uvs[1].x()),
                ),
                bary.z().mul_add(
                    corner_uvs[2].y(),
                    fma_first(bary.x(), corner_uvs[0].y(), bary.y(), corner_uvs[1].y()),
                ),
            );
        });

        // Normalize to 0..1 over this island's UV bounding box. Non-finite
        // interpolants (a degenerate parameterization corner) collapse to
        // the box center so the accessor never emits NaN or infinity.
        let mut min_u = 0.0;
        let mut max_u = 0.0;
        let mut min_v = 0.0;
        let mut max_v = 0.0;
        let mut have_finite = false;
        for uv in &uvs {
            if !uv.x().is_finite() || !uv.y().is_finite() {
                continue;
            }
            if !have_finite {
                min_u = uv.x();
                max_u = uv.x();
                min_v = uv.y();
                max_v = uv.y();
                have_finite = true;
            } else {
                min_u = min_u.min(uv.x());
                max_u = max_u.max(uv.x());
                min_v = min_v.min(uv.y());
                max_v = max_v.max(uv.y());
            }
        }
        if !have_finite {
            uvs = vec![Vector2::new(0.5, 0.5); uvs.len()];
            self.remeshed_vertex_uvs = uvs;
            return;
        }
        let range_u = max_u - min_u;
        let range_v = max_v - min_v;
        let center_u = (min_u + max_u) * 0.5;
        let center_v = (min_v + max_v) * 0.5;
        for uv in &mut uvs {
            let mut u = if uv.x().is_finite() { uv.x() } else { center_u };
            let mut v = if uv.y().is_finite() { uv.y() } else { center_v };
            u = if range_u > 1e-12 {
                (u - min_u) / range_u
            } else {
                0.5
            };
            v = if range_v > 1e-12 {
                (v - min_v) / range_v
            } else {
                0.5
            };
            *uv = Vector2::new(u, v);
        }
        self.remeshed_vertex_uvs = uvs;
    }

    #[allow(clippy::too_many_arguments)]
    fn find_home_triangle(
        vertices: &[Vector3],
        triangles: &[Vec<usize>],
        tree: &AxisAlignedBoundingBoxTree,
        average_edge_length: f64,
        position: Vector3,
        projected: &mut Vector3,
    ) -> usize {
        let mut home = 0;
        let mut radius = average_edge_length;
        while radius <= average_edge_length * 8.0 {
            let mut query_boxes = vec![AxisAlignedBoundingBox::default()];
            query_boxes[0].update(&Vector3::new(
                position.x() - radius,
                position.y() - radius,
                position.z() - radius,
            ));
            query_boxes[0].update(&Vector3::new(
                position.x() + radius,
                position.y() + radius,
                position.z() + radius,
            ));
            query_boxes[0].update_center();
            let outer = query_boxes[0].clone();
            let query_tree = AxisAlignedBoundingBoxTree::new(query_boxes, vec![0], outer);
            let mut pairs = Vec::new();
            tree.test(tree.root(), &query_tree, query_tree.root(), &mut pairs);
            let mut min_distance2 = f64::MAX;
            for (key, _) in &pairs {
                let triangle = &triangles[*key];
                let candidate = closest_point_on_triangle(
                    position,
                    vertices[triangle[0]],
                    vertices[triangle[1]],
                    vertices[triangle[2]],
                );
                let distance2 = (candidate - position).length_squared();
                if distance2 < min_distance2 {
                    min_distance2 = distance2;
                    *projected = candidate;
                    home = *key;
                }
            }
            if min_distance2 < f64::MAX {
                return home;
            }
            radius *= 2.0;
        }
        // Straggler (smoothing pushed it past the search radius): fall back
        // to a full scan so every vertex still gets a home triangle.
        let mut min_distance2 = f64::MAX;
        for (key, triangle) in triangles.iter().enumerate() {
            let candidate = closest_point_on_triangle(
                position,
                vertices[triangle[0]],
                vertices[triangle[1]],
                vertices[triangle[2]],
            );
            let distance2 = (candidate - position).length_squared();
            if distance2 < min_distance2 {
                min_distance2 = distance2;
                *projected = candidate;
                home = key;
            }
        }
        home
    }
}

/// Closest point on a triangle (mirrors the anonymous-namespace
/// `closestPointOnTriangle`; FMA per the module note).
fn closest_point_on_triangle(p: Vector3, a: Vector3, b: Vector3, c: Vector3) -> Vector3 {
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

    let vc = fma_first_sub(d1, d4, d3, d2);
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let denom = d1 - d3;
        let v = if 0.0 != denom { d1 / denom } else { 0.0 };
        return add_scaled_vec3(a, ab, v);
    }

    let cp = p - c;
    let d5 = Vector3::dot_product(&ab, &cp);
    let d6 = Vector3::dot_product(&ac, &cp);
    if d6 >= 0.0 && d5 <= d6 {
        return c;
    }

    let vb = fma_first_sub(d5, d2, d1, d6);
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let denom = d2 - d6;
        let w = if 0.0 != denom { d2 / denom } else { 0.0 };
        return add_scaled_vec3(a, ac, w);
    }

    let va = fma_first_sub(d3, d6, d5, d4);
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let denom = (d4 - d3) + (d5 - d6);
        let w = if 0.0 != denom { (d4 - d3) / denom } else { 0.0 };
        return add_scaled_vec3(b, c - b, w);
    }

    let denom = va + vb + vc;
    if 0.0 == denom {
        return a;
    }
    let v = vb / denom;
    let w = vc / denom;
    add_two_scaled_vec3(a, ab, v, ac, w)
}
