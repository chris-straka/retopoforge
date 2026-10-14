use super::QuadExtractor;
use crate::mesh_separator::MeshSeparator;
use crate::vector2::Vector2;
use crate::vector3::Vector3;
use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet};

impl<'a> QuadExtractor<'a> {
    fn test_point_in_triangle(
        points: &[Vector3],
        triangle: &[usize],
        test_points: &[usize],
    ) -> bool {
        let triangle_normal = Vector3::normal(
            &points[triangle[0]],
            &points[triangle[1]],
            &points[triangle[2]],
        );
        let mut points_in_3d = Vec::new();
        for it in triangle {
            points_in_3d.push(points[*it]);
        }
        for it in test_points {
            points_in_3d.push(points[*it]);
        }
        let mut points_in_2d = Vec::new();
        let origin = (points[triangle[0]] + points[triangle[1]] + points[triangle[2]]) / 3.0;
        let axis = (points[triangle[0]] - origin).normalized();
        Vector3::project_to_2d(
            &points_in_3d,
            &mut points_in_2d,
            &triangle_normal,
            &axis,
            &origin,
        );
        let a = points_in_2d[0];
        let b = points_in_2d[1];
        let c = points_in_2d[2];
        for point in points_in_2d.iter().skip(3) {
            if Vector2::is_in_triangle(&a, &b, &c, point) {
                return true;
            }
        }
        false
    }

    pub(crate) fn rebuild_half_edges(&mut self) {
        self.half_edges.clear();
        for face in &self.remeshed_polygons {
            for i in 0..face.len() {
                let j = (i + 1) % face.len();
                self.half_edges.insert((face[i], face[j]));
            }
        }
    }

    pub(crate) fn fix_holes(&mut self) {
        let mut loops = Vec::new();
        // Restructure: cloned (the C++ passes the member by const ref while
        // `fix_hole_with_quads` mutates it through `this`).
        self.search_boundaries(&self.half_edges.clone(), &mut loops);
        for loop_ in &mut loops {
            if loop_.len() > 65 {
                self.diagnose(|| format!("Ignore long hole at length:{}\n", loop_.len()));
                continue;
            }
            self.diagnose(|| format!("Fixing hole at length:{}...\n", loop_.len()));
            self.fix_hole_with_quads(loop_, true);
            // fixHoleWithQuads returns with 3 or 4 hole verts left only
            // after pushing that final cap, so a second pass there re-emits
            // the identical quad (doubled face, use-count-3 edges).
            // Continue only on a genuine uncapped remainder.
            if loop_.len() > 4 {
                self.fix_hole_with_quads(loop_, false);
            }
        }
    }

    fn record_half_edges_of_last_polygon(&mut self) {
        let last = self.remeshed_polygons.len() - 1;
        for i in 0..self.remeshed_polygons[last].len() {
            let face_len = self.remeshed_polygons[last].len();
            let j = (i + 1) % face_len;
            let edge = (
                self.remeshed_polygons[last][i],
                self.remeshed_polygons[last][j],
            );
            self.half_edges.insert(edge);
        }
    }

    fn fix_hole_with_quads(&mut self, hole: &mut Vec<usize>, check_score: bool) {
        loop {
            if hole.len() <= 2 {
                self.diagnose(|| {
                    format!("fixHoleWithQuads cancel on edge length:{}\n", hole.len())
                });
                return;
            }

            if 3 == hole.len() {
                self.remeshed_polygons.push(vec![hole[2], hole[1], hole[0]]);
                self.record_half_edges_of_last_polygon();
                return;
            }

            if 4 == hole.len() {
                self.remeshed_polygons
                    .push(vec![hole[3], hole[2], hole[1], hole[0]]);
                self.record_half_edges_of_last_polygon();
                return;
            }

            let mut edge_scores = Vec::with_capacity(hole.len());
            for i in 0..hole.len() {
                let h = (i + hole.len() - 1) % hole.len();
                let j = (i + 1) % hole.len();
                let k = (j + 1) % hole.len();
                let left = (self.remeshed_vertices[hole[h]] - self.remeshed_vertices[hole[i]])
                    .normalized();
                let right = (self.remeshed_vertices[hole[k]] - self.remeshed_vertices[hole[j]])
                    .normalized();
                edge_scores.push((i, Vector3::dot_product(&left, &right)));
            }
            // Restructure: `std::sort` (libc++ introsort) becomes
            // `sort_unstable_by` (pdqsort). Both deterministic, but they
            // order score ties differently; the oracle measures the fallout
            // on symmetric holes (see the report).
            edge_scores.sort_unstable_by(|first, second| {
                first.1.partial_cmp(&second.1).unwrap_or(Ordering::Equal)
            });
            let mut hole_changed = false;
            for edge_index in (0..edge_scores.len()).rev() {
                let score = edge_scores[edge_index];
                if check_score && score.1 <= 0.0 {
                    self.diagnose(|| {
                        format!("fixHoleWithQuads failed, highest score(dot):{}\n", score.1)
                    });
                    return;
                }
                let i = score.0;
                let h = (i + hole.len() - 1) % hole.len();
                let j = (i + 1) % hole.len();
                let k = (j + 1) % hole.len();
                let candidate = vec![hole[k], hole[j], hole[i], hole[h]];
                if self.half_edges.contains(&(candidate[0], candidate[1]))
                    || self.half_edges.contains(&(candidate[1], candidate[2]))
                    || self.half_edges.contains(&(candidate[2], candidate[3]))
                    || self.half_edges.contains(&(candidate[3], candidate[0]))
                {
                    self.diagnose(|| {
                        format!(
                            "fixHoleWithQuads ignore score:{} because conflicts with existed quads\n",
                            score.1
                        )
                    });
                    continue;
                }
                let mut remain_points = Vec::new();
                for (w, point) in hole.iter().enumerate() {
                    if w == i || w == j || w == h || w == k {
                        continue;
                    }
                    remain_points.push(*point);
                }
                if Self::test_point_in_triangle(
                    &self.remeshed_vertices,
                    &[candidate[0], candidate[1], candidate[2]],
                    &remain_points,
                ) || Self::test_point_in_triangle(
                    &self.remeshed_vertices,
                    &[candidate[2], candidate[3], candidate[0]],
                    &remain_points,
                ) {
                    self.diagnose(|| {
                        format!(
                            "fixHoleWithQuads ignore score:{} because other point in the same loop fall into quad plane\n",
                            score.1
                        )
                    });
                    continue;
                }
                self.remeshed_polygons.push(candidate);
                self.record_half_edges_of_last_polygon();

                let mut new_hole = Vec::new();
                for (w, point) in hole.iter().enumerate() {
                    if w == i || w == j {
                        continue;
                    }
                    new_hole.push(*point);
                }
                *hole = new_hole;
                hole_changed = true;
                break;
            }
            if !hole_changed {
                break;
            }
        }
    }

    fn search_boundaries(
        &self,
        half_edges: &BTreeSet<(usize, usize)>,
        loops: &mut Vec<Vec<usize>>,
    ) {
        self.diagnose(|| "Searching boundaries...\n".to_string());

        let mut next_map: BTreeMap<usize, BTreeSet<usize>> = BTreeMap::new();
        for (from, to) in half_edges {
            if half_edges.contains(&(*to, *from)) {
                continue;
            }
            next_map.entry(*from).or_default().insert(*to);
        }

        while !next_map.is_empty() {
            // Loops start at the smallest open vertex (sorted order).
            let Some((&start_vertex, _)) = next_map.first_key_value() else {
                break;
            };
            let mut loop_ = Vec::new();
            let mut validate = false;
            self.diagnose(|| format!("Searching loop from:{start_vertex}\n"));
            let mut current = Some(start_vertex);
            while let Some(vertex) = current {
                if start_vertex == vertex && loop_.len() >= 3 {
                    self.diagnose(|| format!("Found valid loop, size:{}\n", loop_.len()));
                    validate = true;
                    break;
                }
                self.diagnose(|| format!("Loop add vertex:{vertex}\n"));
                loop_.push(vertex);
                let Some(nexts) = next_map.get(&vertex) else {
                    break;
                };
                if nexts.len() != 1 {
                    self.diagnose(|| format!("Break loop, because of next size:{}\n", nexts.len()));
                    break;
                }
                // Invariant: exactly one next vertex (checked above).
                current = nexts.iter().next().copied();
            }
            for v in &loop_ {
                next_map.remove(v);
            }
            if validate {
                loops.push(loop_);
            }
        }

        self.diagnose(|| "Searching boundaries done\n".to_string());
    }

    pub(crate) fn remove_isolated_faces(&mut self) -> bool {
        let mut quads_islands = Vec::new();
        MeshSeparator::split_to_islands(&self.remeshed_polygons, &mut quads_islands);
        if quads_islands.is_empty() {
            return false;
        }
        // `std::max_element` returns the FIRST maximum; Rust's `max_by`
        // returns the last, so this stays a manual strict-`<` loop.
        let mut biggest = 0;
        for (index, island) in quads_islands.iter().enumerate() {
            if quads_islands[biggest].len() < island.len() {
                biggest = index;
            }
        }
        self.remeshed_polygons = std::mem::take(&mut quads_islands[biggest]);
        true
    }

    pub(crate) fn remove_non_manifold_faces(&mut self) -> bool {
        let mut changed = false;
        let mut edge_to_face_map = BTreeMap::new();
        MeshSeparator::build_edge_to_face_map(&self.remeshed_polygons, &mut edge_to_face_map);
        let mut vertex_open_boundary_count_map: BTreeMap<usize, usize> = BTreeMap::new();
        for edge in edge_to_face_map.keys() {
            if edge_to_face_map.contains_key(&(edge.1, edge.0)) {
                continue;
            }
            *vertex_open_boundary_count_map.entry(edge.0).or_default() += 1;
            *vertex_open_boundary_count_map.entry(edge.1).or_default() += 1;
        }
        let mut manifold_faces = Vec::new();
        for face in &self.remeshed_polygons {
            let mut is_non_manifold = false;
            for vertex in face {
                let Some(find_count) = vertex_open_boundary_count_map.get(vertex) else {
                    continue;
                };
                if *find_count > 2 {
                    is_non_manifold = true;
                    break;
                }
            }
            if is_non_manifold {
                changed = true;
                continue;
            }
            manifold_faces.push(face.clone());
        }
        self.remeshed_polygons = manifold_faces;
        changed
    }
}
