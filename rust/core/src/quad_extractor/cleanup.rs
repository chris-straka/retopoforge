use super::CleanupRoute;
use super::QuadExtractor;
use super::types::EdgeFaceIndex;
use crate::par::parallel_each;
use crate::vector3::Vector3;
use std::collections::{BTreeMap, BTreeSet};

impl<'a> QuadExtractor<'a> {
    /// Newell's-method fan normal over raw positions (the cleanup pass
    /// compares before/after positions, not mesh faces).
    fn polygon_normal(positions: &[Vector3]) -> Vector3 {
        let mut normal = Vector3::default();
        if positions.len() < 3 {
            return normal;
        }
        for i in 1..positions.len() - 1 {
            normal += Vector3::cross_product(
                &(positions[i] - positions[0]),
                &(positions[i + 1] - positions[0]),
            );
        }
        normal
    }

    /// The cleanup pass's `walkRoute`: the ladder of rungs from
    /// `start_face` across `start_edge` to the nearest sink, or `None`
    /// when the strip folds back on itself or runs too long.
    ///
    /// Returns the rungs, the dissolved faces, and the sink face
    /// (`usize::MAX` for a border sink). The C++ fills out-params and
    /// reports success separately; the caller discards partial outs on
    /// failure, so one `Option` carries the same outcome.
    fn walk_cleanup_route(
        polygons: &[Vec<usize>],
        edge_faces: &EdgeFaceIndex,
        start_face: usize,
        start_edge: (usize, usize),
    ) -> Option<CleanupRoute> {
        const MAX_ROUTE_LENGTH: usize = 20;
        const NO_FACE: usize = usize::MAX;

        let mut rungs = Vec::new();
        let mut dissolved_faces = BTreeSet::new();
        dissolved_faces.insert(start_face);
        let mut sink_face = NO_FACE;
        let mut rung_vertices = BTreeSet::new();
        let mut current_face = start_face;
        let mut rung = start_edge;
        loop {
            if rungs.len() >= MAX_ROUTE_LENGTH {
                return None;
            }
            // Two rungs sharing a vertex would collapse into each
            // other
            if !rung_vertices.insert(rung.0) {
                return None;
            }
            if !rung_vertices.insert(rung.1) {
                return None;
            }
            rungs.push(rung);
            // Every rung is a real mesh edge, so the entry exists; a
            // missing one reads as a non-manifold stop either way.
            let incident = match edge_faces.get(&rung) {
                Some(incident) => incident,
                _ => return None,
            };
            if 1 == incident.len() {
                return Some((rungs, dissolved_faces, sink_face));
            }
            if 2 != incident.len() {
                return None;
            }
            let neighbor = if incident[0] == current_face {
                incident[1]
            } else {
                incident[0]
            };
            if dissolved_faces.contains(&neighbor) {
                return None;
            }
            let neighbor_face = &polygons[neighbor];
            if 4 != neighbor_face.len() {
                sink_face = neighbor;
                return Some((rungs, dissolved_faces, sink_face));
            }
            let mut entry = neighbor_face.len();
            for i in 0..neighbor_face.len() {
                if Self::edge_of(
                    neighbor_face[i],
                    neighbor_face[(i + 1) % neighbor_face.len()],
                ) == rung
                {
                    entry = i;
                    break;
                }
            }
            if entry >= neighbor_face.len() {
                return None;
            }
            dissolved_faces.insert(neighbor);
            current_face = neighbor;
            rung = Self::edge_of(
                neighbor_face[(entry + 2) % 4],
                neighbor_face[(entry + 3) % 4],
            );
        }
    }

    pub(crate) fn cleanup_triangles(&mut self) {
        self.cleanup_routes(false);
    }

    /// Final sweep for defects the pair-wise passes leave behind: isolated
    /// pentagons (and triangles with fresh routes after the later passes)
    /// collapse through a quad strip into a sink, exactly like the
    /// mid-pipeline triangle cleanup, except a pentagon start shrinks to
    /// a quad instead of dissolving. Deliberately emits no progress
    /// events: the engine oracle pins the progress sequence exactly.
    pub(crate) fn cleanup_residual_routes(&mut self) {
        self.cleanup_routes(true);
    }

    fn cleanup_routes(&mut self, pentagon_starts: bool) {
        if self.remeshed_vertices.is_empty() || self.remeshed_polygons.is_empty() {
            return;
        }

        const NO_FACE: usize = usize::MAX;

        // The route of a triangle is the ladder of edges crossing the
        // quad strip that leads to the nearest sink, a border or another
        // non quad face. Every rung of the ladder collapses in the same
        // step: the strip closes up, the sink loses one side, and the
        // valence of the vertices along the way is left untouched
        // because each pair of side edges merges together with its rung.
        // Collapsing one rung at a time instead lets the surviving
        // vertex take part in the next collapse as well, which grows a
        // fan of slivers around a single point. A pentagon start works
        // the same way, except the start face gives up one side (5 -> 4)
        // instead of dissolving.
        let mut rejected_edges = BTreeSet::new();
        let mut collapsed_vertices = BTreeSet::new();
        let mut collapse_count = 0;
        let mut pentagon_count = 0;
        loop {
            // Sorted-vector rebuild (identical keys/values/order to the
            // `BTreeMap` build, a handful of allocs instead of 100k+).
            let edge_faces = EdgeFaceIndex::build(&self.remeshed_polygons);

            let mut route: Vec<(usize, usize)> = Vec::new();
            let mut route_faces = BTreeSet::new();
            let mut route_sink = NO_FACE;
            let mut route_start = NO_FACE;
            let mut route_start_len = 0;
            for start_face in 0..self.remeshed_polygons.len() {
                let starter = &self.remeshed_polygons[start_face];
                let starter_len = starter.len();
                if (3 != starter_len && !(pentagon_starts && 5 == starter_len))
                    || Self::has_repeated_vertex(starter)
                {
                    continue;
                }
                for i in 0..starter_len {
                    let start_edge = Self::edge_of(starter[i], starter[(i + 1) % starter_len]);
                    if rejected_edges.contains(&start_edge) {
                        continue;
                    }
                    let (candidate_route, candidate_faces, candidate_sink) =
                        match Self::walk_cleanup_route(
                            &self.remeshed_polygons,
                            &edge_faces,
                            start_face,
                            start_edge,
                        ) {
                            Some(found) => found,
                            _ => continue,
                        };
                    // The shorter the ladder, the less of the
                    // surrounding mesh it takes with it
                    if route.is_empty() || candidate_route.len() < route.len() {
                        route = candidate_route;
                        route_faces = candidate_faces;
                        route_sink = candidate_sink;
                        route_start = start_face;
                        route_start_len = starter_len;
                    }
                }
            }
            if route.is_empty() {
                break;
            }

            let mut merged_into: BTreeMap<usize, usize> = BTreeMap::new();
            let mut merged_positions: BTreeMap<usize, Vector3> = BTreeMap::new();
            for rung in &route {
                let (first, second) = *rung;
                // First wins (C++ `insert`); rungs are vertex-disjoint
                // (the walk rejects shared vertices), so the keys are
                // unique either way.
                merged_into.entry(second).or_insert(first);
                merged_positions.entry(first).or_insert(
                    (self.remeshed_vertices[first] + self.remeshed_vertices[second]) * 0.5,
                );
            }

            // Per-face remap is independent given the fixed route maps;
            // validation keeps its serial face-order scan (same checks,
            // same order, same early exit), so the outcome is identical.
            let mut remapped: Vec<(Vec<usize>, bool, bool)> =
                vec![(Vec::new(), false, false); self.remeshed_polygons.len()];
            parallel_each(&mut remapped, |face_index, slot| {
                let face = &self.remeshed_polygons[face_index];
                let mut candidate = Vec::with_capacity(face.len());
                let mut touched = false;
                for vertex in face {
                    let rewritten_vertex = merged_into.get(vertex).copied().unwrap_or(*vertex);
                    if rewritten_vertex != *vertex || merged_positions.contains_key(vertex) {
                        touched = true;
                    }
                    if candidate.is_empty() || candidate[candidate.len() - 1] != rewritten_vertex {
                        candidate.push(rewritten_vertex);
                    }
                }
                if candidate.len() > 1 && candidate[0] == candidate[candidate.len() - 1] {
                    candidate.pop();
                }
                // The strip faces and a triangle sink are meant to
                // disappear, everything else has to come out of the
                // collapse with the shape it went in with, apart from
                // the sink which gives up exactly one side. A pentagon
                // start is the exception to the strip rule: it gives up
                // one side instead of dissolving.
                let start_dissolves = 3 == route_start_len;
                let dissolving = (route_faces.contains(&face_index)
                    && (face_index != route_start || start_dissolves))
                    || (face_index == route_sink && 3 == face.len());
                *slot = (candidate, touched, dissolving);
            });
            let mut rewritten: Vec<Vec<usize>> = Vec::with_capacity(self.remeshed_polygons.len());
            let mut touched_faces = BTreeSet::new();
            let mut touched_edge_counts: BTreeMap<(usize, usize), usize> = BTreeMap::new();
            let mut valid = true;
            for (face_index, (candidate, touched, dissolving)) in remapped.into_iter().enumerate() {
                let face = &self.remeshed_polygons[face_index];
                if dissolving {
                    if candidate.len() >= 3 {
                        valid = false;
                        break;
                    }
                    continue;
                }
                let expected_size = if face_index == route_start && 5 == route_start_len {
                    route_start_len - 1
                } else if face_index == route_sink {
                    face.len() - 1
                } else {
                    face.len()
                };
                if candidate.len() != expected_size || Self::has_repeated_vertex(&candidate) {
                    valid = false;
                    break;
                }
                if touched {
                    let mut before = Vec::with_capacity(face.len());
                    for vertex in face {
                        before.push(self.remeshed_vertices[*vertex]);
                    }
                    let mut after = Vec::with_capacity(candidate.len());
                    for vertex in &candidate {
                        after.push(
                            merged_positions
                                .get(vertex)
                                .copied()
                                .unwrap_or(self.remeshed_vertices[*vertex]),
                        );
                    }
                    if Vector3::dot_product(
                        &Self::polygon_normal(&before),
                        &Self::polygon_normal(&after),
                    ) <= 0.0
                    {
                        valid = false;
                        break;
                    }
                    // A face that duplicates another one must share
                    // every vertex with it, so both of them are among
                    // the faces touched by the collapse
                    if !touched_faces.insert(Self::canonical_face(&candidate)) {
                        valid = false;
                        break;
                    }
                    for i in 0..candidate.len() {
                        let edge =
                            Self::edge_of(candidate[i], candidate[(i + 1) % candidate.len()]);
                        if !merged_positions.contains_key(&edge.0)
                            && !merged_positions.contains_key(&edge.1)
                        {
                            continue;
                        }
                        // Present-or-zero (C++ `operator[]` on a count
                        // map); the entry exists exactly when an
                        // earlier face touched the same edge.
                        let count = touched_edge_counts.entry(edge).or_insert(0);
                        *count += 1;
                        if *count > 2 {
                            valid = false;
                            break;
                        }
                    }
                    if !valid {
                        break;
                    }
                }
                rewritten.push(candidate);
            }
            if !valid {
                rejected_edges.insert(route[0]);
                continue;
            }

            for (vertex, position) in &merged_positions {
                self.remeshed_vertices[*vertex] = *position;
                collapsed_vertices.insert(*vertex);
            }
            self.remeshed_polygons = rewritten;
            collapse_count += 1;
            if 5 == route_start_len {
                pentagon_count += 1;
            }
        }

        if 0 == collapse_count {
            return;
        }

        let compacted = self.compact_vertices(&collapsed_vertices);

        if pentagon_starts {
            self.diagnose(|| {
                format!("Cleanup residual routes:{collapse_count} (pentagons:{pentagon_count})\n")
            });
        } else {
            self.diagnose(|| format!("Cleanup triangle faces:{collapse_count}\n"));
        }
        self.rebuild_half_edges();

        // The rungs met halfway, pull the closed up strips back onto the
        // source mesh
        self.smooth_around_vertices(&compacted, 3, 5);
    }

    pub(crate) fn smooth_around_vertices(
        &mut self,
        seed_vertices: &BTreeSet<usize>,
        rings: usize,
        iterations: usize,
    ) {
        if seed_vertices.is_empty() || self.remeshed_polygons.is_empty() {
            return;
        }

        let mut vertex_faces: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
        for (face_index, face) in self.remeshed_polygons.iter().enumerate() {
            for vertex in face {
                vertex_faces.entry(*vertex).or_default().push(face_index);
            }
        }

        // Rings of faces grown from the seed points, the vertices
        // sitting on the outer border of the patch anchor the smoothing
        let mut patch_faces = BTreeSet::new();
        let mut frontier = BTreeSet::new();
        for vertex in seed_vertices {
            if !vertex_faces.contains_key(vertex) {
                continue;
            }
            frontier.insert(*vertex);
        }
        // Set contents only: every iteration unions whole faces in, so
        // sorted iteration builds the same patch/border/movable sets as
        // the C++ hash order.
        for _ in 0..rings {
            if frontier.is_empty() {
                break;
            }
            let mut next_frontier = BTreeSet::new();
            for vertex in &frontier {
                // Present by construction (frontier vertices are kept
                // only when they have faces).
                for face_index in &vertex_faces[vertex] {
                    if !patch_faces.insert(*face_index) {
                        continue;
                    }
                    for neighbor_vertex in &self.remeshed_polygons[*face_index] {
                        next_frontier.insert(*neighbor_vertex);
                    }
                }
            }
            frontier = next_frontier;
        }

        let mut movable_vertices = BTreeSet::new();
        for face_index in &patch_faces {
            for vertex in &self.remeshed_polygons[*face_index] {
                movable_vertices.insert(*vertex);
            }
        }
        // Restructure: the C++ erases in place while iterating; the
        // mirror collects the border vertices first (same surviving
        // set), because Rust cannot erase during iteration.
        let mut anchored = Vec::new();
        for vertex in &movable_vertices {
            // Present by construction (movable vertices come from mesh
            // faces).
            let mut on_patch_border = false;
            for face_index in &vertex_faces[vertex] {
                if !patch_faces.contains(face_index) {
                    on_patch_border = true;
                    break;
                }
            }
            if on_patch_border {
                anchored.push(*vertex);
            }
        }
        for vertex in anchored {
            movable_vertices.remove(&vertex);
        }

        if movable_vertices.is_empty() {
            return;
        }

        self.smooth_and_project(iterations, Some(&movable_vertices));
    }
}
