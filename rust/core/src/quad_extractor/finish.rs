use super::QuadExtractor;
use super::types::{EdgeFaceIndex, NeighborIndex};
use crate::par::parallel_each;
use crate::progress::ProgressHandler;
use crate::vector3::Vector3;
use std::collections::{BTreeMap, BTreeSet};

impl<'a> QuadExtractor<'a> {
    pub(crate) fn merge_shared_five_edge_faces(&mut self, progress: Option<&ProgressHandler>) {
        if self.remeshed_vertices.is_empty() || self.remeshed_polygons.is_empty() {
            return;
        }

        // Every merge consumes a pair of pentagons, so half the pentagon
        // count is a real ceiling on how many rounds can succeed and
        // makes an honest denominator for the fraction reported below.
        let mut pentagon_count = 0;
        for face in &self.remeshed_polygons {
            if 5 == face.len() {
                pentagon_count += 1;
            }
        }
        let merge_ceiling = (pentagon_count / 2).max(1);

        // The merged point inherits the neighbors of both ends of the
        // collapsed edge. The pair of pentagons is worth trading for a
        // six valence point, past that the singularity left behind is a
        // worse defect than the faces it replaces
        const MAX_MERGED_VALENCE: usize = 6;

        let mut rejected_edges = BTreeSet::new();
        let mut merged_vertices = BTreeSet::new();
        let mut merge_count = 0;
        loop {
            if let Some(handler) = progress {
                handler(
                    f32::min(0.99, merge_count as f32 / merge_ceiling as f32),
                    "Merging shared five edge faces",
                );
            }
            // Sorted-vector rebuild (identical keys/values/order to the
            // `BTreeMap` build, a handful of allocs instead of 150k+).
            let edge_faces = EdgeFaceIndex::build(&self.remeshed_polygons);
            let vertex_neighbors = NeighborIndex::build(&self.remeshed_polygons);

            let mut shared_edge = (0, 0);
            let mut found_shared = false;
            for (edge, faces) in edge_faces.groups() {
                if 2 != faces.len() {
                    continue;
                }
                if rejected_edges.contains(&edge) {
                    continue;
                }
                let mut both_five_edges = true;
                for face_index in faces {
                    let face = &self.remeshed_polygons[*face_index];
                    if 5 != face.len() || Self::has_repeated_vertex(face) {
                        both_five_edges = false;
                        break;
                    }
                }
                if !both_five_edges {
                    continue;
                }
                // Endpoints are present by construction (used vertices).
                let mut neighbors = BTreeSet::new();
                neighbors.extend(vertex_neighbors.get_or_empty(&edge.0).iter().copied());
                neighbors.extend(vertex_neighbors.get_or_empty(&edge.1).iter().copied());
                neighbors.remove(&edge.0);
                neighbors.remove(&edge.1);
                if neighbors.len() > MAX_MERGED_VALENCE {
                    continue;
                }
                shared_edge = edge;
                found_shared = true;
                break;
            }
            if !found_shared {
                break;
            }

            let keep = shared_edge.0;
            let remove = shared_edge.1;
            let unmoved_vertex = self.remeshed_vertices.len();
            let keep_position =
                (self.remeshed_vertices[keep] + self.remeshed_vertices[remove]) * 0.5;
            // Per-face remap is independent given the fixed (keep,
            // remove) pair; validation keeps its serial face-order scan
            // (same checks, same order, same early exit), so the outcome
            // is identical.
            let mut remapped: Vec<(Vec<usize>, bool)> =
                vec![(Vec::new(), false); self.remeshed_polygons.len()];
            parallel_each(&mut remapped, |face_index, slot| {
                let face = &self.remeshed_polygons[face_index];
                let mut face_affected = false;
                for vertex in face {
                    if keep == *vertex || remove == *vertex {
                        face_affected = true;
                        break;
                    }
                }
                let mut candidate = Vec::with_capacity(face.len());
                for vertex in face {
                    let rewritten_vertex = if *vertex == remove { keep } else { *vertex };
                    if candidate.is_empty() || candidate[candidate.len() - 1] != rewritten_vertex {
                        candidate.push(rewritten_vertex);
                    }
                }
                if candidate.len() > 1 && candidate[0] == candidate[candidate.len() - 1] {
                    candidate.pop();
                }
                *slot = (candidate, face_affected);
            });
            let mut rewritten: Vec<Vec<usize>> = Vec::with_capacity(self.remeshed_polygons.len());
            let mut affected = Vec::with_capacity(self.remeshed_polygons.len());
            let mut valid = true;
            for (face, (candidate, face_affected)) in self.remeshed_polygons.iter().zip(remapped) {
                if face_affected {
                    // The two five edge faces become quads, no other face
                    // is allowed to degrade, otherwise the merge is
                    // trading one defect for another
                    if candidate.len() < 3 || (face.len() >= 4 && candidate.len() < 4) {
                        valid = false;
                        break;
                    }
                    if Self::has_repeated_vertex(&candidate) {
                        valid = false;
                        break;
                    }
                    let old_normal = Self::face_normal_with_added(
                        &self.remeshed_vertices,
                        face,
                        unmoved_vertex,
                        Vector3::default(),
                    );
                    let new_normal = Self::face_normal_with_added(
                        &self.remeshed_vertices,
                        &candidate,
                        keep,
                        keep_position,
                    );
                    if Vector3::dot_product(&old_normal, &new_normal) <= 0.0 {
                        valid = false;
                        break;
                    }
                }
                rewritten.push(candidate);
                affected.push(face_affected);
            }

            if valid {
                let mut unique_faces = BTreeSet::new();
                for (face_index, face) in rewritten.iter().enumerate() {
                    if !affected[face_index] {
                        unique_faces.insert(Self::canonical_face(face));
                    }
                }
                let mut keep_edge_counts: BTreeMap<(usize, usize), usize> = BTreeMap::new();
                for (face_index, face) in rewritten.iter().enumerate() {
                    if !valid {
                        break;
                    }
                    if affected[face_index] && !unique_faces.insert(Self::canonical_face(face)) {
                        valid = false;
                        break;
                    }
                    for i in 0..face.len() {
                        let edge = Self::edge_of(face[i], face[(i + 1) % face.len()]);
                        if keep != edge.0 && keep != edge.1 {
                            continue;
                        }
                        // Present-or-zero (C++ `operator[]` on a count
                        // map); the entry exists exactly when an earlier
                        // face touched the same edge.
                        let count = keep_edge_counts.entry(edge).or_insert(0);
                        *count += 1;
                        if *count > 2 {
                            valid = false;
                            break;
                        }
                    }
                }
            }
            if !valid {
                rejected_edges.insert(shared_edge);
                continue;
            }

            self.remeshed_vertices[keep] = keep_position;
            self.remeshed_polygons = rewritten;
            merged_vertices.insert(keep);
            merge_count += 1;
        }

        if 0 == merge_count {
            return;
        }

        let compacted = self.compact_vertices(&merged_vertices);

        // Gate on the parked handler: the caller parks
        // `self.progress_handler` in an `Arc` while this runs (borrow
        // restructure), so `diagnose()` would stay silent here —
        // `progress` is `Some` exactly when the outer handler exists
        // (the C++ `m_progressHandler` gate), with
        // `self.progress_handler` kept as the direct-call fallback.
        // Either way the line also needs the verbose flag (CLI
        // `--verbose` only).
        if self.verbose_dump && (progress.is_some() || self.progress_handler.is_some()) {
            eprint!("Merge shared five edge faces:{merge_count}\n");
        }
        self.rebuild_half_edges();

        // The two pentagons closed up around the merged point, pull the
        // patch back onto the source mesh
        self.smooth_around_vertices(&compacted, 3, 5);
    }

    pub(crate) fn collapse_three_valence_corners(&mut self) {
        if self.remeshed_vertices.is_empty() || self.remeshed_polygons.is_empty() {
            return;
        }

        const MIN_SIDE_VALENCE: usize = 5;

        let mut collapse_count = 0;
        let mut collapsed_vertices = BTreeSet::new();
        loop {
            let mut edge_faces: BTreeMap<(usize, usize), Vec<usize>> = BTreeMap::new();
            let mut vertex_neighbors: BTreeMap<usize, BTreeSet<usize>> = BTreeMap::new();
            let mut vertex_faces: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
            for (face_index, face) in self.remeshed_polygons.iter().enumerate() {
                for i in 0..face.len() {
                    let j = (i + 1) % face.len();
                    edge_faces
                        .entry(Self::edge_of(face[i], face[j]))
                        .or_default()
                        .push(face_index);
                    vertex_neighbors.entry(face[i]).or_default().insert(face[j]);
                    vertex_neighbors.entry(face[j]).or_default().insert(face[i]);
                }
                for vertex in face {
                    vertex_faces.entry(*vertex).or_default().push(face_index);
                }
            }

            let mut border_vertices = BTreeSet::new();
            for (edge, faces) in &edge_faces {
                if 2 == faces.len() {
                    continue;
                }
                border_vertices.insert(edge.0);
                border_vertices.insert(edge.1);
            }

            let mut touched_vertices = BTreeSet::new();
            let mut replaced_faces: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
            let mut removed_faces = BTreeSet::new();
            let mut round_collapse_count = 0;
            for face_index in 0..self.remeshed_polygons.len() {
                let quad = &self.remeshed_polygons[face_index];
                if 4 != quad.len() || Self::has_repeated_vertex(quad) {
                    continue;
                }
                let mut on_border = false;
                for vertex in quad {
                    if border_vertices.contains(vertex) {
                        on_border = true;
                        break;
                    }
                }
                if on_border {
                    continue;
                }
                for i in 0..4 {
                    let corner = quad[i];
                    let left_side = quad[(i + 1) % 4];
                    let opposite = quad[(i + 2) % 4];
                    let right_side = quad[(i + 3) % 4];
                    // Present by construction (quad corners are used).
                    if 3 != vertex_neighbors[&corner].len() {
                        continue;
                    }
                    // Present by construction (quad corners are used).
                    if 3 != vertex_faces[&corner].len() {
                        continue;
                    }
                    // Present by construction (quad corners are used).
                    if vertex_neighbors[&left_side].len() < MIN_SIDE_VALENCE
                        || vertex_neighbors[&right_side].len() < MIN_SIDE_VALENCE
                    {
                        continue;
                    }
                    if vertex_neighbors[&corner].contains(&opposite) {
                        continue;
                    }
                    let mut shared_num = 0;
                    for neighbor in &vertex_neighbors[&corner] {
                        if vertex_neighbors[&opposite].contains(neighbor) {
                            shared_num += 1;
                        }
                    }
                    if 2 != shared_num {
                        continue;
                    }

                    let mut affected_faces = vertex_faces[&corner].clone();
                    affected_faces.extend(vertex_faces[&opposite].iter().copied());
                    affected_faces.sort_unstable();
                    affected_faces.dedup();
                    let mut touched = false;
                    for affected in &affected_faces {
                        for vertex in &self.remeshed_polygons[*affected] {
                            if touched_vertices.contains(vertex) {
                                touched = true;
                                break;
                            }
                        }
                        if touched {
                            break;
                        }
                    }
                    if touched {
                        continue;
                    }

                    let added_vertex = self.remeshed_vertices.len();
                    let added_position =
                        (self.remeshed_vertices[corner] + self.remeshed_vertices[opposite]) * 0.5;

                    let mut rewritten = Vec::with_capacity(affected_faces.len());
                    let mut valid = true;
                    for affected in &affected_faces {
                        if face_index == *affected {
                            continue;
                        }
                        let face = &self.remeshed_polygons[*affected];
                        let mut candidate = Vec::with_capacity(face.len());
                        for vertex in face {
                            let rewritten_vertex = if corner == *vertex || opposite == *vertex {
                                added_vertex
                            } else {
                                *vertex
                            };
                            if candidate.is_empty()
                                || candidate[candidate.len() - 1] != rewritten_vertex
                            {
                                candidate.push(rewritten_vertex);
                            }
                        }
                        if candidate.len() > 1 && candidate[0] == candidate[candidate.len() - 1] {
                            candidate.pop();
                        }
                        if candidate.len() != face.len() || Self::has_repeated_vertex(&candidate) {
                            valid = false;
                            break;
                        }
                        if Vector3::dot_product(
                            &Self::face_normal_with_added(
                                &self.remeshed_vertices,
                                face,
                                added_vertex,
                                added_position,
                            ),
                            &Self::face_normal_with_added(
                                &self.remeshed_vertices,
                                &candidate,
                                added_vertex,
                                added_position,
                            ),
                        ) <= 0.0
                        {
                            valid = false;
                            break;
                        }
                        rewritten.push(candidate);
                    }
                    if valid {
                        let mut unique_faces = BTreeSet::new();
                        let mut added_edge_counts: BTreeMap<(usize, usize), usize> =
                            BTreeMap::new();
                        for face in &rewritten {
                            if !unique_faces.insert(Self::canonical_face(face)) {
                                valid = false;
                                break;
                            }
                            for j in 0..face.len() {
                                if !valid {
                                    break;
                                }
                                let edge = Self::edge_of(face[j], face[(j + 1) % face.len()]);
                                if added_vertex != edge.0 && added_vertex != edge.1 {
                                    continue;
                                }
                                // Present-or-zero (C++ `operator[]` on a
                                // count map).
                                let count = added_edge_counts.entry(edge).or_insert(0);
                                *count += 1;
                                if *count > 2 {
                                    valid = false;
                                }
                            }
                            if !valid {
                                break;
                            }
                        }
                    }
                    if !valid {
                        continue;
                    }

                    self.remeshed_vertices.push(added_position);
                    removed_faces.insert(face_index);
                    let mut rewritten_index = 0;
                    for affected in &affected_faces {
                        if face_index == *affected {
                            continue;
                        }
                        for vertex in &self.remeshed_polygons[*affected] {
                            touched_vertices.insert(*vertex);
                        }
                        // First wins (C++ `insert`); affected faces are
                        // distinct within one adoption and the touched
                        // guard blocks overlaps across adoptions, so this
                        // never overwrites either way.
                        replaced_faces
                            .entry(*affected)
                            .or_insert_with(|| rewritten[rewritten_index].clone());
                        rewritten_index += 1;
                    }
                    for vertex in quad {
                        touched_vertices.insert(*vertex);
                    }
                    collapsed_vertices.insert(added_vertex);
                    collapsed_vertices.insert(left_side);
                    collapsed_vertices.insert(right_side);
                    round_collapse_count += 1;
                    break;
                }
            }

            if 0 == round_collapse_count {
                break;
            }

            let mut rewritten = Vec::with_capacity(self.remeshed_polygons.len());
            for (face_index, face) in self.remeshed_polygons.iter().enumerate() {
                if removed_faces.contains(&face_index) {
                    continue;
                }
                if let Some(find_rewritten) = replaced_faces.get(&face_index) {
                    rewritten.push(find_rewritten.clone());
                    continue;
                }
                rewritten.push(face.clone());
            }
            self.remeshed_polygons = rewritten;
            collapse_count += round_collapse_count;
        }

        if 0 == collapse_count {
            return;
        }

        let compacted = self.compact_vertices(&collapsed_vertices);

        self.diagnose(|| format!("Collapse three valence corners:{collapse_count}\n"));
        self.rebuild_half_edges();

        self.smooth_around_vertices(&compacted, 3, 5);
    }
}
