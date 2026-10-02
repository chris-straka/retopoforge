use super::QuadExtractor;
use super::types::{EdgeFaceIndex, FaceCountIndex, NeighborIndex};
use crate::par::parallel_each;
use crate::vector3::Vector3;
use std::collections::{BTreeMap, BTreeSet};

impl<'a> QuadExtractor<'a> {
    /// Shared vertex-compaction epilogue (identical in the five collapse /
    /// merge passes): drops vertices no face references (first-use order),
    /// rewrites faces, and remaps the seed set. Returns the remapped seeds.
    pub(crate) fn compact_vertices(&mut self, seed_vertices: &BTreeSet<usize>) -> BTreeSet<usize> {
        let mut old_to_new = BTreeMap::new();
        let mut compacted_vertices = Vec::new();
        for face in &self.remeshed_polygons {
            for vertex in face {
                if old_to_new.contains_key(vertex) {
                    continue;
                }
                old_to_new.insert(*vertex, compacted_vertices.len());
                compacted_vertices.push(self.remeshed_vertices[*vertex]);
            }
        }
        for face in &mut self.remeshed_polygons {
            for vertex in face {
                *vertex = old_to_new[vertex];
            }
        }
        self.remeshed_vertices = compacted_vertices;
        let mut compacted_seeds = BTreeSet::new();
        for vertex in seed_vertices {
            if let Some(new) = old_to_new.get(vertex) {
                compacted_seeds.insert(*new);
            }
        }
        compacted_seeds
    }

    pub(crate) fn collapse_three_valence_diagonals(&mut self) {
        if self.remeshed_vertices.is_empty() || self.remeshed_polygons.is_empty() {
            return;
        }

        const NO_VERTEX: usize = usize::MAX;
        const NO_FACE: usize = usize::MAX;
        // The quad closes up along the diagonal, so the two corners left
        // standing each hand one neighbor over to the point in the middle.
        // Three neighbors is as low as a corner may go before it turns into
        // a doublet, so the sides need four
        const MIN_SIDE_VALENCE: usize = 4;

        // Two three valence points facing each other across a quad are the
        // two halves of a single point which the extraction split apart,
        // the quad between them is the gap. Collapsing the diagonal puts a
        // point back in the middle carrying the four neighbors the pair had
        // between them, and takes the quad away along with both
        // singularities, the sides paying for it with one neighbor each
        let mut rejected_diagonals = BTreeSet::new();
        let mut collapsed_vertices = BTreeSet::new();
        let mut collapse_count = 0;
        loop {
            // Sorted-vector rebuild (identical keys/values/order to the
            // `BTreeMap` builds, a handful of allocs instead of 200k+).
            let edge_faces = EdgeFaceIndex::build(&self.remeshed_polygons);
            let vertex_neighbors = NeighborIndex::build(&self.remeshed_polygons);
            let vertex_face_counts = FaceCountIndex::build(&self.remeshed_polygons);

            // A three valence point on a border is what a border looks
            // like, not a defect
            let mut border_vertices = BTreeSet::new();
            for (edge, faces) in edge_faces.groups() {
                if 2 == faces.len() {
                    continue;
                }
                border_vertices.insert(edge.0);
                border_vertices.insert(edge.1);
            }

            let neighbor_count = |vertex_neighbors: &NeighborIndex, vertex: usize| {
                vertex_neighbors
                    .get(&vertex)
                    .map_or(0, |neighbors| neighbors.len())
            };

            let mut collapsing_face = NO_FACE;
            let mut first = NO_VERTEX;
            let mut second = NO_VERTEX;
            let mut left = NO_VERTEX;
            let mut right = NO_VERTEX;
            for face_index in 0..self.remeshed_polygons.len() {
                if NO_FACE != collapsing_face {
                    break;
                }
                let face = &self.remeshed_polygons[face_index];
                if 4 != face.len() || Self::has_repeated_vertex(face) {
                    continue;
                }
                let mut on_border = false;
                for vertex in face {
                    if border_vertices.contains(vertex) {
                        on_border = true;
                        break;
                    }
                }
                if on_border {
                    continue;
                }
                for i in 0..2 {
                    let diagonal_first = face[i];
                    let diagonal_second = face[i + 2];
                    if rejected_diagonals.contains(&Self::edge_of(diagonal_first, diagonal_second))
                    {
                        continue;
                    }
                    if 3 != neighbor_count(&vertex_neighbors, diagonal_first)
                        || 3 != neighbor_count(&vertex_neighbors, diagonal_second)
                    {
                        continue;
                    }
                    // A closed fan of three faces is the only shape a three
                    // valence point may have here, anything else is not the
                    // pair this is looking for
                    if 3 != vertex_face_counts
                        .get(&diagonal_first)
                        .copied()
                        .unwrap_or(0)
                        || 3 != vertex_face_counts
                            .get(&diagonal_second)
                            .copied()
                            .unwrap_or(0)
                    {
                        continue;
                    }
                    let diagonal_left = face[(i + 1) % 4];
                    let diagonal_right = face[(i + 3) % 4];
                    if neighbor_count(&vertex_neighbors, diagonal_left) < MIN_SIDE_VALENCE
                        || neighbor_count(&vertex_neighbors, diagonal_right) < MIN_SIDE_VALENCE
                    {
                        continue;
                    }
                    // The two fans are only allowed to meet at the sides of
                    // the quad, any other point shared between them, an edge
                    // included, would fold the faces of the merged point
                    // over each other
                    if vertex_neighbors
                        .get(&diagonal_first)
                        .is_some_and(|neighbors| neighbors.contains(&diagonal_second))
                    {
                        continue;
                    }
                    let mut shared_num = 0;
                    if let Some(neighbors) = vertex_neighbors.get(&diagonal_first) {
                        for neighbor in neighbors {
                            if vertex_neighbors
                                .get(&diagonal_second)
                                .is_some_and(|second_neighbors| second_neighbors.contains(neighbor))
                            {
                                shared_num += 1;
                            }
                        }
                    }
                    if 2 != shared_num {
                        continue;
                    }
                    collapsing_face = face_index;
                    first = diagonal_first;
                    second = diagonal_second;
                    left = diagonal_left;
                    right = diagonal_right;
                    break;
                }
            }
            if NO_FACE == collapsing_face {
                break;
            }

            let added_vertex = self.remeshed_vertices.len();
            let added_position =
                (self.remeshed_vertices[first] + self.remeshed_vertices[second]) * 0.5;

            // Per-face remap is independent given the fixed collapse
            // pair; validation keeps its serial face-order scan (same
            // checks, same order, same early exit), so the outcome is
            // identical.
            let mut remapped: Vec<(Option<Vec<usize>>, bool)> =
                vec![(None, false); self.remeshed_polygons.len()];
            parallel_each(&mut remapped, |face_index, slot| {
                if collapsing_face == face_index {
                    return;
                }
                let face = &self.remeshed_polygons[face_index];
                let mut face_affected = false;
                for vertex in face {
                    if first == *vertex || second == *vertex {
                        face_affected = true;
                        break;
                    }
                }
                if !face_affected {
                    *slot = (Some(face.clone()), false);
                    return;
                }
                let mut candidate = Vec::with_capacity(face.len());
                for vertex in face {
                    let rewritten_vertex = if first == *vertex || second == *vertex {
                        added_vertex
                    } else {
                        *vertex
                    };
                    if candidate.is_empty() || candidate[candidate.len() - 1] != rewritten_vertex {
                        candidate.push(rewritten_vertex);
                    }
                }
                if candidate.len() > 1 && candidate[0] == candidate[candidate.len() - 1] {
                    candidate.pop();
                }
                *slot = (Some(candidate), true);
            });
            let mut rewritten: Vec<Vec<usize>> = Vec::with_capacity(self.remeshed_polygons.len());
            let mut affected = Vec::with_capacity(self.remeshed_polygons.len());
            let mut valid = true;
            for (face_index, (candidate, face_affected)) in remapped.into_iter().enumerate() {
                if !valid {
                    break;
                }
                let Some(candidate) = candidate else {
                    continue;
                };
                let face = &self.remeshed_polygons[face_index];
                if !face_affected {
                    rewritten.push(candidate);
                    affected.push(false);
                    continue;
                }
                // The quad is the only face allowed to disappear, the fans
                // around the pair keep every side they came in with
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
                affected.push(true);
            }

            if valid {
                let mut unique_faces = BTreeSet::new();
                for (face_index, face) in rewritten.iter().enumerate() {
                    if !affected[face_index] {
                        unique_faces.insert(Self::canonical_face(face));
                    }
                }
                let mut added_edge_counts: BTreeMap<(usize, usize), usize> = BTreeMap::new();
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
                        if added_vertex != edge.0 && added_vertex != edge.1 {
                            continue;
                        }
                        *added_edge_counts.entry(edge).or_default() += 1;
                        if added_edge_counts[&edge] > 2 {
                            valid = false;
                            break;
                        }
                    }
                }
            }
            if !valid {
                rejected_diagonals.insert(Self::edge_of(first, second));
                continue;
            }

            self.remeshed_vertices.push(added_position);
            self.remeshed_polygons = rewritten;
            collapsed_vertices.insert(added_vertex);
            collapsed_vertices.insert(left);
            collapsed_vertices.insert(right);
            collapse_count += 1;
        }

        if 0 == collapse_count {
            return;
        }

        // The two collapsed points are left with no face of their own
        let compacted = self.compact_vertices(&collapsed_vertices);

        self.diagnose(|| format!("Collapse three valence diagonals:{collapse_count}\n"));
        self.rebuild_half_edges();

        // The point in the middle came from the diagonal, not from the
        // source mesh, pull the patch which closed up around it back onto
        // the surface
        self.smooth_around_vertices(&compacted, 3, 5);
    }

    pub(crate) fn merge_double_shared_edge_quads(&mut self) {
        if self.remeshed_vertices.is_empty() || self.remeshed_polygons.is_empty() {
            return;
        }

        let index_of_vertex = |face: &[usize], vertex: usize| {
            for (i, corner) in face.iter().enumerate() {
                if vertex == *corner {
                    return i;
                }
            }
            face.len()
        };

        let mut merge_num = 0;
        let mut merged_vertices = BTreeSet::new();
        loop {
            let mut vertex_neighbors: BTreeMap<usize, BTreeSet<usize>> = BTreeMap::new();
            let mut vertex_faces: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
            for (face_index, face) in self.remeshed_polygons.iter().enumerate() {
                for i in 0..face.len() {
                    let j = (i + 1) % face.len();
                    vertex_neighbors.entry(face[i]).or_default().insert(face[j]);
                    vertex_neighbors.entry(face[j]).or_default().insert(face[i]);
                }
                for vertex in face {
                    vertex_faces.entry(*vertex).or_default().push(face_index);
                }
            }

            let mut middle_vertices = BTreeSet::new();
            for (source, neighbors) in &vertex_neighbors {
                if 2 != neighbors.len() {
                    continue;
                }
                match vertex_faces.get(source) {
                    Some(find_faces) if 2 == find_faces.len() => {}
                    _ => continue,
                }
                middle_vertices.insert(*source);
            }

            let mut existing_faces = BTreeSet::new();
            for face in &self.remeshed_polygons {
                existing_faces.insert(Self::canonical_face(face));
            }

            let mut touched_vertices = BTreeSet::new();
            let mut replaced_faces: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
            let mut removed_faces = BTreeSet::new();
            let mut round_merge_num = 0;
            for middle in middle_vertices {
                // Present with two faces by construction (checked above).
                let faces = match vertex_faces.get(&middle) {
                    Some(faces) if 2 == faces.len() => faces.clone(),
                    _ => continue,
                };
                let first_face_index = faces[0];
                let second_face_index = faces[1];
                if first_face_index == second_face_index {
                    continue;
                }
                let first_face = &self.remeshed_polygons[first_face_index];
                let second_face = &self.remeshed_polygons[second_face_index];
                if 4 != first_face.len() || 4 != second_face.len() {
                    continue;
                }
                if Self::has_repeated_vertex(first_face) || Self::has_repeated_vertex(second_face) {
                    continue;
                }
                let first_at = index_of_vertex(first_face, middle);
                let second_at = index_of_vertex(second_face, middle);
                if first_at >= first_face.len() || second_at >= second_face.len() {
                    continue;
                }
                let previous = first_face[(first_at + 3) % 4];
                let next = first_face[(first_at + 1) % 4];
                if previous != second_face[(second_at + 1) % 4]
                    || next != second_face[(second_at + 3) % 4]
                {
                    continue;
                }
                let first_apex = first_face[(first_at + 2) % 4];
                let second_apex = second_face[(second_at + 2) % 4];
                if first_apex == second_apex {
                    continue;
                }
                let mut touched = false;
                for vertex in [middle, previous, next, first_apex, second_apex] {
                    if touched_vertices.contains(&vertex) {
                        touched = true;
                        break;
                    }
                }
                if touched {
                    continue;
                }

                let merged = vec![first_apex, previous, second_apex, next];
                if Self::has_repeated_vertex(&merged) {
                    continue;
                }
                if existing_faces.contains(&Self::canonical_face(&merged)) {
                    continue;
                }
                let first_normal = Self::face_normal_of(&self.remeshed_vertices, first_face);
                let second_normal = Self::face_normal_of(&self.remeshed_vertices, second_face);
                if Vector3::dot_product(
                    &(first_normal + second_normal),
                    &Self::face_normal_of(&self.remeshed_vertices, &merged),
                ) <= 0.0
                {
                    continue;
                }

                existing_faces.insert(Self::canonical_face(&merged));
                // First wins (C++ `insert`); keys are unique here (the
                // touched guard skips faces claimed by an earlier middle),
                // so this never overwrites either way.
                replaced_faces
                    .entry(first_face_index)
                    .or_insert(merged.clone());
                removed_faces.insert(second_face_index);
                touched_vertices.extend(merged.iter().copied());
                touched_vertices.insert(middle);
                merged_vertices.extend(merged.iter().copied());
                round_merge_num += 1;
            }

            if 0 == round_merge_num {
                break;
            }

            let mut rewritten =
                Vec::with_capacity(self.remeshed_polygons.len().saturating_sub(round_merge_num));
            for (face_index, face) in self.remeshed_polygons.iter().enumerate() {
                if removed_faces.contains(&face_index) {
                    continue;
                }
                if let Some(find_replaced) = replaced_faces.get(&face_index) {
                    rewritten.push(find_replaced.clone());
                    continue;
                }
                rewritten.push(face.clone());
            }
            self.remeshed_polygons = rewritten;
            merge_num += round_merge_num;
        }

        if 0 == merge_num {
            return;
        }

        let compacted = self.compact_vertices(&merged_vertices);

        self.diagnose(|| format!("Merge double shared edge quads:{merge_num}\n"));
        self.rebuild_half_edges();

        self.smooth_around_vertices(&compacted, 3, 5);
    }

    pub(crate) fn merge_three_and_five_valence_triangles(&mut self) {
        if self.remeshed_vertices.is_empty() || self.remeshed_polygons.is_empty() {
            return;
        }

        const MIN_VALENCE: usize = 3;

        let mut merge_num = 0;
        let mut merged_vertices = BTreeSet::new();
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

            let mut existing_faces = BTreeSet::new();
            for face in &self.remeshed_polygons {
                existing_faces.insert(Self::canonical_face(face));
            }

            let mut touched_vertices = BTreeSet::new();
            let mut replaced_faces: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
            let mut removed_faces = BTreeSet::new();
            let mut round_merge_num = 0;
            for (edge, faces) in &edge_faces {
                if 2 != faces.len() {
                    continue;
                }

                let mut three = edge.0;
                let mut five = edge.1;
                // Present by construction (edge endpoints are used).
                if 3 != vertex_neighbors[&three].len() {
                    std::mem::swap(&mut three, &mut five);
                }
                if 3 != vertex_neighbors[&three].len() || 5 != vertex_neighbors[&five].len() {
                    continue;
                }
                if border_vertices.contains(&three) || border_vertices.contains(&five) {
                    continue;
                }

                let mut three_fan = Vec::new();
                let mut five_fan = Vec::new();
                if !Self::fan_around(
                    &self.remeshed_polygons,
                    &vertex_faces,
                    &edge_faces,
                    three,
                    &mut three_fan,
                ) || 3 != three_fan.len()
                {
                    continue;
                }
                if !Self::fan_around(
                    &self.remeshed_polygons,
                    &vertex_faces,
                    &edge_faces,
                    five,
                    &mut five_fan,
                ) || 5 != five_fan.len()
                {
                    continue;
                }
                let three_triangle_num =
                    match Self::count_fan_triangles(&self.remeshed_polygons, &three_fan) {
                        Some(three_triangle_num) => three_triangle_num,
                        _ => continue,
                    };
                let five_triangle_num =
                    match Self::count_fan_triangles(&self.remeshed_polygons, &five_fan) {
                        Some(five_triangle_num) => five_triangle_num,
                        _ => continue,
                    };
                if 1 != three_triangle_num || 1 != five_triangle_num {
                    continue;
                }

                let shared_faces = BTreeSet::from([faces[0], faces[1]]);
                let mut shared_at = five_fan.len();
                for i in 0..five_fan.len() {
                    if shared_faces.contains(&five_fan[i])
                        && shared_faces.contains(&five_fan[(i + 1) % five_fan.len()])
                    {
                        shared_at = i;
                        break;
                    }
                }
                if shared_at >= five_fan.len() {
                    continue;
                }
                let triangle_face = five_fan[(shared_at + 3) % five_fan.len()];
                if 3 != self.remeshed_polygons[triangle_face].len() {
                    continue;
                }

                let mut best_score = 0;
                let mut best_quads: Vec<Vec<usize>> = Vec::new();
                let mut best_run = Vec::new();
                let mut best_octagon = Vec::new();
                let mut best_position = Vector3::default();
                for direction in 0..2 {
                    let middle_face =
                        five_fan[(shared_at + if 0 == direction { 2 } else { 4 }) % five_fan.len()];
                    if 4 != self.remeshed_polygons[middle_face].len() {
                        continue;
                    }
                    let mut run = three_fan.clone();
                    run.push(middle_face);
                    run.push(triangle_face);

                    let mut directed_edges = BTreeSet::new();
                    for face_index in &run {
                        let face = &self.remeshed_polygons[*face_index];
                        for i in 0..face.len() {
                            directed_edges.insert((face[i], face[(i + 1) % face.len()]));
                        }
                    }
                    let mut boundary_next = BTreeMap::new();
                    let mut buried_edges = Vec::new();
                    let mut simple_boundary = true;
                    for directed_edge in &directed_edges {
                        let (from, to) = *directed_edge;
                        if directed_edges.contains(&(to, from)) {
                            if from < to {
                                buried_edges.push(Self::edge_of(from, to));
                            }
                            continue;
                        }
                        // Rust `insert` overwrites on a duplicate key where
                        // C++ `insert` keeps the first, but the map is
                        // discarded on this path either way.
                        if boundary_next.insert(from, to).is_some() {
                            simple_boundary = false;
                            break;
                        }
                    }
                    if !simple_boundary || 8 != boundary_next.len() || 5 != buried_edges.len() {
                        continue;
                    }
                    let mut buried_at_defect = true;
                    let mut buried_counts: BTreeMap<usize, usize> = BTreeMap::new();
                    for buried in &buried_edges {
                        let (first, second) = *buried;
                        if three != first && three != second && five != first && five != second {
                            buried_at_defect = false;
                            break;
                        }
                        *buried_counts.entry(first).or_insert(0) += 1;
                        *buried_counts.entry(second).or_insert(0) += 1;
                    }
                    // Present-or-zero (C++ `operator[]` on a count map).
                    if !buried_at_defect || 3 != buried_counts.get(&three).copied().unwrap_or(0) {
                        continue;
                    }

                    let mut octagon = Vec::with_capacity(8);
                    let mut walk = five;
                    for _ in 0..8 {
                        octagon.push(walk);
                        match boundary_next.get(&walk) {
                            Some(next) => walk = *next,
                            _ => break,
                        }
                    }
                    if 8 != octagon.len() || walk != five || Self::has_repeated_vertex(&octagon) {
                        continue;
                    }
                    let mut touched = false;
                    for corner in &octagon {
                        if touched_vertices.contains(corner) {
                            touched = true;
                            break;
                        }
                    }
                    if touched || touched_vertices.contains(&three) {
                        continue;
                    }

                    let mut new_score = 0;
                    let mut lopsided = false;
                    for (i, corner) in octagon.iter().enumerate() {
                        // Present by construction (octagon corners are used
                        // vertices); buried edges are distinct mesh edges,
                        // so the count never exceeds the valence.
                        let valence = vertex_neighbors[corner].len()
                            - buried_counts.get(corner).copied().unwrap_or(0)
                            + usize::from(0 == i % 2);
                        if valence < MIN_VALENCE {
                            lopsided = true;
                        }
                        new_score += Self::valence_score(valence);
                        if lopsided {
                            break;
                        }
                    }
                    if lopsided {
                        continue;
                    }
                    // Present by construction (three is used).
                    let mut old_score = Self::valence_score(vertex_neighbors[&three].len());
                    for corner in &octagon {
                        old_score += Self::valence_score(vertex_neighbors[corner].len());
                    }
                    if new_score > old_score {
                        continue;
                    }
                    if !best_quads.is_empty() && new_score >= best_score {
                        continue;
                    }

                    let added_vertex = self.remeshed_vertices.len();
                    let mut added_position = Vector3::default();
                    for i in (0..8).step_by(2) {
                        added_position += self.remeshed_vertices[octagon[i]];
                    }
                    added_position /= 4.0;

                    let mut quads = Vec::with_capacity(4);
                    for i in (0..8).step_by(2) {
                        quads.push(vec![
                            octagon[i],
                            octagon[(i + 1) % 8],
                            octagon[(i + 2) % 8],
                            added_vertex,
                        ]);
                    }

                    let mut old_normal = Vector3::default();
                    for face_index in &run {
                        old_normal += Self::face_normal_with_added(
                            &self.remeshed_vertices,
                            &self.remeshed_polygons[*face_index],
                            added_vertex,
                            added_position,
                        );
                    }
                    let mut shaped = true;
                    for quad in &quads {
                        if Vector3::dot_product(
                            &old_normal,
                            &Self::face_normal_with_added(
                                &self.remeshed_vertices,
                                quad,
                                added_vertex,
                                added_position,
                            ),
                        ) <= 0.0
                            || existing_faces.contains(&Self::canonical_face(quad))
                        {
                            shaped = false;
                            break;
                        }
                    }
                    if !shaped {
                        continue;
                    }

                    best_score = new_score;
                    best_quads = quads;
                    best_run = run;
                    best_octagon = octagon;
                    best_position = added_position;
                }
                if best_quads.is_empty() {
                    continue;
                }

                let added_vertex = self.remeshed_vertices.len();
                self.remeshed_vertices.push(best_position);
                for quad in &best_quads {
                    existing_faces.insert(Self::canonical_face(quad));
                }
                for (i, face_index) in best_run.iter().enumerate() {
                    // First wins (C++ `insert`); the run faces are
                    // distinct (the middle/triangle faces sit outside
                    // the edge's two faces), so this never overwrites
                    // either way.
                    if i < best_quads.len() {
                        replaced_faces
                            .entry(*face_index)
                            .or_insert(best_quads[i].clone());
                    } else {
                        removed_faces.insert(*face_index);
                    }
                }
                touched_vertices.extend(best_octagon.iter().copied());
                touched_vertices.insert(three);
                merged_vertices.extend(best_octagon.iter().copied());
                merged_vertices.insert(added_vertex);
                round_merge_num += 1;
            }

            if 0 == round_merge_num {
                break;
            }

            let mut rewritten = Vec::with_capacity(self.remeshed_polygons.len());
            for (face_index, face) in self.remeshed_polygons.iter().enumerate() {
                if removed_faces.contains(&face_index) {
                    continue;
                }
                if let Some(find_replaced) = replaced_faces.get(&face_index) {
                    rewritten.push(find_replaced.clone());
                    continue;
                }
                rewritten.push(face.clone());
            }
            self.remeshed_polygons = rewritten;
            merge_num += round_merge_num;
        }

        if 0 == merge_num {
            return;
        }

        let compacted = self.compact_vertices(&merged_vertices);

        self.diagnose(|| format!("Merge three and five valence triangles:{merge_num}\n"));
        self.rebuild_half_edges();

        self.smooth_around_vertices(&compacted, 3, 5);
    }
}
