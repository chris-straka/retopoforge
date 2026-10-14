use super::QuadExtractor;
use crate::vector3::Vector3;
use std::collections::{BTreeMap, BTreeSet};

impl<'a> QuadExtractor<'a> {
    /// Shared `hasRepeatedVertex` lambda (repeated verbatim in every
    /// cleanup pass in the C++).
    pub(crate) fn has_repeated_vertex(face: &[usize]) -> bool {
        let unique: BTreeSet<usize> = face.iter().copied().collect();
        unique.len() != face.len()
    }

    /// Shared `canonicalFace` lambda: lexicographically smallest rotation
    /// of either winding.
    pub(crate) fn canonical_face(face: &[usize]) -> Vec<usize> {
        let mut best = Vec::new();
        let reversed: Vec<usize> = face.iter().rev().copied().collect();
        for winding in [face, &reversed] {
            for start in 0..winding.len() {
                let mut candidate = Vec::with_capacity(winding.len());
                for i in 0..winding.len() {
                    candidate.push(winding[(start + i) % winding.len()]);
                }
                if best.is_empty() || candidate < best {
                    best = candidate;
                }
            }
        }
        best
    }

    /// Shared `valenceScore` lambda: distance from the four neighbors a
    /// quad point wants.
    pub(crate) fn valence_score(valence: usize) -> i32 {
        if valence > 4 {
            (valence - 4) as i32
        } else {
            (4 - valence) as i32
        }
    }

    /// Shared plain `faceNormal` lambda (fan around `face[0]`, positions
    /// read straight from the mesh).
    pub(crate) fn face_normal_of(vertices: &[Vector3], face: &[usize]) -> Vector3 {
        let mut normal = Vector3::default();
        // `saturating_sub` keeps the C++ loop condition (`i + 1 < size`,
        // empty for short faces) total on degenerate input.
        for i in 1..face.len().saturating_sub(1) {
            normal += Vector3::cross_product(
                &(vertices[face[i]] - vertices[face[0]]),
                &(vertices[face[i + 1]] - vertices[face[0]]),
            );
        }
        normal
    }

    /// Shared `faceNormal` lambda with a phantom vertex (the `addedVertex`
    /// / `movedVertex` pattern): `added_vertex` reads `added_position`.
    pub(crate) fn face_normal_with_added(
        vertices: &[Vector3],
        face: &[usize],
        added_vertex: usize,
        added_position: Vector3,
    ) -> Vector3 {
        let position_of = |vertex: usize| {
            if vertex == added_vertex {
                added_position
            } else {
                vertices[vertex]
            }
        };
        let mut normal = Vector3::default();
        if face.len() < 3 {
            return normal;
        }
        let origin = position_of(face[0]);
        for i in 1..face.len() - 1 {
            normal += Vector3::cross_product(
                &(position_of(face[i]) - origin),
                &(position_of(face[i + 1]) - origin),
            );
        }
        normal
    }

    /// Shared `cornerScore` lambda (mean |dot| of corner legs, negated).
    pub(crate) fn corner_score_of(vertices: &[Vector3], quad: &[usize]) -> f64 {
        let mut total = 0.0;
        for i in 0..quad.len() {
            let h = (i + quad.len() - 1) % quad.len();
            let j = (i + 1) % quad.len();
            let left = (vertices[quad[h]] - vertices[quad[i]]).normalized();
            let right = (vertices[quad[j]] - vertices[quad[i]]).normalized();
            total += Vector3::dot_product(&left, &right).abs();
        }
        -total / quad.len() as f64
    }

    /// Shared `flowScoreAt` lambda (best incoming-edge alignment outside
    /// the skipped neighbors, 0 when none).
    pub(crate) fn flow_score_at(
        vertices: &[Vector3],
        vertex_neighbors: &BTreeMap<usize, BTreeSet<usize>>,
        vertex: usize,
        direction: Vector3,
        skip_neighbors: &BTreeSet<usize>,
    ) -> f64 {
        let mut best = -1.0;
        let Some(find_neighbors) = vertex_neighbors.get(&vertex) else {
            return 0.0;
        };
        let mut found = false;
        for neighbor in find_neighbors {
            if skip_neighbors.contains(neighbor) {
                continue;
            }
            let incoming = (vertices[vertex] - vertices[*neighbor]).normalized();
            let score = Vector3::dot_product(&incoming, &direction);
            if score > best {
                best = score;
            }
            found = true;
        }
        if found { best } else { 0.0 }
    }

    pub(crate) fn split_six_edge_faces(&mut self) {
        if self.remeshed_vertices.is_empty() || self.remeshed_polygons.is_empty() {
            return;
        }

        let mut vertex_neighbors: BTreeMap<usize, BTreeSet<usize>> = BTreeMap::new();
        for face in &self.remeshed_polygons {
            for i in 0..face.len() {
                let j = (i + 1) % face.len();
                vertex_neighbors.entry(face[i]).or_default().insert(face[j]);
                vertex_neighbors.entry(face[j]).or_default().insert(face[i]);
            }
        }

        let mut split_num = 0;
        let mut polygons = Vec::with_capacity(self.remeshed_polygons.len());
        for face in &self.remeshed_polygons {
            if 6 != face.len() {
                polygons.push(face.clone());
                continue;
            }
            if Self::has_repeated_vertex(face) {
                polygons.push(face.clone());
                continue;
            }

            let mut best_corner = -1;
            let mut best_score = 0.0;
            for i in 0..3 {
                let a = face[i];
                let b = face[i + 3];
                if vertex_neighbors
                    .get(&a)
                    .is_some_and(|neighbors| neighbors.contains(&b))
                {
                    continue;
                }
                let direction =
                    (self.remeshed_vertices[b] - self.remeshed_vertices[a]).normalized();
                let a_face_neighbors = BTreeSet::from([face[(i + 5) % 6], face[(i + 1) % 6], b]);
                let b_face_neighbors = BTreeSet::from([face[(i + 2) % 6], face[(i + 4) % 6], a]);
                let mut score = Self::flow_score_at(
                    &self.remeshed_vertices,
                    &vertex_neighbors,
                    a,
                    direction,
                    &a_face_neighbors,
                ) + Self::flow_score_at(
                    &self.remeshed_vertices,
                    &vertex_neighbors,
                    b,
                    -direction,
                    &b_face_neighbors,
                );
                score += Self::corner_score_of(
                    &self.remeshed_vertices,
                    &[face[i], face[(i + 1) % 6], face[(i + 2) % 6], face[i + 3]],
                );
                score += Self::corner_score_of(
                    &self.remeshed_vertices,
                    &[face[i + 3], face[(i + 4) % 6], face[(i + 5) % 6], face[i]],
                );
                if -1 == best_corner || score > best_score {
                    best_corner = i as i32;
                    best_score = score;
                }
            }
            if -1 == best_corner {
                self.diagnose(|| "Six edge face kept, no diagonal available\n".to_string());
                polygons.push(face.clone());
                continue;
            }

            let i = best_corner as usize;
            polygons.push(vec![
                face[i],
                face[(i + 1) % 6],
                face[(i + 2) % 6],
                face[i + 3],
            ]);
            polygons.push(vec![
                face[i + 3],
                face[(i + 4) % 6],
                face[(i + 5) % 6],
                face[i],
            ]);
            split_num += 1;
            vertex_neighbors
                .entry(face[i])
                .or_default()
                .insert(face[i + 3]);
            vertex_neighbors
                .entry(face[i + 3])
                .or_default()
                .insert(face[i]);
        }

        if 0 == split_num {
            return;
        }

        self.diagnose(|| format!("Split six edge faces:{split_num}\n"));
        self.remeshed_polygons = polygons;
        self.rebuild_half_edges();
    }

    pub(crate) fn split_seven_edge_faces(&mut self) {
        if self.remeshed_vertices.is_empty() || self.remeshed_polygons.is_empty() {
            return;
        }

        let mut vertex_neighbors: BTreeMap<usize, BTreeSet<usize>> = BTreeMap::new();
        for face in &self.remeshed_polygons {
            for i in 0..face.len() {
                let j = (i + 1) % face.len();
                vertex_neighbors.entry(face[i]).or_default().insert(face[j]);
                vertex_neighbors.entry(face[j]).or_default().insert(face[i]);
            }
        }

        let mut split_num = 0;
        let mut polygons = Vec::with_capacity(self.remeshed_polygons.len());
        for face in &self.remeshed_polygons {
            if 7 != face.len() {
                polygons.push(face.clone());
                continue;
            }
            if Self::has_repeated_vertex(face) {
                polygons.push(face.clone());
                continue;
            }

            let mut best_corner = -1;
            let mut best_score = 0.0;
            for i in 0..7 {
                let a = face[i];
                let b = face[(i + 3) % 7];
                if vertex_neighbors
                    .get(&a)
                    .is_some_and(|neighbors| neighbors.contains(&b))
                {
                    continue;
                }
                let direction =
                    (self.remeshed_vertices[b] - self.remeshed_vertices[a]).normalized();
                let a_face_neighbors = BTreeSet::from([face[(i + 6) % 7], face[(i + 1) % 7], b]);
                let b_face_neighbors = BTreeSet::from([face[(i + 2) % 7], face[(i + 4) % 7], a]);
                let mut score = Self::flow_score_at(
                    &self.remeshed_vertices,
                    &vertex_neighbors,
                    a,
                    direction,
                    &a_face_neighbors,
                ) + Self::flow_score_at(
                    &self.remeshed_vertices,
                    &vertex_neighbors,
                    b,
                    -direction,
                    &b_face_neighbors,
                );
                score += Self::corner_score_of(
                    &self.remeshed_vertices,
                    &[
                        face[i],
                        face[(i + 1) % 7],
                        face[(i + 2) % 7],
                        face[(i + 3) % 7],
                    ],
                );
                score += Self::corner_score_of(
                    &self.remeshed_vertices,
                    &[
                        face[(i + 3) % 7],
                        face[(i + 4) % 7],
                        face[(i + 5) % 7],
                        face[(i + 6) % 7],
                        face[i],
                    ],
                );
                if -1 == best_corner || score > best_score {
                    best_corner = i as i32;
                    best_score = score;
                }
            }
            if -1 == best_corner {
                self.diagnose(|| "Seven edge face kept, no diagonal available\n".to_string());
                polygons.push(face.clone());
                continue;
            }

            let i = best_corner as usize;
            polygons.push(vec![
                face[i],
                face[(i + 1) % 7],
                face[(i + 2) % 7],
                face[(i + 3) % 7],
            ]);
            polygons.push(vec![
                face[(i + 3) % 7],
                face[(i + 4) % 7],
                face[(i + 5) % 7],
                face[(i + 6) % 7],
                face[i],
            ]);
            split_num += 1;
            vertex_neighbors
                .entry(face[i])
                .or_default()
                .insert(face[(i + 3) % 7]);
            vertex_neighbors
                .entry(face[(i + 3) % 7])
                .or_default()
                .insert(face[i]);
        }

        if 0 == split_num {
            return;
        }

        self.diagnose(|| format!("Split seven edge faces:{split_num}\n"));
        self.remeshed_polygons = polygons;
        self.rebuild_half_edges();
    }

    /// Shared `fanAround` lambda (identical in the three fan passes):
    /// walks the closed face fan around `vertex` through shared edges.
    /// The C++ reads `edgeFaces` through non-const `operator[]`, inserting
    /// an empty entry on missing keys; the mirror uses `get`. The inserted
    /// empties are unobservable: a missing edge reads as size != 2 either
    /// way (failing the fan walk and, in the one pass whose main loop
    /// iterates `edgeFaces`, its `2 != faces.size()` guard), and the maps
    /// are rebuilt every round.
    pub(crate) fn fan_around(
        polygons: &[Vec<usize>],
        vertex_faces: &BTreeMap<usize, Vec<usize>>,
        edge_faces: &BTreeMap<(usize, usize), Vec<usize>>,
        vertex: usize,
        fan: &mut Vec<usize>,
    ) -> bool {
        fan.clear();
        let Some(find_faces) = vertex_faces.get(&vertex) else {
            return false;
        };
        if find_faces.is_empty() {
            return false;
        }
        let start_face = find_faces[0];
        let mut current_face = start_face;
        loop {
            fan.push(current_face);
            if fan.len() > find_faces.len() {
                return false;
            }
            let face = &polygons[current_face];
            let mut at = face.len();
            for (i, corner) in face.iter().enumerate() {
                if vertex == *corner {
                    at = i;
                    break;
                }
            }
            if at >= face.len() {
                return false;
            }
            let next_edge = Self::edge_of(vertex, face[(at + 1) % face.len()]);
            let Some(incident) = edge_faces.get(&next_edge) else {
                return false;
            };
            if 2 != incident.len() {
                return false;
            }
            current_face = if incident[0] == current_face {
                incident[1]
            } else {
                incident[0]
            };
            if current_face == start_face {
                break;
            }
        }
        fan.len() == find_faces.len()
    }

    pub(crate) fn convert_triangle_and_five_edge_fans(&mut self) {
        if self.remeshed_vertices.is_empty() || self.remeshed_polygons.is_empty() {
            return;
        }

        const MAX_VALENCE: usize = 6;
        const MIN_FAN_VALENCE: usize = 4;
        const MAX_FAN_VALENCE: usize = 6;

        let mut convert_num = 0;
        let mut converted_vertices = BTreeSet::new();
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
            let mut round_convert_num = 0;
            let mut fan_vertices = BTreeSet::new();
            for (source, neighbors) in &vertex_neighbors {
                if neighbors.len() >= MIN_FAN_VALENCE && neighbors.len() <= MAX_FAN_VALENCE {
                    fan_vertices.insert(*source);
                }
            }
            for vertex in fan_vertices {
                if touched_vertices.contains(&vertex) || border_vertices.contains(&vertex) {
                    continue;
                }
                let mut fan = Vec::new();
                let neighbor_count = vertex_neighbors
                    .get(&vertex)
                    .map_or(0, |neighbors| neighbors.len());
                if !Self::fan_around(
                    &self.remeshed_polygons,
                    &vertex_faces,
                    &edge_faces,
                    vertex,
                    &mut fan,
                ) || fan.len() != neighbor_count
                {
                    continue;
                }

                let mut triangle_position = fan.len();
                let mut pentagon_position = fan.len();
                let mut well_formed = true;
                for i in 0..fan.len() {
                    if !well_formed {
                        break;
                    }
                    let face = &self.remeshed_polygons[fan[i]];
                    if Self::has_repeated_vertex(face) {
                        well_formed = false;
                    } else if 3 == face.len() {
                        if triangle_position < fan.len() {
                            well_formed = false;
                        }
                        triangle_position = i;
                    } else if 5 == face.len() {
                        if pentagon_position < fan.len() {
                            well_formed = false;
                        }
                        pentagon_position = i;
                    } else if 4 != face.len() {
                        well_formed = false;
                    }
                }
                if !well_formed || triangle_position >= fan.len() || pentagon_position >= fan.len()
                {
                    continue;
                }

                let forward_gap = (pentagon_position + fan.len() - triangle_position) % fan.len();
                let backward_gap = fan.len() - forward_gap;
                if 2 != forward_gap.min(backward_gap) {
                    continue;
                }
                let run_start = if forward_gap <= backward_gap {
                    triangle_position
                } else {
                    pentagon_position
                };
                let mut run = Vec::with_capacity(3);
                for i in 0..3 {
                    run.push(fan[(run_start + i) % fan.len()]);
                }

                let mut directed_edges = BTreeSet::new();
                for face_index in &run {
                    let face = &self.remeshed_polygons[*face_index];
                    for i in 0..face.len() {
                        directed_edges.insert((face[i], face[(i + 1) % face.len()]));
                    }
                }
                let mut boundary_next: BTreeMap<usize, usize> = BTreeMap::new();
                let mut buried_edges = Vec::new();
                let mut simple_boundary = true;
                for (from, to) in &directed_edges {
                    if directed_edges.contains(&(*to, *from)) {
                        if from < to {
                            buried_edges.push(Self::edge_of(*from, *to));
                        }
                        continue;
                    }
                    if boundary_next.insert(*from, *to).is_some() {
                        simple_boundary = false;
                        break;
                    }
                }
                if !simple_boundary || 8 != boundary_next.len() || 2 != buried_edges.len() {
                    continue;
                }
                let mut buried_at_fan_vertex = true;
                for edge in &buried_edges {
                    if vertex != edge.0 && vertex != edge.1 {
                        buried_at_fan_vertex = false;
                        break;
                    }
                }
                if !buried_at_fan_vertex {
                    continue;
                }

                let mut octagon = Vec::with_capacity(8);
                let mut walk = vertex;
                for _ in 0..8 {
                    octagon.push(walk);
                    let Some(find_next) = boundary_next.get(&walk) else {
                        break;
                    };
                    walk = *find_next;
                }
                if 8 != octagon.len() || walk != vertex || Self::has_repeated_vertex(&octagon) {
                    continue;
                }
                let mut touched = false;
                for corner in &octagon {
                    if touched_vertices.contains(corner) {
                        touched = true;
                        break;
                    }
                }
                if touched {
                    continue;
                }

                let mut lopsided = false;
                for i in (0..8).step_by(2) {
                    if lopsided {
                        break;
                    }
                    // Wrapping: mirrors C++ unsigned arithmetic exactly
                    // (underflow is believed impossible here — buried edges
                    // are mesh edges, a subset of the counted neighbors —
                    // but wrap matches the C++ either way, where plain `-`
                    // would panic in debug builds).
                    let mut valence = vertex_neighbors
                        .get(&octagon[i])
                        .map_or(0, |neighbors| neighbors.len());
                    for edge in &buried_edges {
                        if octagon[i] == edge.0 || octagon[i] == edge.1 {
                            valence = valence.wrapping_sub(1);
                        }
                    }
                    valence = valence.wrapping_add(1);
                    if valence > MAX_VALENCE {
                        lopsided = true;
                    }
                }
                if lopsided {
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
                    {
                        shaped = false;
                        break;
                    }
                }
                if !shaped {
                    continue;
                }

                self.remeshed_vertices.push(added_position);
                for (i, quad) in quads.into_iter().enumerate() {
                    if i < run.len() {
                        self.remeshed_polygons[run[i]] = quad;
                    } else {
                        self.remeshed_polygons.push(quad);
                    }
                }
                touched_vertices.extend(octagon.iter().copied());
                converted_vertices.extend(octagon.iter().copied());
                converted_vertices.insert(added_vertex);
                round_convert_num += 1;
            }

            if 0 == round_convert_num {
                break;
            }
            convert_num += round_convert_num;
        }

        if 0 == convert_num {
            return;
        }

        self.diagnose(|| format!("Convert triangle and five edge fans:{convert_num}\n"));
        self.rebuild_half_edges();

        self.smooth_around_vertices(&converted_vertices, 3, 5);
    }
}
