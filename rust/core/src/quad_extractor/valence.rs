use super::QuadExtractor;
use crate::vector3::Vector3;
use std::collections::{BTreeMap, BTreeSet};

impl<'a> QuadExtractor<'a> {
    /// The 3/5-valence merge's `countTriangles`: number of triangles in
    /// `fan`, or `None` when a face repeats a vertex or is neither a
    /// triangle nor a quad.
    pub(crate) fn count_fan_triangles(polygons: &[Vec<usize>], fan: &[usize]) -> Option<usize> {
        let mut triangle_num = 0;
        for face_index in fan {
            let face = &polygons[*face_index];
            if Self::has_repeated_vertex(face) {
                return None;
            }
            if 3 == face.len() {
                triangle_num += 1;
            } else if 4 != face.len() {
                return None;
            }
        }
        Some(triangle_num)
    }

    pub(crate) fn split_high_valence_triangle_fans(&mut self) {
        if self.remeshed_vertices.is_empty() || self.remeshed_polygons.is_empty() {
            return;
        }

        const MIN_VALENCE: usize = 3;
        const MIN_FAN_VALENCE: usize = 5;
        const TRIANGLE_PAYMENT: i32 = 2;

        let mut split_num = 0;
        let mut split_vertices = BTreeSet::new();
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
            let mut round_split_num = 0;
            let mut fan_vertices = BTreeSet::new();
            for (source, neighbors) in &vertex_neighbors {
                if neighbors.len() >= MIN_FAN_VALENCE {
                    fan_vertices.insert(*source);
                }
            }
            for vertex in fan_vertices {
                if touched_vertices.contains(&vertex) || border_vertices.contains(&vertex) {
                    continue;
                }
                let mut fan = Vec::new();
                // Present by construction (fan vertices come from the map).
                let valence = vertex_neighbors[&vertex].len();
                if !Self::fan_around(
                    &self.remeshed_polygons,
                    &vertex_faces,
                    &edge_faces,
                    vertex,
                    &mut fan,
                ) || fan.len() != valence
                {
                    continue;
                }

                let mut best_score = 0;
                let mut best_quads: Vec<Vec<usize>> = Vec::new();
                let mut best_run = Vec::new();
                let mut best_octagon = Vec::new();
                let mut best_position = Vector3::default();
                for run_start in 0..fan.len() {
                    let odd_size = self.remeshed_polygons[fan[run_start]].len();
                    if 3 != odd_size && 4 != odd_size {
                        continue;
                    }
                    let even_size = if 3 == odd_size { 4 } else { 3 };
                    let mut run = Vec::with_capacity(4);
                    for i in 0..4 {
                        let face_index = fan[(run_start + i) % fan.len()];
                        let face = &self.remeshed_polygons[face_index];
                        if face.len() != (if 0 == i % 2 { odd_size } else { even_size })
                            || Self::has_repeated_vertex(face)
                        {
                            break;
                        }
                        run.push(face_index);
                    }
                    if 4 != run.len() {
                        continue;
                    }

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
                    if !simple_boundary || 8 != boundary_next.len() || 3 != buried_edges.len() {
                        continue;
                    }
                    let mut buried_at_fan_vertex = true;
                    let mut buried_counts: BTreeMap<usize, usize> = BTreeMap::new();
                    for buried in &buried_edges {
                        let (first, second) = *buried;
                        if vertex != first && vertex != second {
                            buried_at_fan_vertex = false;
                            break;
                        }
                        *buried_counts.entry(first).or_insert(0) += 1;
                        *buried_counts.entry(second).or_insert(0) += 1;
                    }
                    if !buried_at_fan_vertex {
                        continue;
                    }

                    let mut octagon = Vec::with_capacity(8);
                    let mut walk = vertex;
                    for _ in 0..8 {
                        octagon.push(walk);
                        match boundary_next.get(&walk) {
                            Some(next) => walk = *next,
                            _ => break,
                        }
                    }
                    if 8 != octagon.len() || walk != vertex || Self::has_repeated_vertex(&octagon) {
                        continue;
                    }
                    let broken_at = octagon[4];
                    if !buried_edges.contains(&Self::edge_of(vertex, broken_at)) {
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
                    let mut old_score = TRIANGLE_PAYMENT;
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
                    let added_position =
                        (self.remeshed_vertices[vertex] + self.remeshed_vertices[broken_at]) * 0.5;

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

                    best_score = new_score;
                    best_quads = quads;
                    best_run = run;
                    best_octagon = octagon;
                    best_position = added_position;
                }
                if best_quads.is_empty() {
                    continue;
                }

                split_vertices.insert(self.remeshed_vertices.len());
                self.remeshed_vertices.push(best_position);
                for (i, face_index) in best_run.iter().enumerate() {
                    self.remeshed_polygons[*face_index] = best_quads[i].clone();
                }
                touched_vertices.extend(best_octagon.iter().copied());
                split_vertices.extend(best_octagon.iter().copied());
                round_split_num += 1;
            }

            if 0 == round_split_num {
                break;
            }
            split_num += round_split_num;
        }

        if 0 == split_num {
            return;
        }

        self.diagnose(|| format!("Split high valence triangle fans:{split_num}\n"));
        self.rebuild_half_edges();

        self.smooth_around_vertices(&split_vertices, 3, 5);
    }

    pub(crate) fn collapse_three_valence_edge_pairs(&mut self) {
        if self.remeshed_vertices.is_empty() || self.remeshed_polygons.is_empty() {
            return;
        }

        const MIN_VALENCE: usize = 3;
        const HEXAGON_SIZE: usize = 6;
        const PATCH_FACE_NUM: usize = 4;
        const PATCH_BURIED_EDGE_NUM: usize = 5;

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
            let mut removed_faces = BTreeSet::new();
            let mut added_faces: Vec<Vec<usize>> = Vec::new();
            let mut round_collapse_count = 0;
            for (edge, faces) in &edge_faces {
                if 2 != faces.len() {
                    continue;
                }
                let first = edge.0;
                let second = edge.1;
                // Present by construction (edge endpoints are used).
                if 3 != vertex_neighbors[&first].len() || 3 != vertex_neighbors[&second].len() {
                    continue;
                }
                // Present by construction (edge endpoints are used).
                if 3 != vertex_faces[&first].len() || 3 != vertex_faces[&second].len() {
                    continue;
                }

                let mut patch_faces = vertex_faces[&first].clone();
                patch_faces.extend(vertex_faces[&second].iter().copied());
                patch_faces.sort_unstable();
                patch_faces.dedup();
                if PATCH_FACE_NUM != patch_faces.len() {
                    continue;
                }
                let mut usable = true;
                for face_index in &patch_faces {
                    let face = &self.remeshed_polygons[*face_index];
                    if 4 != face.len() || Self::has_repeated_vertex(face) {
                        usable = false;
                        break;
                    }
                    for vertex in face {
                        if border_vertices.contains(vertex) || touched_vertices.contains(vertex) {
                            usable = false;
                            break;
                        }
                    }
                    if !usable {
                        break;
                    }
                }
                if !usable {
                    continue;
                }

                let mut directed_edges = BTreeSet::new();
                for face_index in &patch_faces {
                    let face = &self.remeshed_polygons[*face_index];
                    for i in 0..face.len() {
                        directed_edges.insert((face[i], face[(i + 1) % face.len()]));
                    }
                }
                // `std::map` here (ordered; the convert/merge/split
                // passes use `unordered_map`, but theirs is lookup-only).
                // `buried_counts` is `unordered_map` yet lookup-only, so a
                // `BTreeMap` is exact for both.
                let mut boundary_next = BTreeMap::new();
                let mut buried_counts: BTreeMap<usize, usize> = BTreeMap::new();
                let mut buried_edge_num = 0;
                let mut simple_boundary = true;
                for directed_edge in &directed_edges {
                    let (from, to) = *directed_edge;
                    if directed_edges.contains(&(to, from)) {
                        if from < to {
                            buried_edge_num += 1;
                            *buried_counts.entry(from).or_insert(0) += 1;
                            *buried_counts.entry(to).or_insert(0) += 1;
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
                if !simple_boundary
                    || HEXAGON_SIZE != boundary_next.len()
                    || PATCH_BURIED_EDGE_NUM != buried_edge_num
                {
                    continue;
                }

                let mut hexagon = Vec::with_capacity(HEXAGON_SIZE);
                // Ordered `std::map`, so `begin()->first` is the smallest
                // boundary key.
                let start = *boundary_next.keys().next().unwrap_or(&usize::MAX);
                let mut walk = start;
                for _ in 0..HEXAGON_SIZE {
                    hexagon.push(walk);
                    match boundary_next.get(&walk) {
                        Some(next) => walk = *next,
                        _ => break,
                    }
                }
                if HEXAGON_SIZE != hexagon.len()
                    || walk != start
                    || Self::has_repeated_vertex(&hexagon)
                {
                    continue;
                }

                let mut old_score = Self::valence_score(vertex_neighbors[&first].len())
                    + Self::valence_score(vertex_neighbors[&second].len());
                for corner in &hexagon {
                    old_score += Self::valence_score(vertex_neighbors[corner].len());
                }
                let mut old_normal = Vector3::default();
                for face_index in &patch_faces {
                    old_normal += Self::face_normal_of(
                        &self.remeshed_vertices,
                        &self.remeshed_polygons[*face_index],
                    );
                }

                let mut best_score = 0;
                let mut best_flow = 0.0;
                let mut best_quads: Vec<Vec<usize>> = Vec::new();
                for i in 0..HEXAGON_SIZE / 2 {
                    let across = i + HEXAGON_SIZE / 2;
                    let corner = hexagon[i];
                    let opposite = hexagon[across];
                    // Present by construction (hexagon corners are used).
                    if vertex_neighbors[&corner].contains(&opposite) {
                        continue;
                    }

                    let mut new_score = 0;
                    let mut lopsided = false;
                    for (j, hexagon_corner) in hexagon.iter().enumerate() {
                        // Present by construction (hexagon corners are
                        // used); buried edges are distinct mesh edges, so
                        // the count never exceeds the valence.
                        let valence = vertex_neighbors[hexagon_corner].len()
                            - buried_counts.get(hexagon_corner).copied().unwrap_or(0)
                            + usize::from(i == j || across == j);
                        if valence < MIN_VALENCE {
                            lopsided = true;
                        }
                        new_score += Self::valence_score(valence);
                        if lopsided {
                            break;
                        }
                    }
                    if lopsided || new_score > old_score {
                        continue;
                    }

                    let quads = vec![
                        vec![
                            corner,
                            hexagon[(i + 1) % HEXAGON_SIZE],
                            hexagon[(i + 2) % HEXAGON_SIZE],
                            opposite,
                        ],
                        vec![
                            opposite,
                            hexagon[(across + 1) % HEXAGON_SIZE],
                            hexagon[(across + 2) % HEXAGON_SIZE],
                            corner,
                        ],
                    ];
                    let mut folded = false;
                    for quad in &quads {
                        if Vector3::dot_product(
                            &old_normal,
                            &Self::face_normal_of(&self.remeshed_vertices, quad),
                        ) <= 0.0
                        {
                            folded = true;
                            break;
                        }
                    }
                    if folded {
                        continue;
                    }

                    let span = self.remeshed_vertices[opposite] - self.remeshed_vertices[corner];
                    let direction = span.normalized();
                    let corner_skip = BTreeSet::from([
                        hexagon[(i + 1) % HEXAGON_SIZE],
                        hexagon[(i + HEXAGON_SIZE - 1) % HEXAGON_SIZE],
                        first,
                        second,
                    ]);
                    let opposite_skip = BTreeSet::from([
                        hexagon[(across + 1) % HEXAGON_SIZE],
                        hexagon[(across + HEXAGON_SIZE - 1) % HEXAGON_SIZE],
                        first,
                        second,
                    ]);
                    let mut new_flow = Self::flow_score_at(
                        &self.remeshed_vertices,
                        &vertex_neighbors,
                        corner,
                        direction,
                        &corner_skip,
                    ) + Self::flow_score_at(
                        &self.remeshed_vertices,
                        &vertex_neighbors,
                        opposite,
                        -direction,
                        &opposite_skip,
                    );
                    for quad in &quads {
                        new_flow += Self::corner_score_of(&self.remeshed_vertices, quad);
                    }
                    if !best_quads.is_empty()
                        && (new_score > best_score
                            || (new_score == best_score && new_flow <= best_flow))
                    {
                        continue;
                    }

                    best_score = new_score;
                    best_flow = new_flow;
                    best_quads = quads;
                }
                if best_quads.is_empty() {
                    continue;
                }

                for face_index in &patch_faces {
                    removed_faces.insert(*face_index);
                    for vertex in &self.remeshed_polygons[*face_index] {
                        touched_vertices.insert(*vertex);
                    }
                }
                for quad in best_quads {
                    added_faces.push(quad);
                }
                collapsed_vertices.extend(hexagon.iter().copied());
                round_collapse_count += 1;
            }

            if 0 == round_collapse_count {
                break;
            }

            let mut rewritten =
                Vec::with_capacity(self.remeshed_polygons.len() + added_faces.len());
            for (face_index, face) in self.remeshed_polygons.iter().enumerate() {
                if removed_faces.contains(&face_index) {
                    continue;
                }
                rewritten.push(face.clone());
            }
            for face in added_faces {
                rewritten.push(face);
            }
            self.remeshed_polygons = rewritten;
            collapse_count += round_collapse_count;
        }

        if 0 == collapse_count {
            return;
        }

        let compacted = self.compact_vertices(&collapsed_vertices);

        self.diagnose(|| format!("Collapse three valence edge pairs:{collapse_count}\n"));
        self.rebuild_half_edges();

        self.smooth_around_vertices(&compacted, 3, 5);
    }

    /// Corner index where the directed edge `from -> to` enters `face`
    /// (`face.len()` on a miss): the edge switch's `findDirectedEdge`.
    fn find_directed_entry(face: &[usize], from: usize, to: usize) -> usize {
        for i in 0..face.len() {
            if from == face[i] && to == face[(i + 1) % face.len()] {
                return i;
            }
        }
        face.len()
    }

    pub(crate) fn switch_high_valence_edges(&mut self) {
        if self.remeshed_vertices.is_empty() || self.remeshed_polygons.is_empty() {
            return;
        }

        // Two quads sharing an edge make a hexagon with three diagonals,
        // the shared edge being one of them. Switching to another
        // diagonal takes one neighbor away from each end of the shared
        // edge and hands one to each end of the new diagonal, no point is
        // added, removed or moved
        let mut switch_num = 0;
        let mut switched_vertices = BTreeSet::new();
        loop {
            let mut edge_faces: BTreeMap<(usize, usize), Vec<usize>> = BTreeMap::new();
            let mut vertex_neighbors: BTreeMap<usize, BTreeSet<usize>> = BTreeMap::new();
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
            }

            // Four neighbors is the wrong target on a border, leave those
            // alone
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

            // A second switch reaching into the same hexagon would work
            // off the face indices and the valences the first one left
            // behind
            let mut touched_vertices = BTreeSet::new();
            let mut round_switch_num = 0;
            for (key, value) in &edge_faces {
                if 2 != value.len() {
                    continue;
                }

                let a = key.0;
                let b = key.1;
                // Present by construction (edge endpoints are used).
                let a_valence = vertex_neighbors[&a].len();
                let b_valence = vertex_neighbors[&b].len();
                if a_valence <= 4 || b_valence <= 4 {
                    continue;
                }
                if border_vertices.contains(&a) || border_vertices.contains(&b) {
                    continue;
                }

                let mut first = value[0];
                let mut second = value[1];
                if 4 != self.remeshed_polygons[first].len()
                    || 4 != self.remeshed_polygons[second].len()
                {
                    continue;
                }
                let mut first_entry =
                    Self::find_directed_entry(&self.remeshed_polygons[first], a, b);
                if first_entry >= self.remeshed_polygons[first].len() {
                    std::mem::swap(&mut first, &mut second);
                    first_entry = Self::find_directed_entry(&self.remeshed_polygons[first], a, b);
                }
                let second_entry = Self::find_directed_entry(&self.remeshed_polygons[second], b, a);
                if first_entry >= self.remeshed_polygons[first].len()
                    || second_entry >= self.remeshed_polygons[second].len()
                {
                    continue;
                }

                // The hexagon runs b, c, d, a, e, f, keeping the winding
                // of both quads
                let first_face = &self.remeshed_polygons[first];
                let second_face = &self.remeshed_polygons[second];
                let c = first_face[(first_entry + 2) % 4];
                let d = first_face[(first_entry + 3) % 4];
                let e = second_face[(second_entry + 2) % 4];
                let f = second_face[(second_entry + 3) % 4];
                let hexagon = BTreeSet::from([a, b, c, d, e, f]);
                if 6 != hexagon.len() {
                    continue;
                }
                let mut touched = false;
                for vertex in &hexagon {
                    if touched_vertices.contains(vertex) {
                        touched = true;
                        break;
                    }
                }
                if touched {
                    continue;
                }

                let old_normal = Self::face_normal_of(&self.remeshed_vertices, first_face)
                    + Self::face_normal_of(&self.remeshed_vertices, second_face);
                let old_valence_score =
                    Self::valence_score(a_valence) + Self::valence_score(b_valence);

                let mut best_gain = 0;
                let mut best_shape = 0.0;
                let mut best_face1 = Vec::new();
                let mut best_face2 = Vec::new();
                for candidate in 0..2 {
                    let x = if 0 == candidate { c } else { d };
                    let y = if 0 == candidate { e } else { f };
                    if border_vertices.contains(&x) || border_vertices.contains(&y) {
                        continue;
                    }
                    // The new diagonal would land on an edge which is
                    // already there
                    // Present by construction (hexagon corners are used).
                    if vertex_neighbors[&x].contains(&y) {
                        continue;
                    }
                    let x_valence = vertex_neighbors[&x].len();
                    let y_valence = vertex_neighbors[&y].len();
                    let gain = old_valence_score
                        + Self::valence_score(x_valence)
                        + Self::valence_score(y_valence)
                        - (Self::valence_score(a_valence - 1)
                            + Self::valence_score(b_valence - 1)
                            + Self::valence_score(x_valence + 1)
                            + Self::valence_score(y_valence + 1));
                    if gain <= 0 {
                        continue;
                    }
                    let face1 = if 0 == candidate {
                        vec![c, d, a, e]
                    } else {
                        vec![d, a, e, f]
                    };
                    let face2 = if 0 == candidate {
                        vec![e, f, b, c]
                    } else {
                        vec![f, b, c, d]
                    };
                    if Vector3::dot_product(
                        &old_normal,
                        &Self::face_normal_of(&self.remeshed_vertices, &face1),
                    ) <= 0.0
                        || Vector3::dot_product(
                            &old_normal,
                            &Self::face_normal_of(&self.remeshed_vertices, &face2),
                        ) <= 0.0
                    {
                        continue;
                    }
                    if existing_faces.contains(&Self::canonical_face(&face1))
                        || existing_faces.contains(&Self::canonical_face(&face2))
                    {
                        continue;
                    }
                    let shape = Self::corner_score_of(&self.remeshed_vertices, &face1)
                        + Self::corner_score_of(&self.remeshed_vertices, &face2);
                    if gain < best_gain || (gain == best_gain && shape <= best_shape) {
                        continue;
                    }
                    best_gain = gain;
                    best_shape = shape;
                    best_face1 = face1;
                    best_face2 = face2;
                }
                if best_face1.is_empty() {
                    continue;
                }

                existing_faces.remove(&Self::canonical_face(&self.remeshed_polygons[first]));
                existing_faces.remove(&Self::canonical_face(&self.remeshed_polygons[second]));
                existing_faces.insert(Self::canonical_face(&best_face1));
                existing_faces.insert(Self::canonical_face(&best_face2));
                self.remeshed_polygons[first] = best_face1;
                self.remeshed_polygons[second] = best_face2;
                touched_vertices.extend(hexagon.iter().copied());
                switched_vertices.extend(hexagon.iter().copied());
                round_switch_num += 1;
            }

            if 0 == round_switch_num {
                break;
            }
            switch_num += round_switch_num;
        }

        if 0 == switch_num {
            return;
        }

        self.diagnose(|| format!("Switch high valence edges:{switch_num}\n"));
        self.rebuild_half_edges();

        // The switched quads kept their points, pull the reconnected
        // patches back into shape
        self.smooth_around_vertices(&switched_vertices, 3, 5);
    }
}
