use super::QuadExtractor;
use super::geom::{add_scaled_vec3, sub_scaled_vec3};
use super::types::ConnectionInfo;
use crate::double_utils::is_zero;
use crate::position_key::PositionKey;
use crate::vector3::Vector3;
use std::collections::{BTreeMap, BTreeSet};
use std::f64::consts::PI;

impl<'a> QuadExtractor<'a> {
    /// Normalized undirected edge (mirrors the `makeEdge`/`edgeOf` lambdas
    /// repeated across the C++; one shared helper, identical behavior).
    #[inline]
    pub(crate) fn edge_of(a: usize, b: usize) -> (usize, usize) {
        (a.min(b), a.max(b))
    }

    #[allow(clippy::too_many_arguments)]
    fn split_at_connection(
        connection_infos: &mut BTreeMap<(usize, usize), ConnectionInfo>,
        cross_point_map: &mut BTreeMap<PositionKey, usize>,
        cross_points: &mut Vec<Vector3>,
        source_triangles: &mut Vec<usize>,
        connections: &mut BTreeSet<(usize, usize)>,
        branches_of_point: &mut BTreeMap<usize, BTreeSet<usize>>,
        crossing_position: Vector3,
        crossing_edge: (usize, usize),
    ) -> usize {
        let Some(info) = connection_infos.get(&crossing_edge).copied() else {
            return usize::MAX;
        };
        if let Some(existing) = cross_point_map.get(&PositionKey::from_vector(&crossing_position)) {
            return *existing;
        }
        let new_point_index = cross_points.len();
        cross_point_map.insert(
            PositionKey::from_vector(&crossing_position),
            new_point_index,
        );
        cross_points.push(crossing_position);
        source_triangles.push(info.triangle_index);
        let (edge_first, edge_second) = crossing_edge;
        connections.remove(&(edge_first, edge_second));
        connections.remove(&(edge_second, edge_first));
        connection_infos.remove(&crossing_edge);
        // C++ `operator[]` + erase (both endpoints are present: the branch
        // map tracks `connections` exactly).
        branches_of_point
            .entry(edge_first)
            .or_default()
            .remove(&edge_second);
        branches_of_point
            .entry(edge_second)
            .or_default()
            .remove(&edge_first);
        for endpoint in [edge_first, edge_second] {
            connections.insert((endpoint, new_point_index));
            connection_infos.insert(Self::edge_of(endpoint, new_point_index), info);
            branches_of_point
                .entry(endpoint)
                .or_default()
                .insert(new_point_index);
            branches_of_point
                .entry(new_point_index)
                .or_default()
                .insert(endpoint);
        }
        new_point_index
    }

    #[allow(clippy::too_many_arguments)]
    fn add_walk_connection(
        connection_infos: &mut BTreeMap<(usize, usize), ConnectionInfo>,
        added_connections: &mut BTreeSet<(usize, usize)>,
        connections: &mut BTreeSet<(usize, usize)>,
        source_triangles: &[usize],
        branches_of_point: &mut BTreeMap<usize, BTreeSet<usize>>,
        from_point_index: usize,
        to_point_index: usize,
    ) -> bool {
        if from_point_index == to_point_index {
            return false;
        }
        let edge = Self::edge_of(from_point_index, to_point_index);
        if connection_infos.contains_key(&edge) {
            return false;
        }
        connections.insert((from_point_index, to_point_index));
        connection_infos.insert(
            edge,
            ConnectionInfo {
                triangle_index: source_triangles[to_point_index],
                coord_index: -1,
                integer: 0,
            },
        );
        added_connections.insert(edge);
        branches_of_point
            .entry(from_point_index)
            .or_default()
            .insert(to_point_index);
        branches_of_point
            .entry(to_point_index)
            .or_default()
            .insert(from_point_index);
        true
    }

    #[allow(clippy::too_many_arguments)]
    fn find_node_ahead(
        cross_points: &[Vector3],
        branches_of_point: &BTreeMap<usize, BTreeSet<usize>>,
        local_edges: &BTreeMap<(usize, usize), ConnectionInfo>,
        behind_points: &BTreeSet<usize>,
        nearby_radius: f64,
        ahead_cosine_threshold: f64,
        position: Vector3,
        direction: Vector3,
    ) -> usize {
        let mut nearest = usize::MAX;
        let mut nearest_distance = nearby_radius;
        for edge in local_edges.keys() {
            for endpoint in [edge.0, edge.1] {
                if behind_points.contains(&endpoint) {
                    continue;
                }
                let Some(branches) = branches_of_point.get(&endpoint) else {
                    continue;
                };
                if branches.len() <= 2 {
                    continue;
                }
                let offset = cross_points[endpoint] - position;
                let distance = offset.length();
                if is_zero(distance) || distance >= nearest_distance {
                    continue;
                }
                if Vector3::dot_product(&(offset / distance), &direction) < ahead_cosine_threshold {
                    continue;
                }
                nearest = endpoint;
                nearest_distance = distance;
            }
        }
        nearest
    }

    #[allow(clippy::too_many_arguments)]
    fn find_crossing(
        vertices: &[Vector3],
        triangles: &[Vec<usize>],
        cross_points: &[Vector3],
        local_edges: &BTreeMap<(usize, usize), ConnectionInfo>,
        behind_points: &BTreeSet<usize>,
        parallel_cosine_threshold: f64,
        tolerance: f64,
        position: Vector3,
        direction: Vector3,
        limit_distance: f64,
        also_blocked_point: usize,
    ) -> Option<(Vector3, (usize, usize), usize)> {
        let mut found = None;
        let mut nearest_distance = limit_distance;
        for (edge, info) in local_edges {
            let mut blocked = false;
            for endpoint in [edge.0, edge.1] {
                if endpoint == also_blocked_point || behind_points.contains(&endpoint) {
                    blocked = true;
                    break;
                }
            }
            if blocked {
                continue;
            }
            let from = cross_points[edge.0];
            let to = cross_points[edge.1];
            let edge_vector = to - from;
            if (Vector3::dot_product(&direction, &edge_vector.normalized())).abs()
                > parallel_cosine_threshold
            {
                continue;
            }
            let offset = from - position;
            let a = Vector3::dot_product(&edge_vector, &edge_vector);
            let b = Vector3::dot_product(&direction, &edge_vector);
            let c = Vector3::dot_product(&direction, &offset);
            let d = Vector3::dot_product(&edge_vector, &offset);
            // FMA: `a - b * b` and `(b * c - d) / denominator`.
            let denominator = (-b).mul_add(b, a);
            let mut edge_ratio = if is_zero(denominator) {
                0.0
            } else {
                b.mul_add(c, -d) / denominator
            };
            // Manual clamp like the C++ (NaN passes through on both sides).
            #[allow(clippy::manual_clamp)]
            if edge_ratio < 0.0 {
                edge_ratio = 0.0;
            } else if edge_ratio > 1.0 {
                edge_ratio = 1.0;
            }
            // Unfused `from + edgeVector * edgeRatio` (see module FMA note).
            let point_on_edge = add_scaled_vec3(from, edge_vector, edge_ratio);
            let distance = Vector3::dot_product(&(point_on_edge - position), &direction);
            if distance <= tolerance || distance >= nearest_distance {
                continue;
            }
            // Unfused `position + direction * distance` (see module FMA note).
            let point_on_ray = add_scaled_vec3(position, direction, distance);
            let miss = point_on_edge - point_on_ray;
            let miss_triangle = &triangles[info.triangle_index];
            let miss_normal = Vector3::normal(
                &vertices[miss_triangle[0]],
                &vertices[miss_triangle[1]],
                &vertices[miss_triangle[2]],
            );
            // Unfused `miss - missNormal * dot` (see module FMA note).
            let projected_miss =
                sub_scaled_vec3(miss, miss_normal, Vector3::dot_product(&miss, &miss_normal));
            if projected_miss.length() > tolerance {
                continue;
            }
            found = Some((point_on_edge, *edge, info.triangle_index));
            nearest_distance = distance;
        }
        found
    }

    pub(crate) fn hold_singular_lines(
        &mut self,
        cross_points: &mut Vec<Vector3>,
        source_triangles: &mut Vec<usize>,
        connections: &mut BTreeSet<(usize, usize)>,
    ) {
        let Some(singular_vertices) = self.singular_vertices else {
            return;
        };
        if singular_vertices.is_empty() {
            return;
        }

        const RING_COUNT: usize = 10;
        const MAX_WALK_STEPS: usize = 32;
        let ahead_cosine_threshold = (PI * 30.0 / 180.0).cos();
        const PARALLEL_COSINE_THRESHOLD: f64 = 0.9;

        let mut cross_point_map = BTreeMap::new();
        for (i, point) in cross_points.iter().enumerate() {
            // First index wins on duplicates (C++ `std::map::insert`).
            cross_point_map
                .entry(PositionKey::from_vector(point))
                .or_insert(i);
        }

        let mut branches_of_point: BTreeMap<usize, BTreeSet<usize>> = BTreeMap::new();
        for (first, second) in connections.iter() {
            branches_of_point.entry(*first).or_default().insert(*second);
            branches_of_point.entry(*second).or_default().insert(*first);
        }

        let mut triangles_around_vertex: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
        for (triangle_index, triangle) in self.triangles.iter().enumerate() {
            for vertex_index in triangle {
                triangles_around_vertex
                    .entry(*vertex_index)
                    .or_default()
                    .push(triangle_index);
            }
        }

        let mut added_connections = 0;

        let mut starved_cones = 0;
        let mut walked_cones = 0;
        // Copied out: the loop calls `&mut self` helpers.
        let singular_list = singular_vertices.to_vec();
        for singular_vertex_index in singular_list {
            if singular_vertex_index >= self.vertices.len() {
                continue;
            }
            let singular_position = self.vertices[singular_vertex_index];

            let singular_point_index =
                match cross_point_map.get(&PositionKey::from_vector(&singular_position)) {
                    Some(index) => *index,
                    None => continue,
                };
            let coming_from_point_index = match branches_of_point.get(&singular_point_index) {
                Some(branches) if branches.len() == 1 => {
                    // Invariant: exactly one branch (checked above).
                    match branches.iter().next().copied() {
                        Some(only) => only,
                        None => continue,
                    }
                }
                _ => continue,
            };
            starved_cones += 1;

            let mut neighbor_triangles = BTreeSet::new();
            {
                let mut ring_vertices = BTreeSet::from([singular_vertex_index]);
                for _ in 0..RING_COUNT {
                    let mut next_ring_vertices = BTreeSet::new();
                    for vertex_index in &ring_vertices {
                        let Some(find_triangles) = triangles_around_vertex.get(vertex_index) else {
                            continue;
                        };
                        for triangle_index in find_triangles {
                            neighbor_triangles.insert(*triangle_index);
                            for corner in &self.triangles[*triangle_index] {
                                next_ring_vertices.insert(*corner);
                            }
                        }
                    }
                    ring_vertices = next_ring_vertices;
                }
            }
            if neighbor_triangles.is_empty() {
                continue;
            }

            let mut local_edges = BTreeMap::new();
            let mut total_edge_length = 0.0;
            for (first_point, second_point) in connections.iter() {
                let edge = Self::edge_of(*first_point, *second_point);
                let Some(find_info) = self.connection_infos.get(&edge) else {
                    continue;
                };
                if !neighbor_triangles.contains(&find_info.triangle_index) {
                    continue;
                }
                // First wins (C++ `insert`); skip on duplicate.
                if local_edges.contains_key(&edge) {
                    continue;
                }
                local_edges.insert(edge, *find_info);
                total_edge_length += (cross_points[edge.0] - cross_points[edge.1]).length();
            }
            if local_edges.len() < 3 {
                continue;
            }
            let average_edge_length = total_edge_length / local_edges.len() as f64;
            if is_zero(average_edge_length) {
                continue;
            }
            let tolerance = 0.5 * average_edge_length;
            let nearby_radius = 12.0 * average_edge_length;

            let mut cone_normal = Vector3::default();
            {
                let Some(find_triangles) = triangles_around_vertex.get(&singular_vertex_index)
                else {
                    continue;
                };
                for triangle_index in find_triangles {
                    let triangle = &self.triangles[*triangle_index];
                    cone_normal += Vector3::normal(
                        &self.vertices[triangle[0]],
                        &self.vertices[triangle[1]],
                        &self.vertices[triangle[2]],
                    );
                }
                cone_normal = cone_normal.normalized();
                if cone_normal.is_zero() {
                    continue;
                }
            }
            // Deferred init: the C++ seeds this with the coming-from point
            // and unconditionally overwrites it below.
            let seed_tail;
            {
                let mut coord_index = -1;
                let mut integer = 0;
                if let Some(find_info) = self.connection_infos.get(&Self::edge_of(
                    singular_point_index,
                    coming_from_point_index,
                )) {
                    coord_index = find_info.coord_index;
                    integer = find_info.integer;
                }
                let mut previous = singular_point_index;
                let mut current = coming_from_point_index;
                for _ in 0..MAX_WALK_STEPS {
                    let Some(find_branches) = branches_of_point.get(&current) else {
                        break;
                    };
                    if find_branches.len() > 2 {
                        break;
                    }
                    let mut next = usize::MAX;
                    for neighbor in find_branches {
                        if *neighbor == previous {
                            continue;
                        }
                        let Some(find_next_info) = self
                            .connection_infos
                            .get(&Self::edge_of(current, *neighbor))
                        else {
                            continue;
                        };
                        if coord_index >= 0
                            && (find_next_info.coord_index != coord_index
                                || find_next_info.integer != integer)
                        {
                            continue;
                        }
                        next = *neighbor;
                        break;
                    }
                    if usize::MAX == next {
                        break;
                    }
                    previous = current;
                    current = next;
                }
                seed_tail = cross_points[current];
            }
            let mut seed_direction = singular_position - seed_tail;
            // Unfused `seedDirection - coneNormal * dot` (see module FMA note).
            seed_direction = sub_scaled_vec3(
                seed_direction,
                cone_normal,
                Vector3::dot_product(&seed_direction, &cone_normal),
            )
            .normalized();
            if seed_direction.is_zero() {
                continue;
            }

            let behind_points = BTreeSet::from([singular_point_index, coming_from_point_index]);

            #[derive(Clone, Copy)]
            struct WalkCrossing {
                position: Vector3,
                edge: (usize, usize),
                triangle_index: usize,
            }

            let mut path = Vec::new();
            let mut node_point_index = usize::MAX;
            {
                let mut walk_position = singular_position;
                let mut walk_direction = seed_direction;
                for _ in 0..MAX_WALK_STEPS {
                    let mut limit_distance = f64::MAX;
                    if usize::MAX == node_point_index {
                        node_point_index = Self::find_node_ahead(
                            cross_points,
                            &branches_of_point,
                            &local_edges,
                            &behind_points,
                            nearby_radius,
                            ahead_cosine_threshold,
                            walk_position,
                            walk_direction,
                        );
                    }
                    if usize::MAX != node_point_index {
                        let offset = cross_points[node_point_index] - walk_position;
                        limit_distance = offset.length();
                        walk_direction = offset.normalized();
                    }
                    let Some((crossing_position, crossing_edge, crossing_triangle)) =
                        Self::find_crossing(
                            self.vertices,
                            self.triangles,
                            cross_points,
                            &local_edges,
                            &behind_points,
                            PARALLEL_COSINE_THRESHOLD,
                            tolerance,
                            walk_position,
                            walk_direction,
                            limit_distance,
                            node_point_index,
                        )
                    else {
                        break;
                    };
                    let crossing = WalkCrossing {
                        position: crossing_position,
                        edge: crossing_edge,
                        triangle_index: crossing_triangle,
                    };
                    path.push(crossing);
                    walk_position = crossing.position;
                    let triangle = &self.triangles[crossing.triangle_index];
                    let triangle_normal = Vector3::normal(
                        &self.vertices[triangle[0]],
                        &self.vertices[triangle[1]],
                        &self.vertices[triangle[2]],
                    );
                    // Unfused `walkDirection - triangleNormal * dot` (see module FMA note).
                    let flattened = sub_scaled_vec3(
                        walk_direction,
                        triangle_normal,
                        Vector3::dot_product(&walk_direction, &triangle_normal),
                    );
                    if !flattened.is_zero() {
                        walk_direction = flattened.normalized();
                    }
                }
            }
            if usize::MAX == node_point_index {
                continue;
            }

            let mut previous_point_index = singular_point_index;
            for crossing in &path {
                let point_index = Self::split_at_connection(
                    &mut self.connection_infos,
                    &mut cross_point_map,
                    cross_points,
                    source_triangles,
                    connections,
                    &mut branches_of_point,
                    crossing.position,
                    crossing.edge,
                );
                if usize::MAX == point_index {
                    continue;
                }
                if Self::add_walk_connection(
                    &mut self.connection_infos,
                    &mut self.added_connections,
                    connections,
                    source_triangles,
                    &mut branches_of_point,
                    previous_point_index,
                    point_index,
                ) {
                    added_connections += 1;
                }
                previous_point_index = point_index;
            }
            if Self::add_walk_connection(
                &mut self.connection_infos,
                &mut self.added_connections,
                connections,
                source_triangles,
                &mut branches_of_point,
                previous_point_index,
                node_point_index,
            ) {
                added_connections += 1;
            }
            walked_cones += 1;
        }

        self.diagnose(|| {
            format!(
                "Hold singular lines walked {walked_cones} of {starved_cones} starved cone(s), added {added_connections} connection(s)\n"
            )
        });
    }
}
