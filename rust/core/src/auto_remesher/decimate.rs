use super::AutoRemesher;
use super::{DECIMATE_TRIGGER_RATIO, cxx_max, cxx_min};
use crate::par::parallel_each;
use crate::vector3::Vector3;
use std::sync::atomic::{AtomicI64, AtomicUsize, Ordering};

const DECIMATE_TARGET_RATIO: f64 = 4.0;

fn mark_sharp_edge_vertices(
    vertices: &[Vector3],
    indices: &[u32],
    sharp_edge_radians: f64,
    vertex_lock: &mut [u8],
) {
    let face_count = indices.len() / 3;

    let mut face_normals = vec![Vector3::default(); face_count];
    parallel_each(&mut face_normals, |i, normal| {
        *normal = Vector3::normal(
            &vertices[indices[i * 3] as usize],
            &vertices[indices[i * 3 + 1] as usize],
            &vertices[indices[i * 3 + 2] as usize],
        );
    });

    // The C++ worker is one face with a serial 3-edge inner loop; here the
    // worker is one flat (face, corner) slot — each worker owns a distinct
    // slot either way, so the values are identical.
    let mut edges = vec![(0u64, 0u32); face_count * 3];
    parallel_each(&mut edges, |slot, edge| {
        let i = slot / 3;
        let j = slot % 3;
        let mut first = indices[i * 3 + j];
        let mut second = indices[i * 3 + (j + 1) % 3];
        if first > second {
            std::mem::swap(&mut first, &mut second);
        }
        *edge = ((u64::from(first) << 32) | u64::from(second), i as u32);
    });
    // Every (edge, face) pair is distinct, so the parallel sort produces
    // the same order the serial one did (C++ comment, kept): any correct
    // sort of distinct keys yields one order, and `sort_unstable` is
    // deterministic for a given input.
    edges.sort_unstable();

    for i in 0..edges.len().saturating_sub(1) {
        if edges[i].0 != edges[i + 1].0 {
            continue;
        }
        if Vector3::angle(
            &face_normals[edges[i].1 as usize],
            &face_normals[edges[i + 1].1 as usize],
        ) < sharp_edge_radians
        {
            continue;
        }
        vertex_lock[(edges[i].0 >> 32) as usize] |= crate::decimator::VERTEX_PRIORITY;
        vertex_lock[(edges[i].0 & 0xffff_ffff) as usize] |= crate::decimator::VERTEX_PRIORITY;
    }
}

#[derive(Debug, Default)]
pub struct DecimationStats {
    pub time_us: AtomicI64,
    pub islands_decimated: AtomicUsize,
    pub islands_considered: AtomicUsize,
    pub triangles_before: AtomicUsize,
    pub triangles_after: AtomicUsize,
}

impl AutoRemesher {
    /// Mirrors `calculateAverageEdgeLength` (dead code on both sides:
    /// defined but never called in the C++ either).
    #[allow(dead_code)]
    fn calculate_average_edge_length(vertices: &[Vector3], faces: &[Vec<usize>]) -> f64 {
        let mut sum_of_length = 0.0;
        let mut edge_count = 0usize;
        for face in faces {
            for i in 0..face.len() {
                let j = (i + 1) % face.len();
                sum_of_length += (vertices[face[i]] - vertices[face[j]]).length();
                edge_count += 1;
            }
        }
        if edge_count == 0 {
            return 0.0;
        }
        sum_of_length / edge_count as f64
    }

    pub(crate) fn initialize_voxel_size(&mut self) {
        let area = Self::calculate_mesh_area(&self.vertices, &self.triangles);
        let triangle_area = area / self.target_triangle_count as f64;
        self.voxel_size = (triangle_area / (0.86602540378 * 0.5)).sqrt();
        // (C++ AUTO_REMESHER_DEBUG stderr omitted: the flag is never
        // defined by the build; likewise at every site below.)
    }

    fn calculate_mesh_area(vertices: &[Vector3], triangles: &[Vec<usize>]) -> f64 {
        let mut area = 0.0;
        for it in triangles {
            area += Vector3::area(&vertices[it[0]], &vertices[it[1]], &vertices[it[2]]);
        }
        area
    }

    /// Mirrors `decimateIfTooDense`. The `stats` pointer is non-null at the
    /// single call site, so it transcribes as a plain reference; the FFI
    /// calls transcribe the C++ meshoptimizer calls argument by argument.
    pub(crate) fn decimate_if_too_dense(
        vertices: &mut Vec<Vector3>,
        triangles: &mut Vec<Vec<usize>>,
        voxel_size: f64,
        sharp_edge_degrees: f64,
        _island_index: usize,
        stats: &DecimationStats,
    ) -> bool {
        stats.islands_considered.fetch_add(1, Ordering::SeqCst);

        if vertices.is_empty() || triangles.is_empty() || voxel_size <= 0.0 {
            return false;
        }

        if vertices.len() > u32::MAX as usize {
            return false;
        }

        let target_triangle_area = voxel_size * voxel_size * 0.86602540378 * 0.5;
        if target_triangle_area <= 0.0 {
            return false;
        }
        let island_target_triangle_count =
            Self::calculate_mesh_area(vertices, triangles) / target_triangle_area;
        if island_target_triangle_count < 1.0 {
            return false;
        }

        if (triangles.len() as f64) < island_target_triangle_count * DECIMATE_TRIGGER_RATIO {
            return false;
        }

        let decimate_triangle_count =
            (island_target_triangle_count * DECIMATE_TARGET_RATIO) as usize;

        let mut indices = Vec::with_capacity(triangles.len() * 3);
        for triangle in triangles.iter() {
            if triangle.len() != 3 {
                return false;
            }
            for &corner in triangle {
                indices.push(corner as u32);
            }
        }

        let mut lower_bound = vertices[0];
        let mut upper_bound = vertices[0];
        for position in vertices.iter() {
            for i in 0..3 {
                lower_bound[i] = cxx_min(lower_bound[i], position[i]);
                upper_bound[i] = cxx_max(upper_bound[i], position[i]);
            }
        }
        let center = (lower_bound + upper_bound) * 0.5;

        let mut positions = Vec::with_capacity(vertices.len() * 3);
        for position in vertices.iter() {
            positions.push((position.x() - center.x()) as f32);
            positions.push((position.y() - center.y()) as f32);
            positions.push((position.z() - center.z()) as f32);
        }
        Self::snap_decimator_input(&mut positions, &lower_bound, &upper_bound);

        // Native Rust decimator (the only path since the item-6 flip;
        // the meshoptimizer FFI is deleted).
        let (remap, welded_vertex_count) =
            crate::decimator::generate_vertex_remap(&indices, &positions);
        crate::decimator::remap_index_buffer_in_place(&mut indices, &remap);
        let welded_positions =
            crate::decimator::remap_vertex_buffer(&positions, &remap, welded_vertex_count);
        let mut welded_vertices = vec![Vector3::default(); welded_vertex_count];
        for (i, &r) in remap.iter().enumerate() {
            if r != u32::MAX {
                welded_vertices[r as usize] = vertices[i];
            }
        }

        let mut vertex_lock = Vec::new();
        if sharp_edge_degrees > 0.0 {
            vertex_lock.resize(welded_vertex_count, 0);
            mark_sharp_edge_vertices(
                &welded_vertices,
                &indices,
                sharp_edge_degrees * (std::f64::consts::PI / 180.0),
                &mut vertex_lock,
            );
        }

        let lock_opt = if vertex_lock.is_empty() {
            None
        } else {
            Some(vertex_lock.as_slice())
        };
        let decimated: Vec<u32> = crate::decimator::simplify(
            &welded_positions,
            &indices,
            lock_opt,
            decimate_triangle_count * 3,
        )
        .0;

        if decimated.len() < 3 || decimated.len() >= indices.len() {
            return false;
        }

        let mut output_index_of_welded = vec![usize::MAX; welded_vertex_count];
        let mut decimated_vertices = Vec::new();
        let mut decimated_triangles = Vec::with_capacity(decimated.len() / 3);
        let mut i = 0;
        while i + 2 < decimated.len() {
            let mut triangle = vec![0usize; 3];
            for j in 0..3 {
                let welded_index = decimated[i + j];
                if output_index_of_welded[welded_index as usize] == usize::MAX {
                    output_index_of_welded[welded_index as usize] = decimated_vertices.len();
                    decimated_vertices.push(welded_vertices[welded_index as usize]);
                }
                triangle[j] = output_index_of_welded[welded_index as usize];
            }
            decimated_triangles.push(triangle);
            i += 3;
        }

        stats.islands_decimated.fetch_add(1, Ordering::SeqCst);
        stats
            .triangles_before
            .fetch_add(triangles.len(), Ordering::SeqCst);
        stats
            .triangles_after
            .fetch_add(decimated_triangles.len(), Ordering::SeqCst);

        // Research probe (RETOPO_DUMP_DECIMATED=dir): writes the decimated
        // island as OBJ for cross-seed combinatorics comparison. No state
        // touched.
        if let Some(dir) = std::env::var_os("RETOPO_DUMP_DECIMATED") {
            let path = std::path::Path::new(&dir).join(format!("decim_island{_island_index}.obj"));
            let mut obj = String::new();
            for v in decimated_vertices.iter() {
                obj.push_str(&format!("v {} {} {}\n", v.x(), v.y(), v.z()));
            }
            for t in decimated_triangles.iter() {
                obj.push_str(&format!("f {} {} {}\n", t[0] + 1, t[1] + 1, t[2] + 1));
            }
            let _ = std::fs::write(path, obj);
        }

        *vertices = decimated_vertices;
        *triangles = decimated_triangles;
        true
    }

    /// Noise canonicalization for the decimator input (flat f32
    /// xyz, centered). Sub-visible input noise (1e-9 of the diagonal)
    /// survives the f32 cast near the bbox center and flips meshopt's
    /// collapse order globally (~23% of decimated faces differ across
    /// noise seeds). Snapping to a 1e-6-diagonal grid kills the noise
    /// uniformly (grid >> noise, grid << edges — the tightest bench
    /// mesh has min-edge 1.8e-5 of its diagonal), leaving only rare
    /// grid-boundary flips local (measured: 77% -> 91% face overlap).
    /// Degenerate bounds skip snapping (positions untouched).
    pub(crate) fn snap_decimator_input(positions: &mut [f32], lower: &Vector3, upper: &Vector3) {
        let diag = (*upper - *lower).length();
        let step = diag / 1e6;
        if !step.is_finite() || step <= 0.0 {
            return;
        }
        let g = step as f32;
        if !g.is_finite() || g <= 0.0 {
            return;
        }
        for c in positions.iter_mut() {
            *c = (*c / g).round() * g;
        }
    }
}
