use super::AutoRemesher;
use super::DecimationStats;
use super::{cxx_max, cxx_min};
use crate::density::Density;
use crate::isotropic_remesher::IsotropicRemesher;
use crate::par::{parallel_each, parallel_each_zip2};
use crate::progress::ProgressHandler;
use crate::vector3::Vector3;
use std::sync::atomic::{AtomicI64, Ordering};
use std::time::Instant;

impl AutoRemesher {
    /// Mirrors `resample`. The stats/time pointers are non-null at the
    /// single call site (plain references); only the progress handler is
    /// genuinely optional (`None` in quiet mode). The decimated-output
    /// pointers are likewise always non-null (plain `&mut`). The handler
    /// moves in by value where the C++ passes a pointer to its local: the
    /// downstream remesher takes ownership either way (the C++ copies the
    /// `std::function` into its member), so exactly one live handler
    /// exists during the call on both sides.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn resample(
        vertices: &mut Vec<Vector3>,
        triangles: &mut Vec<Vec<usize>>,
        voxel_size: f64,
        adaptivity: f64,
        sharp_edge_degrees: f64,
        smooth_normal_degrees: f64,
        // (Used only by the C++ AUTO_REMESHER_DEBUG prints, which compile
        // out; kept for the 1:1 signature.)
        _island_index: usize,
        decimation_stats: &DecimationStats,
        adaptive_field_time_us: &AtomicI64,
        progress_handler: Option<ProgressHandler>,
        decimated_vertices_out: &mut Vec<Vector3>,
        decimated_triangles_out: &mut Vec<Vec<usize>>,
        density_in: &[f64],
        density_out: &mut Vec<f64>,
    ) {
        // Local density control, default off: every block below is guarded
        // on densityActive, so a run without a mask executes the exact same
        // statements (and floating-point ops) as before.
        let mut island_density = Vec::new();
        if !density_in.is_empty() && density_in.len() == vertices.len() {
            island_density = Density::normalize_field(density_in);
        }
        let mut density_active = !island_density.is_empty();
        let mut positions_before_decimate = Vec::new();
        if density_active {
            positions_before_decimate = vertices.clone();
        }

        let t_decimate_start = Instant::now();
        let decimated = Self::decimate_if_too_dense(
            vertices,
            triangles,
            voxel_size,
            sharp_edge_degrees,
            _island_index,
            decimation_stats,
        );
        if density_active && decimated {
            // Decimation retopologized the island: carry the mask across by
            // nearest position, then re-normalize (a degenerate map falls
            // back to uniform, which normalizes back to OFF).
            island_density = Density::normalize_field(&Density::resample_nearest(
                &positions_before_decimate,
                &island_density,
                vertices,
            ));
            density_active = !island_density.is_empty();
        }
        decimation_stats.time_us.fetch_add(
            t_decimate_start.elapsed().as_micros() as i64,
            Ordering::SeqCst,
        );

        *decimated_vertices_out = vertices.clone();
        *decimated_triangles_out = triangles.clone();

        let t_field_start = Instant::now();
        let mut vertex_target_lengths = Vec::new();
        let density_usable = density_active && island_density.len() == vertices.len();
        if (adaptivity > 0.0 || density_usable) && !vertices.is_empty() {
            // A target-length field redistributes the uniform triangle
            // budget. The field is deliberately computed on the input mesh:
            // IsotropicRemesher propagates it to vertices created by edge
            // splits.
            let min_ratio = 0.35;
            let max_ratio = 3.0;
            let epsilon = 1e-12;

            // Do not add face normals directly from parallel workers:
            // adjacent faces write to the same vertex. Compute faces in
            // parallel, then do the small accumulation pass serially.
            let mut face_normals = vec![Vector3::default(); triangles.len()];
            let mut face_areas = vec![0.0; triangles.len()];
            parallel_each_zip2(&mut face_normals, &mut face_areas, |i, normal, area| {
                let tri = &triangles[i];
                *area = Vector3::area(&vertices[tri[0]], &vertices[tri[1]], &vertices[tri[2]]);
                if *area > epsilon {
                    *normal =
                        Vector3::normal(&vertices[tri[0]], &vertices[tri[1]], &vertices[tri[2]]);
                }
            });

            let mut normals = vec![Vector3::default(); vertices.len()];
            let mut neighbors: Vec<Vec<usize>> = Vec::new();
            neighbors.resize_with(vertices.len(), Vec::new);
            for i in 0..triangles.len() {
                let tri = &triangles[i];
                if face_areas[i] <= epsilon {
                    continue;
                }
                let weighted_normal = face_normals[i] * face_areas[i];
                for j in 0..3 {
                    normals[tri[j]] += weighted_normal;
                    neighbors[tri[j]].push(tri[(j + 1) % 3]);
                    neighbors[tri[j]].push(tri[(j + 2) % 3]);
                }
            }
            parallel_each_zip2(&mut normals, &mut neighbors, |_, normal, ring| {
                normal.normalize();
                ring.sort_unstable();
                ring.dedup();
            });

            // Mean normal variation per unit length is less sensitive to a
            // single bad triangle than the previous maximum-one-ring
            // estimate.
            let mut vertex_curvature = vec![0.0; vertices.len()];
            parallel_each(&mut vertex_curvature, |v, curvature| {
                let ring = &neighbors[v];
                if ring.is_empty() || normals[v].length_squared() <= epsilon {
                    return;
                }
                let mut weighted_curvature = 0.0;
                let mut total_weight = 0.0;
                for &u in ring {
                    let length = (vertices[u] - vertices[v]).length();
                    if length <= epsilon || normals[u].length_squared() <= epsilon {
                        continue;
                    }
                    let mut cosine = Vector3::dot_product(&normals[v], &normals[u]);
                    cosine = cxx_min(1.0, cxx_max(-1.0, cosine));
                    weighted_curvature += cosine.acos();
                    total_weight += length;
                }
                if total_weight > epsilon {
                    *curvature = weighted_curvature / total_weight;
                }
            });

            // A percentile reference prevents a few very sharp/noisy
            // vertices from making the rest of the surface appear flat.
            let mut non_zero_curvatures = Vec::with_capacity(vertex_curvature.len());
            for &curvature in &vertex_curvature {
                if curvature > epsilon {
                    non_zero_curvatures.push(curvature);
                }
            }
            let have_curvature_reference = adaptivity > 0.0 && !non_zero_curvatures.is_empty();
            if have_curvature_reference || density_usable {
                vertex_target_lengths.resize(vertices.len(), 0.0);
                let mut importance = vec![1.0; vertices.len()];
                if have_curvature_reference {
                    let reference_index = (non_zero_curvatures.len() - 1) * 3 / 4;
                    // `std::nth_element` with `<`: both place the rank-`k`
                    // value at `k`, so the reference value is identical on
                    // NaN-free inputs (NaN curvatures make the comparator
                    // inconsistent on both sides — unspecified either way).
                    let (_, reference, _) = non_zero_curvatures
                        .select_nth_unstable_by(reference_index, |a, b| {
                            a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal)
                        });
                    let curvature_reference = *reference;
                    let strength = cxx_min(adaptivity, 2.0) * 7.0;
                    parallel_each(&mut importance, |v, slot| {
                        let normalized = cxx_min(
                            4.0,
                            vertex_curvature[v] / cxx_max(curvature_reference, epsilon),
                        );
                        // FMA audit (site A): Clang fuses the outer
                        // multiply-add (IR: plain fmul, then
                        // llvm.fmuladd(t, normalized, slot)); the first
                        // multiply stays unfused. Transcribed explicitly.
                        *slot = (strength * normalized).mul_add(normalized, *slot);
                    });
                    if density_usable {
                        // Fold the mask in multiplicatively: local triangle
                        // density scales by the mask while the
                        // average-importance normalization below keeps the
                        // island budget fixed.
                        for v in 0..vertices.len() {
                            importance[v] *= island_density[v];
                        }
                    }
                } else {
                    // Flat island under a mask (or adaptivity off): the mask
                    // alone drives the target-length field.
                    importance.clone_from(&island_density);
                }

                // Keep integral(area / h^2) equal to the uniform field,
                // which preserves the budget implied by voxelSize while
                // moving triangles from flat regions to detailed ones.
                let mut total_area = 0.0;
                let mut weighted_importance = 0.0;
                for i in 0..triangles.len() {
                    let tri = &triangles[i];
                    total_area += face_areas[i];
                    weighted_importance += face_areas[i]
                        * (importance[tri[0]] + importance[tri[1]] + importance[tri[2]])
                        / 3.0;
                }
                if total_area > epsilon {
                    let average_importance = weighted_importance / total_area;
                    for v in 0..vertices.len() {
                        let mut multiplier = (average_importance / importance[v]).sqrt();
                        multiplier = cxx_min(max_ratio, cxx_max(min_ratio, multiplier));
                        vertex_target_lengths[v] = voxel_size * multiplier;
                    }
                } else {
                    vertex_target_lengths.clear();
                }
            }
        }
        adaptive_field_time_us
            .fetch_add(t_field_start.elapsed().as_micros() as i64, Ordering::SeqCst);

        let mut positions_before_remesh = Vec::new();
        if density_usable {
            positions_before_remesh.clone_from(vertices);
        }
        let mut isotropic_remesher = IsotropicRemesher::new(vertices, triangles);
        // NOTE: the C++ re-checks `*progressHandler` (the std::function
        // null state); the only constructed handler is a live closure, so
        // the check is dead and has no counterpart.
        if let Some(handler) = progress_handler {
            isotropic_remesher.set_progress_handler(handler);
        }
        isotropic_remesher.set_target_edge_length(voxel_size);
        if !vertex_target_lengths.is_empty() {
            isotropic_remesher.set_vertex_target_edge_lengths(&vertex_target_lengths);
        }
        isotropic_remesher.set_sharp_edge_degrees(sharp_edge_degrees);
        isotropic_remesher.set_smooth_normal_degrees(smooth_normal_degrees);
        isotropic_remesher.remesh();
        *vertices = isotropic_remesher.remeshed_vertices().to_vec();
        *triangles = isotropic_remesher.remeshed_triangles().to_vec();
        // Research probe (RETOPO_DUMP_STAGES=dir): stage-1 working mesh.
        // Research probe (kept for item-7 stage analysis); no state
        // touched.
        if let Some(dir) = std::env::var_os("RETOPO_DUMP_STAGES") {
            let path = std::path::Path::new(&dir)
                .join(format!("stage1_working_island{_island_index}.obj"));
            let mut obj = String::new();
            for v in vertices.iter() {
                obj.push_str(&format!("v {} {} {}\n", v.x(), v.y(), v.z()));
            }
            for t in triangles.iter() {
                obj.push_str(&format!("f {} {} {}\n", t[0] + 1, t[1] + 1, t[2] + 1));
            }
            let _ = std::fs::write(path, obj);
        }
        density_out.clear();
        if density_usable {
            // Isotropic remeshing retopologized the island: carry the mask
            // to the new vertices so the parameterizer can modulate its
            // scaling field. A degenerate map normalizes back to OFF
            // downstream.
            *density_out = Density::normalize_field(&Density::resample_nearest(
                &positions_before_remesh,
                &island_density,
                vertices,
            ));
        }
    }
}
