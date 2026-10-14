//! Port of `core/isotropicremesher.*` (`retopo.core.isotropic_remesher`).
//!
//! Line-by-line mirror: same struct, same setters in the same order, same
//! defaults (`target_edge_length = 0`, `sharp_edge_degrees = 60`,
//! `smooth_normal_degrees = 0`, 3 remesh iterations), same output loops.
//! The core-to-thirdparty `Vector3` conversion is the identity here (both
//! sides use [`crate::vector3::Vector3`]).
//!
//! Two deliberate restructures, both proven by the differential oracle
//! (`rust/core/tests/isotropic_remesher_diff.rs`):
//! - The borrowed C++ out-params become borrowed slices / references with a
//!   lifetime (`&'a [f64]`, `&'a HashSet<usize>`).
//! - `remesh` round-trips the progress handler through the kernel
//!   (take it in, take it back afterwards): `Box<dyn Fn>` is not cloneable
//!   like `std::function`, and a plain move would drop the handler after
//!   the first call while the C++ keeps its copy.
//! - `set_constraint_vertices` stores a set the C++ `remesh()` never reads
//!   (dead setter, kept for API parity).

use crate::iso_remesh_kernel::IsoRemeshKernel;
use crate::progress::ProgressHandler;
use crate::vector3::Vector3;
use std::collections::HashSet;

/// Isotropic remesher (mirrors `AutoRemesher::IsotropicRemesher`).
pub struct IsotropicRemesher<'a> {
    vertices: Vec<Vector3>,
    triangles: Vec<Vec<usize>>,
    #[allow(dead_code)]
    constraint_vertices: Option<&'a HashSet<usize>>,
    vertex_target_edge_lengths: Option<&'a [f64]>,
    target_edge_length: f64,
    sharp_edge_degrees: f64,
    smooth_normal_degrees: f64,
    progress_handler: Option<ProgressHandler>,
    remeshed_vertices: Vec<Vector3>,
    remeshed_triangles: Vec<Vec<usize>>,
    /// Report kernel construction anomalies on stderr (CLI `--verbose`).
    verbose: bool,
}

impl<'a> IsotropicRemesher<'a> {
    /// Mirrors `m_remeshIterations` (hardcoded 3, no setter in the C++).
    const REMESH_ITERATIONS: usize = 3;

    /// Mirrors the constructor (copies its inputs like the C++ member
    /// initializers do).
    #[must_use]
    pub fn new(vertices: &[Vector3], triangles: &[Vec<usize>]) -> Self {
        Self {
            vertices: vertices.to_vec(),
            triangles: triangles.to_vec(),
            constraint_vertices: None,
            vertex_target_edge_lengths: None,
            target_edge_length: 0.0,
            sharp_edge_degrees: 60.0,
            smooth_normal_degrees: 0.0,
            progress_handler: None,
            remeshed_vertices: Vec::new(),
            remeshed_triangles: Vec::new(),
            verbose: false,
        }
    }

    /// Report non-triangle / repeated-halfedge input on stderr, one
    /// summary line each (the C++ printed a line per occurrence on every
    /// run; off by default so `--quiet` and default runs stay clean).
    pub fn set_verbose(&mut self, verbose: bool) {
        self.verbose = verbose;
    }

    /// Mirrors `setConstraintVertices` (stored; the C++ `remesh()` never
    /// reads it, and neither does this port).
    pub fn set_constraint_vertices(&mut self, constraint_vertices: &'a HashSet<usize>) {
        self.constraint_vertices = Some(constraint_vertices);
    }

    /// Mirrors `setVertexTargetEdgeLengths`.
    pub fn set_vertex_target_edge_lengths(&mut self, target_lengths: &'a [f64]) {
        self.vertex_target_edge_lengths = Some(target_lengths);
    }

    /// Mirrors `setTargetEdgeLength`.
    pub fn set_target_edge_length(&mut self, edge_length: f64) {
        self.target_edge_length = edge_length;
    }

    /// Mirrors `setSharpEdgeDegrees`.
    pub fn set_sharp_edge_degrees(&mut self, degrees: f64) {
        self.sharp_edge_degrees = degrees;
    }

    /// Mirrors `setSmoothNormalDegrees`.
    pub fn set_smooth_normal_degrees(&mut self, degrees: f64) {
        self.smooth_normal_degrees = degrees;
    }

    /// Mirrors `setProgressHandler`.
    pub fn set_progress_handler(&mut self, progress_handler: ProgressHandler) {
        self.progress_handler = Some(progress_handler);
    }

    /// Mirrors `remeshedVertices`.
    #[must_use]
    pub fn remeshed_vertices(&self) -> &[Vector3] {
        &self.remeshed_vertices
    }

    /// Mirrors `remeshedTriangles`.
    #[must_use]
    pub fn remeshed_triangles(&self) -> &[Vec<usize>] {
        &self.remeshed_triangles
    }

    /// Mirrors `remesh` (appends to the output vectors on every call, like
    /// the C++ `push_back` loops; always returns `true`: the C++
    /// `nullptr` check is dead since the mesh is built in the constructor).
    pub fn remesh(&mut self) -> bool {
        let mut remesher = IsoRemeshKernel::new(&self.vertices, &self.triangles);
        if self.verbose {
            let (non_triangles, repeated) = remesher.build_anomalies();
            if non_triangles > 0 {
                eprintln!("Found non-triangle faces:{non_triangles}");
            }
            if repeated > 0 {
                eprintln!("Found repeated halfedges:{repeated}");
            }
        }
        if self.target_edge_length > 0.0 {
            remesher.set_target_edge_length(self.target_edge_length);
        }
        if let Some(target_lengths) = self.vertex_target_edge_lengths {
            remesher.set_vertex_target_edge_lengths(target_lengths);
        }
        remesher.set_sharp_edge_included_angle(180.0 - self.sharp_edge_degrees);
        remesher.set_smooth_normal_degrees(self.smooth_normal_degrees);
        if let Some(handler) = self.progress_handler.take() {
            remesher.set_progress_handler(handler);
        }
        remesher.remesh(Self::REMESH_ITERATIONS);
        if let Some(handler) = remesher.take_progress_handler() {
            self.progress_handler = Some(handler);
        }

        let halfedge_mesh = remesher.remeshed_halfedge_mesh();
        let mut output_index = 0;
        let mut vertex = halfedge_mesh.move_to_next_vertex(None);
        while let Some(current) = vertex {
            vertex = halfedge_mesh.move_to_next_vertex(Some(current));
            halfedge_mesh.vertex_mut(current).output_index = output_index;
            output_index += 1;
            self.remeshed_vertices
                .push(halfedge_mesh.vertex(current).position);
        }
        let mut face = halfedge_mesh.move_to_next_face(None);
        while let Some(current) = face {
            face = halfedge_mesh.move_to_next_face(Some(current));
            let start = halfedge_mesh.face(current).halfedge;
            let previous = halfedge_mesh.halfedge(start).previous_halfedge;
            let next = halfedge_mesh.halfedge(start).next_halfedge;
            self.remeshed_triangles.push(vec![
                halfedge_mesh
                    .vertex(halfedge_mesh.halfedge(previous).start_vertex)
                    .output_index,
                halfedge_mesh
                    .vertex(halfedge_mesh.halfedge(start).start_vertex)
                    .output_index,
                halfedge_mesh
                    .vertex(halfedge_mesh.halfedge(next).start_vertex)
                    .output_index,
            ]);
        }

        true
    }

    /// Mirrors `debugExportObj` (`%f` formatting, 1-based face indices). The
    /// C++ null-derefs when the file cannot be opened; this port panics
    /// instead (debug helper, off the solve path).
    pub fn debug_export_obj(&self, filename: &str) {
        use std::fmt::Write as _;
        let mut text = String::new();
        for vertex in &self.remeshed_vertices {
            let _ = writeln!(text, "v {:.6} {:.6} {:.6}", vertex[0], vertex[1], vertex[2]);
        }
        for triangle in &self.remeshed_triangles {
            let _ = writeln!(
                text,
                "f {} {} {}",
                triangle[0] + 1,
                triangle[1] + 1,
                triangle[2] + 1
            );
        }
        std::fs::write(filename, text).expect("debug_export_obj: cannot open file");
    }
}
