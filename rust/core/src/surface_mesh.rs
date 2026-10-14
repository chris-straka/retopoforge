//! Port of `core/surfacemesh.*` (`retopo.core.surface_mesh`).
//!
//! Line-by-line mirror of `AutoRemesher::SurfaceMesh`: same fields in the
//! same order, same constructor passes (triangle filter, corner fan
//! threading, opposite pairing, detach verification), same corner-index
//! arithmetic including the `(corner + 1) % 3` / `(corner + 2) % 3`
//! formulations.
//!
//! All floating point goes through the ported [`crate::vector3`] methods
//! (`Sub`, [`Vector3::normal`], [`Vector3::dot_product`],
//! [`Vector3::cross_product`], [`Vector3::length`]), which already
//! transcribe Clang's FMA fusion with explicit `mul_add`, plus `acos`
//! (proven bitwise by the vector oracle's `angle` cases) and a sequential
//! accumulate-then-divide in [`SurfaceMesh::average_edge_length`]. There
//! is no multiply-add site of its own here, so no FMA audit beyond reusing
//! those methods.
//!
//! [`Vector3::normal`]: crate::vector3::Vector3::normal
//! [`Vector3::dot_product`]: crate::vector3::Vector3::dot_product
//! [`Vector3::cross_product`]: crate::vector3::Vector3::cross_product
//! [`Vector3::length`]: crate::vector3::Vector3::length

use crate::vector3::Vector3;

/// Half-edge triangle mesh container (mirrors `AutoRemesher::SurfaceMesh`).
#[derive(Clone, Debug, Default)]
pub struct SurfaceMesh {
    positions: Vec<Vector3>,
    triangles: Vec<[usize; 3]>,
    opposite_corners: Vec<usize>,
    corners_around_vertex: Vec<Vec<usize>>,
}

impl SurfaceMesh {
    /// Missing-corner sentinel (mirrors `SurfaceMesh::npos`).
    pub const NPOS: usize = usize::MAX;

    /// Builds the mesh, skipping raw faces that are not triangles.
    ///
    /// Mirrors the C++ constructor exactly: the filter pass, the
    /// `nextAroundVertex`/`vertexCorner` fan threading, the opposite-corner
    /// pairing walk, and the `detach` verification sweep.
    #[must_use]
    pub fn new(positions: &[Vector3], triangles: &[Vec<usize>]) -> Self {
        let mut mesh = Self {
            positions: positions.to_vec(),
            triangles: Vec::with_capacity(triangles.len()),
            opposite_corners: vec![Self::NPOS; 3 * triangles.len()],
            corners_around_vertex: vec![Vec::new(); positions.len()],
        };
        for triangle in triangles {
            if triangle.len() != 3 {
                continue;
            }
            mesh.triangles.push([triangle[0], triangle[1], triangle[2]]);
        }
        mesh.opposite_corners
            .assign(3 * mesh.triangles.len(), Self::NPOS);
        let mut next_around_vertex = vec![Self::NPOS; mesh.corner_count()];
        let mut vertex_corner = vec![Self::NPOS; mesh.vertex_count()];
        for f in 0..mesh.face_count() {
            for local in 0..3 {
                let c = 3 * f + local;
                let vertex = mesh.corner_vertex(c);
                if vertex >= mesh.vertex_count() {
                    continue;
                }
                mesh.corners_around_vertex[vertex].push(c);
                next_around_vertex[c] = vertex_corner[vertex];
                vertex_corner[vertex] = c;
            }
        }
        for f1 in 0..mesh.face_count() {
            for local in 0..3 {
                let c1 = 3 * f1 + local;
                if mesh.opposite_corners[c1] != Self::NPOS {
                    continue;
                }
                let v2 = mesh.corner_vertex(mesh.next_corner(c1));
                let mut c2 = next_around_vertex[c1];
                while c2 != Self::NPOS {
                    if c2 != c1 {
                        let c3 = mesh.previous_corner(c2);
                        if mesh.corner_vertex(c3) == v2 && mesh.opposite_corners[c3] == Self::NPOS {
                            mesh.opposite_corners[c1] = c3;
                            mesh.opposite_corners[c3] = c1;
                            break;
                        }
                    }
                    c2 = next_around_vertex[c2];
                }
            }
        }

        for c in 0..mesh.corner_count() {
            let f2 = mesh.adjacent_face(c);
            if f2 == Self::NPOS {
                continue;
            }
            let f1 = mesh.corner_face(c);
            let mut c2 = Self::NPOS;
            for local in 0..3 {
                let candidate = 3 * f2 + local;
                if mesh.adjacent_face(candidate) == f1 {
                    c2 = candidate;
                    break;
                }
            }
            if c2 == Self::NPOS {
                mesh.detach(c);
                continue;
            }
            if mesh.corner_vertex(c) != mesh.corner_vertex(mesh.next_corner(c2)) {
                mesh.detach(c);
                mesh.detach(c2);
            }
        }
        mesh
    }

    /// Clears the pairing on both sides (mirrors the `detach` lambda).
    fn detach(&mut self, c: usize) {
        if c == Self::NPOS || c >= self.opposite_corners.len() {
            return;
        }
        let mate = self.opposite_corners[c];
        self.opposite_corners[c] = Self::NPOS;
        if mate != Self::NPOS
            && mate < self.opposite_corners.len()
            && self.opposite_corners[mate] == c
        {
            self.opposite_corners[mate] = Self::NPOS;
        }
    }

    #[inline]
    #[must_use]
    pub fn vertex_count(&self) -> usize {
        self.positions.len()
    }

    #[inline]
    #[must_use]
    pub fn face_count(&self) -> usize {
        self.triangles.len()
    }

    #[inline]
    #[must_use]
    pub fn corner_count(&self) -> usize {
        3 * self.triangles.len()
    }

    #[inline]
    #[must_use]
    pub fn corner_face(&self, corner: usize) -> usize {
        corner / 3
    }

    #[inline]
    #[must_use]
    pub fn corner_local(&self, corner: usize) -> usize {
        corner % 3
    }

    #[inline]
    #[must_use]
    pub fn corner_vertex(&self, corner: usize) -> usize {
        self.triangles[corner / 3][corner % 3]
    }

    #[inline]
    #[must_use]
    pub fn next_corner(&self, corner: usize) -> usize {
        3 * (corner / 3) + (corner + 1) % 3
    }

    #[inline]
    #[must_use]
    pub fn previous_corner(&self, corner: usize) -> usize {
        3 * (corner / 3) + (corner + 2) % 3
    }

    #[inline]
    #[must_use]
    pub fn opposite_corner(&self, corner: usize) -> usize {
        self.opposite_corners[corner]
    }

    #[inline]
    #[must_use]
    pub fn adjacent_face(&self, corner: usize) -> usize {
        let opposite = self.opposite_corner(corner);
        if opposite == Self::NPOS {
            Self::NPOS
        } else {
            self.corner_face(opposite)
        }
    }

    #[inline]
    #[must_use]
    pub fn is_boundary_corner(&self, corner: usize) -> bool {
        self.opposite_corner(corner) == Self::NPOS
    }

    #[inline]
    #[must_use]
    pub fn position(&self, vertex: usize) -> &Vector3 {
        &self.positions[vertex]
    }

    #[inline]
    #[must_use]
    pub fn triangle(&self, face: usize) -> &[usize; 3] {
        &self.triangles[face]
    }

    #[inline]
    #[must_use]
    pub fn corners_around_vertex(&self, vertex: usize) -> &Vec<usize> {
        &self.corners_around_vertex[vertex]
    }

    #[inline]
    #[must_use]
    pub fn edge_vector(&self, corner: usize) -> Vector3 {
        *self.position(self.corner_vertex(self.next_corner(corner)))
            - *self.position(self.corner_vertex(corner))
    }

    #[inline]
    #[must_use]
    pub fn face_normal(&self, face: usize) -> Vector3 {
        let face_vertices = self.triangle(face);
        Vector3::normal(
            self.position(face_vertices[0]),
            self.position(face_vertices[1]),
            self.position(face_vertices[2]),
        )
    }

    #[must_use]
    pub fn normal_angle(&self, corner: usize) -> f64 {
        let neighbour = self.adjacent_face(corner);
        if neighbour == Self::NPOS {
            return std::f64::consts::PI;
        }
        let first = self.face_normal(self.corner_face(corner));
        let second = self.face_normal(neighbour);
        let cosine = Vector3::dot_product(&first, &second);
        // Mirrors `std::max(-1.0, std::min(1.0, cosine))` exactly, including
        // the NaN case (NaN -> 1.0); `clamp` would keep NaN instead.
        let cosine = (-1.0_f64).max(cosine.min(1.0));
        let sign = if Vector3::dot_product(
            &Vector3::cross_product(&first, &second),
            &self.edge_vector(corner),
        ) > 0.0
        {
            -1.0
        } else {
            1.0
        };
        sign * cosine.acos()
    }

    #[must_use]
    pub fn average_edge_length(&self) -> f64 {
        let mut total = 0.0;
        let mut count = 0usize;
        for c in 0..self.corner_count() {
            let opposite = self.opposite_corner(c);
            if opposite != Self::NPOS && opposite < c {
                continue;
            }
            total += self.edge_vector(c).length();
            count += 1;
        }
        if count == 0 {
            0.0
        } else {
            total / count as f64
        }
    }
}

/// `std::vector::assign` has no std equivalent; the constructor needs the
/// fill-replace after the filter pass shrinks the triangle list.
trait Assign<T: Clone> {
    fn assign(&mut self, n: usize, value: T);
}

impl<T: Clone> Assign<T> for Vec<T> {
    fn assign(&mut self, n: usize, value: T) {
        self.clear();
        self.resize(n, value);
    }
}
