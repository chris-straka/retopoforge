// Unit tests for retopo.core.surface_mesh (core/surfacemesh.cpp).
// Pure half-edge topology + arithmetic: genuinely std-only logic.
// Plain assert-style main, no third-party framework.
// NOTE: CHECK instead of <cassert> assert(), which is a no-op under NDEBUG.
import retopo.core.double_utils;
import retopo.core.surface_mesh;
import retopo.core.vector3;

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <vector>

static int g_failures = 0;

#define CHECK(cond)                                                                                  \
    do {                                                                                             \
        if (!(cond)) {                                                                               \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                               \
            ++g_failures;                                                                            \
        }                                                                                            \
    } while (0)

int main()
{
    using AutoRemesher::Double::isEqual;
    using AutoRemesher::SurfaceMesh;
    using AutoRemesher::Vector3;
    constexpr size_t npos = SurfaceMesh::npos;

    // Single triangle in the XY plane: every corner is a boundary corner.
    {
        const std::vector<Vector3> positions { Vector3(0, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0) };
        const std::vector<std::vector<size_t>> triangles { { 0, 1, 2 } };
        const SurfaceMesh mesh(positions, triangles);
        CHECK(mesh.vertexCount() == 3);
        CHECK(mesh.faceCount() == 1);
        CHECK(mesh.cornerCount() == 3);
        CHECK(mesh.cornerFace(2) == 0);
        CHECK(mesh.cornerLocal(2) == 2);
        CHECK(mesh.cornerVertex(1) == 1);
        CHECK(mesh.nextCorner(2) == 0);
        CHECK(mesh.previousCorner(0) == 2);
        const std::array<size_t, 3> expectedTri { 0, 1, 2 };
        CHECK(mesh.triangle(0) == expectedTri);
        for (size_t c = 0; c < 3; ++c) {
            CHECK(mesh.oppositeCorner(c) == npos);
            CHECK(mesh.adjacentFace(c) == npos);
            CHECK(mesh.isBoundaryCorner(c));
            CHECK(mesh.normalAngle(c) == M_PI);
        }
        CHECK(mesh.cornersAroundVertex(0) == std::vector<size_t> { 0 });
        CHECK(mesh.cornersAroundVertex(1) == std::vector<size_t> { 1 });
        CHECK(mesh.cornersAroundVertex(2) == std::vector<size_t> { 2 });
        // Corner 0 runs v0 -> v1, i.e. the +X unit edge.
        const Vector3 edge = mesh.edgeVector(0);
        CHECK(edge.x() == 1.0);
        CHECK(edge.y() == 0.0);
        CHECK(edge.z() == 0.0);
        // Counter-clockwise in XY: +Z normal.
        const Vector3 normal = mesh.faceNormal(0);
        CHECK(normal.x() == 0.0);
        CHECK(normal.y() == 0.0);
        CHECK(normal.z() == 1.0);
        // Edges 1, sqrt(2), 1.
        CHECK(isEqual(mesh.averageEdgeLength(), (2.0 + std::sqrt(2.0)) / 3.0));
    }

    // Unit square as two triangles: one shared (diagonal) edge, the rest
    // boundary. Face 0 = {0,1,2}, face 1 = {0,2,3}; corners 0..5.
    {
        const std::vector<Vector3> positions {
            Vector3(0, 0, 0), Vector3(1, 0, 0), Vector3(1, 1, 0), Vector3(0, 1, 0)
        };
        const std::vector<std::vector<size_t>> triangles { { 0, 1, 2 }, { 0, 2, 3 } };
        const SurfaceMesh mesh(positions, triangles);
        CHECK(mesh.vertexCount() == 4);
        CHECK(mesh.faceCount() == 2);
        CHECK(mesh.cornerCount() == 6);
        // Corner 2 (face 0 edge v2 -> v0) pairs with corner 3
        // (face 1 edge v0 -> v2); nothing else pairs.
        CHECK(mesh.oppositeCorner(2) == 3);
        CHECK(mesh.oppositeCorner(3) == 2);
        CHECK(mesh.adjacentFace(2) == 1);
        CHECK(mesh.adjacentFace(3) == 0);
        CHECK(!mesh.isBoundaryCorner(2));
        CHECK(!mesh.isBoundaryCorner(3));
        for (const size_t c : { 0, 1, 4, 5 }) {
            CHECK(mesh.oppositeCorner(c) == npos);
            CHECK(mesh.adjacentFace(c) == npos);
            CHECK(mesh.isBoundaryCorner(c));
        }
        CHECK(mesh.cornersAroundVertex(0) == (std::vector<size_t> { 0, 3 }));
        CHECK(mesh.cornersAroundVertex(1) == (std::vector<size_t> { 1 }));
        CHECK(mesh.cornersAroundVertex(2) == (std::vector<size_t> { 2, 4 }));
        CHECK(mesh.cornersAroundVertex(3) == (std::vector<size_t> { 5 }));
        // Coplanar neighbours: zero dihedral angle across the diagonal.
        CHECK(mesh.normalAngle(2) == 0.0);
        CHECK(mesh.normalAngle(3) == 0.0);
        // Four unit boundary edges + one sqrt(2) diagonal.
        CHECK(isEqual(mesh.averageEdgeLength(), (4.0 + std::sqrt(2.0)) / 5.0));
    }

    // Closed tetrahedron (outward orientation): no boundary anywhere.
    {
        const std::vector<Vector3> positions {
            Vector3(0, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0), Vector3(0, 0, 1)
        };
        const std::vector<std::vector<size_t>> triangles {
            { 0, 2, 1 }, { 0, 1, 3 }, { 0, 3, 2 }, { 1, 2, 3 }
        };
        const SurfaceMesh mesh(positions, triangles);
        CHECK(mesh.vertexCount() == 4);
        CHECK(mesh.faceCount() == 4);
        CHECK(mesh.cornerCount() == 12);
        for (size_t c = 0; c < 12; ++c) {
            CHECK(mesh.oppositeCorner(c) != npos);
            CHECK(mesh.adjacentFace(c) != npos);
            CHECK(!mesh.isBoundaryCorner(c));
            // Pairing is symmetric.
            CHECK(mesh.oppositeCorner(mesh.oppositeCorner(c)) == c);
        }
        for (size_t v = 0; v < 4; ++v)
            CHECK(mesh.cornersAroundVertex(v).size() == 3);
        // Corner 0 is face 0's edge v0 -> v2; its reverse is corner 8
        // (face 2's edge v2 -> v0).
        CHECK(mesh.oppositeCorner(0) == 8);
        // Bottom face (z = 0, outward): -Z normal.
        const Vector3 bottom = mesh.faceNormal(0);
        CHECK(bottom.x() == 0.0);
        CHECK(bottom.y() == 0.0);
        CHECK(bottom.z() == -1.0);
        // Slanted face BCD: (1,1,1) normalized.
        const Vector3 slanted = mesh.faceNormal(3);
        const double unit = 1.0 / std::sqrt(3.0);
        CHECK(isEqual(slanted.x(), unit));
        CHECK(isEqual(slanted.y(), unit));
        CHECK(isEqual(slanted.z(), unit));
        // Three unit edges + three sqrt(2) edges.
        CHECK(isEqual(mesh.averageEdgeLength(), (3.0 + 3.0 * std::sqrt(2.0)) / 6.0));
    }

    // Non-triangle faces are skipped; empty input is an empty mesh.
    {
        const std::vector<Vector3> positions { Vector3(0, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0) };
        const std::vector<std::vector<size_t>> triangles { { 0, 1 }, { 0, 1, 2, 0 }, { 0, 1, 2 } };
        const SurfaceMesh mesh(positions, triangles);
        CHECK(mesh.faceCount() == 1);
        CHECK(mesh.cornerCount() == 3);
        const std::array<size_t, 3> expectedTri { 0, 1, 2 };
        CHECK(mesh.triangle(0) == expectedTri);
    }
    {
        const SurfaceMesh mesh({}, {});
        CHECK(mesh.vertexCount() == 0);
        CHECK(mesh.faceCount() == 0);
        CHECK(mesh.cornerCount() == 0);
        CHECK(mesh.averageEdgeLength() == 0.0);
    }

    if (g_failures == 0)
        std::printf("PASS test_surfacemesh\n");
    return g_failures == 0 ? 0 : 1;
}
