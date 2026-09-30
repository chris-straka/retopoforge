// Unit + engine tests for mirror-symmetry constraints (core/symmetry.*).
// Plain assert-style main, no third-party framework.
// NOTE: CHECK instead of <cassert> assert(), which is a no-op under NDEBUG.
import retopo.core.auto_remesher;
import retopo.core.symmetry;
import retopo.core.vector3;

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <map>
#include <tuple>
#include <vector>

static int g_failures = 0;

#define CHECK(cond)                                                                                  \
    do {                                                                                             \
        if (!(cond)) {                                                                               \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                               \
            ++g_failures;                                                                            \
        }                                                                                            \
    } while (0)

namespace {

using Remesher = AutoRemesher::AutoRemesher;
using Symmetry = AutoRemesher::Symmetry;
using SymmetryPlane = AutoRemesher::SymmetryPlane;
using Vector3 = AutoRemesher::Vector3;

// Subdivided cube centered at the origin, welded across face edges. With
// `bumpy`, vertices are displaced radially by f(x, z) = 0.1*sin(2x+0.5) *
// sin(2z+0.3), which keeps exact Y symmetry while breaking X and Z symmetry.
void buildCube(int subdivisions, bool bumpy, std::vector<Vector3>& vertices,
    std::vector<std::vector<size_t>>& triangles)
{
    vertices.clear();
    triangles.clear();
    std::map<std::tuple<long long, long long, long long>, size_t> welded;
    const auto vertexIndex = [&](const Vector3& point) {
        const auto key = std::make_tuple(static_cast<long long>(std::llround(point.x() * 1e9)),
            static_cast<long long>(std::llround(point.y() * 1e9)),
            static_cast<long long>(std::llround(point.z() * 1e9)));
        const auto inserted = welded.insert({ key, vertices.size() });
        if (inserted.second)
            vertices.push_back(point);
        return inserted.first->second;
    };
    // (fixedAxis, fixedSign, uAxis, vAxis) per face; (u, v) ordered so that
    // every face winds outward (single connected island).
    const int faces[6][4] = {
        { 0, 1, 1, 2 }, { 0, -1, 2, 1 }, { 1, 1, 2, 0 },
        { 1, -1, 0, 2 }, { 2, 1, 0, 1 }, { 2, -1, 1, 0 },
    };
    for (const auto& face : faces) {
        std::vector<std::vector<size_t>> grid(subdivisions + 1,
            std::vector<size_t>(subdivisions + 1));
        for (int i = 0; i <= subdivisions; ++i) {
            for (int j = 0; j <= subdivisions; ++j) {
                Vector3 point;
                point[static_cast<size_t>(face[0])] = static_cast<double>(face[1]);
                point[static_cast<size_t>(face[2])] = -1.0 + 2.0 * i / subdivisions;
                point[static_cast<size_t>(face[3])] = -1.0 + 2.0 * j / subdivisions;
                if (bumpy) {
                    const double bump = 0.1 * std::sin(2.0 * point.x() + 0.5)
                        * std::sin(2.0 * point.z() + 0.3);
                    point = point + point.normalized() * bump;
                }
                grid[static_cast<size_t>(i)][static_cast<size_t>(j)] = vertexIndex(point);
            }
        }
        for (int i = 0; i < subdivisions; ++i) {
            for (int j = 0; j < subdivisions; ++j) {
                const size_t a = grid[static_cast<size_t>(i)][static_cast<size_t>(j)];
                const size_t b = grid[static_cast<size_t>(i + 1)][static_cast<size_t>(j)];
                const size_t c = grid[static_cast<size_t>(i + 1)][static_cast<size_t>(j + 1)];
                const size_t d = grid[static_cast<size_t>(i)][static_cast<size_t>(j + 1)];
                triangles.push_back({ a, b, c });
                triangles.push_back({ a, c, d });
            }
        }
    }
}

double boundingDiagonal(const std::vector<Vector3>& points)
{
    Vector3 lower = points.front(), upper = points.front();
    for (const auto& point : points) {
        for (size_t i = 0; i < 3; ++i) {
            lower[i] = std::min(lower[i], point[i]);
            upper[i] = std::max(upper[i], point[i]);
        }
    }
    return (upper - lower).length();
}

// Largest distance from any vertex's mirror to its nearest vertex.
double maxMirrorDeviation(const std::vector<Vector3>& points, const SymmetryPlane& plane)
{
    double worst = 0.0;
    for (const auto& point : points) {
        const Vector3 mirrored = Symmetry::mirrorPoint(point, plane);
        double best = -1.0;
        for (const auto& other : points) {
            const double distance = (other - mirrored).length();
            if (best < 0.0 || distance < best)
                best = distance;
        }
        worst = std::max(worst, best);
    }
    return worst;
}

}

int main()
{
    // mirrorPoint / mirrorDirection basics.
    {
        const SymmetryPlane plane { 1, 2.0, 1.0 };
        CHECK(plane.valid());
        const Vector3 mirrored = Symmetry::mirrorPoint(Vector3(1.0, 5.0, 3.0), plane);
        CHECK(mirrored.x() == 1.0 && mirrored.y() == -1.0 && mirrored.z() == 3.0);
        const Vector3 direction = Symmetry::mirrorDirection(Vector3(1.0, 1.0, 0.0), plane);
        CHECK(direction.x() == 1.0 && direction.y() == -1.0 && direction.z() == 0.0);
        CHECK(!SymmetryPlane().valid());
        const Vector3 identity = Symmetry::mirrorPoint(Vector3(1.0, 2.0, 3.0), SymmetryPlane());
        CHECK((identity - Vector3(1.0, 2.0, 3.0)).length() == 0.0);
    }

    // Detection on a plain cube: symmetric about every axis, score 1.0.
    std::vector<Vector3> cubeVertices;
    std::vector<std::vector<size_t>> cubeTriangles;
    buildCube(4, false, cubeVertices, cubeTriangles);
    CHECK(!cubeVertices.empty() && !cubeTriangles.empty());
    {
        const SymmetryPlane detected = Symmetry::detectPlane(cubeVertices);
        CHECK(detected.valid());
        CHECK(detected.score > 0.999);
        CHECK(std::fabs(detected.offset) < 1e-12);
        // Ties prefer Y, then Z, then X.
        CHECK(detected.axis == 1);
        const SymmetryPlane fixed = Symmetry::fixedPlane(cubeVertices, 2);
        CHECK(fixed.axis == 2 && fixed.score > 0.999);
        CHECK(!Symmetry::fixedPlane(cubeVertices, 7).valid());
        CHECK(!Symmetry::detectPlane({}).valid());
    }

    // Detection on the bumpy cube: Y only.
    std::vector<Vector3> bumpyVertices;
    std::vector<std::vector<size_t>> bumpyTriangles;
    buildCube(4, true, bumpyVertices, bumpyTriangles);
    {
        const SymmetryPlane detected = Symmetry::detectPlane(bumpyVertices);
        CHECK(detected.axis == 1);
        CHECK(detected.score > 0.999);
        const double diagonal = boundingDiagonal(bumpyVertices);
        const double tolerance = std::max(0.01 * diagonal, 1e-9);
        CHECK(Symmetry::scorePlane(bumpyVertices, 0, 0.0, tolerance) < 0.75);
        CHECK(Symmetry::scorePlane(bumpyVertices, 2, 0.0, tolerance) < 0.75);
    }

    // A sheared half breaks every plane: best score stays below threshold.
    // (Shifting y for x > 0 moves Y-mirror pairs apart and off the
    // bounding-box center, so no axis-aligned plane matches.)
    {
        std::vector<Vector3> sheared = bumpyVertices;
        for (auto& vertex : sheared) {
            if (vertex.x() > 0.0)
                vertex[1] += 0.3;
        }
        const SymmetryPlane detected = Symmetry::detectPlane(sheared);
        CHECK(detected.score < 0.75);
    }

    // symmetrizeVertices: exact pairing after a one-sided perturbation.
    {
        std::vector<Vector3> perturbed = bumpyVertices;
        for (auto& vertex : perturbed) {
            if (vertex.y() > 0.0)
                vertex[1] += 0.01;
        }
        const SymmetryPlane plane { 1, 0.0, 1.0 };
        CHECK(maxMirrorDeviation(perturbed, plane) > 1e-6);
        Symmetry::symmetrizeVertices(perturbed, plane);
        CHECK(perturbed.size() == bumpyVertices.size());
        CHECK(maxMirrorDeviation(perturbed, plane) < 1e-9);
        // An invalid plane is a no-op.
        const std::vector<Vector3> before = perturbed;
        Symmetry::symmetrizeVertices(perturbed, SymmetryPlane());
        CHECK(perturbed.size() == before.size()
            && maxMirrorDeviation(perturbed, plane) < 1e-9);
    }

    // symmetrizeFrameField: a mirrored triangle pair averages under the
    // cross's 4-way symmetry (here (1,0,0) vs (0,0,1) coincide at (1,0,0)).
    {
        const std::vector<Vector3> vertices = {
            Vector3(0.0, 1.0, 0.0), Vector3(1.0, 1.0, 0.0), Vector3(0.0, 1.0, 1.0),
            Vector3(0.0, -1.0, 0.0), Vector3(1.0, -1.0, 0.0), Vector3(0.0, -1.0, 1.0),
        };
        const std::vector<std::vector<size_t>> triangles = { { 0, 1, 2 }, { 3, 4, 5 } };
        std::vector<Vector3> field = { Vector3(1.0, 0.0, 0.0), Vector3(0.0, 0.0, 1.0) };
        Symmetry::symmetrizeFrameField(vertices, triangles, field, SymmetryPlane { 1, 0.0, 1.0 });
        CHECK((field[0] - Vector3(1.0, 0.0, 0.0)).length() < 1e-9);
        CHECK((field[1] - Symmetry::mirrorDirection(field[0], SymmetryPlane { 1, 0.0, 1.0 })).length() < 1e-9);
        // No-op on size mismatch or an invalid plane.
        std::vector<Vector3> untouched = { Vector3(1.0, 0.0, 0.0) };
        Symmetry::symmetrizeFrameField(vertices, triangles, untouched, SymmetryPlane { 1, 0.0, 1.0 });
        CHECK(untouched.size() == 1 && untouched[0].x() == 1.0);
    }

    // End to end, auto plane: symmetric input yields symmetric output verts.
    {
        Remesher remesher(bumpyVertices, bumpyTriangles);
        remesher.setTargetTriangleCount(400);
        remesher.setSymmetryEnabled(true);
        CHECK(remesher.remesh());
        CHECK(remesher.symmetryPlaneAxis() == 1);
        CHECK(remesher.symmetryPlaneScore() > 0.95);
        const auto& outVertices = remesher.remeshedVertices();
        const auto& outQuads = remesher.remeshedQuads();
        CHECK(!outVertices.empty() && !outQuads.empty());
        const SymmetryPlane used { remesher.symmetryPlaneAxis(), remesher.symmetryPlaneOffset(), 1.0 };
        const double diagonal = boundingDiagonal(outVertices);
        const double deviation = maxMirrorDeviation(outVertices, used);
        std::printf("symmetric run: %zu verts %zu quads, plane Y @ %.6f, max mirror deviation %.3e (diagonal %.3f)\n",
            outVertices.size(), outQuads.size(), used.offset, deviation, diagonal);
        CHECK(deviation < 1e-6 * diagonal + 1e-9);
        for (const auto& quad : outQuads) {
            for (const size_t index : quad)
                CHECK(index < outVertices.size());
        }
    }

    // End to end, explicit plane the input lacks: graceful fallback to
    // unconstrained output (axis -1) instead of a mangled mesh.
    {
        Remesher remesher(bumpyVertices, bumpyTriangles);
        remesher.setTargetTriangleCount(400);
        remesher.setSymmetryEnabled(true);
        remesher.setSymmetryPlane(2);
        CHECK(remesher.remesh());
        CHECK(remesher.symmetryPlaneAxis() == -1);
        CHECK(!remesher.remeshedVertices().empty());
    }

    // Default off: no symmetry calls, plane stays unset.
    {
        Remesher remesher(bumpyVertices, bumpyTriangles);
        remesher.setTargetTriangleCount(400);
        CHECK(remesher.remesh());
        CHECK(remesher.symmetryPlaneAxis() == -1);
        CHECK(!remesher.remeshedVertices().empty());
    }

    if (g_failures == 0)
        std::printf("PASS test_symmetry\n");
    return g_failures == 0 ? 0 : 1;
}
