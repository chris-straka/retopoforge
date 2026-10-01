// Differential oracle dump for the surfacemesh Rust port (wave 2,
// rs-surfacemesh lane). Generates fixed edge-case meshes plus seeded random
// face soups (degenerate/empty/nonmanifold/out-of-range inputs included),
// queries every SurfaceMesh accessor with the C++ engine, and prints inputs
// + outputs in a token format that rust/core/tests/surface_mesh_diff.rs
// replays. Also times a large grid build for the runtime ratio (3 samples).
//
// Build-only helper: not registered with ctest. Run it and redirect stdout
// to tests/fixtures/surfacemesh_diff.txt, then commit the fixture.
import retopo.core.surface_mesh;
import retopo.core.vector3;

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <vector>

using AutoRemesher::SurfaceMesh;
using AutoRemesher::Vector3;

static constexpr size_t kNpos = SurfaceMesh::npos;

// splitmix64: the edge-case meshes below are fixed, the random cases draw
// from this stream with a fixed seed, so the fixture is reproducible.
static std::uint64_t g_state = 0x5EED5EED5EED5EEDull;

static std::uint64_t nextU64()
{
    std::uint64_t z = (g_state += 0x9E3779B97F4A7C15ull);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
}

static std::uint64_t below(std::uint64_t n)
{
    return nextU64() % n;
}

static void printDouble(double x)
{
    std::printf(" %.17g", x);
}

static void printIndex(size_t x)
{
    if (x == kNpos)
        std::printf(" N");
    else
        std::printf(" %zu", x);
}

// True when every kept triangle only references valid vertices, i.e. the
// float accessors (edgeVector/faceNormal/normalAngle/averageEdgeLength)
// are in-bounds. Out-of-range cases still dump full topology; the replay
// skips the float section exactly like this tool does.
static bool floatsSafe(const SurfaceMesh& mesh, size_t vertexCount)
{
    for (size_t f = 0; f < mesh.faceCount(); ++f) {
        const auto& tri = mesh.triangle(f);
        for (size_t k = 0; k < 3; ++k) {
            if (tri[k] >= vertexCount)
                return false;
        }
    }
    return true;
}

static void dumpCase(const std::vector<Vector3>& positions,
    const std::vector<std::vector<size_t>>& triangles)
{
    const SurfaceMesh mesh(positions, triangles);
    const size_t nv = positions.size();
    std::printf("CASE %zu %zu\n", nv, triangles.size());
    for (const auto& p : positions) {
        std::printf("v");
        printDouble(p.x());
        printDouble(p.y());
        printDouble(p.z());
        std::printf("\n");
    }
    for (const auto& t : triangles) {
        std::printf("t %zu", t.size());
        for (size_t v : t)
            std::printf(" %zu", v);
        std::printf("\n");
    }
    const bool fp = floatsSafe(mesh, nv);
    std::printf("R %zu %zu %zu %d\n", mesh.vertexCount(), mesh.faceCount(),
        mesh.cornerCount(), fp ? 1 : 0);
    std::printf("OPP %zu", mesh.cornerCount());
    for (size_t c = 0; c < mesh.cornerCount(); ++c)
        printIndex(mesh.oppositeCorner(c));
    std::printf("\n");
    std::printf("ADJ %zu", mesh.cornerCount());
    for (size_t c = 0; c < mesh.cornerCount(); ++c)
        printIndex(mesh.adjacentFace(c));
    std::printf("\n");
    std::printf("BND %zu", mesh.cornerCount());
    for (size_t c = 0; c < mesh.cornerCount(); ++c)
        std::printf(" %d", mesh.isBoundaryCorner(c) ? 1 : 0);
    std::printf("\n");
    std::printf("CAV %zu\n", mesh.vertexCount());
    for (size_t v = 0; v < mesh.vertexCount(); ++v) {
        const auto& cav = mesh.cornersAroundVertex(v);
        std::printf("c %zu", cav.size());
        for (size_t c : cav)
            std::printf(" %zu", c);
        std::printf("\n");
    }
    if (!fp)
        return;
    std::printf("AVG");
    printDouble(mesh.averageEdgeLength());
    std::printf("\n");
    std::printf("NANG %zu", mesh.cornerCount());
    for (size_t c = 0; c < mesh.cornerCount(); ++c)
        printDouble(mesh.normalAngle(c));
    std::printf("\n");
    std::printf("EDGE %zu", mesh.cornerCount());
    for (size_t c = 0; c < mesh.cornerCount(); ++c) {
        const Vector3 e = mesh.edgeVector(c);
        printDouble(e.x());
        printDouble(e.y());
        printDouble(e.z());
    }
    std::printf("\n");
    std::printf("FNORM %zu", mesh.faceCount());
    for (size_t f = 0; f < mesh.faceCount(); ++f) {
        const Vector3 n = mesh.faceNormal(f);
        printDouble(n.x());
        printDouble(n.y());
        printDouble(n.z());
    }
    std::printf("\n");
}

static void dumpEdgeCases()
{
    const Vector3 o(0, 0, 0);
    const Vector3 x(1, 0, 0);
    const Vector3 y(0, 1, 0);
    // 1. Empty mesh.
    dumpCase({}, {});
    // 2. Single triangle in the XY plane (golden).
    dumpCase({ o, x, y }, { { 0, 1, 2 } });
    // 3. Unit square as two triangles (golden).
    dumpCase({ o, x, Vector3(1, 1, 0), Vector3(0, 1, 0) },
        { { 0, 1, 2 }, { 0, 2, 3 } });
    // 4. Closed tetrahedron (golden).
    dumpCase({ o, x, y, Vector3(0, 0, 1) },
        { { 0, 2, 1 }, { 0, 1, 3 }, { 0, 3, 2 }, { 1, 2, 3 } });
    // 5. Non-triangle faces are skipped (golden).
    dumpCase({ o, x, y }, { { 0, 1 }, { 0, 1, 2, 0 }, { 0, 1, 2 } });
    // 6. Out-of-range vertex indices: topology only, no float section.
    dumpCase({ o, x, y }, { { 0, 1, 9 }, { 0, 1, 2 } });
    // 7. Duplicate faces, same orientation: no pairing, all boundary.
    dumpCase({ o, x, y }, { { 0, 1, 2 }, { 0, 1, 2 } });
    // 8. Flipped duplicate: every corner pairs across the two faces.
    dumpCase({ o, x, y }, { { 0, 1, 2 }, { 0, 2, 1 } });
    // 9. Nonmanifold star: three faces around edge (0,1).
    dumpCase({ o, x, y, Vector3(0, 0, 1), Vector3(1, 1, 1) },
        { { 0, 1, 2 }, { 1, 0, 3 }, { 0, 1, 4 } });
    // 10. Degenerate faces: repeated vertex, all-equal triple, normal tri.
    dumpCase({ o, x, y }, { { 0, 0, 1 }, { 2, 2, 2 }, { 0, 1, 2 } });
    // 11. Collinear positions: zero-area face, zero face normal.
    dumpCase({ o, x, Vector3(2, 0, 0) }, { { 0, 1, 2 } });
    // 12. Coincident positions: zero-length edge.
    dumpCase({ o, o, y }, { { 0, 1, 2 } });
    // 13. Empty raw face mixed with a normal triangle.
    dumpCase({ o, x, y }, { {}, { 0, 1, 2 } });
    // 14. Fan: five tris around center vertex 0.
    dumpCase({ o, x, y, Vector3(1, 1, 0), Vector3(0, 2, 0), Vector3(-1, 1, 0) },
        { { 0, 1, 2 }, { 0, 2, 3 }, { 0, 3, 4 }, { 0, 4, 5 }, { 0, 5, 1 } });
    // 15. Strip of consistently wound triangles, one island.
    dumpCase({ o, x, Vector3(2, 0, 0), Vector3(0, 1, 0), Vector3(1, 1, 0), Vector3(2, 1, 0) },
        { { 0, 1, 4 }, { 0, 4, 3 }, { 1, 2, 5 }, { 1, 5, 4 } });
    // 16. Bowtie: two tris sharing only vertex 2, no shared edge.
    dumpCase({ o, x, y, Vector3(3, 0, 0), Vector3(3, 1, 0) },
        { { 0, 1, 2 }, { 2, 3, 4 } });
    // 17. Two disjoint tris.
    dumpCase({ o, x, y, Vector3(5, 5, 5), Vector3(6, 5, 5), Vector3(5, 6, 5) },
        { { 0, 1, 2 }, { 3, 4, 5 } });
    // 18. Same-orientation neighbours: shared geometric edge, no reversed
    // directed edge -> the detach verification sweep must leave both
    // boundary.
    dumpCase({ o, x, y, Vector3(1, 1, 0) },
        { { 0, 1, 2 }, { 0, 1, 3 } });
    // 19. 3x3 grid patch (8 tris), interior valence-6 vertex.
    {
        std::vector<Vector3> pos;
        for (size_t yy = 0; yy < 3; ++yy)
            for (size_t xx = 0; xx < 3; ++xx)
                pos.emplace_back(double(xx), double(yy), 0.0);
        auto id = [](size_t xx, size_t yy) { return yy * 3 + xx; };
        std::vector<std::vector<size_t>> tris;
        for (size_t yy = 0; yy < 2; ++yy)
            for (size_t xx = 0; xx < 2; ++xx)
                tris.push_back({ id(xx, yy), id(xx + 1, yy), id(xx + 1, yy + 1) }),
                    tris.push_back({ id(xx, yy), id(xx + 1, yy + 1), id(xx, yy + 1) });
        dumpCase(pos, tris);
    }
    // 20. Slanted tetrahedron with non-axis coordinates (FP exercise).
    dumpCase({ Vector3(0.25, -1.5, 0.75), Vector3(2.125, 0.5, -0.375),
                 Vector3(-0.875, 1.25, 2.5), Vector3(1.75, -0.125, 1.0625) },
        { { 0, 2, 1 }, { 0, 1, 3 }, { 0, 3, 2 }, { 1, 2, 3 } });
    // 21. Non-planar quad: nonzero dihedral angle across the diagonal.
    dumpCase({ o, x, Vector3(1, 1, 0.5), Vector3(0, 1, 0) },
        { { 0, 1, 2 }, { 0, 2, 3 } });
    // 22. Faces but zero vertices: everything out of range.
    dumpCase({}, { { 0, 1, 2 } });
    // 23. Huge vertex index (2^32): topology only.
    dumpCase({ o, x, y },
        { { 0, 1, static_cast<size_t>(0x100000000ull) }, { 0, 1, 2 } });
    // 24. Vertices but no faces.
    dumpCase({ o, x, y }, {});
    // 25. Large magnitude (1e150) and tiny magnitude (1e-150) positions.
    dumpCase({ Vector3(1e150, 0, 0), Vector3(0, 1e150, 0), Vector3(0, 0, 1e150) },
        { { 0, 1, 2 } });
    dumpCase({ Vector3(1e-150, 0, 0), Vector3(0, 1e-150, 0), Vector3(0, 0, 1e-150) },
        { { 0, 1, 2 } });
    // 27. Quad soup with inconsistent winding across the shared edge.
    dumpCase({ o, x, Vector3(1, 1, 0), Vector3(0, 1, 0) },
        { { 0, 1, 2 }, { 3, 2, 0 } });
}

static Vector3 randomPosition()
{
    // Half the time snap to the 0..3 integer lattice so coincident
    // vertices, collinear triples, and zero-area faces occur often;
    // otherwise uniform-ish in [-10, 10).
    auto coord = []() {
        if (below(2) == 0)
            return double(below(4));
        return double(nextU64() % 2000) / 100.0 - 10.0;
    };
    return Vector3(coord(), coord(), coord());
}

static std::vector<size_t> randomRawFace(size_t vertexCount)
{
    // 15%: non-triangle arity (skipped by the constructor).
    size_t k = 3;
    if (below(100) < 15) {
        static const size_t kArity[] = { 0, 1, 2, 4, 5 };
        k = kArity[below(5)];
    }
    std::vector<size_t> face;
    for (size_t i = 0; i < k; ++i) {
        // 3%: out-of-range index (small overshoot or huge). Higher rates
        // push most multi-face cases into the topology-only bucket and
        // starve the float section of the replay.
        if (below(100) < 3) {
            if (below(2) == 0)
                face.push_back(vertexCount + below(3));
            else
                face.push_back(static_cast<size_t>(0x100000000ull) + below(8));
        } else if (vertexCount == 0) {
            face.push_back(below(4));
        } else {
            face.push_back(below(vertexCount));
        }
    }
    return face;
}

static void dumpRandomCase()
{
    size_t nv = below(11); // 0..10 vertices on a tiny pool: degenerate
    // positions, duplicate faces, and nonmanifold stars all occur.
    std::vector<Vector3> positions;
    for (size_t i = 0; i < nv; ++i)
        positions.push_back(randomPosition());
    size_t nt = below(13); // 0..12 raw faces.
    std::vector<std::vector<size_t>> triangles;
    for (size_t i = 0; i < nt; ++i) {
        // 10%: duplicate an earlier face verbatim.
        if (!triangles.empty() && below(10) == 0)
            triangles.push_back(triangles[below(triangles.size())]);
        else
            triangles.push_back(randomRawFace(nv));
    }
    dumpCase(positions, triangles);
}

// Large timing input: W(x)H triangulated grid with deterministic integer
// relief (exact on both sides, no RNG). The Rust timing test builds the
// identical mesh.
static void makeGrid(size_t w, size_t h, std::vector<Vector3>& positions,
    std::vector<std::vector<size_t>>& triangles)
{
    positions.reserve((w + 1) * (h + 1));
    for (size_t yy = 0; yy <= h; ++yy) {
        for (size_t xx = 0; xx <= w; ++xx) {
            double z = 0.1 * double((xx * 7 + yy * 13) % 5);
            positions.emplace_back(double(xx), double(yy), z);
        }
    }
    triangles.reserve(w * h * 2);
    auto id = [w](size_t xx, size_t yy) { return yy * (w + 1) + xx; };
    for (size_t yy = 0; yy < h; ++yy) {
        for (size_t xx = 0; xx < w; ++xx) {
            size_t a = id(xx, yy);
            size_t b = id(xx + 1, yy);
            size_t c = id(xx + 1, yy + 1);
            size_t d = id(xx, yy + 1);
            triangles.push_back({ a, b, c });
            triangles.push_back({ a, c, d });
        }
    }
}

static void timeGrid()
{
    std::vector<Vector3> positions;
    std::vector<std::vector<size_t>> triangles;
    makeGrid(200, 200, positions, triangles); // 80,000 triangles
    for (int sample = 0; sample < 3; ++sample) {
        auto t0 = std::chrono::steady_clock::now();
        const SurfaceMesh mesh(positions, triangles);
        double angleSum = 0.0;
        for (size_t c = 0; c < mesh.cornerCount(); ++c)
            angleSum += mesh.normalAngle(c);
        double avg = mesh.averageEdgeLength();
        auto t1 = std::chrono::steady_clock::now();
        double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        std::printf("T surfacemesh verts=%zu faces=%zu anglesum=%.17g avg=%.17g ms=%.3f\n",
            mesh.vertexCount(), mesh.faceCount(), angleSum, avg, ms);
    }
}

static constexpr int kRandomCases = 220;

int main()
{
    std::printf("SMESH1\n");
    dumpEdgeCases();
    for (int i = 0; i < kRandomCases; ++i)
        dumpRandomCase();
    timeGrid();
    return 0;
}
