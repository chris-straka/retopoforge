// Differential oracle dump for the isotropicremesher Rust port.
// Drives the C++ wrapper (AutoRemesher::IsotropicRemesher) and, for kernel-only
// options, the thirdparty kernel directly, over fixed edge-case meshes plus
// seeded random meshes, and prints inputs + outputs + progress records in a
// token format that rust/core/tests/isotropic_remesher_diff.rs replays.
//
// Doubles print as hex bit patterns (exact, no decimal round-trip), progress
// fractions as f32 hex bits. Also times a large grid remesh for the runtime
// ratio (3 samples).
//
// Build-only helper: not registered with ctest. Run it and redirect stdout
// to tests/fixtures/isoremesh_diff.txt, then commit the fixture.
#include <isotropichalfedgemesh.h>
#include <isotropicremesher.h>
import retopo.core.isotropic_remesher;
import retopo.core.vector3;

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>
#include <utility>
#include <vector>


static std::uint64_t g_state = 0x1501E9E50CE4uLL;

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

static void printBits(double v)
{
    std::uint64_t w;
    std::memcpy(&w, &v, sizeof(w));
    std::printf("%016llx", (unsigned long long)w);
}

static void printV3(const AutoRemesher::Vector3& v)
{
    printBits(v.x());
    std::printf(" ");
    printBits(v.y());
    std::printf(" ");
    printBits(v.z());
}

static void printV3tp(const ::Vector3& v)
{
    printBits(v.x());
    std::printf(" ");
    printBits(v.y());
    std::printf(" ");
    printBits(v.z());
}

// Random double across scales: small ints, quarters, wide range, tiny.
static double randDouble()
{
    switch (below(6)) {
    case 0:
        return static_cast<double>(static_cast<int>(below(21)) - 10);
    case 1:
        return (static_cast<int>(below(41)) - 20) / 4.0;
    case 2:
        return (static_cast<double>(nextU64() % 2000000) - 1000000.0) / 100.0;
    case 3:
        return (static_cast<double>(nextU64() % 2000000) - 1000000.0) * 1e-9;
    case 4:
        return std::ldexp((static_cast<double>(nextU64() % 2000000) - 1000000.0) / 1000000.0,
            static_cast<int>(below(40)) - 20);
    default:
        return (static_cast<int>(below(2001)) - 1000) / 1000.0;
    }
}

struct Params {
    double targetEdgeLength = 0; // 0 = unset (use mesh average)
    // Random cases store a multiplier of the mesh's own average edge length
    // instead (resolved in dumpCase): absolute small targets on huge random
    // soups would refine without bound.
    bool telIsMultiplier = false;
    double sharpEdgeDegrees = 60;
    double smoothNormalDegrees = 0;
    bool perVertexLengths = false;
    double perVertexScale[3] = { 0.5, 1.0, 2.0 };
    size_t targetTriangleCount = 0; // kernel-direct cases only
    size_t iterations = 3; // kernel-direct cases only
};

static Params randomParams(bool kernel)
{
    Params p;
    // Target edge length as a multiple of the mesh average edge length:
    // small multiples split heavily, large ones collapse heavily, 0 keeps
    // the mesh average.
    p.telIsMultiplier = true;
    switch (below(10)) {
    case 0:
    case 1:
    case 2:
    case 3:
        p.targetEdgeLength = 0;
        break;
    case 4:
        p.targetEdgeLength = 0.25;
        break;
    case 5:
        p.targetEdgeLength = 0.5;
        break;
    case 6:
        p.targetEdgeLength = 1.5;
        break;
    default:
        p.targetEdgeLength = 3.0;
        break;
    }
    static const double kSharp[] = { 0, 15, 60, 60, 120, 180 };
    p.sharpEdgeDegrees = kSharp[below(6)];
    switch (below(10)) {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
        p.smoothNormalDegrees = 0;
        break;
    case 7:
        p.smoothNormalDegrees = 15;
        break;
    case 8:
        p.smoothNormalDegrees = 30;
        break;
    default:
        p.smoothNormalDegrees = 60;
        break;
    }
    p.perVertexLengths = below(4) == 0;
    if (kernel) {
        static const size_t kTtc[] = { 0, 0, 4, 10, 50 };
        p.targetTriangleCount = kTtc[below(5)];
        static const size_t kIters[] = { 0, 1, 3, 3, 5 };
        p.iterations = kIters[below(5)];
    }
    return p;
}

struct ProgRec {
    std::uint32_t bits;
    std::string name;
};

static std::vector<ProgRec> runWrapper(const std::vector<AutoRemesher::Vector3>& vertices,
    const std::vector<std::vector<size_t>>& triangles,
    const Params& p,
    const std::vector<double>* lengths,
    bool* ok,
    std::vector<AutoRemesher::Vector3>* outVertices,
    std::vector<std::vector<size_t>>* outTriangles)
{
    std::vector<ProgRec> prog;
    AutoRemesher::IsotropicRemesher remesher(vertices, triangles);
    if (nullptr != lengths)
        remesher.setVertexTargetEdgeLengths(lengths);
    if (p.targetEdgeLength > 0)
        remesher.setTargetEdgeLength(p.targetEdgeLength);
    remesher.setSharpEdgeDegrees(p.sharpEdgeDegrees);
    remesher.setSmoothNormalDegrees(p.smoothNormalDegrees);
    remesher.setProgressHandler([&prog](float fraction, const char* name) {
        std::uint32_t w;
        std::memcpy(&w, &fraction, sizeof(w));
        prog.push_back({ w, name });
    });
    *ok = remesher.remesh();
    *outVertices = remesher.remeshedVertices();
    *outTriangles = remesher.remeshedTriangles();
    return prog;
}

static std::vector<ProgRec> runKernel(const std::vector<AutoRemesher::Vector3>& vertices,
    const std::vector<std::vector<size_t>>& triangles,
    const Params& p,
    const std::vector<double>* lengths,
    bool* ok,
    std::vector<::Vector3>* outVertices,
    std::vector<std::vector<size_t>>* outTriangles,
    double* avgLen)
{
    std::vector<ProgRec> prog;
    std::vector<::Vector3> inputVertices;
    inputVertices.reserve(vertices.size());
    for (const auto& position : vertices)
        inputVertices.push_back(::Vector3(position.x(), position.y(), position.z()));
    ::IsotropicRemesher remesher(&inputVertices, &triangles);
    *avgLen = remesher.initialAverageEdgeLength();
    if (nullptr != lengths)
        remesher.setVertexTargetEdgeLengths(lengths);
    if (p.targetEdgeLength > 0)
        remesher.setTargetEdgeLength(p.targetEdgeLength);
    if (p.targetTriangleCount > 0)
        remesher.setTargetTriangleCount(p.targetTriangleCount);
    remesher.setSharpEdgeIncludedAngle(180.0 - p.sharpEdgeDegrees);
    remesher.setSmoothNormalDegrees(p.smoothNormalDegrees);
    remesher.setProgressHandler([&prog](float fraction, const char* name) {
        std::uint32_t w;
        std::memcpy(&w, &fraction, sizeof(w));
        prog.push_back({ w, name });
    });
    remesher.remesh(p.iterations);
    IsotropicHalfedgeMesh* mesh = remesher.remeshedHalfedgeMesh();
    if (nullptr == mesh) {
        *ok = false;
        return prog;
    }
    *ok = true;
    size_t outputIndex = 0;
    for (IsotropicHalfedgeMesh::Vertex* vertex = mesh->moveToNextVertex(nullptr);
        nullptr != vertex;
        vertex = mesh->moveToNextVertex(vertex)) {
        vertex->outputIndex = outputIndex++;
        outVertices->push_back(vertex->position);
    }
    for (IsotropicHalfedgeMesh::Face* face = mesh->moveToNextFace(nullptr);
        nullptr != face;
        face = mesh->moveToNextFace(face)) {
        outTriangles->push_back(std::vector<size_t> {
            face->halfedge->previousHalfedge->startVertex->outputIndex,
            face->halfedge->startVertex->outputIndex,
            face->halfedge->nextHalfedge->startVertex->outputIndex });
    }
    return prog;
}

static void dumpProg(const std::vector<ProgRec>& prog)
{
    for (const auto& rec : prog)
        std::printf("P %08x %s\n", rec.bits, rec.name.c_str());
}

static double meanEdgeLength(const std::vector<AutoRemesher::Vector3>& vertices,
    const std::vector<std::vector<size_t>>& triangles)
{
    double total = 0;
    size_t count = 0;
    for (const auto& tri : triangles) {
        for (size_t i = 0; i < 3; ++i) {
            size_t j = (i + 1) % 3;
            total += (vertices[tri[i]] - vertices[tri[j]]).length();
            ++count;
        }
    }
    if (0 == count)
        return 0;
    return total / count;
}

static void dumpCase(int id,
    bool kernel,
    const std::vector<AutoRemesher::Vector3>& vertices,
    const std::vector<std::vector<size_t>>& triangles,
    const Params& p)
{
    Params q = p;
    double refLen = meanEdgeLength(vertices, triangles);
    bool usableRef = refLen > 0 && std::isfinite(refLen);
    if (q.telIsMultiplier) {
        q.telIsMultiplier = false;
        q.targetEdgeLength = (q.targetEdgeLength > 0 && usableRef)
            ? q.targetEdgeLength * refLen
            : 0;
    }
    // Per-vertex lengths scale with the mesh too (absolute scales are only
    // safe on unit meshes); a degenerate reference falls back to 1.0, which
    // cannot split zero-length edges.
    double vlenRef = usableRef ? refLen : 1.0;
    std::vector<double> lengthsStorage;
    const std::vector<double>* lengths = nullptr;
    if (q.perVertexLengths) {
        lengthsStorage.reserve(vertices.size());
        for (size_t i = 0; i < vertices.size(); ++i) {
            // Occasional 0.0 exercises the "use global default" branch.
            lengthsStorage.push_back(
                below(5) == 0 ? 0.0 : q.perVertexScale[below(3)] * vlenRef);
        }
        lengths = &lengthsStorage;
    }

    bool ok = false;
    std::vector<AutoRemesher::Vector3> outVerticesW;
    std::vector<::Vector3> outVerticesK;
    std::vector<std::vector<size_t>> outTriangles;
    std::vector<ProgRec> prog;
    double avgLen = 0;
    if (kernel) {
        prog = runKernel(vertices, triangles, q, lengths, &ok, &outVerticesK, &outTriangles, &avgLen);
    } else {
        prog = runWrapper(vertices, triangles, q, lengths, &ok, &outVerticesW, &outTriangles);
    }

    std::fprintf(stderr, "case %d mode=%c nv=%zu nt=%zu tel=%.4g sharp=%.0f smooth=%.0f vtel=%d ttc=%zu iters=%zu -> outv=%zu outt=%zu\n",
        id, kernel ? 'K' : 'W', vertices.size(), triangles.size(),
        q.targetEdgeLength, q.sharpEdgeDegrees, q.smoothNormalDegrees,
        q.perVertexLengths ? 1 : 0, q.targetTriangleCount, q.iterations,
        kernel ? outVerticesK.size() : outVerticesW.size(), outTriangles.size());
    std::printf("CASE %d MODE=%c NV=%zu NT=%zu TEL=%.17g SHARP=%.17g SMOOTH=%.17g VTEL=%d TTC=%zu ITERS=%zu\n",
        id, kernel ? 'K' : 'W', vertices.size(), triangles.size(),
        q.targetEdgeLength, q.sharpEdgeDegrees, q.smoothNormalDegrees,
        q.perVertexLengths ? 1 : 0, q.targetTriangleCount, q.iterations);
    for (const auto& v : vertices) {
        std::printf("V ");
        printV3(v);
        std::printf("\n");
    }
    for (const auto& t : triangles)
        std::printf("T %zu %zu %zu\n", t[0], t[1], t[2]);
    if (nullptr != lengths) {
        std::printf("VLEN");
        for (double l : *lengths) {
            std::printf(" ");
            printBits(l);
        }
        std::printf("\n");
    }
    std::printf("R ok=%d nv=%zu nt=%zu np=%zu",
        ok ? 1 : 0,
        kernel ? outVerticesK.size() : outVerticesW.size(),
        outTriangles.size(), prog.size());
    if (kernel) {
        std::printf(" avg=");
        printBits(avgLen);
    }
    std::printf("\n");
    if (kernel) {
        for (const auto& v : outVerticesK) {
            std::printf("OV ");
            printV3tp(v);
            std::printf("\n");
        }
    } else {
        for (const auto& v : outVerticesW) {
            std::printf("OV ");
            printV3(v);
            std::printf("\n");
        }
    }
    for (const auto& t : outTriangles)
        std::printf("OT %zu %zu %zu\n", t[0], t[1], t[2]);
    dumpProg(prog);
}

// --- Fixed edge-case meshes -----------------------------------------------

static std::vector<AutoRemesher::Vector3> tetraVerts()
{
    return { { 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
}

static std::vector<std::vector<size_t>> tetraTris()
{
    return { { 0, 2, 1 }, { 0, 1, 3 }, { 0, 3, 2 }, { 1, 2, 3 } };
}

static std::vector<AutoRemesher::Vector3> cubeVerts()
{
    return { { 0, 0, 0 }, { 1, 0, 0 }, { 1, 1, 0 }, { 0, 1, 0 },
        { 0, 0, 1 }, { 1, 0, 1 }, { 1, 1, 1 }, { 0, 1, 1 } };
}

static std::vector<std::vector<size_t>> cubeTris()
{
    // 12 triangles, sharp 90-degree edges (feature_edges path).
    return { { 0, 2, 1 }, { 0, 3, 2 }, { 4, 5, 6 }, { 4, 6, 7 }, { 0, 1, 5 },
        { 0, 5, 4 }, { 2, 3, 7 }, { 2, 7, 6 }, { 0, 4, 7 }, { 0, 7, 3 },
        { 1, 2, 6 }, { 1, 6, 5 } };
}

static std::vector<std::vector<size_t>> gridTris(size_t w, size_t h)
{
    std::vector<std::vector<size_t>> tris;
    auto id = [w](size_t x, size_t y) { return y * (w + 1) + x; };
    for (size_t y = 0; y < h; ++y) {
        for (size_t x = 0; x < w; ++x) {
            size_t a = id(x, y);
            size_t b = id(x + 1, y);
            size_t c = id(x + 1, y + 1);
            size_t d = id(x, y + 1);
            tris.push_back({ a, b, c });
            tris.push_back({ a, c, d });
        }
    }
    return tris;
}

static std::vector<AutoRemesher::Vector3> gridVerts(size_t w, size_t h)
{
    std::vector<AutoRemesher::Vector3> verts;
    for (size_t y = 0; y <= h; ++y)
        for (size_t x = 0; x <= w; ++x)
            verts.push_back({ static_cast<double>(x), static_cast<double>(y), 0 });
    return verts;
}

static void dumpFixedCases(int* id)
{
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    Params dflt;
    // Tetrahedron, default params.
    dumpCase((*id)++, false, tetraVerts(), tetraTris(), dflt);
    // Single triangle (all-boundary).
    dumpCase((*id)++, false, { { 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 } }, { { 0, 1, 2 } }, dflt);
    // Quad (two triangles, one interior edge).
    dumpCase((*id)++, false, { { 0, 0, 0 }, { 1, 0, 0 }, { 1, 1, 0 }, { 0, 1, 0 } },
        { { 0, 1, 2 }, { 0, 2, 3 } }, dflt);
    // Cube: sharp edges at several thresholds (feature_edges vs boundary).
    for (double sharp : { 0.0, 60.0, 120.0, 180.0 }) {
        Params p = dflt;
        p.sharpEdgeDegrees = sharp;
        dumpCase((*id)++, false, cubeVerts(), cubeTris(), p);
    }
    // Open 4x4 grid (boundary featuring).
    dumpCase((*id)++, false, gridVerts(4, 4), gridTris(4, 4), dflt);
    // Target edge length extremes on the tetra (split-heavy / collapse-heavy).
    for (double tel : { 0.05, 5.0 }) {
        Params p = dflt;
        p.targetEdgeLength = tel;
        dumpCase((*id)++, false, tetraVerts(), tetraTris(), p);
    }
    // Smooth-normal (PN) path on tetra and cube.
    for (double smooth : { 15.0, 60.0 }) {
        Params p = dflt;
        p.smoothNormalDegrees = smooth;
        dumpCase((*id)++, false, tetraVerts(), tetraTris(), p);
        dumpCase((*id)++, false, cubeVerts(), cubeTris(), p);
    }
    // Degenerate triangles: collinear, repeated corner, all-equal.
    dumpCase((*id)++, false, { { 0, 0, 0 }, { 1, 0, 0 }, { 2, 0, 0 } }, { { 0, 1, 2 } }, dflt);
    dumpCase((*id)++, false, tetraVerts(),
        { { 0, 2, 1 }, { 0, 1, 3 }, { 0, 3, 3 }, { 1, 2, 3 } }, dflt);
    dumpCase((*id)++, false, tetraVerts(),
        { { 0, 2, 1 }, { 1, 1, 1 }, { 0, 3, 2 }, { 1, 2, 3 } }, dflt);
    // Duplicate faces and non-manifold edge (3 tris sharing one edge).
    dumpCase((*id)++, false, tetraVerts(),
        { { 0, 2, 1 }, { 0, 2, 1 }, { 0, 1, 3 }, { 0, 3, 2 } }, dflt);
    dumpCase((*id)++, false, { { 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 }, { 1, 1, 1 } },
        { { 0, 1, 2 }, { 0, 1, 3 }, { 1, 0, 4 } }, dflt);
    // Repeated directed edge (halfedge-map collision path).
    dumpCase((*id)++, false, { { 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 }, { 1, 1, 0 } },
        { { 0, 1, 2 }, { 0, 1, 3 } }, dflt);
    // Two disjoint tetras (multi-component).
    {
        auto v = tetraVerts();
        auto t = tetraTris();
        for (const auto& p : tetraVerts())
            v.push_back({ p.x() + 5, p.y(), p.z() });
        for (const auto& tri : tetraTris())
            t.push_back({ tri[0] + 4, tri[1] + 4, tri[2] + 4 });
        dumpCase((*id)++, false, v, t, dflt);
    }
    // Empty mesh, vertices without triangles, triangle plus unused vertices.
    dumpCase((*id)++, false, {}, {}, dflt);
    dumpCase((*id)++, false, { { 0, 0, 0 }, { 1, 1, 1 } }, {}, dflt);
    dumpCase((*id)++, false, { { 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 }, { 9, 9, 9 } },
        { { 0, 1, 2 } }, dflt);
    // Scale extremes: tiny and huge triangles.
    dumpCase((*id)++, false, { { 0, 0, 0 }, { 1e-9, 0, 0 }, { 0, 1e-9, 0 } },
        { { 0, 1, 2 } }, dflt);
    dumpCase((*id)++, false, { { 0, 0, 0 }, { 1e9, 0, 0 }, { 0, 1e9, 0 } },
        { { 0, 1, 2 } }, dflt);
    // Needle triangle (near-degenerate area vs edge length).
    dumpCase((*id)++, false, { { 0, 0, 0 }, { 1, 0, 0 }, { 0.5, 1e-12, 0 } },
        { { 0, 1, 2 } }, dflt);
    // Non-finite vertices (robustness: must not crash).
    dumpCase((*id)++, false, { { nan, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 } },
        { { 0, 1, 2 } }, dflt);
    dumpCase((*id)++, false, { { 0, 0, 0 }, { inf, 0, 0 }, { 0, 1, 0 } },
        { { 0, 1, 2 } }, dflt);
    dumpCase((*id)++, false, tetraVerts(),
        { { 0, 2, 1 }, { 0, 1, 3 }, { 0, 3, 2 }, { 1, 2, 3 } },
        [] { Params p; p.smoothNormalDegrees = 30; return p; }());
    // Per-vertex target lengths on tetra and cube.
    dumpCase((*id)++, false, tetraVerts(), tetraTris(),
        [] { Params p; p.perVertexLengths = true; return p; }());
    dumpCase((*id)++, false, cubeVerts(), cubeTris(),
        [] { Params p; p.perVertexLengths = true; p.sharpEdgeDegrees = 15; return p; }());

    // Kernel-direct fixed cases: triangle-count target, iteration counts.
    for (size_t ttc : { size_t(4), size_t(50) }) {
        Params p = dflt;
        p.targetTriangleCount = ttc;
        dumpCase((*id)++, true, tetraVerts(), tetraTris(), p);
    }
    for (size_t iters : { size_t(0), size_t(1), size_t(5) }) {
        Params p = dflt;
        p.iterations = iters;
        dumpCase((*id)++, true, cubeVerts(), cubeTris(), p);
    }
    {
        Params p = dflt;
        p.targetTriangleCount = 20;
        p.smoothNormalDegrees = 30;
        dumpCase((*id)++, true, cubeVerts(), cubeTris(), p);
    }
}

// --- Random meshes --------------------------------------------------------

static std::vector<AutoRemesher::Vector3> randomPool(size_t n)
{
    std::vector<AutoRemesher::Vector3> pool;
    for (size_t i = 0; i < n; ++i)
        pool.push_back({ randDouble(), randDouble(), randDouble() });
    return pool;
}

// Triangle soup on a small vertex pool: collisions, degenerate windings,
// duplicate faces and non-manifold stars all occur.
static void randomSoup(std::vector<AutoRemesher::Vector3>* vertices,
    std::vector<std::vector<size_t>>* triangles)
{
    size_t pool = 1 + below(8);
    *vertices = randomPool(pool);
    size_t ntris = below(13);
    for (size_t i = 0; i < ntris; ++i) {
        if (!triangles->empty() && below(10) == 0) {
            triangles->push_back((*triangles)[below(triangles->size())]);
            continue;
        }
        std::vector<size_t> tri = { below(pool), below(pool), below(pool) };
        // 15%: collapse a corner onto another (degenerate).
        if (below(100) < 15)
            tri[below(3)] = tri[0];
        triangles->push_back(tri);
    }
}

// Perturbed grid patch (open mesh with boundary), unit-ish scale.
static void randomGrid(std::vector<AutoRemesher::Vector3>* vertices,
    std::vector<std::vector<size_t>>* triangles)
{
    size_t w = 1 + below(5);
    size_t h = 1 + below(5);
    *vertices = gridVerts(w, h);
    if (below(2) == 0) {
        for (auto& v : *vertices) {
            v = { v.x() + randDouble() * 0.2, v.y() + randDouble() * 0.2,
                randDouble() * 0.3 };
        }
    }
    *triangles = gridTris(w, h);
}

// Perturbed tetrahedron / octahedron (closed mesh), unit-ish scale.
static void randomBlob(std::vector<AutoRemesher::Vector3>* vertices,
    std::vector<std::vector<size_t>>* triangles)
{
    if (below(2) == 0) {
        *vertices = tetraVerts();
        *triangles = tetraTris();
    } else {
        *vertices = { { 1, 0, 0 }, { -1, 0, 0 }, { 0, 1, 0 },
            { 0, -1, 0 }, { 0, 0, 1 }, { 0, 0, -1 } };
        *triangles = { { 4, 0, 2 }, { 4, 2, 1 }, { 4, 1, 3 }, { 4, 3, 0 },
            { 5, 2, 0 }, { 5, 1, 2 }, { 5, 3, 1 }, { 5, 0, 3 } };
    }
    double jitter = below(2) == 0 ? 0.1 : 0.4;
    for (auto& v : *vertices) {
        v = { v.x() + randDouble() * jitter, v.y() + randDouble() * jitter,
            v.z() + randDouble() * jitter };
    }
}

static void dumpRandomCase(int id, bool kernel)
{
    Params p = randomParams(kernel);
    std::vector<AutoRemesher::Vector3> vertices;
    std::vector<std::vector<size_t>> triangles;
    // A triangle-count target derives its edge length from the total area,
    // which mixed-scale soups can push orders of magnitude below their
    // longest edges (unbounded splitting), so TTC cases use grids/blobs.
    std::uint64_t gen = below(kernel ? 2 : 3);
    if (kernel && p.targetTriangleCount > 0 && gen == 0)
        gen = 1;
    switch (gen) {
    case 0:
        randomSoup(&vertices, &triangles);
        break;
    case 1:
        randomGrid(&vertices, &triangles);
        break;
    default:
        randomBlob(&vertices, &triangles);
        break;
    }
    dumpCase(id, kernel, vertices, triangles, p);
}

// Large timing input: W(x)H grid with a pure-formula z bump (no RNG: the
// Rust timing test builds the identical mesh).
static std::vector<AutoRemesher::Vector3> timingVerts(size_t w, size_t h)
{
    std::vector<AutoRemesher::Vector3> verts;
    for (size_t y = 0; y <= h; ++y) {
        for (size_t x = 0; x <= w; ++x) {
            double z = static_cast<double>((x * y) % 7) * 0.05;
            verts.push_back({ static_cast<double>(x), static_cast<double>(y), z });
        }
    }
    return verts;
}

static void timeGrid()
{
    auto vertices = timingVerts(80, 80);
    auto triangles = gridTris(80, 80);
    for (int sample = 0; sample < 3; ++sample) {
        AutoRemesher::IsotropicRemesher remesher(vertices, triangles);
        auto t0 = std::chrono::steady_clock::now();
        bool ok = remesher.remesh();
        auto t1 = std::chrono::steady_clock::now();
        double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        std::printf("T isoremesh nv=%zu nt=%zu outv=%zu outt=%zu ok=%d ms=%.3f\n",
            vertices.size(), triangles.size(), remesher.remeshedVertices().size(),
            remesher.remeshedTriangles().size(), ok ? 1 : 0, ms);
    }
}

static constexpr int kRandomWrapperCases = 180;
static constexpr int kRandomKernelCases = 30;

int main()
{
    std::printf("ISOREMESH1\n");
    int id = 0;
    dumpFixedCases(&id);
    for (int i = 0; i < kRandomWrapperCases; ++i)
        dumpRandomCase(id++, false);
    for (int i = 0; i < kRandomKernelCases; ++i)
        dumpRandomCase(id++, true);
    timeGrid();
    return 0;
}
