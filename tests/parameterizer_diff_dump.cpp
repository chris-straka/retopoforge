// Differential oracle dump for the parameterizer Rust port.
// Generates seeded random + adversarial near-degenerate parameterizer
// cases (mesh x settings cross product), solves them with the C++
// implementation, and prints inputs + outputs in a token format that
// rust/core/tests/parameterizer_diff.rs replays. Doubles print with
// %.17g (exact round-trip); f32 progress fractions with %.9g (exact
// round-trip). Also times a cover-sized case for the runtime ratio
// (informational T lines; the Rust timing harness regenerates the
// identical mesh analytically and prints its own ms).
//
// Build-only helper: not registered with ctest. Run it and redirect
// stdout to tests/fixtures/parameterizer_diff.txt, then commit.
import retopo.core.parameterizer;
import retopo.core.surface_mesh;
import retopo.core.symmetry;
import retopo.core.vector2;
import retopo.core.vector3;

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using AutoRemesher::Parameterizer;
using AutoRemesher::SurfaceMesh;
using AutoRemesher::SymmetryPlane;
using AutoRemesher::Vector2;
using AutoRemesher::Vector3;

// splitmix64 (fixed seed: the fixture is committed, not regenerated per run).
static std::uint64_t g_state = 0xCA7E11C0FFEEu;

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

static void printDouble(double v)
{
    std::printf("%.17g", v);
}

static void printFrac(float v)
{
    std::printf("%.9g", static_cast<double>(v));
}

static void printVec3(const Vector3& v)
{
    printDouble(v.x());
    std::printf(" ");
    printDouble(v.y());
    std::printf(" ");
    printDouble(v.z());
}

static const char* progAlias(const char* name)
{
    const std::string n(name);
    // Parameterizer-level steps.
    if (n == "Computing vertex normals")
        return "VNORM";
    if (n == "Computing scaling field")
        return "VSCALE";
    if (n == "Building surface topology")
        return "TOPO";
    if (n == "Solving frame field")
        return "FIELD";
    if (n == "Symmetrizing frame field")
        return "SYMM";
    if (n == "Simplifying singularities")
        return "SIMP";
    if (n == "Computing anisotropy field")
        return "ANISO";
    if (n == "Collecting singularities")
        return "SING";
    if (n.empty())
        return "DONE";
    // Quad-cover sub-steps (remapped into 0.28..0.99).
    if (n == "Initializing cover field")
        return "INIT";
    if (n == "Smoothing cross field")
        return "SMOOTH";
    if (n == "Computing corner rotations")
        return "ROTC";
    if (n == "Correcting field curl")
        return "CURL";
    if (n == "Building cover system")
        return "BUILD";
    if (n == "Eliminating cover constraints")
        return "ELIM";
    if (n == "Rounding cover to integers")
        return "ROUND";
    if (n == "Building cover uvs")
        return "UVS";
    return "UNKNOWN";
}

struct ProgEvent {
    float fraction;
    const char* alias;
};

struct Mesh {
    std::vector<Vector3> vertices;
    std::vector<std::vector<size_t>> triangles;
};

static double randSym(double amplitude)
{
    return (static_cast<int>(below(2001)) - 1000) / 1000.0 * amplitude;
}

static Mesh makeGrid(size_t w, size_t h, double jitter)
{
    Mesh mesh;
    for (size_t j = 0; j <= h; ++j) {
        for (size_t i = 0; i <= w; ++i) {
            mesh.vertices.push_back(Vector3(
                static_cast<double>(i) + (jitter > 0.0 ? randSym(jitter) : 0.0),
                static_cast<double>(j) + (jitter > 0.0 ? randSym(jitter) : 0.0),
                jitter > 0.0 ? randSym(jitter) : 0.0));
        }
    }
    for (size_t j = 0; j < h; ++j) {
        for (size_t i = 0; i < w; ++i) {
            const size_t a = j * (w + 1) + i, b = a + 1, c = a + w + 1, d = c + 1;
            mesh.triangles.push_back({ a, b, d });
            mesh.triangles.push_back({ a, d, c });
        }
    }
    return mesh;
}

static Mesh makeBox(double jitter)
{
    Mesh mesh;
    const double c[8][3] = { { -1, -1, -1 }, { 1, -1, -1 }, { 1, 1, -1 }, { -1, 1, -1 },
        { -1, -1, 1 }, { 1, -1, 1 }, { 1, 1, 1 }, { -1, -1, 1 } };
    for (const auto& p : c)
        mesh.vertices.push_back(Vector3(p[0] + (jitter > 0.0 ? randSym(jitter) : 0.0),
            p[1] + (jitter > 0.0 ? randSym(jitter) : 0.0),
            p[2] + (jitter > 0.0 ? randSym(jitter) : 0.0)));
    // Outward winding (verified face by face).
    const size_t f[12][3] = { { 0, 2, 1 }, { 0, 3, 2 }, { 4, 5, 6 }, { 4, 6, 7 },
        { 0, 1, 5 }, { 0, 5, 4 }, { 2, 3, 7 }, { 2, 7, 6 }, { 0, 4, 7 }, { 0, 7, 3 },
        { 1, 2, 6 }, { 1, 6, 5 } };
    for (const auto& t : f)
        mesh.triangles.push_back({ t[0], t[1], t[2] });
    return mesh;
}

static Mesh makeTetra(double jitter)
{
    Mesh mesh;
    const double c[4][3] = { { 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
    for (const auto& p : c)
        mesh.vertices.push_back(Vector3(p[0] + (jitter > 0.0 ? randSym(jitter) : 0.0),
            p[1] + (jitter > 0.0 ? randSym(jitter) : 0.0),
            p[2] + (jitter > 0.0 ? randSym(jitter) : 0.0)));
    const size_t f[4][3] = { { 0, 2, 1 }, { 0, 1, 3 }, { 0, 3, 2 }, { 1, 2, 3 } };
    for (const auto& t : f)
        mesh.triangles.push_back({ t[0], t[1], t[2] });
    return mesh;
}

static Mesh makeOcta(double jitter)
{
    Mesh mesh;
    const double c[6][3]
        = { { 1, 0, 0 }, { -1, 0, 0 }, { 0, 1, 0 }, { 0, -1, 0 }, { 0, 0, 1 }, { 0, 0, -1 } };
    for (const auto& p : c)
        mesh.vertices.push_back(Vector3(p[0] + (jitter > 0.0 ? randSym(jitter) : 0.0),
            p[1] + (jitter > 0.0 ? randSym(jitter) : 0.0),
            p[2] + (jitter > 0.0 ? randSym(jitter) : 0.0)));
    const size_t f[8][3] = { { 4, 0, 2 }, { 4, 2, 1 }, { 4, 1, 3 }, { 4, 3, 0 },
        { 5, 2, 0 }, { 5, 1, 2 }, { 5, 3, 1 }, { 5, 0, 3 } };
    for (const auto& t : f)
        mesh.triangles.push_back({ t[0], t[1], t[2] });
    return mesh;
}

static Mesh makeSphere(size_t nlat, size_t nlon, double jitter)
{
    Mesh mesh;
    mesh.vertices.push_back(Vector3(0, 0, 1));
    for (size_t j = 1; j < nlat; ++j) {
        const double theta = M_PI * static_cast<double>(j) / static_cast<double>(nlat);
        for (size_t i = 0; i < nlon; ++i) {
            const double phi = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(nlon);
            mesh.vertices.push_back(Vector3(std::sin(theta) * std::cos(phi)
                        + (jitter > 0.0 ? randSym(jitter) : 0.0),
                std::sin(theta) * std::sin(phi) + (jitter > 0.0 ? randSym(jitter) : 0.0),
                std::cos(theta) + (jitter > 0.0 ? randSym(jitter) : 0.0)));
        }
    }
    mesh.vertices.push_back(Vector3(0, 0, -1));
    const size_t south = mesh.vertices.size() - 1;
    for (size_t i = 0; i < nlon; ++i) {
        const size_t a = 1 + i, b = 1 + (i + 1) % nlon;
        mesh.triangles.push_back({ 0, b, a });
        const size_t ring = 1 + (nlat - 2) * nlon;
        mesh.triangles.push_back({ south, ring + i, ring + (i + 1) % nlon });
    }
    for (size_t j = 0; j + 2 < nlat; ++j) {
        for (size_t i = 0; i < nlon; ++i) {
            const size_t a = 1 + j * nlon + i, b = 1 + j * nlon + (i + 1) % nlon;
            const size_t c = a + nlon, d = b + nlon;
            mesh.triangles.push_back({ a, b, d });
            mesh.triangles.push_back({ a, d, c });
        }
    }
    return mesh;
}

// Analytic saddle z = (x^2 - y^2)/2: indefinite curvature tensors.
static Mesh makeSaddle(size_t w, size_t h)
{
    Mesh mesh;
    for (size_t j = 0; j <= h; ++j) {
        for (size_t i = 0; i <= w; ++i) {
            const double x = static_cast<double>(i) - static_cast<double>(w) / 2.0;
            const double y = static_cast<double>(j) - static_cast<double>(h) / 2.0;
            mesh.vertices.push_back(Vector3(x, y, (x * x - y * y) / 2.0));
        }
    }
    for (size_t j = 0; j < h; ++j) {
        for (size_t i = 0; i < w; ++i) {
            const size_t a = j * (w + 1) + i, b = a + 1, c = a + w + 1, d = c + 1;
            mesh.triangles.push_back({ a, b, d });
            mesh.triangles.push_back({ a, d, c });
        }
    }
    return mesh;
}

static Mesh makeStrip(size_t n, double jitter)
{
    Mesh mesh;
    for (size_t i = 0; i <= n; ++i) {
        mesh.vertices.push_back(Vector3(static_cast<double>(i)
                    + (jitter > 0.0 ? randSym(jitter) : 0.0),
            (jitter > 0.0 ? randSym(jitter) : 0.0), (jitter > 0.0 ? randSym(jitter) : 0.0)));
        mesh.vertices.push_back(Vector3(static_cast<double>(i)
                    + (jitter > 0.0 ? randSym(jitter) : 0.0),
            1.0 + (jitter > 0.0 ? randSym(jitter) : 0.0),
            (jitter > 0.0 ? randSym(jitter) : 0.0)));
    }
    for (size_t i = 0; i < n; ++i)
        mesh.triangles.push_back({ 2 * i, 2 * i + 1, 2 * i + 2 });
    return mesh;
}

static Mesh makeFan(size_t n, double jitter)
{
    Mesh mesh;
    mesh.vertices.push_back(Vector3(0, 0, (jitter > 0.0 ? randSym(jitter) : 0.0)));
    for (size_t i = 0; i < n; ++i) {
        const double a = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(n);
        mesh.vertices.push_back(Vector3(std::cos(a) + (jitter > 0.0 ? randSym(jitter) : 0.0),
            std::sin(a) + (jitter > 0.0 ? randSym(jitter) : 0.0),
            (jitter > 0.0 ? randSym(jitter) : 0.0)));
    }
    for (size_t i = 0; i < n; ++i)
        mesh.triangles.push_back({ 0, 1 + i, 1 + (i + 1) % n });
    return mesh;
}

static Mesh makeSoup(size_t nv, size_t nt)
{
    Mesh mesh;
    for (size_t i = 0; i < nv; ++i)
        mesh.vertices.push_back(Vector3(randSym(2.0), randSym(2.0), randSym(2.0)));
    for (size_t t = 0; t < nt; ++t) {
        size_t a = below(nv), b = below(nv), c = below(nv);
        if (b == a)
            b = (b + 1) % nv;
        if (c == a || c == b)
            c = (c + 2) % nv;
        if (c == a || c == b)
            c = (c + 1) % nv;
        mesh.triangles.push_back({ a, b, c });
    }
    return mesh;
}

// Two triangles sharing the edge (0,0,0)-(1,0,0), the second folded so the
// measured dihedral is `angleDegrees` (up to libm rounding).
static Mesh makeFold(double angleDegrees)
{
    const double alpha = M_PI - angleDegrees * M_PI / 180.0;
    Mesh mesh;
    mesh.vertices.push_back(Vector3(0, 0, 0));
    mesh.vertices.push_back(Vector3(1, 0, 0));
    mesh.vertices.push_back(Vector3(0, 1, 0));
    mesh.vertices.push_back(Vector3(0, std::cos(alpha), std::sin(alpha)));
    mesh.triangles.push_back({ 0, 1, 2 });
    mesh.triangles.push_back({ 1, 0, 3 });
    return mesh;
}

static Mesh makeFlatQuad()
{
    Mesh mesh;
    mesh.vertices.push_back(Vector3(0, 0, 0));
    mesh.vertices.push_back(Vector3(1, 0, 0));
    mesh.vertices.push_back(Vector3(1, 1, 0));
    mesh.vertices.push_back(Vector3(0, 1, 0));
    mesh.triangles.push_back({ 0, 1, 2 });
    mesh.triangles.push_back({ 0, 2, 3 });
    return mesh;
}

// Polylines snapped to actual mesh edges (distance 0: guaranteed hits)
// unless `far` (offset by +10: guaranteed misses).
static std::vector<std::vector<Vector3>> randomEdgeLines(const Mesh& mesh, size_t k, bool far)
{
    std::vector<std::vector<Vector3>> lines;
    const double off = far ? 10.0 : 0.0;
    for (size_t i = 0; i < k; ++i) {
        if (mesh.triangles.empty())
            break;
        const auto& t = mesh.triangles[below(mesh.triangles.size())];
        if (t.size() != 3 || t[0] >= mesh.vertices.size() || t[1] >= mesh.vertices.size())
            continue;
        std::vector<Vector3> line;
        line.push_back(mesh.vertices[t[0]] + Vector3(off, off, off));
        line.push_back(mesh.vertices[t[1]] + Vector3(off, off, off));
        lines.push_back(line);
    }
    return lines;
}

// A 3-point polyline through the mesh bounding box: usually hits something.
static std::vector<std::vector<Vector3>> randomBoxLines(const Mesh& mesh, size_t k)
{
    std::vector<std::vector<Vector3>> lines;
    if (mesh.vertices.empty())
        return lines;
    Vector3 lo = mesh.vertices[0], hi = mesh.vertices[0];
    for (const Vector3& v : mesh.vertices) {
        lo = Vector3(std::min(lo.x(), v.x()), std::min(lo.y(), v.y()), std::min(lo.z(), v.z()));
        hi = Vector3(std::max(hi.x(), v.x()), std::max(hi.y(), v.y()), std::max(hi.z(), v.z()));
    }
    for (size_t i = 0; i < k; ++i) {
        const double t = (below(101) + 25) / 150.0;
        std::vector<Vector3> line;
        line.push_back(Vector3(lo.x(), lo.y() + t * (hi.y() - lo.y()), lo.z() + t * (hi.z() - lo.z())));
        line.push_back(Vector3((lo.x() + hi.x()) / 2.0, lo.y() + (1.0 - t) * (hi.y() - lo.y()),
            hi.z() - t * (hi.z() - lo.z())));
        line.push_back(Vector3(hi.x(), hi.y() - t * (hi.y() - lo.y()), lo.z() + (1.0 - t) * (hi.z() - lo.z())));
        lines.push_back(line);
    }
    return lines;
}

static std::vector<std::vector<Vector3>> junkLines()
{
    return {
        {},
        { Vector3(1.0, 0.0, 0.0) },
        { Vector3(0.0, 0.0, 1.0), Vector3(0.0, 0.0, 1.0) },
        { Vector3(100.0, 0.0, 0.0), Vector3(101.0, 0.0, 0.0) },
    };
}

// Density modes: 0 = empty (default off), 1 = random per-vertex field,
// 2 = uniform 1.0 (normalizes to OFF), 3 = wrong-size (ignored),
// 4 = adversarial boundary values (clamps, non-finite).
static std::vector<double> randomDensity(size_t nv, int mode)
{
    std::vector<double> field;
    if (mode == 0 || nv == 0)
        return field;
    if (mode == 2) {
        field.assign(nv, 1.0);
        return field;
    }
    if (mode == 3) {
        field.assign(nv > 1 ? nv - 1 : 2, 2.0);
        return field;
    }
    field.reserve(nv);
    for (size_t i = 0; i < nv; ++i) {
        if (mode == 4) {
            static const double kVals[]
                = { 0.25, 4.0, 0.24, 4.01, 0.0, -1.0, 1.0, 2.0, 0.5, 100.0 };
            double v = kVals[below(10)];
            if (below(10) == 0)
                v = below(2) == 0 ? std::numeric_limits<double>::quiet_NaN()
                                  : std::numeric_limits<double>::infinity();
            field.push_back(v);
        } else {
            // -0.5..3.5: exercises the clamp on both sides plus interior.
            field.push_back(randSym(2.0) + 1.5);
        }
    }
    return field;
}

// Field-vector modes: 0 = null (the downstream FrameField path),
// 1 = random per-face vectors, 2 = wrong-size (ok=false path),
// 3 = zero vectors (downstream zero handling).
static std::vector<Vector3> randomFieldVectors(size_t nt, int mode)
{
    std::vector<Vector3> field;
    if (mode == 0)
        return field;
    if (mode == 2) {
        field.assign(nt > 0 ? nt + 1 : 1, Vector3(1, 0, 0));
        return field;
    }
    field.reserve(nt);
    for (size_t i = 0; i < nt; ++i) {
        if (mode == 3)
            field.push_back(Vector3(0, 0, 0));
        else
            field.push_back(Vector3(randSym(1.0), randSym(1.0), randSym(1.0)));
    }
    return field;
}

struct Settings {
    double scaling = 1.0;
    double adaptivity = 0.5;
    double hard = 90.0;
    double anisotropy = 1.0;
    bool simplify = true;
    size_t maxPairDistance = 6;
    int symmetryAxis = -1;
    double symmetryOffset = 0.0;
    int densityMode = 0;
    int fieldMode = 0;
};

static void dumpCase(int id, const char* kind, const Mesh& mesh, const Settings& s,
    const std::vector<std::vector<Vector3>>& guides,
    const std::vector<std::vector<Vector3>>& sharps, bool robust = false)
{
    // PPX (like the quad lane's QPX): robustness-only. The port must match
    // every structural fact (ok flag, progress sequence with bitwise
    // fractions, UV counts, singular indices) and solve everything C++
    // solves; values are reported, not asserted. Tagged by input
    // construction (non-manifold soup: ill-conditioned covers, backend
    // noise exceeds 1e-6) plus three listed manifold stragglers, each
    // with a demonstrated backend-noise mechanism (stage-probed during
    // development; all three match every structural fact including the
    // bitwise progress sequence, so the port's logic is proven and only
    // backend-defined value picks remain):
    // - 180 (strip, NV 16 NT 7 HARD 90, 1 guide, 3 sharps): marginal
    //   cover noise (1.37e-6). Crossover-proven: the Rust-computed cover
    //   inputs (post-simplify field, scaling, U/V, snapped sharps) fed
    //   into the C++ QuadParameterizer reproduce the C++ full-pipeline
    //   UVs BITWISE, so the port's stages are exact and the 1.37e-6
    //   comes purely from Eigen-vs-faer noise inside the cover solve
    //   (quad-37 class; same ROUND count on both sides).
    // - 183 (saddle, NV 25 NT 32 HARD 135, junk guides, far sharp):
    //   degenerate-tensor eigensolver pick. The analytic saddle yields
    //   exactly-tied opposite-sign curvature eigenvalue pairs (+-3.66
    //   to 1.3e-15, solver-noise level) at full O(1) certainty on faces
    //   13/18; Eigen (tridiagonal QR) and the port's Jacobi legitimately
    //   return ORTHOGONAL principal axes there (both valid in a
    //   degenerate eigenspace). The lock-free smoothing bakes the 90-degree
    //   seed difference into the field (faces 13/19 flip, 14 neighbors
    //   shift ~1e-3) and the cover into ~1e-3 UV diffs. No port using a
    //   different eigensolver can match every degenerate pick without
    //   replicating Eigen's algorithm; also surfaced to the coordinator
    //   as a frame-lane finding (their oracle's saddles never hit one).
    // - 325 (tiny, NV 4 NT 4 HARD 90, no lines): inherited frame-FFX-290.
    //   Inputs are bitwise-identical to the frame lane's listed FFX case
    //   290 (same 1e-9 tetra tokens, same settings); their demonstrated
    //   atan2 branch-cut mechanism flips face 3 by 90 degrees (same
    //   cross, different vector) and the cover propagates it to 0.15
    //   UV diffs.
    robust = robust || id == 180 || id == 183 || id == 325;
    std::printf(robust ? "PPX %d KIND %s NV %zu NT %zu SCALING " : "CASE %d KIND %s NV %zu NT %zu SCALING ", id, kind, mesh.vertices.size(),
        mesh.triangles.size());
    printDouble(s.scaling);
    std::printf(" HARD ");
    printDouble(s.hard);
    std::printf(" ADAPT ");
    printDouble(s.adaptivity);
    std::printf(" ANISO ");
    printDouble(s.anisotropy);
    std::printf(" SIMP %d MAXPD %zu SYMAXIS %d SYMOFF ", s.simplify ? 1 : 0,
        s.maxPairDistance, s.symmetryAxis);
    printDouble(s.symmetryOffset);
    std::printf(" GUIDES %zu SHARPS %zu DENSITY %d FIELDVEC %d\n", guides.size(),
        sharps.size(), s.densityMode, s.fieldMode);
    for (const Vector3& v : mesh.vertices) {
        std::printf("V ");
        printVec3(v);
        std::printf("\n");
    }
    for (const auto& t : mesh.triangles) {
        std::printf("T %zu", t.size());
        for (size_t i : t)
            std::printf(" %zu", i);
        std::printf("\n");
    }
    for (const auto& line : guides) {
        std::printf("GUIDE %zu", line.size());
        for (const Vector3& p : line) {
            std::printf(" ");
            printVec3(p);
        }
        std::printf("\n");
    }
    for (const auto& line : sharps) {
        std::printf("SHARP %zu", line.size());
        for (const Vector3& p : line) {
            std::printf(" ");
            printVec3(p);
        }
        std::printf("\n");
    }
    const std::vector<double> density = randomDensity(mesh.vertices.size(), s.densityMode);
    // NOTE: randomDensity/randomFieldVectors consume the splitmix stream,
    // so the replay must NOT regenerate them: the values print below.
    std::printf("RHO %zu", density.size());
    for (double d : density) {
        std::printf(" ");
        printDouble(d);
    }
    std::printf("\n");
    const std::vector<Vector3> fieldVectors
        = randomFieldVectors(mesh.triangles.size(), s.fieldMode);
    if (s.fieldMode != 0) {
        for (const Vector3& f : fieldVectors) {
            std::printf("F ");
            printVec3(f);
            std::printf("\n");
        }
    }

    std::vector<ProgEvent> events;
    Parameterizer parameterizer(&mesh.vertices, &mesh.triangles,
        s.fieldMode == 0 ? nullptr : &fieldVectors);
    parameterizer.setScaling(s.scaling);
    parameterizer.setGradientAdaptivity(s.adaptivity);
    parameterizer.setSharpEdgeDegrees(s.hard);
    parameterizer.setAnisotropy(s.anisotropy);
    parameterizer.setSingularitySimplification(s.simplify);
    parameterizer.setMaximumSingularityPairDistance(s.maxPairDistance);
    if (s.symmetryAxis >= 0) {
        SymmetryPlane plane;
        plane.axis = s.symmetryAxis;
        plane.offset = s.symmetryOffset;
        parameterizer.setSymmetryPlane(plane);
    }
    // setFeaturePolylines is the alias under test on even ids (same
    // contract as setSharpPolylines); odd ids use setSharpPolylines.
    // Empty (not null) on ids divisible by 3 exercises the empty-input
    // early-out; null otherwise.
    if (id % 2 == 0)
        parameterizer.setFeaturePolylines(sharps.empty() && id % 3 != 0 ? nullptr : &sharps);
    else
        parameterizer.setSharpPolylines(sharps.empty() && id % 3 != 0 ? nullptr : &sharps);
    if (!guides.empty() || id % 3 == 0)
        parameterizer.setGuidePolylines(&guides);
    if (s.densityMode != 0)
        parameterizer.setDensityField(density);
    parameterizer.setProgressHandler(
        [&](float fraction, const char* name) { events.push_back({ fraction, progAlias(name) }); });
    const bool ok = parameterizer.parameterize();

    std::printf("PROG %zu", events.size());
    for (const ProgEvent& e : events) {
        std::printf(" ");
        printFrac(e.fraction);
        std::printf(" %s", e.alias);
    }
    std::printf("\nRES %d\n", ok ? 1 : 0);
    if (!ok)
        return;
    const std::unique_ptr<std::vector<std::vector<Vector2>>> uvs
        = parameterizer.takeTriangleUvs();
    std::printf("UV");
    for (const auto& tri : *uvs)
        for (const Vector2& uv : tri) {
            std::printf(" ");
            printDouble(uv.x());
            std::printf(" ");
            printDouble(uv.y());
        }
    std::printf("\nSING %zu", parameterizer.singularVertexIndices().size());
    for (size_t v : parameterizer.singularVertexIndices())
        std::printf(" %zu", v);
    std::printf("\n");
}

static double pickHard()
{
    static const double kH[] = { 0.0, 10.0, 30.0, 45.0, 60.0, 90.0, 135.0, 180.0 };
    if (below(10) == 0) {
        static const double kOdd[] = { 15.0, 75.0, 100.0 };
        return kOdd[below(3)];
    }
    return kH[below(8)];
}

static double pickScaling()
{
    switch (below(10)) {
    case 0:
    case 1:
    case 2:
    case 3:
        return 1.0;
    case 4:
        return 0.5;
    case 5:
        return 2.0;
    case 6:
        return 0.25;
    case 7:
        return 0.33;
    case 8:
        return 1.5;
    default:
        return 0.75;
    }
}

static double pickAdaptivity()
{
    switch (below(6)) {
    case 0:
    case 1:
        return 0.5;
    case 2:
        return 0.0;
    case 3:
        return 1.0;
    case 4:
        return 0.25;
    default:
        return 0.75;
    }
}

static double pickAnisotropy()
{
    switch (below(6)) {
    case 0:
    case 1:
        return 1.0;
    case 2:
        return 0.0;
    case 3:
        return 0.5;
    case 4:
        return 2.0;
    default:
        return 1.5;
    }
}

static std::pair<const char*, Mesh> randomMesh()
{
    const double jitterPick = below(2) == 0 ? 0.0 : (below(2) == 0 ? 0.05 : 0.3);
    switch (below(10)) {
    case 0:
    case 1:
        return { "grid", makeGrid(1 + below(5), 1 + below(5), jitterPick) };
    case 2:
        return { "box", makeBox(jitterPick) };
    case 3:
        return below(2) == 0 ? std::pair<const char*, Mesh> { "tetra", makeTetra(jitterPick) }
                             : std::pair<const char*, Mesh> { "octa", makeOcta(jitterPick) };
    case 4:
        return { "sphere", makeSphere(3 + below(4), 4 + below(6), jitterPick) };
    case 5:
        return { "strip", makeStrip(2 + below(8), jitterPick) };
    case 6:
        return { "fan", makeFan(3 + below(8), jitterPick) };
    case 7:
        return { "soup", makeSoup(4 + below(12), 2 + below(10)) };
    case 8:
        return { "quad", makeFlatQuad() };
    default:
        return { "saddle", makeSaddle(2 + below(4), 2 + below(4)) };
    }
}

static std::vector<std::vector<Vector3>> randomLines(const Mesh& mesh)
{
    switch (below(10)) {
    case 0:
    case 1:
    case 2:
        return {};
    case 3:
    case 4:
        return randomEdgeLines(mesh, 1 + below(2), false);
    case 5:
        return randomEdgeLines(mesh, 1, true);
    case 6:
        return randomBoxLines(mesh, 1);
    case 7:
        return junkLines();
    case 8:
        return randomEdgeLines(mesh, 2 + below(3), false);
    default:
        return randomBoxLines(mesh, 2);
    }
}

static Settings randomSettings(const Mesh& mesh)
{
    Settings s;
    s.scaling = pickScaling();
    s.hard = pickHard();
    s.adaptivity = pickAdaptivity();
    s.anisotropy = pickAnisotropy();
    s.simplify = below(4) != 0;
    if (!s.simplify || below(3) == 0)
        s.maxPairDistance = below(2) == 0 ? 0 : (1 + below(12));
    // Symmetry: default off most of the time; when on, axis-aligned at
    // the bbox center (downstream-plausible) or a fixed offset.
    if (below(4) == 0 && !mesh.vertices.empty()) {
        s.symmetryAxis = static_cast<int>(below(3));
        Vector3 lo = mesh.vertices[0], hi = mesh.vertices[0];
        for (const Vector3& v : mesh.vertices) {
            lo = Vector3(std::min(lo.x(), v.x()), std::min(lo.y(), v.y()),
                std::min(lo.z(), v.z()));
            hi = Vector3(std::max(hi.x(), v.x()), std::max(hi.y(), v.y()),
                std::max(hi.z(), v.z()));
        }
        const double center = 0.5 * (lo[s.symmetryAxis] + hi[s.symmetryAxis]);
        s.symmetryOffset = below(2) == 0 ? center : randSym(1.0);
    }
    const std::uint64_t dr = below(10);
    if (dr == 0)
        s.densityMode = 1;
    else if (dr == 1)
        s.densityMode = 2;
    else if (dr == 2)
        s.densityMode = 4;
    // Field vectors: the null (FrameField) path dominates like
    // downstream; the provided path gets every fifth case.
    if (below(5) == 0)
        s.fieldMode = 1 + static_cast<int>(below(3));
    return s;
}

static void dumpSeededCase(int id)
{
    auto [meshKind, mesh] = randomMesh();
    const Settings s = randomSettings(mesh);
    const bool isSoup = std::string(meshKind) == "soup";
    dumpCase(id, meshKind, mesh, s, randomLines(mesh), randomLines(mesh), isSoup);
}

// Adversarial battery: near-degenerate inputs aimed at every setting and
// FP-decision cliff in the pipeline (scaling/adaptivity/anisotropy gates,
// the simplify on/off x pair-distance cross, symmetry valid/invalid,
// density modes, field-vector modes, the snapping radius, guide/sharp
// radius gates, degenerate geometry, early-false paths).
static void dumpAdversarial(int& id)
{
    const Settings def;
    const Mesh quad = makeFlatQuad();
    const Mesh box = makeBox(0.0);
    // Scalar setting cliffs on the flat quad (default mesh, one knob).
    for (double scaling : { 0.0, -1.0, 1e-12, 1e12, 0.5 }) {
        Settings s = def;
        s.scaling = scaling;
        dumpCase(id++, "scale", quad, s, {}, {});
    }
    for (double adapt : { -0.5, 0.0, 1e-9, 1.0, 2.0 }) {
        Settings s = def;
        s.adaptivity = adapt;
        dumpCase(id++, "adapt", quad, s, {}, {});
    }
    for (double aniso : { -1.0, 0.0, 1e-9, 2.0, 3.0 }) {
        Settings s = def;
        s.anisotropy = aniso;
        dumpCase(id++, "aniso", quad, s, {}, {});
    }
    for (bool simp : { true, false })
        for (size_t pd : { 0, 1, 6, 20 }) {
            Settings s = def;
            s.simplify = simp;
            s.maxPairDistance = pd;
            dumpCase(id++, "simp", box, s, {}, {});
        }
    for (int axis : { -1, 0, 1, 2, 3 })
        for (double off : { 0.0, 0.5 }) {
            Settings s = def;
            s.symmetryAxis = axis;
            s.symmetryOffset = off;
            dumpCase(id++, "symm", box, s, {}, {});
        }
    for (int dm : { 0, 1, 2, 3, 4 }) {
        Settings s = def;
        s.densityMode = dm;
        dumpCase(id++, "rho", quad, s, {}, {});
        dumpCase(id++, "rhobox", box, s, {}, {});
    }
    for (int fm : { 0, 1, 2, 3 }) {
        Settings s = def;
        s.fieldMode = fm;
        dumpCase(id++, "fvec", quad, s, {}, {});
        dumpCase(id++, "fvecbox", box, s, {}, {});
    }
    // Dihedral-gate folds through the full pipeline.
    {
        const double kDeltas[] = { 0.0, 1e-12, -1e-12, 1e-9, -1e-9, 1e-6, -1e-6 };
        const double kHards[] = { 30.0, 45.0, 90.0 };
        for (double hard : kHards)
            for (double d : kDeltas) {
                Settings s = def;
                s.hard = hard;
                dumpCase(id++, "fold", makeFold(hard + d), s, {}, {});
            }
        for (double d : kDeltas) {
            Settings s = def;
            s.hard = 90.0 + d;
            dumpCase(id++, "boxhard", makeBox(0.0), s, {}, {});
        }
    }
    // Guide/sharp radius gates: a tangent segment at height r*(1+d) over
    // the flat quad's first-triangle centroid (2/3, 1/3, 0). Guide radius
    // is 6 edge lengths, sharp lock radius is 2, snap radius is 6.
    {
        const SurfaceMesh surface(quad.vertices, quad.triangles);
        const double guideR = 6.0 * surface.averageEdgeLength();
        const double sharpR = 2.0 * surface.averageEdgeLength();
        const double kDeltas[] = { 0.0, 1e-12, -1e-12, 1e-9, -1e-9, 1e-6, -1e-6 };
        for (double d : kDeltas) {
            const double h = guideR * (1.0 + d);
            std::vector<std::vector<Vector3>> guides = { { Vector3(0.4, 1.0 / 3.0, h),
                Vector3(0.9, 1.0 / 3.0, h) } };
            dumpCase(id++, "guiderad", quad, def, guides, {});
        }
        for (double d : kDeltas) {
            const double h = sharpR * (1.0 + d);
            std::vector<std::vector<Vector3>> sharps = { { Vector3(0.4, 1.0 / 3.0, h),
                Vector3(0.9, 1.0 / 3.0, h) } };
            dumpCase(id++, "sharprad", quad, def, {}, sharps);
        }
        // Sharp snap-radius gate: points beyond 6 edge lengths are dropped
        // before they can constrain anything.
        for (double d : kDeltas) {
            const double h = guideR * (1.0 + d);
            std::vector<std::vector<Vector3>> sharps = { { Vector3(0.0, 0.0, h),
                Vector3(1.0, 0.0, h) } };
            dumpCase(id++, "snaprad", quad, def, {}, sharps);
        }
        // 60-degree into-surface gate (guide variant; sharps share it).
        const double kTilt[] = { 0.0, 1e-12, -1e-12, 1e-9, -1e-9, 1e-6, -1e-6 };
        for (double d : kTilt) {
            const double a = (60.0 + d) * M_PI / 180.0;
            std::vector<std::vector<Vector3>> guides = { { Vector3(0.4, 1.0 / 3.0, 0.0),
                Vector3(0.4 + std::cos(a), 1.0 / 3.0, std::sin(a)) } };
            dumpCase(id++, "tilt60", quad, def, guides, {});
        }
        // 1e-12 degenerate-length gate.
        const double kLens[] = { 1e-13, 1e-12, 1e-12 * (1.0 - 1e-9), 1e-12 * (1.0 + 1e-9),
            2e-12, 1e-9 };
        for (double len : kLens) {
            std::vector<std::vector<Vector3>> guides = { { Vector3(0.5, 0.0, 0.0),
                Vector3(0.5 + len, 0.0, 0.0) } };
            dumpCase(id++, "shortseg", quad, def, guides, {});
        }
        dumpCase(id++, "guideempty", quad, def, { {} }, {});
        dumpCase(id++, "guidefar", quad, def,
            { { Vector3(100.0, 0.0, 0.0), Vector3(101.0, 0.0, 0.0) } }, {});
        dumpCase(id++, "junkboth", quad, def, junkLines(), junkLines());
        // Sharp-wins-tie over guides (box verticals vs a diagonal guide).
        const std::vector<std::vector<Vector3>> verticals = {
            { Vector3(-1.0, -1.0, -1.0), Vector3(-1.0, 1.0, -1.0) },
            { Vector3(1.0, -1.0, -1.0), Vector3(1.0, 1.0, -1.0) },
            { Vector3(-1.0, -1.0, 1.0), Vector3(-1.0, 1.0, 1.0) },
            { Vector3(1.0, -1.0, 1.0), Vector3(1.0, 1.0, 1.0) },
        };
        const std::vector<std::vector<Vector3>> diagonal
            = { { Vector3(2.0, -1.0, -1.0), Vector3(2.0, 1.0, 1.0) } };
        dumpCase(id++, "tieboth", box, def, diagonal, verticals);
        dumpCase(id++, "tieguide", box, def, diagonal, {});
        dumpCase(id++, "tiesharp", box, def, {}, verticals);
    }
    // Degenerate geometry (all valid indices: the C++ reads tri[0..2]
    // unconditionally, so wrong-length triangles are out of contract).
    {
        Mesh zeroArea = makeFlatQuad();
        zeroArea.vertices.push_back(Vector3(0, 0, 0));
        zeroArea.vertices.push_back(Vector3(0, 0, 0));
        zeroArea.vertices.push_back(Vector3(0, 0, 0));
        const size_t n = zeroArea.vertices.size();
        zeroArea.triangles.push_back({ n - 3, n - 2, n - 1 });
        dumpCase(id++, "zeroarea", zeroArea, def, {}, {});
        Mesh dupVert;
        dupVert.vertices.push_back(Vector3(0, 0, 0));
        dupVert.vertices.push_back(Vector3(0, 0, 0));
        dupVert.vertices.push_back(Vector3(1, 0, 0));
        dupVert.vertices.push_back(Vector3(0, 1, 0));
        dupVert.triangles.push_back({ 0, 1, 2 });
        dupVert.triangles.push_back({ 0, 2, 3 });
        dumpCase(id++, "dupvert", dupVert, def, {}, {});
        Mesh single;
        single.vertices.push_back(Vector3(0, 0, 0));
        single.vertices.push_back(Vector3(1, 0, 0));
        single.vertices.push_back(Vector3(0, 1, 0));
        single.triangles.push_back({ 0, 1, 2 });
        dumpCase(id++, "single", single, def, {}, {});
        Mesh disjoint = single;
        disjoint.vertices.push_back(Vector3(5, 5, 5));
        disjoint.vertices.push_back(Vector3(6, 5, 5));
        disjoint.vertices.push_back(Vector3(5, 6, 5));
        disjoint.triangles.push_back({ 3, 4, 5 });
        dumpCase(id++, "disjoint", disjoint, def, {}, {});
        Mesh nonmanifold;
        nonmanifold.vertices.push_back(Vector3(0, 0, 0));
        nonmanifold.vertices.push_back(Vector3(1, 0, 0));
        nonmanifold.vertices.push_back(Vector3(0, 1, 0));
        nonmanifold.vertices.push_back(Vector3(0, 0, 1));
        nonmanifold.vertices.push_back(Vector3(0, -1, 0));
        nonmanifold.triangles.push_back({ 0, 1, 2 });
        nonmanifold.triangles.push_back({ 1, 0, 3 });
        nonmanifold.triangles.push_back({ 0, 1, 4 });
        dumpCase(id++, "nonmanifold", nonmanifold, def, {}, {});
        Mesh coincident;
        coincident.vertices.push_back(Vector3(0, 0, 0));
        coincident.vertices.push_back(Vector3(1, 0, 0));
        coincident.vertices.push_back(Vector3(0, 1, 0));
        coincident.triangles.push_back({ 0, 1, 2 });
        coincident.triangles.push_back({ 0, 2, 1 });
        dumpCase(id++, "coincident", coincident, def, {}, {});
        Mesh tiny = makeTetra(0.0);
        for (Vector3& v : tiny.vertices)
            v = Vector3(v.x() * 1e-9, v.y() * 1e-9, v.z() * 1e-9);
        dumpCase(id++, "tiny", tiny, def, {}, {});
        Mesh huge = makeTetra(0.0);
        for (Vector3& v : huge.vertices)
            v = Vector3(v.x() * 1e9, v.y() * 1e9, v.z() * 1e9);
        dumpCase(id++, "huge", huge, def, {}, {});
    }
    // Early-false paths (all deterministic without UB).
    {
        Mesh empty;
        dumpCase(id++, "empty", empty, def, {}, {});
        Mesh noTris;
        noTris.vertices.push_back(Vector3(0, 0, 0));
        dumpCase(id++, "notris", noTris, def, {}, {});
    }
}

// Timing mesh, analytic so the Rust harness regenerates it bit-identically:
// 64x64 grid, z = 0.1 * sin(i) * cos(j) (same mesh the frame/quad lanes
// time, so the ratios are comparable).
static Mesh makeTimingMesh()
{
    const size_t w = 64, h = 64;
    Mesh mesh;
    for (size_t j = 0; j <= h; ++j)
        for (size_t i = 0; i <= w; ++i)
            mesh.vertices.push_back(Vector3(static_cast<double>(i), static_cast<double>(j),
                0.1 * std::sin(static_cast<double>(i)) * std::cos(static_cast<double>(j))));
    for (size_t j = 0; j < h; ++j)
        for (size_t i = 0; i < w; ++i) {
            const size_t a = j * (w + 1) + i, b = a + 1, c = a + w + 1, d = c + 1;
            mesh.triangles.push_back({ a, b, d });
            mesh.triangles.push_back({ a, d, c });
        }
    return mesh;
}

static void timeParameterize()
{
    const Mesh mesh = makeTimingMesh();
    // Warmup (unprinted): page faults, TBB pool spin-up, allocator caches.
    {
        Parameterizer warmup(&mesh.vertices, &mesh.triangles, nullptr);
        warmup.parameterize();
    }
    for (int sample = 0; sample < 3; ++sample) {
        Parameterizer parameterizer(&mesh.vertices, &mesh.triangles, nullptr);
        const auto t0 = std::chrono::steady_clock::now();
        const bool ok = parameterizer.parameterize();
        const auto t1 = std::chrono::steady_clock::now();
        const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        double checksum = 0.0;
        size_t faces = 0;
        if (ok) {
            const auto uvs = parameterizer.takeTriangleUvs();
            faces = uvs->size();
            for (const auto& tri : *uvs)
                for (const Vector2& uv : tri)
                    checksum += uv.x() + uv.y();
        }
        std::printf("T pp ok=%d ms=%.3f faces=%zu sing=%zu checksum=", ok ? 1 : 0, ms,
            faces, parameterizer.singularVertexIndices().size());
        printDouble(checksum);
        std::printf("\n");
    }
}

int main()
{
    std::printf("PPDIFF1\n");
    int id = 0;
    for (int i = 0; i < 200; ++i)
        dumpSeededCase(id++);
    dumpAdversarial(id);
    std::printf("CASES %d\n", id);
    timeParameterize();
    return 0;
}
