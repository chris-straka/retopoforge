// Differential oracle dump for the quadparameterizer Rust port.
// Generates seeded random + adversarial near-degenerate parameterize cases,
// solves them with the C++ implementation, and prints inputs + outputs in a
// token format that rust/core/tests/quad_parameterizer_diff.rs replays.
// Doubles print with %.17g (exact round-trip); f32 progress fractions with
// %.9g (exact round-trip). Also times a cover-sized case for the runtime
// ratio (informational T lines; the Rust timing harness regenerates the
// identical mesh analytically and prints its own ms).
//
// Build-only helper: not registered with ctest. Run it and redirect stdout
// to tests/fixtures/quadparam_diff.txt, then commit the fixture.
import retopo.core.progress;
import retopo.core.quad_parameterizer;
import retopo.core.surface_mesh;
import retopo.core.vector2;
import retopo.core.vector3;

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

using AutoRemesher::ProgressHandler;
using AutoRemesher::QuadParameterizer;
using AutoRemesher::SurfaceMesh;
using AutoRemesher::Vector2;
using AutoRemesher::Vector3;

// splitmix64 (fixed seed: the fixture is committed, not regenerated per run).
static std::uint64_t g_state = 0x51ADE17CE9u;

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
        { -1, -1, 1 }, { 1, -1, 1 }, { 1, 1, 1 }, { -1, 1, 1 } };
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

static Mesh makeSoup(size_t nv, size_t nt, bool allowDegenerate)
{
    Mesh mesh;
    for (size_t i = 0; i < nv; ++i)
        mesh.vertices.push_back(Vector3(randSym(2.0), randSym(2.0), randSym(2.0)));
    for (size_t t = 0; t < nt; ++t) {
        size_t a = below(nv), b = below(nv), c = below(nv);
        if (!allowDegenerate) {
            if (b == a)
                b = (b + 1) % nv;
            if (c == a || c == b)
                c = (c + 2) % nv;
            if (c == a || c == b)
                c = (c + 1) % nv;
        }
        mesh.triangles.push_back({ a, b, c });
    }
    return mesh;
}

// Two triangles sharing the edge (0,0,0)-(1,0,0), the second folded so the
// measured dihedral is `angleDegrees` (up to libm rounding): tri A lies in
// z=0 with normal +z, tri B's normal is (0, sin, -cos), so
// dot(nA, nB) = -cos(alpha) and the measured angle is PI - alpha.
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

// Flat boundary quad with per-face guidance at `angleDegrees` to the +x
// boundary edge: targets the 10-degree alignment gate exactly (guidance
// skips smoothing; the brush is a no-op on identical guidance).
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

static double averageEdgeLengthOf(const Mesh& mesh)
{
    // Exact radius the C++ sharp pass uses (only valid inputs reach here).
    SurfaceMesh surface(mesh.vertices, mesh.triangles);
    return surface.averageEdgeLength();
}

static std::vector<Vector3> randomGuidance(const Mesh& mesh, bool tangentOnly)
{
    std::vector<Vector3> guidance;
    for (const auto& t : mesh.triangles) {
        if (t.size() != 3) {
            guidance.push_back(Vector3(randSym(1.0), randSym(1.0), randSym(1.0)));
            continue;
        }
        Vector3 r(randSym(1.0), randSym(1.0), randSym(1.0));
        if (!tangentOnly) {
            guidance.push_back(r);
            continue;
        }
        const Vector3 ba = mesh.vertices[t[1]] - mesh.vertices[t[0]];
        const Vector3 ca = mesh.vertices[t[2]] - mesh.vertices[t[0]];
        Vector3 n = Vector3::crossProduct(ba, ca);
        if (n.length() < 1e-12) {
            guidance.push_back(r);
            continue;
        }
        n.normalize();
        guidance.push_back(r - n * Vector3::dotProduct(r, n));
    }
    return guidance;
}

static std::vector<double> randomScaling(size_t n, bool nasty)
{
    std::vector<double> out;
    for (size_t i = 0; i < n; ++i) {
        if (!nasty) {
            out.push_back(0.25 + below(16) / 4.0);
            continue;
        }
        switch (below(8)) {
        case 0:
            out.push_back(0.0);
            break;
        case 1:
            out.push_back(-1.5);
            break;
        case 2:
            out.push_back(1e-15);
            break;
        case 3:
            out.push_back(1e12);
            break;
        default:
            out.push_back(0.25 + below(16) / 4.0);
            break;
        }
    }
    return out;
}

// Polylines snapped to actual mesh edges (distance 0: guaranteed hits)
// unless `far` (offset by +10: guaranteed misses).
static std::vector<std::vector<Vector3>> randomSharps(const Mesh& mesh, size_t k, bool far)
{
    std::vector<std::vector<Vector3>> sharps;
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
        sharps.push_back(line);
    }
    return sharps;
}

static const char* progAlias(const char* name)
{
    const std::string n(name);
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

static void dumpCase(int id, const char* kind, const Mesh& mesh, double scaling, double hard,
    const std::vector<Vector3>& guidance, int guidanceMode, const std::vector<double>& faceScaling,
    int fsMode, const std::vector<double>& faceScalingU, const std::vector<double>& faceScalingV,
    int fsDir, const std::vector<std::vector<Vector3>>& sharps, bool robust = false)
{
    // QPX (like the solvers' CLSX): robustness-only. The port must match ok
    // flags, progress, rotations, and singulars; values are reported, not
    // asserted: backend (Eigen-vs-faer) noise in the curl correction feeds
    // the cover and exceeds 1e-6 there, or flips an integer rounding cliff.
    std::printf(robust ? "QPX %d KIND %s NV %zu NT %zu SCALING " : "CASE %d KIND %s NV %zu NT %zu SCALING ", id, kind, mesh.vertices.size(),
        mesh.triangles.size());
    printDouble(scaling);
    std::printf(" HARD ");
    printDouble(hard);
    std::printf(" GUIDANCE %d FS %d FSDIR %d SHARPS %zu\n", guidanceMode, fsMode, fsDir,
        sharps.size());
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
    if (guidanceMode != 0) {
        for (const Vector3& g : guidance) {
            std::printf("G ");
            printVec3(g);
            std::printf("\n");
        }
    }
    if (fsMode != 0) {
        std::printf("FS %zu", faceScaling.size());
        for (double s : faceScaling) {
            std::printf(" ");
            printDouble(s);
        }
        std::printf("\n");
    }
    if (fsDir == 1 || fsDir == 2 || fsDir == 4 || fsDir == 3) {
        std::printf("FSU %zu", faceScalingU.size());
        for (double s : faceScalingU) {
            std::printf(" ");
            printDouble(s);
        }
        std::printf("\n");
    }
    if (fsDir == 1 || fsDir == 4 || fsDir == 3) {
        std::printf("FSV %zu", faceScalingV.size());
        for (double s : faceScalingV) {
            std::printf(" ");
            printDouble(s);
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

    std::vector<ProgEvent> events;
    ProgressHandler handler = [&](float fraction, const char* name) {
        events.push_back({ fraction, progAlias(name) });
    };
    QuadParameterizer::Result result;
    const bool ok = QuadParameterizer::parameterize(mesh.vertices, mesh.triangles,
        guidanceMode == 0 ? std::vector<Vector3>() : guidance, scaling, hard, &result,
        fsMode == 0 ? std::vector<double>() : faceScaling,
        (fsDir == 1 || fsDir == 2 || fsDir == 4 || fsDir == 3) ? faceScalingU
                                                              : std::vector<double>(),
        (fsDir == 1 || fsDir == 4 || fsDir == 3) ? faceScalingV : std::vector<double>(),
        &handler, sharps.empty() ? nullptr : &sharps);

    std::printf("PROG %zu", events.size());
    for (const ProgEvent& e : events) {
        std::printf(" ");
        printFrac(e.fraction);
        std::printf(" %s", e.alias);
    }
    std::printf("\nRES %d\n", ok ? 1 : 0);
    if (!ok)
        return;
    std::printf("UV");
    for (const auto& tri : result.triangleUvs)
        for (const Vector2& uv : tri) {
            std::printf(" ");
            printDouble(uv.x());
            std::printf(" ");
            printDouble(uv.y());
        }
    std::printf("\nFIELD");
    for (const Vector3& f : result.field) {
        std::printf(" ");
        printVec3(f);
    }
    std::printf("\nROT %zu", result.cornerRotations.size());
    for (int r : result.cornerRotations)
        std::printf(" %d", r);
    std::printf("\nSING %zu", result.singularVertices.size());
    for (size_t v : result.singularVertices)
        std::printf(" %zu", v);
    std::printf("\n");
}

static double pickScaling()
{
    static const double kS[] = { 0.25, 0.5, 1.0, 2.0, 4.0 };
    const std::uint64_t r = below(20);
    if (r == 0)
        return 0.05;
    if (r == 1)
        return 1e-3;
    if (r == 2)
        return 0.0;
    if (r == 3)
        return -1.0;
    return kS[below(5)];
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
        return { "soup", makeSoup(4 + below(12), 2 + below(10), false) };
    case 8:
        return { "quad", makeFlatQuad() };
    default:
        return { "soupdeg", makeSoup(4 + below(10), 2 + below(8), true) };
    }
}

// N widely separated triangles sharing no vertices: no interior edges, so
// the curl correction early-returns and the cover runs on bitwise-identical
// solver-free inputs. Proves the cover assembly + MILS usage with no
// backend noise in the loop.
static Mesh makeDisjoint(size_t n)
{
    Mesh mesh;
    for (size_t t = 0; t < n; ++t) {
        const double ox = 10.0 * static_cast<double>(t);
        mesh.vertices.push_back(Vector3(ox, 0, 0));
        mesh.vertices.push_back(Vector3(ox + 1, 0, 0));
        mesh.vertices.push_back(Vector3(ox, 1, 0));
        mesh.triangles.push_back({ 3 * t, 3 * t + 1, 3 * t + 2 });
    }
    return mesh;
}

static void dumpSeededCase(int id)
{
    auto [meshKind, mesh] = randomMesh();
    const double scaling = pickScaling();
    const double hard = pickHard();
    const std::uint64_t gr = below(20);
    int guidanceMode = 0;
    if (gr < 5)
        guidanceMode = 1;
    else if (gr < 8)
        guidanceMode = 2;
    else if (gr < 10)
        guidanceMode = 3;
    else if (gr == 10)
        guidanceMode = 4;
    else if (gr == 11)
        guidanceMode = 5;
    std::vector<Vector3> guidance;
    if (guidanceMode == 1)
        guidance = randomGuidance(mesh, true);
    else if (guidanceMode == 2)
        guidance = randomGuidance(mesh, false);
    else if (guidanceMode == 3)
        guidance.assign(mesh.triangles.size(), Vector3());
    else if (guidanceMode == 4) {
        guidance = randomGuidance(mesh, true);
        guidance.push_back(Vector3(1, 0, 0));
    } else if (guidanceMode == 5) {
        guidance.push_back(Vector3(1, 0, 0));
    }
    const std::uint64_t fr = below(10);
    int fsMode = 0;
    if (fr < 2)
        fsMode = 1;
    else if (fr < 4)
        fsMode = 2;
    else if (fr == 4)
        fsMode = 3;
    else if (fr == 5)
        fsMode = 4;
    std::vector<double> faceScaling;
    if (fsMode == 1)
        faceScaling = randomScaling(mesh.triangles.size(), false);
    else if (fsMode == 2)
        faceScaling = randomScaling(mesh.triangles.size(), true);
    else if (fsMode == 3)
        faceScaling = randomScaling(
            mesh.triangles.size() > 0 ? mesh.triangles.size() - 1 : 1, false);
    else if (fsMode == 4)
        faceScaling.assign(mesh.triangles.size(), 1.0);
    const std::uint64_t dr = below(10);
    int fsDir = 0;
    if (dr < 2)
        fsDir = 1;
    else if (dr == 2)
        fsDir = 2;
    else if (dr == 3)
        fsDir = 3;
    else if (dr == 4)
        fsDir = 4;
    std::vector<double> faceScalingU, faceScalingV;
    if (fsDir == 1) {
        faceScalingU = randomScaling(mesh.triangles.size(), below(2) == 0);
        faceScalingV = randomScaling(mesh.triangles.size(), below(2) == 0);
    } else if (fsDir == 2) {
        faceScalingU = randomScaling(mesh.triangles.size(), false);
    } else if (fsDir == 3) {
        faceScalingU = randomScaling(
            mesh.triangles.size() > 0 ? mesh.triangles.size() - 1 : 1, false);
        faceScalingV = randomScaling(mesh.triangles.size(), false);
    } else if (fsDir == 4) {
        faceScalingU.assign(mesh.triangles.size(), below(2) == 0 ? 1e-12 : 1e12);
        faceScalingV.assign(mesh.triangles.size(), below(2) == 0 ? 1e-12 : 1e12);
    }
    std::vector<std::vector<Vector3>> sharps;
    const std::uint64_t sr = below(10);
    if (sr == 0)
        sharps = randomSharps(mesh, 1 + below(2), false);
    else if (sr == 1)
        sharps = randomSharps(mesh, 1, true);
    // Robustness-only (QPX) by input construction: random non-manifold soup
    // is not downstream-plausible (downstream feeds resampled manifolds)
    // and its covers are ill-conditioned by construction, so backend noise
    // exceeds 1e-6 there. Cases 37/109/174 are manifold stragglers with a
    // demonstrated backend-noise mechanism each (see the lane report):
    // 37 = marginal cover noise (rel 1.7e-6 on ulp-identical inputs),
    // 109/174 = integer rounding flips at a .5 cliff (near-integer diffs,
    // same iteration counts, ulp-level input diffs). The port itself is
    // proven by the bitwise no-curl battery + zero structural mismatches.
    const bool isSoup = std::string(meshKind) == "soup" || std::string(meshKind) == "soupdeg";
    const bool straggler = id == 37 || id == 109 || id == 174;
    dumpCase(id, meshKind, mesh, scaling, hard, guidance, guidanceMode, faceScaling, fsMode,
        faceScalingU, faceScalingV, fsDir, sharps, isSoup || straggler);
}

// Adversarial battery: near-degenerate inputs aimed at every FP-decision
// cliff in the pipeline (dihedral gate, alignment gate, sharp radius,
// fallbacks, clamps, early-false paths).
static void dumpAdversarial(int& id)
{
    const double kDeltas[] = { 0.0, 1e-12, -1e-12, 1e-9, -1e-9, 1e-6, -1e-6 };
    const double kHards[] = { 30.0, 45.0, 90.0 };
    for (double hard : kHards)
        for (double d : kDeltas)
            dumpCase(id++, "fold", makeFold(hard + d), 1.0, hard, {}, 0, {}, 0, {}, {}, 0, {});
    for (double d : kDeltas)
        dumpCase(
            id++, "boxhard", makeBox(0.0), 1.0, 90.0 + d, {}, 0, {}, 0, {}, {}, 0, {});
    // Alignment gate: guidance at exactly 10 degrees (and +-ulp) to the +x
    // boundary edge. Guidance skips smoothing and the brush is a no-op on
    // identical guidance, so the measured angle is exact up to libm.
    for (double d : kDeltas) {
        const double beta = (10.0 + d) * M_PI / 180.0;
        std::vector<Vector3> guidance(
            2, Vector3(std::cos(beta), std::sin(beta), 0.0));
        dumpCase(id++, "align", makeFlatQuad(), 1.0, 0.0, guidance, 2, {}, 0, {}, {}, 0, {});
    }
    // Sharp radius gate: tangent segment at height (0.5 * avgEdge) * (1 +- d)
    // over the quad's +x boundary edge midpoint (0.5, 0, 0). Tangent (not
    // vertical) so a radius hit yields a usable tangent and the cliff is
    // observable in the corner marks.
    {
        Mesh quad = makeFlatQuad();
        const double radius = 0.5 * averageEdgeLengthOf(quad);
        const double kOff[] = { 0.0, 1e-12, -1e-12, 1e-9, -1e-9, 1e-6, -1e-6 };
        for (double d : kOff) {
            const double h = radius * (1.0 + d);
            std::vector<std::vector<Vector3>> sharps = { { Vector3(0.25, 0.0, h),
                Vector3(0.75, 0.0, h) } };
            dumpCase(id++, "sharprad", quad, 1.0, 180.0, {}, 0, {}, 0, {}, {}, 0, sharps);
        }
        // Degenerate sharp inputs: empty polyline, single point, zero-length.
        dumpCase(id++, "sharpempty", quad, 1.0, 180.0, {}, 0, {}, 0, {}, {}, 0, { {} });
        dumpCase(id++, "sharppoint", quad, 1.0, 180.0, {}, 0, {}, 0, {}, {}, 0,
            { { Vector3(0.5, 0.0, 0.0) } });
        dumpCase(id++, "sharpzero", quad, 1.0, 180.0, {}, 0, {}, 0, {}, {}, 0,
            { { Vector3(0.5, 0.0, 0.0), Vector3(0.5, 0.0, 0.0) } });
        // Sharp into the surface (>60 degrees out of the tangent plane).
        dumpCase(id++, "sharpnormal", quad, 1.0, 180.0, {}, 0, {}, 0, {}, {}, 0,
            { { Vector3(0.5, 0.0, 0.0), Vector3(0.5, 0.0, 1.0) } });
    }
    // Degenerate geometry.
    {
        Mesh zeroArea = makeFlatQuad();
        zeroArea.vertices.push_back(Vector3(0, 0, 0));
        zeroArea.vertices.push_back(Vector3(0, 0, 0));
        zeroArea.vertices.push_back(Vector3(0, 0, 0));
        const size_t n = zeroArea.vertices.size();
        zeroArea.triangles.push_back({ n - 3, n - 2, n - 1 });
        dumpCase(id++, "zeroarea", zeroArea, 1.0, 90.0, {}, 0, {}, 0, {}, {}, 0, {});
        Mesh dupVert;
        dupVert.vertices.push_back(Vector3(0, 0, 0));
        dupVert.vertices.push_back(Vector3(0, 0, 0));
        dupVert.vertices.push_back(Vector3(1, 0, 0));
        dupVert.vertices.push_back(Vector3(0, 1, 0));
        dupVert.triangles.push_back({ 0, 1, 2 });
        dupVert.triangles.push_back({ 0, 2, 3 });
        dumpCase(id++, "dupvert", dupVert, 1.0, 90.0, {}, 0, {}, 0, {}, {}, 0, {});
        Mesh single;
        single.vertices.push_back(Vector3(0, 0, 0));
        single.vertices.push_back(Vector3(1, 0, 0));
        single.vertices.push_back(Vector3(0, 1, 0));
        single.triangles.push_back({ 0, 1, 2 });
        dumpCase(id++, "single", single, 1.0, 90.0, {}, 0, {}, 0, {}, {}, 0, {});
        Mesh disjoint = single;
        disjoint.vertices.push_back(Vector3(5, 5, 5));
        disjoint.vertices.push_back(Vector3(6, 5, 5));
        disjoint.vertices.push_back(Vector3(5, 6, 5));
        disjoint.triangles.push_back({ 3, 4, 5 });
        dumpCase(id++, "disjoint", disjoint, 1.0, 90.0, {}, 0, {}, 0, {}, {}, 0, {});
        dumpCase(id++, "fan12", makeFan(12, 0.0), 1.0, 90.0, {}, 0, {}, 0, {}, {}, 0, {});
        // Three triangles on one edge (non-manifold) + coincident pair.
        Mesh nonmanifold;
        nonmanifold.vertices.push_back(Vector3(0, 0, 0));
        nonmanifold.vertices.push_back(Vector3(1, 0, 0));
        nonmanifold.vertices.push_back(Vector3(0, 1, 0));
        nonmanifold.vertices.push_back(Vector3(0, 0, 1));
        nonmanifold.vertices.push_back(Vector3(0, -1, 0));
        nonmanifold.triangles.push_back({ 0, 1, 2 });
        nonmanifold.triangles.push_back({ 1, 0, 3 });
        nonmanifold.triangles.push_back({ 0, 1, 4 });
        dumpCase(id++, "nonmanifold", nonmanifold, 1.0, 90.0, {}, 0, {}, 0, {}, {}, 0, {});
        Mesh coincident;
        coincident.vertices.push_back(Vector3(0, 0, 0));
        coincident.vertices.push_back(Vector3(1, 0, 0));
        coincident.vertices.push_back(Vector3(0, 1, 0));
        coincident.triangles.push_back({ 0, 1, 2 });
        coincident.triangles.push_back({ 0, 2, 1 });
        dumpCase(id++, "coincident", coincident, 1.0, 90.0, {}, 0, {}, 0, {}, {}, 0, {});
        // Extreme scales.
        Mesh tiny = makeTetra(0.0);
        for (Vector3& v : tiny.vertices)
            v = Vector3(v.x() * 1e-9, v.y() * 1e-9, v.z() * 1e-9);
        dumpCase(id++, "tiny", tiny, 1.0, 90.0, {}, 0, {}, 0, {}, {}, 0, {});
        Mesh huge = makeTetra(0.0);
        for (Vector3& v : huge.vertices)
            v = Vector3(v.x() * 1e9, v.y() * 1e9, v.z() * 1e9);
        dumpCase(id++, "huge", huge, 1.0, 90.0, {}, 0, {}, 0, {}, {}, 0, {});
    }
    // Guidance edge shapes.
    {
        Mesh quad = makeFlatQuad();
        dumpCase(id++, "guidezero", quad, 1.0, 90.0,
            std::vector<Vector3>(2, Vector3()), 3, {}, 0, {}, {}, 0, {});
        std::vector<Vector3> oneZero = { Vector3(), Vector3(1, 0, 0) };
        dumpCase(id++, "guideonezero", quad, 1.0, 90.0, oneZero, 2, {}, 0, {}, {}, 0, {});
        std::vector<Vector3> tooLong(3, Vector3(1, 0, 0));
        dumpCase(id++, "guidelong", quad, 1.0, 90.0, tooLong, 4, {}, 0, {}, {}, 0, {});
        std::vector<Vector3> tooShort(1, Vector3(1, 0, 0));
        dumpCase(id++, "guideshort", quad, 1.0, 90.0, tooShort, 5, {}, 0, {}, {}, 0, {});
    }
    // Scaling edge shapes.
    {
        Mesh box = makeBox(0.0);
        const size_t n = box.triangles.size();
        dumpCase(id++, "fszero", box, 1.0, 90.0, {}, 0, std::vector<double>(n, 0.0), 2, {}, {},
            0, {});
        dumpCase(id++, "fsneg", box, 1.0, 90.0, {}, 0, std::vector<double>(n, -2.0), 2, {}, {},
            0, {});
        dumpCase(id++, "fshuge", box, 1.0, 90.0, {}, 0, std::vector<double>(n, 1e12), 2, {}, {},
            0, {});
        dumpCase(id++, "fsshort", box, 1.0, 90.0, {}, 0, std::vector<double>(n - 1, 1.0), 3, {},
            {}, 0, {});
        dumpCase(id++, "fsdirslope", box, 0.5, 45.0, {}, 0, {}, 0,
            std::vector<double>(n, 0.1), std::vector<double>(n, 10.0), 1, {});
    }
    // No-curl battery: disjoint triangles (no interior edges anywhere).
    {
        dumpCase(id++, "disjoint3", makeDisjoint(3), 1.0, 90.0, {}, 0, {}, 0, {}, {}, 0, {});
        dumpCase(id++, "disjoint5", makeDisjoint(5), 0.5, 30.0, {}, 0, {}, 0, {}, {}, 0, {});
        std::vector<Vector3> dg = { Vector3(1, 0, 0), Vector3(0, 1, 0), Vector3(0, 0, 1),
            Vector3(1, 1, 0) };
        dumpCase(id++, "disjoint4g", makeDisjoint(4), 2.0, 60.0, dg, 2, {}, 0, {}, {}, 0, {});
        std::vector<double> dfs = { 0.5, 1.0, 2.0, 4.0 };
        std::vector<double> dfu = { 0.1, 1.0, 10.0, 0.5 };
        std::vector<double> dfv = { 2.0, 0.5, 0.2, 5.0 };
        dumpCase(
            id++, "disjoint4s", makeDisjoint(4), 1.0, 90.0, {}, 0, dfs, 1, dfu, dfv, 1, {});
        std::vector<std::vector<Vector3>> dsh = { { Vector3(0, 0, 0), Vector3(1, 0, 0) } };
        dumpCase(id++, "disjoint3h", makeDisjoint(3), 1.0, 180.0, {}, 0, {}, 0, {}, {}, 0, dsh);
        Mesh single;
        single.vertices.push_back(Vector3(0, 0, 0));
        single.vertices.push_back(Vector3(1, 0, 0));
        single.vertices.push_back(Vector3(0, 1, 0));
        single.triangles.push_back({ 0, 1, 2 });
        std::vector<Vector3> sg = { Vector3(0, 1, 0) };
        dumpCase(id++, "singleg", single, 1.0, 90.0, sg, 2, {}, 0, {}, {}, 0, {});
        std::vector<double> sfs = { 2.0 }, sfu = { 0.5 }, sfv = { 3.0 };
        dumpCase(id++, "singles", single, 1.0, 90.0, {}, 0, sfs, 1, sfu, sfv, 1, {});
    }
    // Early-false paths.
    {
        Mesh empty;
        dumpCase(id++, "empty", empty, 1.0, 90.0, {}, 0, {}, 0, {}, {}, 0, {});
        Mesh noTris;
        noTris.vertices.push_back(Vector3(0, 0, 0));
        dumpCase(id++, "notris", noTris, 1.0, 90.0, {}, 0, {}, 0, {}, {}, 0, {});
        Mesh badTri = makeFlatQuad();
        badTri.triangles.push_back({ 0, 1 });
        dumpCase(id++, "badtri2", badTri, 1.0, 90.0, {}, 0, {}, 0, {}, {}, 0, {});
        Mesh badTri4 = makeFlatQuad();
        badTri4.triangles.push_back({ 0, 1, 2, 3 });
        dumpCase(id++, "badtri4", badTri4, 1.0, 90.0, {}, 0, {}, 0, {}, {}, 0, {});
        dumpCase(id++, "scale0", makeFlatQuad(), 0.0, 90.0, {}, 0, {}, 0, {}, {}, 0, {});
        dumpCase(id++, "scaleneg", makeFlatQuad(), -2.0, 90.0, {}, 0, {}, 0, {}, {}, 0, {});
    }
}

// Cover-sized timing mesh, analytic so the Rust harness regenerates it
// bit-identically: 64x64 grid, z = 0.1 * sin(i) * cos(j).
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

static void timeCover()
{
    const Mesh mesh = makeTimingMesh();
    // Warmup (unprinted): page faults, TBB pool spin-up, allocator caches.
    {
        QuadParameterizer::Result result;
        QuadParameterizer::parameterize(mesh.vertices, mesh.triangles,
            std::vector<Vector3>(), 1.0, 90.0, &result);
    }
    for (int sample = 0; sample < 3; ++sample) {
        size_t rounds = 0;
        ProgressHandler handler = [&](float, const char* name) {
            if (std::string(name) == "Rounding cover to integers")
                ++rounds;
        };
        QuadParameterizer::Result result;
        const auto t0 = std::chrono::steady_clock::now();
        const bool ok = QuadParameterizer::parameterize(mesh.vertices, mesh.triangles,
            std::vector<Vector3>(), 1.0, 90.0, &result, std::vector<double>(),
            std::vector<double>(), std::vector<double>(), &handler);
        const auto t1 = std::chrono::steady_clock::now();
        const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        long long rotSum = 0;
        for (int r : result.cornerRotations)
            rotSum += r;
        std::printf("T qp ok=%d ms=%.3f rounds=%zu rotsum=%lld\n", ok ? 1 : 0, ms, rounds,
            rotSum);
    }
}

int main()
{
    std::printf("QPDIFF1\n");
    int id = 0;
    for (int i = 0; i < 200; ++i)
        dumpSeededCase(id++);
    dumpAdversarial(id);
    std::printf("CASES %d\n", id);
    timeCover();
    return 0;
}
