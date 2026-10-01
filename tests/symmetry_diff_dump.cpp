// Differential oracle dump for the symmetry Rust port (rs-symmetry lane).
// Seeded splitmix64 cases over mirror/detect/score/symmetrize entry points,
// plus adversarial near-degenerate inputs: score-boundary ulp straddles,
// cell-boundary coordinates, 1e-12 field/tangent length straddles,
// cross-candidate exact ties, self-mirror faces, tie-breaks, empty and
// invalid inputs, and overflow-adjacent offsets proving the FMA
// transcription of `2.0 * offset - x`.
//
// Build-only helper: not registered with ctest. Build Release (the oracle
// is bitwise and Clang fuses `2.0 * offset - x` only under optimization)
// and redirect stdout to tests/fixtures/symmetry_diff.txt, replayed by
// rust/core/tests/symmetry_diff.rs. Doubles print with %.17g (exact
// round-trip, so the replay compares bitwise).
import retopo.core.symmetry;
import retopo.core.vector3;

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <vector>

static std::uint64_t g_state = 0x5EA7E7A11CULL;

static std::uint64_t nextU64()
{
    std::uint64_t z = (g_state += 0x9E3779B97F4A7C15ULL);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
}

static std::uint64_t below(std::uint64_t n)
{
    return nextU64() % n;
}

using AutoRemesher::Symmetry;
using AutoRemesher::SymmetryPlane;
using AutoRemesher::Vector3;

static void printDouble(double v)
{
    std::printf("%.17g", v);
}

static void printV3(const Vector3& v)
{
    printDouble(v.x());
    std::printf(" ");
    printDouble(v.y());
    std::printf(" ");
    printDouble(v.z());
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

static Vector3 randV3()
{
    return Vector3(randDouble(), randDouble(), randDouble());
}

static int randAxis()
{
    // Mostly valid axes, sometimes invalid (must be a no-op / invalid).
    switch (below(8)) {
    case 0:
        return -1;
    case 1:
        return 3;
    case 2:
        return 7;
    default:
        return static_cast<int>(below(3));
    }
}

static void printPlane(const SymmetryPlane& plane)
{
    std::printf("%d ", plane.axis);
    printDouble(plane.offset);
    std::printf(" ");
    printDouble(plane.score);
}

static void dumpMirror(const Vector3& point, int axis, double offset)
{
    const SymmetryPlane plane { axis, offset, 0.0 };
    std::printf("M ");
    printV3(point);
    std::printf(" %d ", axis);
    printDouble(offset);
    std::printf("\n");
    std::printf("r ");
    printV3(Symmetry::mirrorPoint(point, plane));
    std::printf(" ");
    printV3(Symmetry::mirrorDirection(point, plane));
    std::printf("\n");
}

static void dumpDetect(const std::vector<Vector3>& vertices, int probeAxis, double probeOffset, double probeTol)
{
    std::printf("D %zu", vertices.size());
    for (const auto& v : vertices) {
        std::printf(" ");
        printV3(v);
    }
    std::printf(" %d ", probeAxis);
    printDouble(probeOffset);
    std::printf(" ");
    printDouble(probeTol);
    std::printf("\n");
    std::printf("r ");
    printPlane(Symmetry::detectPlane(vertices));
    for (const int axis : { -1, 0, 1, 2, 3 }) {
        std::printf(" ");
        printPlane(Symmetry::fixedPlane(vertices, axis));
    }
    std::printf(" ");
    printDouble(Symmetry::scorePlane(vertices, probeAxis, probeOffset, probeTol));
    std::printf("\n");
}

static void dumpSymmetrizeV(const std::vector<Vector3>& vertices, int axis, double offset)
{
    std::printf("V %zu", vertices.size());
    for (const auto& v : vertices) {
        std::printf(" ");
        printV3(v);
    }
    std::printf(" %d ", axis);
    printDouble(offset);
    std::printf("\n");
    std::vector<Vector3> out = vertices;
    Symmetry::symmetrizeVertices(out, SymmetryPlane { axis, offset, 1.0 });
    std::printf("r %zu", out.size());
    for (const auto& v : out) {
        std::printf(" ");
        printV3(v);
    }
    std::printf("\n");
}

static void dumpFrameField(const std::vector<Vector3>& vertices,
    const std::vector<std::vector<size_t>>& triangles,
    const std::vector<Vector3>& field, int axis, double offset)
{
    std::printf("F %zu", vertices.size());
    for (const auto& v : vertices) {
        std::printf(" ");
        printV3(v);
    }
    std::printf(" %zu", triangles.size());
    for (const auto& tri : triangles) {
        std::printf(" %zu", tri.size());
        for (const size_t index : tri)
            std::printf(" %zu", index);
    }
    std::printf(" %zu", field.size());
    for (const auto& v : field) {
        std::printf(" ");
        printV3(v);
    }
    std::printf(" %d ", axis);
    printDouble(offset);
    std::printf("\n");
    std::vector<Vector3> out = field;
    Symmetry::symmetrizeFrameField(vertices, triangles, out, SymmetryPlane { axis, offset, 1.0 });
    std::printf("r %zu", out.size());
    for (const auto& v : out) {
        std::printf(" ");
        printV3(v);
    }
    std::printf("\n");
}

// Exactly mirrored cloud about (axis, offset): half generated, half mirrored.
static std::vector<Vector3> mirroredCloud(size_t n, int axis, double offset, double noise)
{
    std::vector<Vector3> vertices;
    vertices.reserve(n);
    const SymmetryPlane plane { axis, offset, 0.0 };
    while (vertices.size() + 1 < n) {
        Vector3 v = randV3();
        vertices.push_back(v);
        Vector3 mirror = Symmetry::mirrorPoint(v, plane);
        if (noise > 0.0)
            mirror = mirror + randV3() * noise;
        vertices.push_back(mirror);
    }
    while (vertices.size() < n) {
        Vector3 v = randV3();
        v[static_cast<size_t>(axis)] = offset; // on-plane filler keeps exact symmetry
        vertices.push_back(v);
    }
    return vertices;
}

static void dumpSeeded()
{
    // Mirror: 60 seeded cases.
    for (int i = 0; i < 60; ++i)
        dumpMirror(randV3(), randAxis(), randDouble());

    // Detect: 60 seeded clouds.
    const double tols[] = { 1e-9, 1e-6, 1e-3, 0.01, 0.05, 0.1, 0.5, 1.0, 10.0 };
    const double noises[] = { 0.0, 1e-9, 1e-6, 1e-3, 0.05 };
    for (int i = 0; i < 60; ++i) {
        const size_t n = 1 + below(40);
        std::vector<Vector3> vertices;
        switch (below(4)) {
        case 0:
        case 1: {
            const int axis = static_cast<int>(below(3));
            vertices = mirroredCloud(n, axis, randDouble(), noises[below(5)]);
            break;
        }
        default:
            vertices.reserve(n);
            for (size_t k = 0; k < n; ++k)
                vertices.push_back(randV3());
            break;
        }
        const int probeAxis = randAxis();
        const double probeTol = below(12) == 0 ? 0.0 : (below(12) == 1 ? -0.5 : tols[below(9)]);
        dumpDetect(vertices, probeAxis, randDouble(), probeTol);
    }

    // symmetrizeVertices: 60 seeded clouds.
    for (int i = 0; i < 60; ++i) {
        const size_t n = 1 + below(40);
        const int axis = static_cast<int>(below(3));
        std::vector<Vector3> vertices;
        switch (below(3)) {
        case 0:
            vertices = mirroredCloud(n, axis, randDouble(), noises[below(5)]);
            break;
        default:
            vertices.reserve(n);
            for (size_t k = 0; k < n; ++k)
                vertices.push_back(randV3());
            break;
        }
        const int planeAxis = below(8) == 0 ? -1 : axis;
        dumpSymmetrizeV(vertices, planeAxis, randDouble());
    }

    // symmetrizeFrameField: 60 seeded soups.
    for (int i = 0; i < 60; ++i) {
        const size_t nv = 3 + below(8);
        const size_t nt = 1 + below(6);
        std::vector<Vector3> vertices;
        vertices.reserve(nv);
        for (size_t k = 0; k < nv; ++k)
            vertices.push_back(randV3());
        std::vector<std::vector<size_t>> triangles;
        triangles.reserve(nt);
        for (size_t t = 0; t < nt; ++t) {
            if (below(4) == 0) {
                // Degenerate: repeated index (zero normal -> skip).
                const size_t a = below(nv);
                triangles.push_back({ a, a, below(nv) });
            } else {
                triangles.push_back({ below(nv), below(nv), below(nv) });
            }
        }
        std::vector<Vector3> field;
        field.reserve(nt);
        for (size_t t = 0; t < nt; ++t) {
            const unsigned r = below(9);
            if (r == 0)
                field.push_back(Vector3()); // zero field -> skip
            else if (r == 1)
                field.push_back(randV3() * 1e-13); // tiny field
            else
                field.push_back(randV3());
        }
        const int planeAxis = below(8) == 0 ? -1 : static_cast<int>(below(3));
        dumpFrameField(vertices, triangles, field, planeAxis, randDouble());
    }
}

static void dumpAdversarial()
{
    const double eps = std::numeric_limits<double>::epsilon();
    const double dblMax = std::numeric_limits<double>::max();
    const double denormMin = std::numeric_limits<double>::denorm_min();

    // M: overflow-adjacent offsets prove the FMA transcription: fma(o, 2, -x)
    // vs (2*o) - x differ exactly here (C++ Release fuses; Rust uses mul_add).
    const double offsets[] = { dblMax, -dblMax, 1e308, -1e308, 1e-308, -1e-308,
        denormMin, -denormMin, 0.0, -0.0 };
    const Vector3 mpoints[] = {
        Vector3(dblMax, 0.0, 0.0), Vector3(1.0, 2.0, 3.0), Vector3(-0.0, -0.0, -0.0),
    };
    for (const double offset : offsets) {
        for (const Vector3& point : mpoints)
            dumpMirror(point, static_cast<int>(below(3)), offset);
    }
    dumpMirror(Vector3(1.0, 2.0, 3.0), -1, dblMax); // invalid axis: identity even at extremes

    // D: empty input.
    dumpDetect({}, 1, 0.0, 0.01);
    dumpDetect({}, -1, 0.0, 0.0);
    // D: all-coincident (diagonal 0 -> tolerance 1e-9, every axis scores 1, tie -> Y).
    dumpDetect(std::vector<Vector3>(5, Vector3(1.0, 2.0, 3.0)), 0, 1.0, 1e-9);
    // D: cube corners (exact 3-way tie -> Y).
    {
        std::vector<Vector3> corners;
        for (int x = -1; x <= 1; x += 2)
            for (int y = -1; y <= 1; y += 2)
                for (int z = -1; z <= 1; z += 2)
                    corners.push_back(Vector3(x, y, z));
        dumpDetect(corners, 1, 0.0, 0.01);
    }
    // D: single-vertex score straddles. V=(1,0,0), plane x=0: the mirror is
    // 2.0 away, the only candidate is V itself, so tol=2*(1+k*eps) flips 0/1.
    for (int k = -2; k <= 2; ++k)
        dumpDetect({ Vector3(1.0, 0.0, 0.0) }, 0, 0.0, 2.0 * (1.0 + k * eps));
    // D: cell-boundary coordinates at exact-binary tolerance 0.125.
    dumpDetect({ Vector3(0.0625, 0.0, 0.0) }, 0, 0.0, 0.125);
    dumpDetect({ Vector3(0.0625, 0.0, 0.0) }, 0, 0.0, 0.125 * (1.0 + eps));
    dumpDetect({ Vector3(0.0625, 0.0, 0.0) }, 0, 0.0, 0.125 * (1.0 - 2.0 * eps));
    // D: non-positive tolerance and invalid probe axes.
    dumpDetect({ Vector3(1.0, 0.0, 0.0), Vector3(-1.0, 0.0, 0.0) }, 0, 0.0, 0.0);
    dumpDetect({ Vector3(1.0, 0.0, 0.0), Vector3(-1.0, 0.0, 0.0) }, 0, 0.0, -1.0);
    dumpDetect({ Vector3(1.0, 0.0, 0.0), Vector3(-1.0, 0.0, 0.0) }, 7, 0.0, 0.1);
    dumpDetect({ Vector3(1.0, 0.0, 0.0), Vector3(-1.0, 0.0, 0.0) }, -1, 0.0, 0.1);
    // D: near-tie winners. Y-exact cloud with one extra vertex on the Y plane
    // (Y stays 1.0, X drops a hit -> Y wins); mirrored setup -> X wins.
    {
        const std::vector<Vector3> yExact = {
            Vector3(1.0, 1.0, 0.0), Vector3(1.0, -1.0, 0.0),
            Vector3(-1.0, 1.0, 0.5), Vector3(-1.0, -1.0, 0.5),
            Vector3(0.25, 0.0, -0.5),
        };
        dumpDetect(yExact, 1, 0.0, 0.05);
        const std::vector<Vector3> xExact = {
            Vector3(1.0, 1.0, 0.0), Vector3(-1.0, 1.0, 0.0),
            Vector3(1.0, -1.0, 0.5), Vector3(-1.0, -1.0, 0.5),
            Vector3(0.0, 0.25, -0.5),
        };
        dumpDetect(xExact, 0, 0.0, 0.05);
    }

    // V: single vertex on / off the plane (both self-snap: the only partner).
    dumpSymmetrizeV({ Vector3(1.0, 0.0, 3.0) }, 1, 0.0);
    dumpSymmetrizeV({ Vector3(1.0, 2.0, 3.0) }, 1, 0.0);
    // V: exact pair + duplicate forcing the done[partner] coincide path.
    dumpSymmetrizeV({ Vector3(1.0, 0.0, 0.0), Vector3(1.0, 0.0, 0.0), Vector3(-1.0, 0.0, 0.0) }, 0, 0.0);
    // V: invalid plane (no-op), invalid axis 3 (no-op), empty (no-op).
    dumpSymmetrizeV({ Vector3(1.0, 2.0, 3.0), Vector3(4.0, 5.0, 6.0) }, -1, 0.0);
    dumpSymmetrizeV({ Vector3(1.0, 2.0, 3.0), Vector3(4.0, 5.0, 6.0) }, 3, 0.0);
    dumpSymmetrizeV({}, 1, 0.0);
    // V: all-coincident (every vertex self-snaps onto the plane).
    dumpSymmetrizeV(std::vector<Vector3>(4, Vector3(1.0, 2.0, 3.0)), 2, 0.5);

    // F: cross-candidate exact tie. Tri pair mirrored about Y=0 with normals
    // -/+y... own=(s,0,s) sits at exactly 45 degrees between the tangent
    // (1,0,0) and the perpendicular (0,0,1): both dots are bitwise s, so the
    // strictly-greater rule keeps the tangent.
    {
        const double s = 1.0 / std::sqrt(2.0);
        const std::vector<Vector3> vertices = {
            Vector3(0.0, 1.0, 0.0), Vector3(1.0, 1.0, 0.0), Vector3(0.0, 1.0, 1.0),
            Vector3(0.0, -1.0, 0.0), Vector3(1.0, -1.0, 0.0), Vector3(0.0, -1.0, 1.0),
        };
        const std::vector<std::vector<size_t>> triangles = { { 0, 1, 2 }, { 3, 4, 5 } };
        dumpFrameField(vertices, triangles, { Vector3(s, 0.0, s), Vector3(1.0, 0.0, 0.0) }, 1, 0.0);
        // Same pair, partner field zero -> skip path (both unchanged).
        dumpFrameField(vertices, triangles, { Vector3(1.0, 0.0, 0.0), Vector3() }, 1, 0.0);
        // Own-field 1e-12 length straddles: (t,0,0) is processed iff t > 1e-12.
        for (int k = -1; k <= 1; ++k) {
            const double t = 1e-12 * (1.0 + k * eps);
            dumpFrameField(vertices, triangles, { Vector3(t, 0.0, 0.0), Vector3(1.0, 0.0, 0.0) }, 1, 0.0);
        }
        // Tangent 1e-12 straddles: partner field (t,0,1) projects to (t,~0,0).
        for (int k = -1; k <= 1; ++k) {
            const double t = 1e-12 * (1.0 + k * eps);
            dumpFrameField(vertices, triangles, { Vector3(1.0, 0.0, 0.0), Vector3(t, 0.0, 1.0) }, 1, 0.0);
        }
    }
    // F: single face straddling the plane (partner == self branch).
    {
        const std::vector<Vector3> vertices = { Vector3(1.0, 0.0, 0.0), Vector3(-1.0, 0.0, 0.0), Vector3(0.0, 0.0, 1.0) };
        const std::vector<std::vector<size_t>> triangles = { { 0, 1, 2 } };
        dumpFrameField(vertices, triangles, { Vector3(1.0, 0.0, 0.0) }, 1, 0.0);
        dumpFrameField(vertices, triangles, { Vector3(0.0, 0.0, 1.0) }, 1, 0.0);
        dumpFrameField(vertices, triangles, { Vector3(0.0, 1.0, 0.0) }, 1, 0.0); // field parallel to normal
    }
    // F: partner-done skip. B mirrors onto C which A already claimed.
    {
        const std::vector<Vector3> vertices = {
            Vector3(0.0, 0.0, 0.0), Vector3(1.0, 0.0, 0.0), Vector3(0.0, 1.0, 0.0),
            Vector3(0.001, 0.0, 0.0), Vector3(1.001, 0.0, 0.0), Vector3(0.001, 1.0, 0.0),
            Vector3(0.0, 0.0, 0.0), Vector3(-1.0, 0.0, 0.0), Vector3(0.0, 1.0, 0.0),
        };
        const std::vector<std::vector<size_t>> triangles = { { 0, 1, 2 }, { 3, 4, 5 }, { 6, 7, 8 } };
        dumpFrameField(vertices, triangles,
            { Vector3(1.0, 0.0, 0.0), Vector3(1.0, 0.0, 0.0), Vector3(1.0, 0.0, 0.0) }, 0, 0.0);
    }
    // F: degenerate triangles (repeated index, collinear, short, empty, wide).
    {
        const std::vector<Vector3> vertices = {
            Vector3(0.0, 0.0, 0.0), Vector3(1.0, 0.0, 0.0), Vector3(0.0, 1.0, 0.0),
        };
        dumpFrameField(vertices, { { 0, 0, 1 } }, { Vector3(1.0, 0.0, 0.0) }, 1, 0.0);
        dumpFrameField(vertices, { { 0, 1 }, { 0, 1, 2 } }, { Vector3(1.0, 0.0, 0.0), Vector3(0.0, 0.0, 1.0) }, 1, 0.0);
        dumpFrameField(vertices, { {}, { 0, 1, 2, 0 } }, { Vector3(1.0, 0.0, 0.0), Vector3(0.0, 0.0, 1.0) }, 1, 0.0);
    }
    // F: structural no-ops (size mismatch, empty triangles, invalid plane).
    {
        const std::vector<Vector3> vertices = { Vector3(0.0, 0.0, 0.0), Vector3(1.0, 0.0, 0.0), Vector3(0.0, 1.0, 0.0) };
        dumpFrameField(vertices, { { 0, 1, 2 } }, { Vector3(1.0, 0.0, 0.0), Vector3(0.0, 1.0, 0.0) }, 1, 0.0);
        dumpFrameField(vertices, {}, {}, 1, 0.0);
        dumpFrameField(vertices, { { 0, 1, 2 } }, { Vector3(1.0, 0.0, 0.0) }, -1, 0.0);
    }
}

static constexpr std::uint64_t kTimingSeed = 0x51EED01E55ULL;

// Simple draws for the shared timing stream (mirrored exactly in Rust).
static double timingDouble()
{
    return (static_cast<double>(nextU64() % 2000000) - 1000000.0) / 100000.0;
}

static void timeSymmetryLoop()
{
    g_state = kTimingSeed;
    const size_t n = 20000;
    std::vector<Vector3> vertices(n);
    for (size_t i = 0; i < n; ++i)
        vertices[i] = Vector3(timingDouble(), timingDouble(), timingDouble());
    std::vector<std::vector<size_t>> triangles(n);
    for (size_t i = 0; i < n; ++i)
        triangles[i] = { nextU64() % n, nextU64() % n, nextU64() % n };
    std::vector<Vector3> field(n);
    for (size_t i = 0; i < n; ++i)
        field[i] = Vector3(timingDouble(), timingDouble(), timingDouble());
    auto t0 = std::chrono::steady_clock::now();
    const SymmetryPlane detected = Symmetry::detectPlane(vertices);
    const SymmetryPlane fixed0 = Symmetry::fixedPlane(vertices, 0);
    const SymmetryPlane fixed1 = Symmetry::fixedPlane(vertices, 1);
    const SymmetryPlane fixed2 = Symmetry::fixedPlane(vertices, 2);
    std::vector<Vector3> snapped = vertices;
    Symmetry::symmetrizeVertices(snapped, detected);
    std::vector<Vector3> symField = field;
    Symmetry::symmetrizeFrameField(vertices, triangles, symField, detected);
    double sink = 0.0;
    sink += detected.score + detected.offset;
    sink += fixed0.score + fixed0.offset;
    sink += fixed1.score + fixed1.offset;
    sink += fixed2.score + fixed2.offset;
    for (const auto& v : snapped) {
        sink += v.x();
        sink += v.y();
        sink += v.z();
    }
    for (const auto& v : symField) {
        sink += v.x();
        sink += v.y();
        sink += v.z();
    }
    auto t1 = std::chrono::steady_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    std::printf("T sym ms=%.3f sink=%.17g\n", ms, sink);
}

int main()
{
    std::printf("SDIFF1\n");
    dumpSeeded();
    dumpAdversarial();
    timeSymmetryLoop();
    timeSymmetryLoop();
    timeSymmetryLoop();
    return 0;
}
