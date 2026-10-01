// Differential oracle dump for the guides Rust port (wave 3, rs-guides
// lane). Generates hand-built adversarial cases (degenerate-threshold
// straddles, the 0.5 projection cliff, equidistant first-wins, exact
// radius ties, NaN/inf robustness) plus seeded random mesh-relative
// queries, evaluates Guides::influenceRadius + Guides::tangentNear with
// the C++ engine, and prints inputs + outputs in a token format that
// rust/core/tests/guides_diff.rs replays. Also times a formula-only
// (libm-free, bit-identical on both sides) query batch for the runtime
// ratio (3 samples).
//
// Build-only helper: not registered with ctest. Run it and redirect stdout
// to tests/fixtures/guides_diff.txt, then commit the fixture.
import retopo.core.guides;
import retopo.core.surface_mesh;
import retopo.core.vector3;

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <vector>

using AutoRemesher::Guides;
using AutoRemesher::SurfaceMesh;
using AutoRemesher::Vector3;

// splitmix64: the edge cases below are fixed, the random cases draw from
// this stream with a fixed seed, so the fixture is reproducible.
static std::uint64_t g_state = 0x61DE5A11CE5EED11ull;

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

static void dumpCase(const std::vector<Vector3>& positions,
    const std::vector<std::vector<size_t>>& triangles,
    const std::vector<std::vector<Vector3>>& guides,
    const Vector3& point, const Vector3& normal, double radius)
{
    const SurfaceMesh mesh(positions, triangles);
    const double influence = Guides::influenceRadius(mesh);
    const Vector3 tangent = Guides::tangentNear(guides, point, normal, radius);
    std::printf("CASE %zu %zu %zu\n", positions.size(), triangles.size(), guides.size());
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
    for (const auto& polyline : guides) {
        std::printf("g %zu", polyline.size());
        for (const auto& p : polyline) {
            printDouble(p.x());
            printDouble(p.y());
            printDouble(p.z());
        }
        std::printf("\n");
    }
    std::printf("q");
    printDouble(point.x());
    printDouble(point.y());
    printDouble(point.z());
    printDouble(normal.x());
    printDouble(normal.y());
    printDouble(normal.z());
    printDouble(radius);
    std::printf("\nR");
    printDouble(influence);
    std::printf("\nTAN");
    printDouble(tangent.x());
    printDouble(tangent.y());
    printDouble(tangent.z());
    std::printf("\n");
}

static const std::vector<Vector3> kTriPositions = {
    Vector3(0, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0)
};
static const std::vector<std::vector<size_t>> kTriFaces = { { 0, 1, 2 } };

static void dumpEdgeCases()
{
    const Vector3 o(0, 0, 0);
    const Vector3 x(1, 0, 0);
    const Vector3 y(0, 1, 0);
    const Vector3 z(0, 0, 1);
    // 1. Empty mesh -> influence 0; empty guides -> zero tangent.
    dumpCase({}, {}, {}, o, z, 1.0);
    // 2. Single triangle; no guides at all.
    dumpCase(kTriPositions, kTriFaces, {}, Vector3(0.25, 0.25, 0), z, 6.0);
    // 3. Polylines with < 2 points carry no direction.
    dumpCase(kTriPositions, kTriFaces, { {}, { x }, { y } }, o, z, 6.0);
    // 4. All segments zero-length (duplicate points).
    dumpCase(kTriPositions, kTriFaces, { { x, x, x }, { y, y } }, x, z, 6.0);
    // 5. Segment just below the 1e-12 degenerate threshold -> skipped.
    dumpCase(kTriPositions, kTriFaces, { { o, Vector3(9e-13, 0, 0) } }, o, z, 6.0);
    // 6. Segment just above the threshold, point on it -> selected.
    dumpCase(kTriPositions, kTriFaces, { { o, Vector3(1.1e-12, 0, 0) } }, o, z, 6.0);
    // 7. Non-positive radius is always zero (0, negative, NaN).
    const std::vector<std::vector<Vector3>> seg = { { o, x } };
    dumpCase(kTriPositions, kTriFaces, seg, Vector3(0.5, 0, 0), z, 0.0);
    dumpCase(kTriPositions, kTriFaces, seg, Vector3(0.5, 0, 0), z, -2.0);
    dumpCase(kTriPositions, kTriFaces, seg, Vector3(0.5, 0, 0), z,
        std::numeric_limits<double>::quiet_NaN());
    // 10. Min-subnormal radius: radius^2 underflows to 0, only an exact
    // zero-distance hit (strict < cannot hold) ... still zero either way.
    dumpCase(kTriPositions, kTriFaces, seg, Vector3(0.5, 0, 0), z, 5e-324);
    // 11. Point exactly on the segment middle, normal perpendicular ->
    // tangent is the segment direction.
    dumpCase(kTriPositions, kTriFaces, seg, Vector3(0.5, 0, 0), z, 6.0);
    // 12. Equidistant symmetric pair: strict < keeps the FIRST segment.
    dumpCase(kTriPositions, kTriFaces,
        { { Vector3(1, 1, 0), Vector3(-1, 1, 0) }, { Vector3(1, -1, 0), Vector3(-1, -1, 0) } },
        o, z, 6.0);
    // 13. Nearest is the second segment, not the first.
    dumpCase(kTriPositions, kTriFaces,
        { { Vector3(5, 0, 0), Vector3(6, 0, 0) }, { o, x } },
        Vector3(0.5, 0.1, 0), z, 6.0);
    // 14. Normal parallel to the direction: projection vanishes -> zero.
    dumpCase(kTriPositions, kTriFaces, seg, Vector3(0.5, 0, 0), x, 6.0);
    // 15. Zero normal: tangent is the (unit) direction, normalized.
    dumpCase(kTriPositions, kTriFaces, seg, Vector3(0.5, 0, 0), o, 6.0);
    // 16. Non-unit normal: deterministic, same ops both sides.
    dumpCase(kTriPositions, kTriFaces, seg, Vector3(0.5, 0, 0), Vector3(0, 0, 3), 6.0);
    // 17-19. The 0.5 projection cliff: |tangent| is intended to be exactly
    // 0.5 (|d - (d.n)n| with d=(1,0,0), n=(sqrt(3)/2, 1/2, 0)); FP lands
    // within 1 ulp and both sides must agree. Perturbations straddle it.
    const double s3 = std::sqrt(3.0) / 2.0;
    dumpCase(kTriPositions, kTriFaces, seg, Vector3(0.5, 0, 0), Vector3(s3, 0.5, 0), 6.0);
    dumpCase(kTriPositions, kTriFaces, seg, Vector3(0.5, 0, 0),
        Vector3(s3 * (1.0 + 2e-13), 0.5, 0), 6.0);
    dumpCase(kTriPositions, kTriFaces, seg, Vector3(0.5, 0, 0),
        Vector3(s3 * (1.0 - 2e-13), 0.5, 0), 6.0);
    // 20-21. along clamping: point past the end / before the start.
    dumpCase(kTriPositions, kTriFaces, seg, Vector3(3, 0.5, 0), z, 6.0);
    dumpCase(kTriPositions, kTriFaces, seg, Vector3(-2, 0.5, 0), z, 6.0);
    // 22-23. Exact radius tie: distance exactly 1, radius 1 -> strict <
    // misses -> zero; nextafter up -> selected.
    const std::vector<std::vector<Vector3>> hseg = { { o, Vector3(2, 0, 0) } };
    dumpCase(kTriPositions, kTriFaces, hseg, Vector3(1, 1, 0), z, 1.0);
    dumpCase(kTriPositions, kTriFaces, hseg, Vector3(1, 1, 0), z,
        std::nextafter(1.0, std::numeric_limits<double>::infinity()));
    // 24. Infinite radius: radius^2 is +inf, everything is within.
    dumpCase(kTriPositions, kTriFaces, seg, Vector3(50, 0, 0), z,
        std::numeric_limits<double>::infinity());
    // 25. radius^2 overflows to +inf (1e200).
    dumpCase(kTriPositions, kTriFaces, seg, Vector3(50, 0, 0), z, 1e200);
    // 26. radius^2 is subnormal (1e-200): only a ~zero distance selects.
    dumpCase(kTriPositions, kTriFaces, seg, Vector3(0.5, 0, 0), z, 1e-200);
    dumpCase(kTriPositions, kTriFaces, seg, Vector3(0.5, 1e-100, 0), z, 1e-200);
    // 28. Huge coordinates (1e150): no overflow in the chain.
    dumpCase({ Vector3(1e150, 0, 0), Vector3(0, 1e150, 0), Vector3(0, 0, 1e150) },
        kTriFaces, { { Vector3(1e150, 0, 0), Vector3(0, 1e150, 0) } },
        Vector3(5e149, 5e149, 0), z, 1e150);
    // 29. Tiny coordinates (1e-150): subnormal-adjacent distances.
    dumpCase({ Vector3(1e-150, 0, 0), Vector3(0, 1e-150, 0), Vector3(0, 0, 1e-150) },
        kTriFaces, { { Vector3(1e-150, 0, 0), Vector3(0, 1e-150, 0) } },
        Vector3(5e-151, 5e-151, 0), z, 1e-150);
    // 30. All-coincident mesh: average edge 0 -> influence 0.
    dumpCase({ o, o, o }, kTriFaces, seg, Vector3(0.5, 0, 0), z, 6.0);
    // 31. Guide far outside the radius -> zero.
    dumpCase(kTriPositions, kTriFaces,
        { { Vector3(100, 0, 0), Vector3(101, 0, 0) } }, o, z, 6.0);
    // 32. Closed loop guide (first point repeated).
    dumpCase(kTriPositions, kTriFaces,
        { { x, y, z, x } }, Vector3(0.5, 0.5, 0), Vector3(0, 0, 1), 6.0);
    // 33. Long polyline (200-point circle-ish spiral, exact ints).
    {
        std::vector<Vector3> spiral;
        for (int i = 0; i < 200; ++i)
            spiral.push_back(Vector3(0.05 * (i % 40), 0.05 * (i / 40), (i % 2) * 0.1));
        dumpCase(kTriPositions, kTriFaces, { spiral }, Vector3(1, 0.05, 0.05), z, 6.0);
    }
    // 34. NaN guide coordinate: distances are NaN, never selected -> zero.
    dumpCase(kTriPositions, kTriFaces,
        { { Vector3(std::numeric_limits<double>::quiet_NaN(), 0, 0), x } },
        Vector3(0.5, 0, 0), z, 6.0);
    // 35. NaN query point with a valid nearby segment.
    dumpCase(kTriPositions, kTriFaces, seg,
        Vector3(std::numeric_limits<double>::quiet_NaN(), 0, 0), z, 6.0);
    // 36. NaN normal with a valid nearby segment (NaN-tolerant compare).
    dumpCase(kTriPositions, kTriFaces, seg, Vector3(0.5, 0, 0),
        Vector3(0, 0, std::numeric_limits<double>::quiet_NaN()), 6.0);
    // 37. Inf guide coordinate.
    dumpCase(kTriPositions, kTriFaces,
        { { Vector3(std::numeric_limits<double>::infinity(), 0, 0), x } },
        Vector3(0.5, 0, 0), z, 6.0);
    // 38. Non-triangle mesh faces are skipped by the constructor.
    dumpCase(kTriPositions, { { 0, 1 }, { 0, 1, 2, 0 }, { 0, 1, 2 } }, seg,
        Vector3(0.5, 0, 0), z, 6.0);
    // 39. Tetrahedron mesh (interior edges counted once).
    dumpCase({ o, x, y, z }, { { 0, 2, 1 }, { 0, 1, 3 }, { 0, 3, 2 }, { 1, 2, 3 } },
        seg, Vector3(0.5, 0, 0), z, 6.0);
}

static Vector3 randomPosition()
{
    // Half the time snap to the 0..3 integer lattice so coincident
    // vertices and zero-length segments occur often; otherwise
    // uniform-ish in [-10, 10).
    auto coord = []() {
        if (below(2) == 0)
            return double(below(4));
        return double(nextU64() % 2000) / 100.0 - 10.0;
    };
    return Vector3(coord(), coord(), coord());
}

static std::vector<size_t> randomTriFace(size_t vertexCount)
{
    // In-range triangles only: out-of-range indices would be UB inside
    // averageEdgeLength (unlike the topology-only surface_mesh oracle,
    // influenceRadius always reads positions).
    std::vector<size_t> face;
    for (int i = 0; i < 3; ++i)
        face.push_back(below(vertexCount));
    // 10%: non-triangle arity (skipped by the constructor).
    if (below(10) == 0) {
        static const size_t kArity[] = { 0, 1, 2, 4, 5 };
        face.resize(kArity[below(5)]);
    }
    return face;
}

static Vector3 randomGuidePoint(const std::vector<Vector3>& soFar)
{
    // 12%: repeat the previous point (degenerate segment); 8%: sit
    // ~1e-13 away from it (degenerate-threshold straddle).
    if (!soFar.empty() && below(100) < 12)
        return soFar.back();
    if (!soFar.empty() && below(100) < 8) {
        const Vector3& p = soFar.back();
        return Vector3(p.x() + 1e-13 * double(1 + below(5)),
            p.y() - 1e-13 * double(1 + below(5)), p.z() + 1e-13 * double(1 + below(5)));
    }
    return randomPosition();
}

static void dumpRandomCase()
{
    size_t nv = below(7); // 0..6 vertices.
    std::vector<Vector3> positions;
    for (size_t i = 0; i < nv; ++i)
        positions.push_back(randomPosition());
    size_t nt = nv == 0 ? 0 : below(6); // 0..5 raw faces.
    std::vector<std::vector<size_t>> triangles;
    for (size_t i = 0; i < nt; ++i)
        triangles.push_back(randomTriFace(nv));
    size_t ng = below(4); // 0..3 polylines.
    std::vector<std::vector<Vector3>> guides;
    for (size_t i = 0; i < ng; ++i) {
        size_t np = below(7); // 0..6 points.
        std::vector<Vector3> polyline;
        for (size_t j = 0; j < np; ++j)
            polyline.push_back(randomGuidePoint(polyline));
        guides.push_back(polyline);
    }
    // Query point: uniform, snapped near a guide point, or an exact
    // segment midpoint (distance-0 path).
    Vector3 point = randomPosition();
    size_t totalPoints = 0, totalSegs = 0;
    for (const auto& p : guides) {
        totalPoints += p.size();
        if (p.size() >= 2)
            totalSegs += p.size() - 1;
    }
    const std::uint64_t pick = below(10);
    if (totalPoints > 0 && pick < 2) {
        size_t k = below(totalPoints);
        for (const auto& p : guides) {
            if (k < p.size()) {
                point = Vector3(p[k].x() + 1e-9 * double(1 + below(9)),
                    p[k].y() - 1e-9 * double(1 + below(9)), p[k].z());
                break;
            }
            k -= p.size();
        }
    } else if (totalSegs > 0 && pick < 4) {
        size_t k = below(totalSegs);
        for (const auto& p : guides) {
            if (p.size() >= 2 && k < p.size() - 1) {
                point = (p[k] + p[k + 1]) / 2.0;
                break;
            }
            if (p.size() >= 2)
                k -= p.size() - 1;
        }
    }
    // Normal: unit (downstream-plausible), raw, zero, or parallel to a
    // random segment direction (projection-reject path).
    Vector3 normal = randomPosition().normalized();
    const std::uint64_t npick = below(20);
    if (npick == 0 || npick == 1)
        normal = randomPosition();
    else if (npick == 2)
        normal = Vector3(0, 0, 0);
    else if (npick == 3 && totalSegs > 0) {
        size_t k = below(totalSegs);
        for (const auto& p : guides) {
            if (p.size() >= 2 && k < p.size() - 1) {
                normal = (p[k + 1] - p[k]).normalized();
                break;
            }
            if (p.size() >= 2)
                k -= p.size() - 1;
        }
    }
    // Radius: the downstream pattern (influenceRadius of this mesh) half
    // the time, else mesh-scale random with degenerate/huge outliers.
    double radius;
    if (below(2) == 0) {
        radius = Guides::influenceRadius(SurfaceMesh(positions, triangles));
    } else {
        radius = 0.05 + double(nextU64() % 3000) / 100.0; // [0.05, 30.05).
        const std::uint64_t rpick = below(20);
        if (rpick == 0)
            radius = 0.0;
        else if (rpick == 1)
            radius = -double(1 + below(5));
        else if (rpick == 2)
            radius = 1e-300;
        else if (rpick == 3)
            radius = 1e150;
    }
    dumpCase(positions, triangles, guides, point, normal, radius);
}

// Timing inputs: pure integer formulas (no libm), so both sides build
// bit-identical inputs. The Rust timing test rebuilds them exactly.
static void makeTimingInputs(std::vector<Vector3>& positions,
    std::vector<std::vector<size_t>>& triangles,
    std::vector<std::vector<Vector3>>& guides,
    std::vector<Vector3>& queries, std::vector<Vector3>& normals)
{
    const size_t w = 120, h = 120;
    for (size_t yy = 0; yy <= h; ++yy) {
        for (size_t xx = 0; xx <= w; ++xx) {
            double z = 0.1 * double((xx * 7 + yy * 13) % 5);
            positions.emplace_back(double(xx), double(yy), z);
        }
    }
    auto id = [w](size_t xx, size_t yy) { return yy * (w + 1) + xx; };
    for (size_t yy = 0; yy < h; ++yy) {
        for (size_t xx = 0; xx < w; ++xx)
            triangles.push_back({ id(xx, yy), id(xx + 1, yy), id(xx + 1, yy + 1) }),
                triangles.push_back({ id(xx, yy), id(xx + 1, yy + 1), id(xx, yy + 1) });
    }
    for (size_t line = 0; line < 4; ++line) {
        std::vector<Vector3> polyline;
        for (size_t j = 0; j < 500; ++j)
            polyline.emplace_back(0.5 * double(j), 2.0 * double(j % 2) + double(line),
                1.0 * double(j % 7));
        guides.push_back(polyline);
    }
    for (size_t i = 0; i < 2000; ++i) {
        queries.emplace_back(
            0.25 * double(i % 400), 0.5 * double(i % 11), 0.25 * double(i % 13));
        normals.push_back(
            Vector3(double(i % 5) - 2.0, double(i % 3) - 1.0, 1.0).normalized());
    }
}

static void timeBatch()
{
    std::vector<Vector3> positions;
    std::vector<std::vector<size_t>> triangles;
    std::vector<std::vector<Vector3>> guides;
    std::vector<Vector3> queries, normals;
    makeTimingInputs(positions, triangles, guides, queries, normals);
    const SurfaceMesh mesh(positions, triangles);
    const double influence = Guides::influenceRadius(mesh);
    for (int sample = 0; sample < 3; ++sample) {
        auto t0 = std::chrono::steady_clock::now();
        double checksum = influence;
        for (size_t i = 0; i < queries.size(); ++i) {
            const double radius = (i % 2 == 0) ? influence : 2.0;
            const Vector3 t = Guides::tangentNear(guides, queries[i], normals[i], radius);
            checksum += t.x() + t.y() + t.z();
        }
        auto t1 = std::chrono::steady_clock::now();
        double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        std::printf("T guides queries=%zu influence=%.17g checksum=%.17g ms=%.3f\n",
            queries.size(), influence, checksum, ms);
    }
}

static constexpr int kRandomCases = 220;

int main()
{
    std::printf("GUIDES1\n");
    dumpEdgeCases();
    for (int i = 0; i < kRandomCases; ++i)
        dumpRandomCase();
    timeBatch();
    return 0;
}
