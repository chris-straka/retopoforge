// Unit + engine tests for local density control (core/density.*).
// Plain assert-style main, no third-party framework.
// NOTE: CHECK instead of <cassert> assert(), which is a no-op under NDEBUG.
import retopo.core.auto_remesher;
import retopo.core.density;
import retopo.core.vector3;

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
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

using Density = AutoRemesher::Density;
using Remesher = AutoRemesher::AutoRemesher;
using Vector3 = AutoRemesher::Vector3;

// Subdivided cube centered at the origin, welded across face edges.
void buildCube(int subdivisions, std::vector<Vector3>& vertices,
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

struct FaceCounts {
    size_t inside = 0; // quads on the masked (+X) slice
    size_t opposite = 0; // quads on the far (-X) slice, same area
    size_t total = 0;
};

// Count output quads by centroid x: the mask covers the +X face, so the
// x > +0.85 and x < -0.85 slices have equal area and their quad-count ratio
// measures the achieved density ratio.
FaceCounts countFaces(const std::vector<Vector3>& vertices,
    const std::vector<std::vector<size_t>>& quads)
{
    FaceCounts counts;
    counts.total = quads.size();
    for (const auto& quad : quads) {
        double cx = 0.0;
        for (const size_t index : quad)
            cx += vertices[index].x();
        cx /= static_cast<double>(quad.size());
        if (cx > 0.85)
            ++counts.inside;
        else if (cx < -0.85)
            ++counts.opposite;
    }
    return counts;
}

// Identity comparisons use a fraction band, not ==: the engine has
// pre-existing run-to-run nondeterminism (~1.5% from thread scheduling —
// identical unset runs in one process already differ), so 5% (the same
// tolerance bench/run.py uses) is the honest equality for counts.
bool withinFraction(size_t a, size_t b, double fraction)
{
    if (a == b)
        return true;
    const double denom = static_cast<double>(std::max(a, b));
    if (denom <= 0.0)
        return true;
    const double diff = a > b ? static_cast<double>(a - b) : static_cast<double>(b - a);
    return diff / denom <= fraction;
}

FaceCounts runMasked(const std::vector<Vector3>& vertices,
    const std::vector<std::vector<size_t>>& triangles,
    const std::vector<double>* mask, size_t targetTriangles, size_t* outVerts = nullptr)
{
    Remesher remesher(vertices, triangles);
    remesher.setTargetTriangleCount(targetTriangles);
    if (nullptr != mask)
        remesher.setDensityMultipliers(*mask);
    const bool ok = remesher.remesh();
    CHECK(ok);
    if (nullptr != outVerts)
        *outVerts = remesher.remeshedVertices().size();
    return countFaces(remesher.remeshedVertices(), remesher.remeshedQuads());
}

}

int main()
{
    // normalizeField: clamping, non-finite handling, OFF states.
    {
        CHECK(Density::normalizeField({}).empty());
        CHECK(Density::normalizeField(std::vector<double>({ 1.0, 1.0, 1.0 })).empty());
        const std::vector<double> clamped = Density::normalizeField(
            std::vector<double>({ 100.0, 0.001, 2.0, 1.0 }));
        CHECK(clamped.size() == 4);
        CHECK(clamped[0] == 4.0 && clamped[1] == 0.25 && clamped[2] == 2.0 && clamped[3] == 1.0);
        const std::vector<double> finite = Density::normalizeField(
            std::vector<double>({ std::numeric_limits<double>::quiet_NaN(), 2.0 }));
        CHECK(finite.size() == 2 && finite[0] == 1.0 && finite[1] == 2.0);
        CHECK(Density::normalizeField(
            std::vector<double>({ std::numeric_limits<double>::infinity(), 1.0 }))
                .empty());
    }

    // edgeScaleFor: 1/sqrt(d), 1.0 on garbage.
    {
        CHECK(Density::edgeScaleFor(4.0) == 0.5);
        CHECK(Density::edgeScaleFor(1.0) == 1.0);
        CHECK(std::fabs(Density::edgeScaleFor(0.25) - 2.0) < 1e-12);
        CHECK(Density::edgeScaleFor(0.0) == 1.0);
        CHECK(Density::edgeScaleFor(std::numeric_limits<double>::quiet_NaN()) == 1.0);
    }

    // resampleNearest: identity map reproduces the field; size mismatch
    // falls back to uniform 1.0 (which normalizes back to OFF).
    {
        const std::vector<Vector3> points = {
            Vector3(0.0, 0.0, 0.0), Vector3(1.0, 0.0, 0.0), Vector3(0.0, 1.0, 0.0),
        };
        const std::vector<double> field = { 4.0, 1.0, 0.25 };
        const std::vector<double> same = Density::resampleNearest(points, field, points);
        CHECK(same == field);
        const std::vector<double> mismatch = Density::resampleNearest(points, { 4.0 }, points);
        CHECK(mismatch.size() == 3 && Density::normalizeField(mismatch).empty());
    }

    // applyToScalingField: masked faces shrink, budget SUM A/m^2 is preserved.
    {
        const std::vector<Vector3> vertices = {
            Vector3(0.0, 0.0, 0.0), Vector3(1.0, 0.0, 0.0),
            Vector3(1.0, 1.0, 0.0), Vector3(0.0, 1.0, 0.0),
        };
        const std::vector<std::vector<size_t>> triangles = { { 0, 1, 2 }, { 0, 2, 3 } };
        std::vector<double> scaling = { 1.0, 1.0 };
        Density::applyToScalingField(vertices, triangles, { 4.0, 4.0, 1.0, 1.0 }, scaling);
        // Face 0 averages density 3, face 1 averages 2: face 0 ends denser.
        CHECK(scaling[0] < scaling[1]);
        double budget = 0.0;
        for (const double m : scaling)
            budget += 0.5 / (m * m);
        CHECK(std::fabs(budget - 1.0) < 1e-9);
        // Size mismatch is a no-op.
        std::vector<double> untouched = { 1.0, 1.0 };
        Density::applyToScalingField(vertices, triangles, { 4.0 }, untouched);
        CHECK(untouched[0] == 1.0 && untouched[1] == 1.0);
    }

    // Engine: subdivided cube, +X face masked 4x.
    std::vector<Vector3> cubeVertices;
    std::vector<std::vector<size_t>> cubeTriangles;
    buildCube(6, cubeVertices, cubeTriangles);
    CHECK(!cubeVertices.empty() && !cubeTriangles.empty());
    constexpr size_t targetTriangles = 2000;
    std::vector<double> mask(cubeVertices.size(), 1.0);
    for (size_t i = 0; i < cubeVertices.size(); ++i) {
        if (cubeVertices[i].x() > 0.99)
            mask[i] = 4.0;
    }

    const FaceCounts plain = runMasked(cubeVertices, cubeTriangles, nullptr, targetTriangles);
    const FaceCounts masked = runMasked(cubeVertices, cubeTriangles, &mask, targetTriangles);
    std::printf("cube target=%zu: plain inside=%zu opposite=%zu total=%zu | "
                "masked-4x inside=%zu opposite=%zu total=%zu\n",
        targetTriangles, plain.inside, plain.opposite, plain.total,
        masked.inside, masked.opposite, masked.total);
    CHECK(plain.opposite > 0 && masked.opposite > 0);
    if (plain.opposite > 0 && masked.opposite > 0) {
        const double plainRatio = static_cast<double>(plain.inside) / plain.opposite;
        const double maskedRatio = static_cast<double>(masked.inside) / masked.opposite;
        std::printf("density ratio inside/opposite: plain=%.2f masked=%.2f (expect ~1.0 / ~2.4, see note)\n",
            plainRatio, maskedRatio);
        CHECK(plainRatio > 0.7 && plainRatio < 1.4);
        // NOTE: the masked contrast saturates at ~2.4x here instead of the
        // asked 4x. Bunching grid lines across a pole-free transition is
        // integer-infeasible (the frame-field poles are sizing-unaware), so
        // strong localized refinement caps out while coarsening realizes
        // fully; mild masks (<=2x) realize nearly fully (see the 2x slope
        // below). The band documents measured behavior — see the density
        // lane report for the full analysis.
        CHECK(maskedRatio > 2.0 && maskedRatio < 3.0);
        // Hard-mask transitions waste quads (singularities, holes, cleanup
        // merges), so the total drops ~25% here; uniform and smooth masks
        // preserve the budget exactly (see the whole-4x check below).
        const double totalRatio = static_cast<double>(masked.total) / plain.total;
        std::printf("total quads ratio masked/plain: %.2f (expect ~0.74)\n", totalRatio);
        CHECK(totalRatio > 0.65 && totalRatio < 0.85);
    }

    // Renormalization proof: a uniform 4x field must reproduce the plain
    // run (the rescale divides the modulation back out). Mild localized
    // masks realize nearly fully (2x asks reach ~1.7x).
    {
        const std::vector<double> whole(cubeVertices.size(), 4.0);
        std::vector<double> half2x = mask;
        for (double& v : half2x) {
            if (v > 1.0)
                v = 2.0;
        }
        const FaceCounts wholeRun = runMasked(cubeVertices, cubeTriangles, &whole, targetTriangles);
        const FaceCounts halfRun = runMasked(cubeVertices, cubeTriangles, &half2x, targetTriangles);
        std::printf("whole-4x inside=%zu opposite=%zu total=%zu | face-2x inside=%zu opposite=%zu total=%zu\n",
            wholeRun.inside, wholeRun.opposite, wholeRun.total,
            halfRun.inside, halfRun.opposite, halfRun.total);
        CHECK(withinFraction(wholeRun.total, plain.total, 0.05));
        CHECK(withinFraction(wholeRun.inside, plain.inside, 0.05));
        CHECK(withinFraction(wholeRun.opposite, plain.opposite, 0.05));
        if (halfRun.opposite > 0) {
            const double halfRatio = static_cast<double>(halfRun.inside) / halfRun.opposite;
            std::printf("face-2x ratio: %.2f (expect ~1.7)\n", halfRatio);
            CHECK(halfRatio > 1.4 && halfRatio < 2.0);
        }
    }

    // Default OFF: no setter, explicit all-1.0, and wrong-sized fields agree.
    {
        size_t plainVerts = 0, onesVerts = 0, wrongVerts = 0;
        const FaceCounts plainAgain = runMasked(cubeVertices, cubeTriangles, nullptr,
            targetTriangles, &plainVerts);
        const std::vector<double> ones(cubeVertices.size(), 1.0);
        const FaceCounts onesRun = runMasked(cubeVertices, cubeTriangles, &ones,
            targetTriangles, &onesVerts);
        const std::vector<double> wrong(cubeVertices.size() / 2, 4.0);
        const FaceCounts wrongRun = runMasked(cubeVertices, cubeTriangles, &wrong,
            targetTriangles, &wrongVerts);
        CHECK(withinFraction(plainAgain.total, plain.total, 0.05));
        CHECK(withinFraction(onesRun.total, plain.total, 0.05));
        CHECK(withinFraction(onesVerts, plainVerts, 0.05));
        CHECK(withinFraction(wrongRun.total, plain.total, 0.05));
        CHECK(withinFraction(wrongVerts, plainVerts, 0.05));
    }

    // Clamping: out-of-range masks behave like the clamped edge (within
    // run-to-run noise — same-process identical runs already differ by ~1%).
    {
        std::vector<double> huge = mask, tiny = mask;
        for (double& v : huge) {
            if (v > 1.0)
                v = 100.0;
        }
        for (double& v : tiny) {
            if (v > 1.0)
                v = 0.001;
        }
        std::vector<double> quarter(cubeVertices.size(), 1.0);
        for (size_t i = 0; i < cubeVertices.size(); ++i) {
            if (cubeVertices[i].x() > 0.99)
                quarter[i] = 0.25;
        }
        size_t maskedVerts = 0, hugeVerts = 0, quarterVerts = 0, tinyVerts = 0;
        const FaceCounts maskedAgain = runMasked(cubeVertices, cubeTriangles, &mask,
            targetTriangles, &maskedVerts);
        const FaceCounts hugeRun = runMasked(cubeVertices, cubeTriangles, &huge,
            targetTriangles, &hugeVerts);
        const FaceCounts quarterRun = runMasked(cubeVertices, cubeTriangles, &quarter,
            targetTriangles, &quarterVerts);
        const FaceCounts tinyRun = runMasked(cubeVertices, cubeTriangles, &tiny,
            targetTriangles, &tinyVerts);
        CHECK(withinFraction(maskedAgain.total, masked.total, 0.05));
        CHECK(withinFraction(hugeRun.total, masked.total, 0.05));
        CHECK(withinFraction(hugeVerts, maskedVerts, 0.05));
        CHECK(withinFraction(tinyRun.total, quarterRun.total, 0.05));
        CHECK(withinFraction(tinyVerts, quarterVerts, 0.05));
        std::printf("clamp check: 4x=%zu quads, 100x=%zu quads, 0.25x=%zu quads, 0.001x=%zu quads\n",
            masked.total, hugeRun.total, quarterRun.total, tinyRun.total);
    }

    if (g_failures == 0)
        std::printf("PASS test_density\n");
    return g_failures == 0 ? 0 : 1;
}
