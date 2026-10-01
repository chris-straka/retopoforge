// Differential oracle dump for the density Rust port (wave 2).
// Seeded splitmix64 cases over normalizeField / edgeScaleFor /
// resampleNearest / applyToScalingField, plus adversarial near-degenerate
// inputs (equidistant nearest ties, zero-distance hits, coincident
// sources, zero-area triangles, empty/short faces, out-of-range face
// indices past slot 2, zero/negative/NaN/inf scaling entries, NaN
// coordinates), since several sites branch on FP comparisons.
//
// Build-only helper: not registered with ctest. Run it and redirect stdout
// to tests/fixtures/density_diff.txt, replayed by
// rust/core/tests/density_diff.rs. Doubles print with %.17g (exact
// round-trip, so the replay compares bitwise); NaN/inf print with an
// explicit sign (macOS libc drops the NaN sign).
import retopo.core.density;
import retopo.core.vector3;

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <vector>

static std::uint64_t g_state = 0xD3517A9E0Dull;

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

using AutoRemesher::Density;
using AutoRemesher::Vector3;

static void printDouble(double v)
{
    if (std::isnan(v))
        std::printf("%snan", std::signbit(v) ? "-" : "");
    else if (std::isinf(v))
        std::printf("%sinf", std::signbit(v) ? "-" : "");
    else
        std::printf("%.17g", v);
}

// Small ints/halves promote exact ties/duplicates; wide values exercise
// multi-cell grids. Bounded to +-1e4 so cell coordinates stay in i64
// range even at the 1e-9 minimum cell (1e4/1e-9 = 1e13 << 9e18;
// out-of-range conversion is UB in C++).
static double randCoord()
{
    switch (below(4)) {
    case 0:
        return static_cast<double>(static_cast<int>(below(9)) - 4);
    case 1:
        return (static_cast<int>(below(41)) - 20) / 2.0;
    case 2:
        return (static_cast<double>(nextU64() % 2000000) - 1000000.0) / 100.0;
    default:
        return (static_cast<double>(nextU64() % 2000000) - 1000000.0) / 100.0;
    }
}

static Vector3 randPoint()
{
    return Vector3(randCoord(), randCoord(), randCoord());
}

// Density-ish values across the clamp range, with exact edges and garbage.
static double randDensity()
{
    switch (below(12)) {
    case 0:
        return 1.0;
    case 1:
        return 0.25;
    case 2:
        return 4.0;
    case 3:
        return std::numeric_limits<double>::quiet_NaN();
    case 4:
        return std::numeric_limits<double>::infinity();
    default:
        return (static_cast<double>(nextU64() % 600) - 100.0) / 100.0;
    }
}

// Scaling entries: mostly positive, sometimes zero/negative/garbage.
static double randScale()
{
    switch (below(12)) {
    case 0:
        return 1.0;
    case 1:
        return 0.0;
    case 2:
        return -1.0;
    case 3:
        return std::numeric_limits<double>::quiet_NaN();
    case 4:
        return std::numeric_limits<double>::infinity();
    default:
        return (static_cast<double>(nextU64() % 500) - 150.0) / 100.0;
    }
}

static void dumpNormalize(int count)
{
    for (int i = 0; i < count; ++i) {
        const size_t len = static_cast<size_t>(below(9));
        std::vector<double> field(len);
        for (size_t k = 0; k < len; ++k)
            field[k] = randDensity();
        std::printf("N %zu", len);
        for (double v : field) {
            std::printf(" ");
            printDouble(v);
        }
        std::printf("\n");
        const std::vector<double> out = Density::normalizeField(field);
        std::printf("r %zu", out.size());
        for (double v : out) {
            std::printf(" ");
            printDouble(v);
        }
        std::printf("\n");
    }
}

static void dumpEdge(int count)
{
    for (int i = 0; i < count; ++i) {
        const double v = randDensity();
        std::printf("E ");
        printDouble(v);
        std::printf("\n");
        std::printf("r ");
        printDouble(Density::edgeScaleFor(v));
        std::printf("\n");
    }
}

static void dumpResample(int count)
{
    for (int i = 0; i < count; ++i) {
        size_t nsrc = static_cast<size_t>(1 + below(10));
        size_t ndst = static_cast<size_t>(1 + below(5));
        if (below(16) == 0)
            nsrc = 0;
        if (below(16) == 0)
            ndst = 0;
        std::vector<Vector3> src(nsrc);
        for (size_t k = 0; k < nsrc; ++k) {
            // Exact duplicates exercise nearest-tie order.
            if (k > 0 && below(6) == 0)
                src[k] = src[static_cast<size_t>(below(k))];
            else
                src[k] = randPoint();
        }
        size_t nfield = nsrc;
        if (below(8) == 0)
            nfield = nsrc > 0 ? static_cast<size_t>(below(nsrc + 2)) : 1;
        std::vector<double> field(nfield);
        for (size_t k = 0; k < nfield; ++k)
            field[k] = randDensity();
        std::vector<Vector3> dst(ndst);
        for (size_t k = 0; k < ndst; ++k) {
            if (nsrc > 0 && below(5) == 0) {
                // Exact hit: zero-distance early-out.
                dst[k] = src[static_cast<size_t>(below(nsrc))];
            } else if (nsrc >= 2 && below(8) == 0) {
                // Exact midpoint: equidistant-tie order.
                const Vector3& a = src[static_cast<size_t>(below(nsrc))];
                const Vector3& b = src[static_cast<size_t>(below(nsrc))];
                dst[k] = Vector3((a.x() + b.x()) / 2.0, (a.y() + b.y()) / 2.0,
                    (a.z() + b.z()) / 2.0);
            } else {
                dst[k] = randPoint();
            }
        }
        std::printf("R %zu %zu %zu\n", nsrc, nfield, ndst);
        for (const Vector3& p : src) {
            printDouble(p.x());
            std::printf(" ");
            printDouble(p.y());
            std::printf(" ");
            printDouble(p.z());
            std::printf(" ");
        }
        std::printf("\n");
        for (double v : field) {
            printDouble(v);
            std::printf(" ");
        }
        std::printf("\n");
        for (const Vector3& p : dst) {
            printDouble(p.x());
            std::printf(" ");
            printDouble(p.y());
            std::printf(" ");
            printDouble(p.z());
            std::printf(" ");
        }
        std::printf("\n");
        const std::vector<double> out = Density::resampleNearest(src, field, dst);
        std::printf("r %zu", out.size());
        for (double v : out) {
            std::printf(" ");
            printDouble(v);
        }
        std::printf("\n");
    }
}

static void dumpApply(int count)
{
    for (int i = 0; i < count; ++i) {
        size_t nv = static_cast<size_t>(3 + below(6));
        size_t nt = static_cast<size_t>(1 + below(5));
        if (below(16) == 0)
            nv = 0;
        if (below(16) == 0)
            nt = 0;
        std::vector<Vector3> verts(nv);
        for (size_t k = 0; k < nv; ++k)
            verts[k] = randPoint();
        std::vector<std::vector<size_t>> tris(nt);
        for (size_t k = 0; k < nt; ++k) {
            size_t len = 3;
            switch (below(8)) {
            case 0:
                len = 0;
                break;
            case 1:
                len = 1;
                break;
            case 2:
                len = 2;
                break;
            case 3:
                len = 4;
                break;
            case 4:
                len = 5;
                break;
            default:
                break;
            }
            tris[k].resize(len);
            for (size_t s = 0; s < len; ++s) {
                // Slots 0-2 feed the unchecked area read, so they stay in
                // range; slot 3+ feeds only the guarded average, so it may
                // run past the end (contract: caller averages get 1.0).
                if (nv > 0 && (s < 3 || below(2) == 0))
                    tris[k][s] = static_cast<size_t>(below(nv));
                else
                    tris[k][s] = nv + static_cast<size_t>(below(3));
            }
        }
        size_t ndens = nv, nscal = nt;
        if (below(8) == 0)
            ndens = nv > 0 ? static_cast<size_t>(below(nv + 2)) : 1;
        if (below(8) == 0)
            nscal = nt > 0 ? static_cast<size_t>(below(nt + 2)) : 1;
        std::vector<double> dens(ndens), scal(nscal);
        for (size_t k = 0; k < ndens; ++k)
            dens[k] = randDensity();
        for (size_t k = 0; k < nscal; ++k)
            scal[k] = randScale();
        std::printf("A %zu %zu %zu %zu\n", nv, nt, ndens, nscal);
        for (const Vector3& p : verts) {
            printDouble(p.x());
            std::printf(" ");
            printDouble(p.y());
            std::printf(" ");
            printDouble(p.z());
            std::printf(" ");
        }
        std::printf("\n");
        for (const auto& t : tris) {
            std::printf("t %zu", t.size());
            for (size_t v : t)
                std::printf(" %zu", v);
            std::printf("\n");
        }
        for (double v : dens) {
            printDouble(v);
            std::printf(" ");
        }
        std::printf("\n");
        for (double v : scal) {
            printDouble(v);
            std::printf(" ");
        }
        std::printf("\n");
        Density::applyToScalingField(verts, tris, dens, scal);
        std::printf("r %zu", scal.size());
        for (double v : scal) {
            std::printf(" ");
            printDouble(v);
        }
        std::printf("\n");
    }
}

static void dumpNormalizeOne(const std::vector<double>& field)
{
    std::printf("N %zu", field.size());
    for (double v : field) {
        std::printf(" ");
        printDouble(v);
    }
    std::printf("\n");
    const std::vector<double> out = Density::normalizeField(field);
    std::printf("r %zu", out.size());
    for (double v : out) {
        std::printf(" ");
        printDouble(v);
    }
    std::printf("\n");
}

static void dumpEdgeOne(double v)
{
    std::printf("E ");
    printDouble(v);
    std::printf("\n");
    std::printf("r ");
    printDouble(Density::edgeScaleFor(v));
    std::printf("\n");
}

static void dumpResampleOne(const std::vector<Vector3>& src,
    const std::vector<double>& field, const std::vector<Vector3>& dst)
{
    std::printf("R %zu %zu %zu\n", src.size(), field.size(), dst.size());
    for (const Vector3& p : src) {
        printDouble(p.x());
        std::printf(" ");
        printDouble(p.y());
        std::printf(" ");
        printDouble(p.z());
        std::printf(" ");
    }
    std::printf("\n");
    for (double v : field) {
        printDouble(v);
        std::printf(" ");
    }
    std::printf("\n");
    for (const Vector3& p : dst) {
        printDouble(p.x());
        std::printf(" ");
        printDouble(p.y());
        std::printf(" ");
        printDouble(p.z());
        std::printf(" ");
    }
    std::printf("\n");
    const std::vector<double> out = Density::resampleNearest(src, field, dst);
    std::printf("r %zu", out.size());
    for (double v : out) {
        std::printf(" ");
        printDouble(v);
    }
    std::printf("\n");
}

static void dumpApplyOne(const std::vector<Vector3>& verts,
    const std::vector<std::vector<size_t>>& tris,
    const std::vector<double>& dens, std::vector<double> scal)
{
    std::printf("A %zu %zu %zu %zu\n", verts.size(), tris.size(), dens.size(),
        scal.size());
    for (const Vector3& p : verts) {
        printDouble(p.x());
        std::printf(" ");
        printDouble(p.y());
        std::printf(" ");
        printDouble(p.z());
        std::printf(" ");
    }
    std::printf("\n");
    for (const auto& t : tris) {
        std::printf("t %zu", t.size());
        for (size_t v : t)
            std::printf(" %zu", v);
        std::printf("\n");
    }
    for (double v : dens) {
        printDouble(v);
        std::printf(" ");
    }
    std::printf("\n");
    for (double v : scal) {
        printDouble(v);
        std::printf(" ");
    }
    std::printf("\n");
    Density::applyToScalingField(verts, tris, dens, scal);
    std::printf("r %zu", scal.size());
    for (double v : scal) {
        std::printf(" ");
        printDouble(v);
    }
    std::printf("\n");
}

static void dumpAdversarial()
{
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    const double denorm = std::numeric_limits<double>::denorm_min();
    const double dmax = std::numeric_limits<double>::max();

    // normalizeField edges: OFF states, clamp boundaries, garbage.
    dumpNormalizeOne({});
    dumpNormalizeOne({ 1.0, 1.0, 1.0, 1.0, 1.0 });
    dumpNormalizeOne({ nan, 1.0 });
    dumpNormalizeOne({ inf });
    dumpNormalizeOne({ -inf, -inf });
    dumpNormalizeOne({ 0.25 - 1e-15, 4.0 + 1e-12, 0.0, -0.0, -3.0, 1e300 });
    dumpNormalizeOne({ 0.25, 4.0, 1.0 });

    // edgeScaleFor edges: exact, garbage, extremes.
    for (double v : { 0.25, 1.0, 4.0, 2.0, 3.0, 0.0, -0.0, -1.0, 100.0, 0.001,
             0.25 - 1e-15, 4.0 + 1e-12, denorm, dmax, inf, -inf, nan, -nan })
        dumpEdgeOne(v);

    // resampleNearest: exact-duplicate sources with distinct values pin the
    // first-wins tie order; exact midpoints pin equidistant order.
    dumpResampleOne({ Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0) },
        { 2.0, 3.0 }, { Vector3(0.0, 0.0, 0.0) });
    dumpResampleOne({ Vector3(-1.0, 0.0, 0.0), Vector3(1.0, 0.0, 0.0) },
        { 2.0, 3.0 }, { Vector3(0.0, 0.0, 0.0) });
    // Ulp-off midpoints: the FP tie-break, not the exact one.
    for (int e = -52; e <= -48; ++e) {
        const double off = std::ldexp(1.0, e);
        dumpResampleOne({ Vector3(-1.0, 0.0, 0.0), Vector3(1.0, 0.0, 0.0) },
            { 2.0, 3.0 }, { Vector3(off, 0.0, 0.0), Vector3(-off, 0.0, 0.0) });
    }
    // All-coincident sources: zero diagonal, 1e-9 cell, tiny ring walk.
    dumpResampleOne(
        { Vector3(1.0, 2.0, 3.0), Vector3(1.0, 2.0, 3.0), Vector3(1.0, 2.0, 3.0) },
        { 2.0, 3.0, 4.0 },
        { Vector3(1.0, 2.0, 3.0), Vector3(1.000000001, 2.0, 3.0) });
    // Single source: everything maps to it.
    dumpResampleOne({ Vector3(5.0, -5.0, 0.5) }, { 0.5 },
        { Vector3(0.0, 0.0, 0.0), Vector3(100.0, 100.0, 100.0) });
    // Empty source / empty destination.
    dumpResampleOne({}, {}, { Vector3(0.0, 0.0, 0.0) });
    dumpResampleOne({ Vector3(0.0, 0.0, 0.0) }, { 2.0 }, {});
    // NaN passthrough: field values copy verbatim, no arithmetic.
    dumpResampleOne({ Vector3(0.0, 0.0, 0.0), Vector3(10.0, 0.0, 0.0) },
        { nan, inf }, { Vector3(0.0, 0.0, 0.0), Vector3(10.0, 0.0, 0.0) });
    // NaN/inf coordinates: min/max propagation + saturating cell cast.
    dumpResampleOne({ Vector3(nan, 0.0, 0.0), Vector3(1.0, 1.0, 1.0) },
        { 2.0, 3.0 }, { Vector3(0.0, 0.0, 0.0), Vector3(nan, nan, nan) });
    dumpResampleOne({ Vector3(inf, 0.0, 0.0), Vector3(1.0, 1.0, 1.0) },
        { 2.0, 3.0 }, { Vector3(0.0, 0.0, 0.0) });

    // applyToScalingField: zero-area faces (collinear, duplicated verts).
    const std::vector<Vector3> line = {
        Vector3(0.0, 0.0, 0.0),
        Vector3(1.0, 0.0, 0.0),
        Vector3(2.0, 0.0, 0.0),
        Vector3(0.0, 1.0, 0.0),
    };
    dumpApplyOne(line, { { 0, 1, 2 }, { 0, 1, 3 } }, { 4.0, 4.0, 1.0, 1.0 },
        { 1.0, 1.0 });
    dumpApplyOne(line, { { 0, 0, 0 }, { 0, 1, 3 } }, { 4.0, 4.0, 1.0, 1.0 },
        { 1.0, 1.0 });
    // All faces zero-area: zero budget, no-op (scaling untouched).
    dumpApplyOne(line, { { 0, 1, 2 }, { 0, 0, 0 } }, { 4.0, 4.0, 1.0, 1.0 },
        { 1.0, 1.0 });
    // Empty / short faces: 0/0 density ratio falls back to scale 1.0.
    dumpApplyOne(line, { {}, { 0 }, { 0, 1 }, { 0, 1, 3 } },
        { 4.0, 4.0, 1.0, 1.0 }, { 1.0, 1.0, 1.0, 1.0 });
    // Out-of-range slots past index 2 average as 1.0.
    dumpApplyOne(line, { { 0, 1, 3, 99, 100 }, { 0, 1 }, { 7 } },
        { 4.0, 4.0, 1.0, 1.0 }, { 1.0, 1.0, 1.0 });
    // Zero / negative / garbage scaling entries.
    dumpApplyOne(line, { { 0, 1, 3 }, { 0, 1, 3 } }, { 4.0, 4.0, 1.0, 1.0 },
        { 0.0, 1.0 });
    dumpApplyOne(line, { { 0, 1, 3 }, { 0, 1, 3 } }, { 4.0, 4.0, 1.0, 1.0 },
        { -2.0, -0.0 });
    dumpApplyOne(line, { { 0, 1, 3 }, { 0, 1, 3 } }, { 4.0, 4.0, 1.0, 1.0 },
        { nan, inf });
    dumpApplyOne(line, { { 0, 1, 3 }, { 0, 1, 3 } }, { 4.0, 4.0, 1.0, 1.0 },
        { 0.0, 0.0 });
    // NaN density averages to scale 1.0 for that face.
    dumpApplyOne(line, { { 0, 1, 3 }, { 0, 1, 3 } }, { nan, 4.0, 1.0, 1.0 },
        { 1.0, 1.0 });
    // Size mismatches and empties: no-op, scaling untouched.
    dumpApplyOne(line, { { 0, 1, 3 } }, { 4.0 }, { 1.0 });
    dumpApplyOne(line, { { 0, 1, 3 } }, { 4.0, 4.0, 1.0, 1.0 }, { 1.0, 2.0 });
    dumpApplyOne({}, {}, {}, {});
    dumpApplyOne({}, { { 0, 1, 2 } }, {}, { 1.0 });
}

static constexpr std::uint64_t kTimingSeed = 0xDE4517A11CULL;

// Timing stream: deliberately simple draws (identical order both sides)
// so the Rust timing test regenerates bit-identical inputs.
static double tdouble()
{
    return (static_cast<double>(nextU64() % 20001) - 10000.0) / 100.0;
}

static double tdensity()
{
    return (static_cast<double>(nextU64() % 401) + 25.0) / 100.0;
}

static double timeDensityLoop()
{
    g_state = kTimingSeed;
    const size_t nsrc = 2048, ndst = 2048;
    std::vector<Vector3> src(nsrc), dst(ndst);
    std::vector<double> field(nsrc);
    for (size_t i = 0; i < nsrc; ++i) {
        src[i] = Vector3(tdouble(), tdouble(), tdouble());
        field[i] = tdensity();
    }
    for (size_t i = 0; i < ndst; ++i)
        dst[i] = Vector3(tdouble(), tdouble(), tdouble());
    const size_t nv = 1024, nt = 2048;
    std::vector<Vector3> verts(nv);
    for (size_t i = 0; i < nv; ++i)
        verts[i] = Vector3(tdouble(), tdouble(), tdouble());
    std::vector<std::vector<size_t>> tris(nt);
    for (size_t i = 0; i < nt; ++i)
        tris[i] = { i % nv, (i + 1) % nv, (i + 2) % nv };
    std::vector<double> dens(nv, 1.0), scal(nt, 1.0);
    for (size_t i = 0; i < nv; ++i)
        dens[i] = tdensity();
    auto t0 = std::chrono::steady_clock::now();
    const std::vector<double> resampled
        = Density::resampleNearest(src, field, dst);
    Density::applyToScalingField(verts, tris, dens, scal);
    auto t1 = std::chrono::steady_clock::now();
    double sink = 0.0;
    for (double v : resampled)
        sink += v;
    for (double v : scal)
        sink += v;
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    std::printf("T dens ms=%.3f sink=%.17g\n", ms, sink);
    return ms;
}

int main()
{
    std::printf("DDIFF1\n");
    dumpNormalize(60);
    dumpEdge(40);
    dumpResample(80);
    dumpApply(60);
    dumpAdversarial();
    timeDensityLoop();
    timeDensityLoop();
    timeDensityLoop();
    return 0;
}
