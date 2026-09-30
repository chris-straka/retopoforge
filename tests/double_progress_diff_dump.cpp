// Differential oracle dump for the double_utils + progress port.
// Generates seeded isZero/isEqual cases, solves them with the C++
// implementation, records a seeded ProgressHandler call sequence, and
// prints inputs + outputs in a line format that
// rust/core/tests/double_progress_diff.rs replays. Doubles print with
// %.17g (exact round-trip). Also times a cover loop for the runtime ratio.
//
// Build-only helper: not registered with ctest. Run it and redirect stdout
// to tests/fixtures/double_progress_diff.txt, then commit the fixture.
import retopo.core.double_utils;
import retopo.core.progress;

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

using AutoRemesher::Double::isEqual;
using AutoRemesher::Double::isZero;
using AutoRemesher::ProgressHandler;

// splitmix64: an identical implementation lives in the Rust replay test,
// so the timing input below is generated bit-identically on both sides.
static std::uint64_t g_state = 0xD0B1E77A11CEull;

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

static void printFloat(float v)
{
    std::printf("%.9g", v);
}

// Uniform double in [lo, hi) from 53 random mantissa bits.
static double uniform(double lo, double hi)
{
    constexpr double kUnit = 1.0 / 9007199254740992.0; // 2^-53
    double u = static_cast<double>(nextU64() >> 11) * kUnit;
    return lo + u * (hi - lo);
}

// Random finite double across magnitudes: tiny (epsilon regime),
// small, unit, and large exponents.
static double anyDouble()
{
    switch (below(5)) {
    case 0: {
        // Epsilon neighborhood: multiples of eps around the boundary,
        // where the isZero cliff lives.
        constexpr double eps = std::numeric_limits<double>::epsilon();
        double m = uniform(-4.0, 4.0);
        return m * eps;
    }
    case 1:
        return uniform(-10.0, 10.0);
    case 2:
        return uniform(-1.0e6, 1.0e6);
    case 3: {
        // Random bit pattern with a bounded exponent (finite, any scale).
        std::uint64_t bits = nextU64();
        int exp = static_cast<int>(below(108)) - 53; // 2^-53 .. 2^54
        double mant = 1.0 + static_cast<double>(bits >> 12) / 4503599627370496.0;
        double v = std::ldexp(mant, exp);
        return (bits & 1) ? -v : v;
    }
    default:
        return uniform(-1.0, 1.0);
    }
}

static void dumpZeroCases()
{
    constexpr double eps = std::numeric_limits<double>::epsilon();
    // Fixed specials first: signed zeros, the exact boundary and its float
    // neighbors, infinities, NaN, and the exponent extremes.
    const double specials[] = { 0.0, -0.0, eps, -eps, 0.5 * eps, -0.5 * eps,
        std::nextafter(eps, 0.0), std::nextafter(-eps, 0.0), std::nextafter(eps, 1.0),
        std::nextafter(-eps, -1.0), 2.0 * eps, -2.0 * eps, 1e-10, -1e-10, 1.0, -1.0, 0.1 + 0.2 - 0.3,
        std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::min(),
        std::numeric_limits<double>::max(), std::numeric_limits<double>::denorm_min() };
    for (double v : specials) {
        std::printf("Z ");
        printDouble(v);
        std::printf(" %d\n", isZero(v) ? 1 : 0);
    }
    for (int i = 0; i < 160; ++i) {
        double v = anyDouble();
        std::printf("Z ");
        printDouble(v);
        std::printf(" %d\n", isZero(v) ? 1 : 0);
    }
}

static void dumpEqualCases()
{
    constexpr double eps = std::numeric_limits<double>::epsilon();
    // Fixed pairs: identity, classic fp rounding, the boundary in both
    // directions, sign flips, infinities, NaN.
    const double pairs[][2] = { { 0.0, 0.0 }, { 1.0, 1.0 }, { -5.0, -5.0 }, { 0.1 + 0.2, 0.3 },
        { 0.0, eps }, { 0.0, -eps }, { 0.0, 2.0 * eps }, { 0.0, -2.0 * eps }, { 1.0, 1.0 + 1e-9 },
        { 1.0, 2.0 }, { 1.0, -1.0 }, { -0.0, 0.0 },
        { std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity() },
        { std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity() },
        { std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN() },
        { 1.0, std::numeric_limits<double>::quiet_NaN() } };
    for (const auto& [a, b] : pairs) {
        std::printf("E ");
        printDouble(a);
        std::printf(" ");
        printDouble(b);
        std::printf(" %d\n", isEqual(a, b) ? 1 : 0);
    }
    for (int i = 0; i < 160; ++i) {
        double a = anyDouble();
        double b;
        switch (below(4)) {
        case 0:
            // Near-identical: the isEqual cliff (diff around eps).
            b = a + uniform(-4.0, 4.0) * eps;
            break;
        case 1:
            // Small perturbation, usually but not always within eps.
            b = std::nextafter(a, uniform(-1.0, 1.0) < 0 ? -1e300 : 1e300);
            if (below(2) == 0)
                b = std::nextafter(b, b < a ? -1e300 : 1e300);
            break;
        case 2:
            // Exact copy (must always compare equal, NaN excluded: anyDouble
            // never emits NaN, so a == a holds here).
            b = a;
            break;
        default:
            b = anyDouble();
            break;
        }
        std::printf("E ");
        printDouble(a);
        std::printf(" ");
        printDouble(b);
        std::printf(" %d\n", isEqual(a, b) ? 1 : 0);
    }
}

static void dumpProgressCalls()
{
    static const char* const kNames[] = { "remesh", "weld", "parameterize", "solve", "refine" };
    std::vector<std::pair<float, std::string>> calls;
    ProgressHandler handler = [&](float fraction, const char* name) {
        calls.push_back({ fraction, name });
    };
    // Fixed edges: stage start/end plus a mid-stage call.
    handler(0.0f, "remesh");
    handler(0.5f, "solve");
    handler(1.0f, "weld");
    for (int i = 0; i < 64; ++i) {
        float f = static_cast<float>(uniform(0.0, 1.0));
        const char* name = kNames[below(5)];
        handler(f, name);
    }
    for (const auto& [fraction, name] : calls) {
        std::printf("P ");
        printFloat(fraction);
        std::printf(" %s\n", name.c_str());
    }
}

// Timing input: 4096 doubles regenerated bit-identically by the Rust timing
// test from the same splitmix64 stream (reset to kTimingSeed).
static constexpr std::uint64_t kTimingSeed = 0xD0B1E77A11CEull ^ 0x71E77A11CEull;

static std::vector<double> timingInput()
{
    g_state = kTimingSeed;
    std::vector<double> xs;
    xs.reserve(4096);
    for (int i = 0; i < 4096; ++i)
        xs.push_back(anyDouble());
    return xs;
}

static void timeZeroLoop()
{
    std::vector<double> xs = timingInput();
    // 3 samples, 1M calls each: the function is a single compare, so a
    // short loop measures call + branch cost, not memory.
    for (int s = 0; s < 3; ++s) {
        volatile int sink = 0;
        auto t0 = std::chrono::steady_clock::now();
        for (int i = 0; i < 1000000; ++i)
            sink += isZero(xs[static_cast<size_t>(i) & 4095]) ? 1 : 0;
        auto t1 = std::chrono::steady_clock::now();
        double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        std::printf("T zero calls=1000000 hits=%d ms=%.3f\n", sink, ms);
    }
}

int main()
{
    std::printf("DPDIFF1\n");
    dumpZeroCases();
    dumpEqualCases();
    dumpProgressCalls();
    timeZeroLoop();
    return 0;
}
