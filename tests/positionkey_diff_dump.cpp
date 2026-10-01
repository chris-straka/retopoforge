// Differential oracle dump for the positionkey Rust port (rs-positionkey
// lane). Seeded splitmix64 cases over construction/position/==/<, plus
// adversarial near-quantum-boundary inputs (the FP-decision code here is
// the trunc(x * 100000) quantization: boundary straddles at exact
// multiples of 1e-5, negative half-quantum truncation-toward-zero,
// per-axis ordering precedence, signed zero, denormals, large
// magnitudes).
//
// The quantized integers are private in C++, so the oracle observes the
// full public behavior instead: exact position() round-trip (bitwise)
// plus the pairwise ==/</> matrix over 4 points per case (18 flags),
// which determines the quantization and ordering completely.
//
// Build-only helper: not registered with ctest. Run it and redirect stdout
// to tests/fixtures/positionkey_diff.txt, replayed by
// rust/core/tests/position_key_diff.rs. Doubles print with %.17g (exact
// round-trip, so the replay compares bitwise).
//
// NaN/inf are deliberately NOT dumped: static_cast<long> is UB there
// (practically i64::MIN via cvttsd2si), while Rust's `as i64` saturates,
// so no meaningful expectation exists. That domain is unreachable
// downstream (weld drops NaN before keys are built).
import retopo.core.position_key;
import retopo.core.vector3;

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <vector>

static std::uint64_t g_state = 0x5EEDBEEF42ull;

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

using AutoRemesher::PositionKey;
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

// Random double across scales (same 6-branch shape as vector_diff_dump;
// the timing stream below must match the Rust replay branch for branch).
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

// One case: 4 input points, then the full observable behavior — exact
// position() of each key plus the 6-pair ==/</> matrix.
static void dumpCase(const char* tag, const Vector3& a, const Vector3& b, const Vector3& c,
    const Vector3& d)
{
    std::printf("%s ", tag);
    printV3(a);
    std::printf(" ");
    printV3(b);
    std::printf(" ");
    printV3(c);
    std::printf(" ");
    printV3(d);
    std::printf("\n");
    PositionKey ka(a), kb(b), kc(c), kd(d);
    const PositionKey* ks[4] = { &ka, &kb, &kc, &kd };
    std::printf("r ");
    for (int i = 0; i < 4; ++i) {
        if (i > 0)
            std::printf(" ");
        printV3(ks[i]->position());
    }
    for (int i = 0; i < 4; ++i) {
        for (int j = i + 1; j < 4; ++j) {
            std::printf(" %d %d %d", (*ks[i] == *ks[j]) ? 1 : 0, (*ks[i] < *ks[j]) ? 1 : 0,
                (*ks[j] < *ks[i]) ? 1 : 0);
        }
    }
    std::printf("\n");
}

static void dumpRandom(int count)
{
    for (int i = 0; i < count; ++i) {
        if (i % 2 == 0) {
            // Wide: four independent points.
            dumpCase("K", randV3(), randV3(), randV3(), randV3());
        } else {
            // Clustered: small offsets around a base point, so quantum
            // boundaries are straddled often (offset magnitudes 2^-30
            // .. 2^-6 bracket the 1e-5 quantum from both sides).
            Vector3 base = randV3();
            Vector3 p[4];
            for (int k = 0; k < 4; ++k) {
                double mag = std::ldexp(1.0, -30 + static_cast<int>(below(25)));
                double ox = (static_cast<int>(below(2001)) - 1000) / 1000.0 * mag;
                double oy = (static_cast<int>(below(2001)) - 1000) / 1000.0 * mag;
                double oz = (static_cast<int>(below(2001)) - 1000) / 1000.0 * mag;
                p[k] = Vector3(base.x() + ox, base.y() + oy, base.z() + oz);
            }
            dumpCase("K", p[0], p[1], p[2], p[3]);
        }
    }
}

static double step(double base, int n)
{
    double v = base;
    for (int i = 0; i < (n >= 0 ? n : -n); ++i)
        v = std::nextafter(v, n >= 0 ? std::numeric_limits<double>::infinity() : -std::numeric_limits<double>::infinity());
    return v;
}

static void dumpAdversarial()
{
    // Quantum edges on x: multiples of 1e-5 from zero to 1e7, both signs,
    // probed at +-1/2 ulp. Catches any off-by-one-quantum or
    // rounding-mode slip in trunc(x * 1e5).
    const long long edges[] = { 0, 1, 2, 99999, 100000, 100001, 999999, 1000000, 1000000000LL,
        1000000000000LL };
    for (long long n : edges) {
        for (int sign = 0; sign < 2; ++sign) {
            if (n == 0 && sign == 1)
                continue;
            double base = (sign == 0 ? 1.0 : -1.0) * static_cast<double>(n) / 100000.0;
            dumpCase("A", Vector3(step(base, -2), 0.0, 0.0), Vector3(step(base, -1), 0.0, 0.0),
                Vector3(base, 0.0, 0.0), Vector3(step(base, 1), 0.0, 0.0));
        }
    }
    // Negative half-quantum: truncation goes toward zero, so -1.000005
    // (x*1e5 = -100000.5) keys EQUAL to -1.0. A floor-based port would
    // key it with -1.00001 instead. Positive mirrors included.
    const double halves[] = { 0.5, 1.5, 99999.5, 100000.5 };
    for (double v : halves) {
        for (int sign = 0; sign < 2; ++sign) {
            double s = sign == 0 ? 1.0 : -1.0;
            double x = s * v / 100000.0;
            double mate = s * (v - 0.5) / 100000.0; // quantum partner toward zero
            double away = s * (v + 0.5) / 100000.0; // next quantum away from zero
            dumpCase("A", Vector3(step(x, -1), 0.0, 0.0), Vector3(x, 0.0, 0.0),
                Vector3(mate, 0.0, 0.0), Vector3(away, 0.0, 0.0));
        }
    }
    // Quantum edges on y and z with equal leading axes, forcing the
    // ordering down to the probed axis.
    const long long yzEdges[] = { 0, 1, 100000, -1, -100000 };
    for (long long n : yzEdges) {
        double base = static_cast<double>(n) / 100000.0;
        dumpCase("A", Vector3(5.0, step(base, -1), 7.0), Vector3(5.0, base, 7.0),
            Vector3(5.0, step(base, 1), 7.0), Vector3(5.0, step(base, 2), 7.0));
        dumpCase("A", Vector3(5.0, 7.0, step(base, -1)), Vector3(5.0, 7.0, base),
            Vector3(5.0, 7.0, step(base, 1)), Vector3(5.0, 7.0, step(base, 2)));
    }
    // Ordering precedence across axes: x dominates regardless of y/z,
    // and equal-x falls through to y (incl. a within-quantum x pair).
    dumpCase("A", Vector3(1.0, 0.0, 0.0), Vector3(0.0, 99.0, 99.0), Vector3(0.0, 0.0, 0.0),
        Vector3(1.0, 0.0, 0.0));
    dumpCase("A", Vector3(-5.0, 1e9, 1e9), Vector3(5.0, -1e9, -1e9), Vector3(0.0, 0.0, 0.0),
        Vector3(0.0, 0.0, 0.0));
    dumpCase("A", Vector3(1.000005, 999.0, 999.0), Vector3(1.0, -999.0, -999.0),
        Vector3(1.0, 999.0, 999.0), Vector3(1.000005, -999.0, -999.0));
    // Signed zero: all quantize to 0 (equal), but position() must
    // round-trip the -0.0 sign bit exactly.
    dumpCase("A", Vector3(-0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0), Vector3(0.0, -0.0, 0.0),
        Vector3(0.0, 0.0, -0.0));
    dumpCase("A", Vector3(-0.0, -0.0, -0.0), Vector3(0.0, 0.0, 0.0), Vector3(-0.0, 0.0, -0.0),
        Vector3(0.0, -0.0, 0.0));
    // Tiny: denormals and DBL_MIN all quantize to 0.
    dumpCase("A", Vector3(std::numeric_limits<double>::denorm_min(), 0.0, 0.0),
        Vector3(std::numeric_limits<double>::min(), 0.0, 0.0), Vector3(1e-310, 0.0, 0.0),
        Vector3(0.0, 0.0, 0.0));
    // Large magnitudes (x*1e5 still within int64: max here is 5e18 <
    // 9.2e18), probed at +-1 ulp.
    const double bigs[] = { 1e9, 1e12, 1e13, 5e13 };
    for (double v : bigs) {
        for (int sign = 0; sign < 2; ++sign) {
            double b = sign == 0 ? v : -v;
            dumpCase("A", Vector3(step(b, -1), 0.0, 0.0), Vector3(b, 0.0, 0.0),
                Vector3(step(b, 1), 0.0, 0.0), Vector3(step(b, 2), 0.0, 0.0));
        }
    }
}

static constexpr std::uint64_t kTimingSeed = 0x51CE1CA1EDull;

static void timePositionKeyLoop()
{
    g_state = kTimingSeed;
    const size_t n = 1 << 20;
    std::vector<Vector3> a(n), b(n);
    for (size_t i = 0; i < n; ++i) {
        a[i] = randV3();
        b[i] = randV3();
    }
    auto t0 = std::chrono::steady_clock::now();
    double sink = 0.0;
    for (int rep = 0; rep < 5; ++rep) {
        for (size_t i = 0; i < n; ++i) {
            PositionKey ka(a[i]);
            PositionKey kb(b[i]);
            sink += ka.position().x() + (ka < kb ? 1.0 : 0.0) + (ka == kb ? 2.0 : 0.0);
        }
    }
    auto t1 = std::chrono::steady_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    std::printf("T pk ms=%.3f sink=%.17g\n", ms, sink);
}

int main()
{
    std::printf("PKDIFF1\n");
    dumpRandom(200);
    dumpAdversarial();
    timePositionKeyLoop();
    timePositionKeyLoop();
    timePositionKeyLoop();
    return 0;
}
