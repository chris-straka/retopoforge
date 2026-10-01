// Differential oracle dump for the vector2/vector3 Rust port (wave 2
// foundation). Seeded splitmix64 cases over every method, plus adversarial
// near-degenerate inputs (near-parallel cross, near-collinear normal,
// near-zero normalize around the epsilon boundary, near-cocircular,
// near-edge triangle/left, near-0/pi angles, degenerate barycentric,
// NaN/inf/-0.0), since the port transcribes Clang's FMA fusion with
// explicit mul_add and only ulp-level cases can prove the transcription.
//
// Build-only helper: not registered with ctest. Run it and redirect stdout
// to tests/fixtures/vector_diff.txt, replayed by
// rust/core/tests/vector_diff.rs. Doubles print with %.17g (exact
// round-trip, so the replay compares bitwise).
import retopo.core.vector2;
import retopo.core.vector3;

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

static std::uint64_t g_state = 0xDEC70A5EEDull;

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

using AutoRemesher::Vector2;
using AutoRemesher::Vector3;

static void printDouble(double v)
{
    std::printf("%.17g", v);
}

static void printV2(const Vector2& v)
{
    printDouble(v.x());
    std::printf(" ");
    printDouble(v.y());
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

static Vector2 randV2()
{
    return Vector2(randDouble(), randDouble());
}

static Vector3 randV3()
{
    return Vector3(randDouble(), randDouble(), randDouble());
}

static void dumpV2Random(int count)
{
    for (int i = 0; i < count; ++i) {
        Vector2 a = randV2(), b = randV2(), c = randV2(), p = randV2();
        double s = randDouble();
        std::printf("V2 ");
        printV2(a);
        std::printf(" ");
        printV2(b);
        std::printf(" ");
        printV2(c);
        std::printf(" ");
        printV2(p);
        std::printf(" ");
        printDouble(s);
        std::printf("\n");
        std::printf("r ");
        printDouble(a.lengthSquared());
        std::printf(" ");
        printDouble(a.length());
        std::printf(" ");
        printV2(a.normalized());
        std::printf(" ");
        printDouble(Vector2::dotProduct(a, b));
        std::printf(" %d", a.isOnLeft(b, c) ? 1 : 0);
        std::printf(" ");
        printV2(Vector2::barycentricCoordinates(a, b, c, p));
        std::printf(" %d", Vector2::isInTriangle(a, b, c, p) ? 1 : 0);
        std::printf(" %d", p.isInCircle(a, b, c) ? 1 : 0);
        std::printf(" ");
        printV2(a + b);
        std::printf(" ");
        printV2(a - b);
        std::printf(" ");
        printV2(a * s);
        std::printf(" ");
        printV2(s * a);
        std::printf(" %d %d", (a == b) ? 1 : 0, (a != b) ? 1 : 0);
        std::printf("\n");
        Vector2 m = a;
        m.normalize();
        std::printf("n ");
        printV2(m);
        std::printf("\n");
    }
}

static void dumpV3Random(int count)
{
    for (int i = 0; i < count; ++i) {
        Vector3 a = randV3(), b = randV3(), c = randV3(), p = randV3();
        double s = randDouble();
        std::printf("V3 ");
        printV3(a);
        std::printf(" ");
        printV3(b);
        std::printf(" ");
        printV3(c);
        std::printf(" ");
        printV3(p);
        std::printf(" ");
        printDouble(s);
        std::printf("\n");
        std::printf("r ");
        printDouble(a.lengthSquared());
        std::printf(" ");
        printDouble(a.length());
        std::printf(" ");
        printV3(a.normalized());
        std::printf(" ");
        printDouble(Vector3::dotProduct(a, b));
        std::printf(" ");
        printV3(Vector3::crossProduct(a, b));
        std::printf(" ");
        printV3(Vector3::normal(a, b, c));
        std::printf(" ");
        printDouble(Vector3::angle(a, b));
        std::printf(" %d", a.isZero() ? 1 : 0);
        std::printf(" %d", (a < b) ? 1 : 0);
        std::printf(" ");
        printDouble(Vector3::area(a, b, c));
        std::printf(" ");
        printV3(Vector3::barycentricCoordinates(a, b, c, p));
        std::printf(" ");
        printV3(a + b);
        std::printf(" ");
        printV3(a - b);
        std::printf(" ");
        printV3(-a);
        std::printf(" ");
        printV3(a * s);
        std::printf(" ");
        printV3(s * a);
        std::printf(" ");
        printV3(a * b);
        std::printf(" ");
        printV3(a / b);
        std::printf(" ");
        printV3(a / s);
        std::printf("\n");
        Vector3 m = a;
        m.normalize();
        Vector3 q = a;
        q += b;
        Vector3 t = a;
        t *= s;
        Vector3 u = a;
        u /= s;
        std::printf("n ");
        printV3(m);
        std::printf(" ");
        printV3(q);
        std::printf(" ");
        printV3(t);
        std::printf(" ");
        printV3(u);
        std::printf("\n");
        std::vector<Vector3> in { a, b, c };
        std::vector<Vector2> out2;
        Vector3::project(in, &out2, a, b, c);
        std::printf("p2");
        for (const Vector2& v : out2) {
            std::printf(" ");
            printV2(v);
        }
        std::printf("\n");
        std::vector<Vector3> out3;
        Vector3::project(in, &out3, a, b, c);
        std::printf("p3");
        for (const Vector3& v : out3) {
            std::printf(" ");
            printV3(v);
        }
        std::printf("\n");
    }
}

static void dumpAdversarial()
{
    const double eps = std::numeric_limits<double>::epsilon();
    // Near-zero normalize around the isZero boundary.
    for (int e = -20; e <= -12; ++e) {
        double t = std::ldexp(1.0, e);
        Vector2 v2(t, t);
        Vector3 v3(t, t, t);
        std::printf("Z %.17g\n", t);
        std::printf("r ");
        printV2(v2.normalized());
        std::printf(" ");
        printV3(v3.normalized());
        std::printf(" %d\n", v3.isZero() ? 1 : 0);
    }
    // Near-parallel cross / near-collinear normal / near-0,pi angles.
    for (int i = 0; i < 40; ++i) {
        Vector3 a = randV3();
        double n = a.length();
        if (!(n > 0.0))
            continue;
        Vector3 unit = a / n;
        double k = 1.0 + (below(2) ? 1.0 : -1.0) * std::ldexp(1.0, -30 - static_cast<int>(below(25)));
        Vector3 b(unit.x() * k + randDouble() * 1e-12, unit.y() * k, unit.z() * k);
        Vector3 c = randV3();
        std::printf("P ");
        printV3(a);
        std::printf(" ");
        printV3(b);
        std::printf(" ");
        printV3(c);
        std::printf("\n");
        std::printf("r ");
        printV3(Vector3::crossProduct(a, b));
        std::printf(" ");
        printV3(Vector3::normal(a, b, c));
        std::printf(" ");
        printDouble(Vector3::angle(a, b));
        std::printf(" ");
        printDouble(Vector3::area(a, b, c));
        std::printf("\n");
    }
    // Near-cocircular: fourth point on/near the circle through three.
    for (int i = 0; i < 40; ++i) {
        Vector2 a = randV2(), b = randV2();
        Vector2 c(b.x() + 1.0 + randDouble(), b.y() - 1.0 + randDouble());
        // Circumcenter-ish probe: midpoint + perpendicular offsets.
        Vector2 mid((a.x() + b.x()) / 2.0, (a.y() + b.y()) / 2.0);
        double r = (Vector2::dotProduct(a - mid, a - mid));
        r = std::sqrt(r > 0.0 ? r : 0.0);
        double ang = randDouble();
        double rr = r * (1.0 + (static_cast<int>(below(7)) - 3) * 1e-13);
        Vector2 p(mid.x() + rr * std::cos(ang), mid.y() + rr * std::sin(ang));
        std::printf("C ");
        printV2(a);
        std::printf(" ");
        printV2(b);
        std::printf(" ");
        printV2(c);
        std::printf(" ");
        printV2(p);
        std::printf("\n");
        std::printf("r %d %d %d\n", p.isInCircle(a, b, c) ? 1 : 0,
            Vector2::isInTriangle(a, b, c, p) ? 1 : 0, p.isOnLeft(a, b) ? 1 : 0);
    }
    // Near-boundary epsilon equality.
    for (int i = 0; i < 20; ++i) {
        double x = randDouble();
        for (int step = -2; step <= 2; ++step) {
            double y = x + step * eps;
            std::printf("E %.17g %.17g\n", x, y);
            std::printf("r %d %d\n",
                (Vector2(x, 0.0) == Vector2(y, 0.0)) ? 1 : 0,
                (Vector2(x, 0.0) != Vector2(y, 0.0)) ? 1 : 0);
        }
    }
    // Specials: zeros, NaN, infinities, denormals, extremes.
    double specials[] = { 0.0, -0.0, 1.0, -1.0, std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(), std::nan(""), -std::nan(""), eps, -eps,
        std::numeric_limits<double>::min(), std::numeric_limits<double>::max(),
        std::numeric_limits<double>::denorm_min() };
    for (double x : specials) {
        for (double y : specials) {
            Vector2 v(x, y);
            std::printf("S %.17g %.17g\n", x, y);
            std::printf("r ");
            printDouble(v.lengthSquared());
            std::printf(" ");
            printV2(v.normalized());
            std::printf(" %d\n", (v == v) ? 1 : 0);
        }
    }
    // Ordering incl. NaN fall-through.
    for (double x : specials) {
        Vector3 a(x, 1.0, 2.0), b(0.0, 1.0, 2.0), c(0.0, x, 2.0), d(0.0, 1.0, x);
        std::printf("O %.17g\n", x);
        std::printf("r %d %d %d %d %d %d\n", (a < b) ? 1 : 0, (b < a) ? 1 : 0, (c < d) ? 1 : 0,
            (d < c) ? 1 : 0, (a < a) ? 1 : 0, (Vector3(x, x, x) < Vector3(0.0, 0.0, 0.0)) ? 1 : 0);
    }
    // to_string incl. nan sign.
    for (double x : specials) {
        Vector2 v2(x, -x);
        Vector3 v3(x, 0.5, -x);
        std::printf("T %.17g\n", x);
        std::printf("r [%s] [%s]\n", AutoRemesher::to_string(v2).c_str(),
            AutoRemesher::to_string(v3).c_str());
    }
}

static constexpr std::uint64_t kTimingSeed = 0x9EC70A5C10Cull;

static double timeVectorLoop()
{
    g_state = kTimingSeed;
    const size_t n = 1 << 20;
    std::vector<Vector3> a(n), b(n), c(n);
    for (size_t i = 0; i < n; ++i) {
        a[i] = randV3();
        b[i] = randV3();
        c[i] = randV3();
    }
    auto t0 = std::chrono::steady_clock::now();
    double sink = 0.0;
    for (int rep = 0; rep < 5; ++rep) {
        for (size_t i = 0; i < n; ++i) {
            Vector3 cr = Vector3::crossProduct(a[i], b[i]);
            double d = Vector3::dotProduct(cr, c[i]);
            Vector3 nn = Vector3::normal(a[i], b[i], c[i]);
            sink += d + nn.x() + Vector3::area(a[i], b[i], c[i]);
        }
    }
    auto t1 = std::chrono::steady_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    std::printf("T vec ms=%.3f sink=%.17g\n", ms, sink);
    return ms;
}

int main()
{
    std::printf("VDIFF1\n");
    dumpV2Random(150);
    dumpV3Random(150);
    dumpAdversarial();
    timeVectorLoop();
    timeVectorLoop();
    timeVectorLoop();
    return 0;
}
