// Unit tests for retopo.core.double_utils (core/double.cppm).
// Plain assert-style main, no third-party framework.
// NOTE: CHECK instead of <cassert> assert(), which is a no-op under NDEBUG.
import retopo.core.double_utils;

#include <cmath>
#include <cstdio>
#include <limits>

static int g_failures = 0;

#define CHECK(cond)                                                                                  \
    do {                                                                                             \
        if (!(cond)) {                                                                               \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                               \
            ++g_failures;                                                                            \
        }                                                                                            \
    } while (0)

int main()
{
    using AutoRemesher::Double::isEqual;
    using AutoRemesher::Double::isZero;

    const double eps = std::numeric_limits<double>::epsilon();

    // isZero: |x| <= epsilon.
    CHECK(isZero(0.0));
    CHECK(isZero(-0.0));
    CHECK(isZero(eps)); // boundary: exactly epsilon counts as zero
    CHECK(isZero(-eps));
    CHECK(isZero(0.5 * eps));
    CHECK(isZero(std::nextafter(eps, 0.0))); // just inside the boundary
    CHECK(!isZero(std::nextafter(eps, 1.0))); // just outside the boundary
    CHECK(!isZero(2.0 * eps));
    CHECK(!isZero(-2.0 * eps));
    CHECK(!isZero(1e-10));
    CHECK(!isZero(-1e-10));
    CHECK(!isZero(1.0));

    // isEqual: isZero(a - b).
    CHECK(isEqual(0.0, 0.0));
    CHECK(isEqual(1.0, 1.0));
    CHECK(isEqual(-5.0, -5.0));
    CHECK(isEqual(0.1 + 0.2, 0.3)); // classic fp rounding, diff ~5.6e-17 < eps
    CHECK(isEqual(0.0, eps)); // boundary
    CHECK(isEqual(0.0, -eps));
    CHECK(!isEqual(0.0, 2.0 * eps));
    CHECK(!isEqual(1.0, 1.0 + 1e-9));
    CHECK(!isEqual(1.0, 2.0));
    CHECK(!isEqual(1.0, -1.0));

    if (g_failures == 0)
        std::printf("PASS test_double\n");
    return g_failures == 0 ? 0 : 1;
}
