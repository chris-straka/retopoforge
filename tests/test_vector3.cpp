// Unit tests for retopo.core.vector3 (core/vector3.cppm).
// Plain assert-style main, no third-party framework.
// NOTE: CHECK instead of <cassert> assert(), which is a no-op under NDEBUG.
import retopo.core.double_utils;
import retopo.core.vector3;

#include <cmath>
#include <cstdio>

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
    using AutoRemesher::Vector3;

    // Default construction is the zero vector.
    const Vector3 zero;
    CHECK(zero.x() == 0.0);
    CHECK(zero.y() == 0.0);
    CHECK(zero.z() == 0.0);
    CHECK(zero.isZero());
    CHECK(!Vector3(1.0, 0.0, 0.0).isZero());

    // Length on (1,2,2): sqrt(9) = 3, exactly representable.
    const Vector3 v(1.0, 2.0, 2.0);
    CHECK(v.lengthSquared() == 9.0);
    CHECK(v.length() == 3.0);

    // normalized() on a known vector.
    const Vector3 n = v.normalized();
    CHECK(n.x() == 1.0 / 3.0);
    CHECK(n.y() == 2.0 / 3.0);
    CHECK(n.z() == 2.0 / 3.0);
    CHECK(isEqual(n.length(), 1.0));

    // normalize() mutates in place with the same result.
    Vector3 m(1.0, 2.0, 2.0);
    m.normalize();
    CHECK(m.x() == 1.0 / 3.0);
    CHECK(m.y() == 2.0 / 3.0);
    CHECK(m.z() == 2.0 / 3.0);

    // Zero-vector behavior: normalize is a no-op returning/staying zero.
    const Vector3 zn = zero.normalized();
    CHECK(zn.x() == 0.0);
    CHECK(zn.y() == 0.0);
    CHECK(zn.z() == 0.0);
    CHECK(zero.length() == 0.0);
    Vector3 mz;
    mz.normalize();
    CHECK(mz.x() == 0.0 && mz.y() == 0.0 && mz.z() == 0.0);

    // Dot product on known vectors: 4 - 10 + 18 = 12.
    CHECK(Vector3::dotProduct(Vector3(1.0, 2.0, 3.0), Vector3(4.0, -5.0, 6.0)) == 12.0);
    CHECK(Vector3::dotProduct(Vector3(1.0, 0.0, 0.0), Vector3(0.0, 1.0, 0.0)) == 0.0);
    CHECK(Vector3::dotProduct(zero, v) == 0.0);

    // Cross product: unit axes and a known pair.
    // (1,2,3) x (4,5,6) = (2*6-3*5, 3*4-1*6, 1*5-2*4) = (-3, 6, -3).
    const Vector3 axis = Vector3::crossProduct(Vector3(1.0, 0.0, 0.0), Vector3(0.0, 1.0, 0.0));
    CHECK(axis.x() == 0.0 && axis.y() == 0.0 && axis.z() == 1.0);
    const Vector3 cp = Vector3::crossProduct(Vector3(1.0, 2.0, 3.0), Vector3(4.0, 5.0, 6.0));
    CHECK(cp.x() == -3.0 && cp.y() == 6.0 && cp.z() == -3.0);
    // Parallel vectors give the zero vector.
    const Vector3 par = Vector3::crossProduct(Vector3(1.0, 2.0, 3.0), Vector3(2.0, 4.0, 6.0));
    CHECK(par.x() == 0.0 && par.y() == 0.0 && par.z() == 0.0);

    // Triangle normal: right triangle in the xy-plane faces +z.
    const Vector3 tn = Vector3::normal(Vector3(0.0, 0.0, 0.0), Vector3(1.0, 0.0, 0.0), Vector3(0.0, 1.0, 0.0));
    CHECK(tn.x() == 0.0 && tn.y() == 0.0 && tn.z() == 1.0);
    // Degenerate (collinear) triangle has a zero normal.
    const Vector3 dn = Vector3::normal(Vector3(0.0, 0.0, 0.0), Vector3(1.0, 0.0, 0.0), Vector3(2.0, 0.0, 0.0));
    CHECK(dn.x() == 0.0 && dn.y() == 0.0 && dn.z() == 0.0);

    // Angle between known vectors.
    CHECK(isEqual(Vector3::angle(Vector3(1.0, 0.0, 0.0), Vector3(0.0, 1.0, 0.0)), M_PI / 2.0));
    CHECK(Vector3::angle(Vector3(1.0, 0.0, 0.0), Vector3(1.0, 0.0, 0.0)) == 0.0);

    // Lexicographic ordering.
    CHECK(Vector3(0.0, 0.0, 0.0) < Vector3(1.0, 0.0, 0.0));
    CHECK(Vector3(1.0, 0.0, 0.0) < Vector3(1.0, 1.0, 0.0));
    CHECK(!(Vector3(1.0, 0.0, 0.0) < Vector3(1.0, 0.0, 0.0)));

    if (g_failures == 0)
        std::printf("PASS test_vector3\n");
    return g_failures == 0 ? 0 : 1;
}
