// Unit tests for retopo.core.vector2 (core/vector2.cppm).
// Plain assert-style main, no third-party framework.
// NOTE: CHECK instead of <cassert> assert(), which is a no-op under NDEBUG.
import retopo.core.double_utils;
import retopo.core.vector2;

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
    using AutoRemesher::Vector2;

    // Default construction is the zero vector.
    const Vector2 zero;
    CHECK(zero.x() == 0.0);
    CHECK(zero.y() == 0.0);
    CHECK(zero[0] == 0.0);
    CHECK(zero[1] == 0.0);

    // Setters and element access.
    Vector2 mutableVec;
    mutableVec.setX(1.5);
    mutableVec.setY(-2.5);
    CHECK(mutableVec.x() == 1.5);
    CHECK(mutableVec.y() == -2.5);
    mutableVec[0] = 3.0;
    CHECK(mutableVec.x() == 3.0);
    mutableVec.setData(4.0, 5.0);
    CHECK(mutableVec.x() == 4.0);
    CHECK(mutableVec.y() == 5.0);

    // Length on the 3-4-5 triangle (exactly representable).
    const Vector2 v(3.0, 4.0);
    CHECK(v.lengthSquared() == 25.0);
    CHECK(v.length() == 5.0);
    CHECK(Vector2(1.0, 0.0).length() == 1.0);
    CHECK(isEqual(Vector2(1.0, 1.0).lengthSquared(), 2.0));

    // normalized() on a known vector.
    const Vector2 n = v.normalized();
    CHECK(n.x() == 0.6);
    CHECK(n.y() == 0.8);
    CHECK(isEqual(n.length(), 1.0));

    // normalize() mutates in place with the same result.
    Vector2 m(3.0, 4.0);
    m.normalize();
    CHECK(m.x() == 0.6);
    CHECK(m.y() == 0.8);

    // Zero-vector behavior: normalize is a no-op returning/staying zero.
    const Vector2 zn = zero.normalized();
    CHECK(zn.x() == 0.0);
    CHECK(zn.y() == 0.0);
    CHECK(zero.length() == 0.0);
    CHECK(zero.lengthSquared() == 0.0);
    Vector2 mz;
    mz.normalize();
    CHECK(mz.x() == 0.0);
    CHECK(mz.y() == 0.0);

    // Dot product on known vectors.
    CHECK(Vector2::dotProduct(Vector2(1.0, 2.0), Vector2(3.0, 4.0)) == 11.0);
    CHECK(Vector2::dotProduct(Vector2(1.0, 0.0), Vector2(0.0, 1.0)) == 0.0);
    CHECK(Vector2::dotProduct(Vector2(2.0, -3.0), Vector2(-4.0, 5.0)) == -23.0);
    CHECK(Vector2::dotProduct(zero, v) == 0.0);

    // Arithmetic operators.
    CHECK((Vector2(1.0, 2.0) + Vector2(3.0, 4.0)) == Vector2(4.0, 6.0));
    CHECK((Vector2(3.0, 4.0) - Vector2(1.0, 2.0)) == Vector2(2.0, 2.0));
    CHECK((2.0 * Vector2(1.0, 2.0)) == Vector2(2.0, 4.0));
    CHECK((Vector2(1.0, 2.0) * 2.0) == Vector2(2.0, 4.0));

    // Equality uses epsilon comparison.
    CHECK(Vector2(1.0, 2.0) == Vector2(1.0, 2.0));
    CHECK(Vector2(1.0, 2.0) != Vector2(1.0, 3.0));

    if (g_failures == 0)
        std::printf("PASS test_vector2\n");
    return g_failures == 0 ? 0 : 1;
}
