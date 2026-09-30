// Unit tests for retopo.core.position_key (core/positionkey.cppm).
// Plain assert-style main, no third-party framework.
// NOTE: CHECK instead of <cassert> assert(), which is a no-op under NDEBUG.
import retopo.core.position_key;
import retopo.core.vector3;

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
    using AutoRemesher::PositionKey;
    using AutoRemesher::Vector3;

    // Equality on identical points, from doubles or from a Vector3.
    CHECK(PositionKey(1.0, 2.0, 3.0) == PositionKey(1.0, 2.0, 3.0));
    CHECK(PositionKey(Vector3(1.0, 2.0, 3.0)) == PositionKey(1.0, 2.0, 3.0));
    CHECK(PositionKey(0.0, 0.0, 0.0) == PositionKey(0.0, 0.0, 0.0));

    // Equality is quantized (factor 1e5, truncated): points within one
    // quantum compare equal ...
    CHECK(PositionKey(1.000001, 0.0, 0.0) == PositionKey(1.000009, 0.0, 0.0));
    // ... while points a quantum apart compare unequal.
    CHECK(!(PositionKey(1.0, 0.0, 0.0) == PositionKey(1.0001, 0.0, 0.0)));
    CHECK(!(PositionKey(0.0, 1.0, 0.0) == PositionKey(0.0, 2.0, 0.0)));
    CHECK(!(PositionKey(0.0, 0.0, 1.0) == PositionKey(0.0, 0.0, 2.0)));
    CHECK(!(PositionKey(1.0, 2.0, 3.0) == PositionKey(3.0, 2.0, 1.0)));

    // position() round-trips the exact input coordinates.
    const Vector3 pos = PositionKey(1.5, -2.25, 3.125).position();
    CHECK(pos.x() == 1.5);
    CHECK(pos.y() == -2.25);
    CHECK(pos.z() == 3.125);

    // Ordering is lexicographic on the quantized coordinates.
    CHECK(PositionKey(0.0, 0.0, 0.0) < PositionKey(1.0, 0.0, 0.0));
    CHECK(PositionKey(1.0, 0.0, 0.0) < PositionKey(1.0, 1.0, 0.0));
    CHECK(PositionKey(1.0, 1.0, 0.0) < PositionKey(1.0, 1.0, 1.0));
    CHECK(!(PositionKey(1.0, 0.0, 0.0) < PositionKey(0.0, 0.0, 0.0)));
    CHECK(!(PositionKey(1.0, 2.0, 3.0) < PositionKey(1.0, 2.0, 3.0)));
    // Ordering follows the quantized values.
    CHECK(PositionKey(1.000001, 0.0, 0.0) < PositionKey(1.00002, 0.0, 0.0));
    // Equal keys are equivalent under ordering.
    CHECK(!(PositionKey(1.000001, 0.0, 0.0) < PositionKey(1.000009, 0.0, 0.0)));
    CHECK(!(PositionKey(1.000009, 0.0, 0.0) < PositionKey(1.000001, 0.0, 0.0)));
    // Negative coordinates order numerically.
    CHECK(PositionKey(-2.0, 0.0, 0.0) < PositionKey(-1.0, 0.0, 0.0));
    CHECK(PositionKey(-1.0, 0.0, 0.0) < PositionKey(0.0, 0.0, 0.0));

    if (g_failures == 0)
        std::printf("PASS test_positionkey\n");
    return g_failures == 0 ? 0 : 1;
}
