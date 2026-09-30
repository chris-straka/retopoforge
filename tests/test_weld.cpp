// Unit tests for weld-on-load (AutoRemesher::weldPositionsAndTriangles).
// De-indexed triangle soup welds back to an indexed mesh, index-degenerate
// triangles are dropped, and already-welded input is left untouched.
// Plain assert-style main, no framework.
// NOTE: CHECK instead of <cassert> assert(), which is a no-op under NDEBUG.
#include <AutoRemesher/ObjReader>
import retopo.core.mesh_separator;

#include <cstdio>
#include <string>
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

bool vecEqual(const std::vector<float>& a, const std::vector<float>& b)
{
    return a == b;
}

bool trisEqual(const std::vector<std::vector<size_t>>& a, const std::vector<std::vector<size_t>>& b)
{
    return a == b;
}

size_t islandCount(const std::vector<std::vector<size_t>>& triangles)
{
    std::vector<std::vector<std::vector<size_t>>> islands;
    AutoRemesher::MeshSeparator::splitToIslands(triangles, islands);
    return islands.size();
}

} // namespace

int main()
{
    // De-indexed soup: two triangles sharing an edge, written as six
    // independent vertices, plus one index-degenerate triangle. Welding
    // merges the duplicated edge corners and drops the degenerate face.
    {
        std::vector<float> positions {
            0, 0, 0, // 0
            1, 0, 0, // 1
            0, 1, 0, // 2
            1, 0, 0, // 3 == 1
            1, 1, 0, // 4
            0, 1, 0, // 5 == 2
        };
        std::vector<std::vector<size_t>> triangles { { 0, 1, 2 }, { 3, 4, 5 }, { 0, 0, 1 } };
        AutoRemesher::weldPositionsAndTriangles(&positions, &triangles);
        const std::vector<float> expectedPositions {
            0, 0, 0,
            1, 0, 0,
            0, 1, 0,
            1, 1, 0,
        };
        CHECK(vecEqual(positions, expectedPositions));
        const std::vector<std::vector<size_t>> expectedTriangles { { 0, 1, 2 }, { 1, 3, 2 } };
        CHECK(trisEqual(triangles, expectedTriangles));
    }

    // The welded soup above is one island; unwelded it is two (no shared
    // corner indices, so the splitter cannot join the triangles).
    {
        std::vector<float> positions {
            0, 0, 0,
            1, 0, 0,
            0, 1, 0,
            1, 0, 0,
            1, 1, 0,
            0, 1, 0,
        };
        std::vector<std::vector<size_t>> triangles { { 0, 1, 2 }, { 3, 4, 5 } };
        CHECK(islandCount(triangles) == 2);
        AutoRemesher::weldPositionsAndTriangles(&positions, &triangles);
        CHECK(islandCount(triangles) == 1);
    }

    // Already-welded input is byte-identical after the call (identity fast
    // path): the bench models take this path, so their counts cannot move.
    {
        std::vector<float> positions {
            0, 0, 0,
            1, 0, 0,
            1, 1, 0,
            0, 1, 0,
        };
        std::vector<std::vector<size_t>> triangles { { 0, 1, 2 }, { 0, 2, 3 } };
        const std::vector<float> beforePositions = positions;
        const std::vector<std::vector<size_t>> beforeTriangles = triangles;
        AutoRemesher::weldPositionsAndTriangles(&positions, &triangles);
        CHECK(vecEqual(positions, beforePositions));
        CHECK(trisEqual(triangles, beforeTriangles));
    }

    // A triangle whose three corners are distinct vertices at one position
    // collapses under the remap and is dropped as degenerate.
    {
        std::vector<float> positions {
            0, 0, 0,
            0, 0, 0,
            0, 0, 0,
            1, 0, 0,
            0, 1, 0,
        };
        std::vector<std::vector<size_t>> triangles { { 0, 1, 2 }, { 0, 3, 4 } };
        AutoRemesher::weldPositionsAndTriangles(&positions, &triangles);
        const std::vector<float> expectedPositions {
            0, 0, 0,
            1, 0, 0,
            0, 1, 0,
        };
        CHECK(vecEqual(positions, expectedPositions));
        const std::vector<std::vector<size_t>> expectedTriangles { { 0, 1, 2 } };
        CHECK(trisEqual(triangles, expectedTriangles));
    }

    // No triangles: nothing to weld, positions stay as they are.
    {
        std::vector<float> positions { 0, 0, 0, 1, 0, 0 };
        std::vector<std::vector<size_t>> triangles;
        AutoRemesher::weldPositionsAndTriangles(&positions, &triangles);
        const std::vector<float> expectedPositions { 0, 0, 0, 1, 0, 0 };
        CHECK(vecEqual(positions, expectedPositions));
        CHECK(triangles.empty());
    }

    // Null inputs are ignored, not crashed on.
    {
        std::vector<float> positions { 0, 0, 0 };
        std::vector<std::vector<size_t>> triangles { { 0, 0, 0 } };
        AutoRemesher::weldPositionsAndTriangles(nullptr, &triangles);
        AutoRemesher::weldPositionsAndTriangles(&positions, nullptr);
        CHECK(triangles.size() == 1);
        CHECK(positions.size() == 3);
    }

    if (g_failures == 0)
        std::printf("PASS test_weld\n");
    return g_failures == 0 ? 0 : 1;
}
