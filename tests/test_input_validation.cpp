// Input-validation unit tests: loader index validation and weld drops.
// Covers the corpus-robust hardening without running the engine:
//   - the OBJ loader rejects out-of-range / overflowing face indices
//     (these used to index the vertex array out of bounds downstream),
//   - the weld drops NaN/inf-cornered triangles and reports per-reason
//     counts (one NaN corner used to poison the whole run's mesh area),
//   - the engine's remesh() entry validation rejects bad faces instead of
//     crashing (backstop for every loader, including the GLB one).
// Plain assert-style main, no framework.
// NOTE: CHECK instead of <cassert> assert(), which is a no-op under NDEBUG.
#include <AutoRemesher/AutoRemesher>
#include <AutoRemesher/ObjReader>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <unistd.h>
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

int g_tempCounter = 0;

// Write OBJ text to a temp file, load it, remove the file. Returns the
// loader result; fills positions/triangles/warn/err from the call.
bool loadString(const std::string& objText,
    std::vector<float>* positions,
    std::vector<std::vector<size_t>>* triangles,
    std::string* warn,
    std::string* err)
{
    // PID-suffixed: parallel lanes run the same suite from sibling
    // checkouts, and a fixed temp name lets them clobber each other's
    // files (observed test_cli_glb flake).
    const std::filesystem::path path = std::filesystem::temp_directory_path()
        / ("retopo_validation_test_" + std::to_string(::getpid()) + "_" + std::to_string(g_tempCounter++)
            + ".obj");
    {
        std::ofstream out(path, std::ios::out | std::ios::binary);
        out << objText;
        if (!out) {
            std::printf("FAIL %s:%d: cannot write temp file %s\n", __FILE__, __LINE__, path.c_str());
            ++g_failures;
            return false;
        }
    }
    const bool ok = AutoRemesher::loadObjPositionsAndTriangles(
        path.c_str(), positions, triangles, warn, err);
    std::error_code ec;
    std::filesystem::remove(path, ec);
    return ok;
}

bool allFinite(const std::vector<float>& positions)
{
    for (float value : positions) {
        if (!std::isfinite(value))
            return false;
    }
    return true;
}

} // namespace

int main()
{
    std::vector<float> positions;
    std::vector<std::vector<size_t>> triangles;
    std::string warn;
    std::string err;

    // Out-of-range positive index: the load fails and leaves outputs alone
    // (same contract as the zero-index case in test_objreader). Before the
    // fix this triangle flowed downstream and segfaulted in the island build.
    {
        const std::string obj = "v 0 0 0\n"
                                "v 1 0 0\n"
                                "v 0 1 0\n"
                                "f 1 2 99999\n";
        positions = { 9.0f };
        triangles = { { 7 } };
        CHECK(!loadString(obj, &positions, &triangles, &warn, &err));
        CHECK(err.find("Failed parse") != std::string::npos);
        CHECK(positions == std::vector<float> { 9.0f });
        CHECK((triangles == std::vector<std::vector<size_t>> { { 7 } }));
    }

    // Forward reference (face before its vertices): fails the same way.
    // Before the fix this passed by luck whenever the later vertex count
    // happened to cover the cited indices.
    {
        const std::string obj = "f 1 2 3\n"
                                "v 0 0 0\n"
                                "v 1 0 0\n"
                                "v 0 1 0\n";
        CHECK(!loadString(obj, &positions, &triangles, &warn, &err));
        CHECK(err.find("Failed parse") != std::string::npos);
    }

    // Index outside int range: fails instead of wrapping. 2147483648 is
    // INT_MAX+1; the 20-digit value overflows even strtol (clamps to
    // LONG_MAX) and must fail too.
    {
        const std::string obj = "v 0 0 0\n"
                                "v 1 0 0\n"
                                "v 0 1 0\n"
                                "f 1 2 2147483648\n";
        CHECK(!loadString(obj, &positions, &triangles, &warn, &err));
        CHECK(err.find("Failed parse") != std::string::npos);
    }
    {
        const std::string obj = "v 0 0 0\n"
                                "v 1 0 0\n"
                                "v 0 1 0\n"
                                "f 1 2 99999999999999999999\n";
        CHECK(!loadString(obj, &positions, &triangles, &warn, &err));
        CHECK(err.find("Failed parse") != std::string::npos);
    }

    // A good triangle plus a NaN-cornered one: the weld drops exactly the
    // NaN face, removes its now-orphaned vertices, and reports the reason.
    // No NaN survives in the positions.
    {
        const float nan = std::numeric_limits<float>::quiet_NaN();
        std::vector<float> soupPositions {
            0, 0, 0, // 0
            1, 0, 0, // 1
            0, 1, 0, // 2
            nan, 0, 0, // 3
            9, 0, 0, // 4
            9, 1, 0, // 5
        };
        std::vector<std::vector<size_t>> soupTriangles { { 0, 1, 2 }, { 3, 4, 5 } };
        AutoRemesher::WeldStats stats;
        AutoRemesher::weldPositionsAndTriangles(&soupPositions, &soupTriangles, &stats);
        CHECK(stats.nonFiniteDropped == 1);
        CHECK(stats.degenerateDropped == 0);
        CHECK((soupTriangles == std::vector<std::vector<size_t>> { { 0, 1, 2 } }));
        CHECK(allFinite(soupPositions));
        CHECK(soupPositions.size() == 9);
    }

    // Infinity (including the 1e999-overflow spelling, which strtod rounds
    // to inf) is dropped the same way.
    {
        const float inf = std::numeric_limits<float>::infinity();
        std::vector<float> soupPositions {
            0, 0, 0,
            1, 0, 0,
            inf, 1, 0,
        };
        std::vector<std::vector<size_t>> soupTriangles { { 0, 1, 2 } };
        AutoRemesher::WeldStats stats;
        AutoRemesher::weldPositionsAndTriangles(&soupPositions, &soupTriangles, &stats);
        CHECK(stats.nonFiniteDropped == 1);
        CHECK(soupTriangles.empty());
    }

    // Stats on clean input: zero drops, byte-identical input (identity path).
    {
        std::vector<float> cleanPositions {
            0, 0, 0,
            1, 0, 0,
            1, 1, 0,
            0, 1, 0,
        };
        std::vector<std::vector<size_t>> cleanTriangles { { 0, 1, 2 }, { 0, 2, 3 } };
        const std::vector<float> beforePositions = cleanPositions;
        const std::vector<std::vector<size_t>> beforeTriangles = cleanTriangles;
        AutoRemesher::WeldStats stats;
        AutoRemesher::weldPositionsAndTriangles(&cleanPositions, &cleanTriangles, &stats);
        CHECK(stats.nonFiniteDropped == 0);
        CHECK(stats.degenerateDropped == 0);
        CHECK(cleanPositions == beforePositions);
        CHECK(cleanTriangles == beforeTriangles);
    }

    // Degenerate accounting: an index-degenerate face counts before the
    // remap, and a face whose distinct-but-coincident corners collapse
    // under the remap counts after it.
    {
        std::vector<float> soupPositions {
            0, 0, 0,
            1, 0, 0,
            0, 1, 0,
        };
        std::vector<std::vector<size_t>> soupTriangles { { 0, 1, 2 }, { 0, 0, 1 } };
        AutoRemesher::WeldStats stats;
        AutoRemesher::weldPositionsAndTriangles(&soupPositions, &soupTriangles, &stats);
        CHECK(stats.degenerateDropped == 1);
        CHECK(stats.nonFiniteDropped == 0);
        CHECK(soupTriangles.size() == 1);
    }
    {
        std::vector<float> soupPositions {
            0, 0, 0,
            0, 0, 0,
            0, 0, 0,
            1, 0, 0,
            0, 1, 0,
        };
        std::vector<std::vector<size_t>> soupTriangles { { 0, 1, 2 }, { 0, 3, 4 } };
        AutoRemesher::WeldStats stats;
        AutoRemesher::weldPositionsAndTriangles(&soupPositions, &soupTriangles, &stats);
        CHECK(stats.degenerateDropped == 1);
        CHECK(soupTriangles.size() == 1);
    }

    // Engine backstop: out-of-range corners and non-triangle faces fail
    // remesh() loudly instead of indexing out of bounds. (The loaders
    // reject these first; this covers API misuse and any future loader.)
    {
        const std::vector<AutoRemesher::Vector3> vertices(2);
        const std::vector<std::vector<size_t>> badTriangles { { 0, 1, 5000000 } };
        AutoRemesher::AutoRemesher remesher(vertices, badTriangles);
        remesher.setTargetTriangleCount(20);
        CHECK(!remesher.remesh());
    }
    {
        const std::vector<AutoRemesher::Vector3> vertices(4);
        const std::vector<std::vector<size_t>> badTriangles { { 0, 1 } };
        AutoRemesher::AutoRemesher remesher(vertices, badTriangles);
        remesher.setTargetTriangleCount(20);
        CHECK(!remesher.remesh());
    }

    if (g_failures == 0)
        std::printf("PASS test_input_validation\n");
    return g_failures == 0 ? 0 : 1;
}
