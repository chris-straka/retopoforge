// Unit tests for core/objreader (AutoRemesher::loadObjPositionsAndTriangles).
// Synthetic OBJ strings on known geometry, with hardcoded expected
// positions and triangles. Plain assert-style main, no framework.
// NOTE: CHECK instead of <cassert> assert(), which is a no-op under NDEBUG.
#include <AutoRemesher/ObjReader>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>
#include <unistd.h>

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
    const std::filesystem::path path = std::filesystem::temp_directory_path()
        / ("retopo_unit_test_" + std::to_string(::getpid()) + "_"
            + std::to_string(g_tempCounter++) + ".obj");
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

bool vecEqual(const std::vector<float>& a, const std::vector<float>& b)
{
    return a == b;
}

bool trisEqual(const std::vector<std::vector<size_t>>& a, const std::vector<std::vector<size_t>>& b)
{
    return a == b;
}

} // namespace

int main()
{
    std::vector<float> positions;
    std::vector<std::vector<size_t>> triangles;
    std::string warn;
    std::string err;

    // v/vt/vn lines with full v/vt/vn corners; groups, materials, object
    // names, smoothing groups and comments are ignored.
    {
        const std::string obj = "# comment line\n"
                                "mtllib test.mtl\n"
                                "o TestObject\n"
                                "v 0 0 0\n"
                                "v 1 0 0\n"
                                "v 0 1 0\n"
                                "vt 0 0\n"
                                "vn 0 0 1\n"
                                "vp 0.5\n"
                                "g group1\n"
                                "usemtl mat1\n"
                                "s 1\n"
                                "f 1/1/1 2/1/1 3/1/1\n";
        CHECK(loadString(obj, &positions, &triangles, &warn, &err));
        CHECK(err.empty());
        const std::vector<float> expectedPositions { 0, 0, 0, 1, 0, 0, 0, 1, 0 };
        CHECK(vecEqual(positions, expectedPositions));
        const std::vector<std::vector<size_t>> expectedTriangles { { 0, 1, 2 } };
        CHECK(trisEqual(triangles, expectedTriangles));
    }

    // Corner forms v, v/vt, v//vn and v/vt/vn are equivalent; the vt/vn
    // parts are ignored even when their indices are out of range (9).
    {
        const std::string obj = "v 0 0 0\n"
                                "v 2 0 0\n"
                                "v 0 3 0\n"
                                "f 1 2 3\n"
                                "f 1/1 2/1 3/1\n"
                                "f 1//1 2//1 3//1\n"
                                "f 1/9/9 2/9/9 3/9/9\n";
        CHECK(loadString(obj, &positions, &triangles, &warn, &err));
        CHECK(err.empty());
        const std::vector<float> expectedPositions { 0, 0, 0, 2, 0, 0, 0, 3, 0 };
        CHECK(vecEqual(positions, expectedPositions));
        const std::vector<std::vector<size_t>> expectedTriangles { { 0, 1, 2 }, { 0, 1, 2 }, { 0, 1, 2 }, { 0, 1, 2 } };
        CHECK(trisEqual(triangles, expectedTriangles));
    }

    // Negative (relative) indices count back from the last vertex.
    {
        const std::string obj = "v 0 0 0\n"
                                "v 1 0 0\n"
                                "v 1 1 0\n"
                                "f -3 -2 -1\n";
        CHECK(loadString(obj, &positions, &triangles, &warn, &err));
        CHECK(err.empty());
        const std::vector<float> expectedPositions { 0, 0, 0, 1, 0, 0, 1, 1, 0 };
        CHECK(vecEqual(positions, expectedPositions));
        const std::vector<std::vector<size_t>> expectedTriangles { { 0, 1, 2 } };
        CHECK(trisEqual(triangles, expectedTriangles));
    }

    // Convex quad: ear-clip triangulation starting from the first corner.
    {
        const std::string obj = "v 0 0 0\n"
                                "v 1 0 0\n"
                                "v 1 1 0\n"
                                "v 0 1 0\n"
                                "f 1 2 3 4\n";
        CHECK(loadString(obj, &positions, &triangles, &warn, &err));
        CHECK(err.empty());
        const std::vector<float> expectedPositions { 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0 };
        CHECK(vecEqual(positions, expectedPositions));
        const std::vector<std::vector<size_t>> expectedTriangles { { 0, 1, 2 }, { 0, 2, 3 } };
        CHECK(trisEqual(triangles, expectedTriangles));
    }

    // Concave pentagon (notch at vertex 3 = (1,1)): ear clipping skips the
    // reflex corner (0,2,3) and emits three triangles covering all corners.
    {
        const std::string obj = "v 0 0 0\n"
                                "v 3 0 0\n"
                                "v 1 1 0\n"
                                "v 3 2 0\n"
                                "v 0 2 0\n"
                                "f 1 2 3 4 5\n";
        CHECK(loadString(obj, &positions, &triangles, &warn, &err));
        CHECK(err.empty());
        const std::vector<float> expectedPositions { 0, 0, 0, 3, 0, 0, 1, 1, 0, 3, 2, 0, 0, 2, 0 };
        CHECK(vecEqual(positions, expectedPositions));
        const std::vector<std::vector<size_t>> expectedTriangles { { 0, 1, 2 }, { 2, 3, 4 }, { 0, 2, 4 } };
        CHECK(trisEqual(triangles, expectedTriangles));
        CHECK(triangles.size() == 3); // n-gon yields n-2 triangles
    }

    // CRLF line endings parse identically to LF.
    {
        const std::string obj = "v 0 0 0\r\n"
                                "v 1 0 0\r\n"
                                "v 0 1 0\r\n"
                                "f 1 2 3\r\n";
        CHECK(loadString(obj, &positions, &triangles, &warn, &err));
        CHECK(err.empty());
        const std::vector<float> expectedPositions { 0, 0, 0, 1, 0, 0, 0, 1, 0 };
        CHECK(vecEqual(positions, expectedPositions));
        const std::vector<std::vector<size_t>> expectedTriangles { { 0, 1, 2 } };
        CHECK(trisEqual(triangles, expectedTriangles));
    }

    // Degenerate face with fewer than 3 corners: loads fine, emits nothing.
    {
        const std::string obj = "v 0 0 0\n"
                                "v 1 0 0\n"
                                "f 1 2\n";
        CHECK(loadString(obj, &positions, &triangles, &warn, &err));
        CHECK(err.empty());
        const std::vector<float> expectedPositions { 0, 0, 0, 1, 0, 0 };
        CHECK(vecEqual(positions, expectedPositions));
        CHECK(triangles.empty());
    }

    // Degenerate face with repeated corners passes through as-is.
    {
        const std::string obj = "v 0 0 0\n"
                                "f 1 1 1\n";
        CHECK(loadString(obj, &positions, &triangles, &warn, &err));
        CHECK(err.empty());
        const std::vector<float> expectedPositions { 0, 0, 0 };
        CHECK(vecEqual(positions, expectedPositions));
        const std::vector<std::vector<size_t>> expectedTriangles { { 0, 0, 0 } };
        CHECK(trisEqual(triangles, expectedTriangles));
    }

    // Zero face index is invalid: the load fails and leaves outputs alone.
    {
        const std::string obj = "v 0 0 0\n"
                                "v 1 0 0\n"
                                "v 0 1 0\n"
                                "f 1 0 3\n";
        positions = { 9.0f };
        triangles = { { 7 } };
        CHECK(!loadString(obj, &positions, &triangles, &warn, &err));
        CHECK(err.find("Failed parse") != std::string::npos);
        const std::vector<float> untouchedPositions { 9.0f };
        CHECK(vecEqual(positions, untouchedPositions));
        const std::vector<std::vector<size_t>> untouchedTriangles { { 7 } };
        CHECK(trisEqual(triangles, untouchedTriangles));
    }

    // Missing file: the load fails with a "Cannot open file" error.
    {
        positions = { 9.0f };
        triangles = { { 7 } };
        warn.clear();
        err.clear();
        const bool ok = AutoRemesher::loadObjPositionsAndTriangles(
            "/nonexistent-dir-38f2/does-not-exist.obj", &positions, &triangles, &warn, &err);
        CHECK(!ok);
        CHECK(warn.empty());
        CHECK(err.find("Cannot open file") != std::string::npos);
        const std::vector<float> untouchedPositions { 9.0f };
        CHECK(vecEqual(positions, untouchedPositions));
        const std::vector<std::vector<size_t>> untouchedTriangles { { 7 } };
        CHECK(trisEqual(triangles, untouchedTriangles));
    }

    if (g_failures == 0)
        std::printf("PASS test_objreader\n");
    return g_failures == 0 ? 0 : 1;
}
