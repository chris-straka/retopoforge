// CLI --guides test: runs the BUILT retopo binary (from the build tree,
// never from PATH) and checks the guide-file wiring end to end:
//   1. --help lists --guides.
//   2. A missing guide file fails loudly (exit != 0).
//   3. A malformed guide file fails loudly (exit != 0).
//   4. --guides with a batch directory input is rejected.
//   5. A cube remeshed with an equatorial guide loop exits 0 with a
//      "Guide polylines: 1" line and output vertices that DIFFER from
//      the unguided run (proving the flag reaches the engine, not just
//      the parser — flow-alignment quality itself is test_guides' job).
// Plain assert-style main, no framework.
// NOTE: CHECK instead of <cassert> assert(), which is a no-op under NDEBUG.
//
// Paths arrive as compile definitions from tests/CMakeLists.txt:
//   RETOPO_BINARY  absolute path to the built retopo binary
#ifndef RETOPO_BINARY
#error "RETOPO_BINARY must be defined (absolute path to the built retopo binary)"
#endif

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
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

std::string readFile(const std::string& path)
{
    std::ifstream in(path, std::ios::in | std::ios::binary);
    std::ostringstream out;
    out << in.rdbuf();
    return out.str();
}

void writeFile(const std::string& path, const std::string& text)
{
    std::ofstream out(path, std::ios::out | std::ios::binary);
    out << text;
}

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

bool readObjVertices(const std::string& path, std::vector<Vec3>* verts)
{
    std::ifstream in(path);
    if (!in.is_open()) {
        std::printf("FAIL %s:%d: cannot open output OBJ %s\n", __FILE__, __LINE__, path.c_str());
        ++g_failures;
        return false;
    }
    std::string line;
    while (std::getline(in, line)) {
        if (line.rfind("v ", 0) != 0)
            continue;
        Vec3 v;
        if (3 != std::sscanf(line.c_str() + 2, "%lf %lf %lf", &v.x, &v.y, &v.z)) {
            std::printf("FAIL %s:%d: bad vertex line: %s\n", __FILE__, __LINE__, line.c_str());
            ++g_failures;
            return false;
        }
        verts->push_back(v);
    }
    return true;
}

constexpr const char* kCubeObj = R"(v -1 -1 -1
v 1 -1 -1
v 1 1 -1
v -1 1 -1
v -1 -1 1
v 1 -1 1
v 1 1 1
v -1 1 1
f 1 4 3
f 1 3 2
f 5 6 7
f 5 7 8
f 1 2 6
f 1 6 5
f 4 8 7
f 4 7 3
f 1 5 8
f 1 8 4
f 2 3 7
f 2 7 6
)";

constexpr const char* kLoopGuides = R"(# equatorial loop around Y
1.2 0 0
0 0 1.2
-1.2 0 0
0 0 -1.2
1.2 0 0
)";

} // namespace

int main()
{
    const std::string binary = RETOPO_BINARY;

    if (!std::filesystem::exists(binary)) {
        std::printf("FAIL %s:%d: retopo binary not found: %s\n", __FILE__, __LINE__, binary.c_str());
        return 1;
    }

    // PID-suffixed subdir: parallel lanes run the same suite from sibling
    // checkouts, and fixed tmp names let them clobber each other's files.
    const std::filesystem::path tmp = std::filesystem::temp_directory_path()
        / ("retopo_cli_guides_" + std::to_string(::getpid()));
    std::error_code ec0;
    std::filesystem::create_directories(tmp, ec0);
    const std::filesystem::path helpOut = tmp / "retopo_cli_guides_help.txt";
    const std::filesystem::path errOut = tmp / "retopo_cli_guides_err.txt";
    const std::filesystem::path cubeIn = tmp / "retopo_cli_guides_cube.obj";
    const std::filesystem::path loopFile = tmp / "retopo_cli_guides_loop.txt";
    const std::filesystem::path badFile = tmp / "retopo_cli_guides_bad.txt";
    const std::filesystem::path plainOut = tmp / "retopo_cli_guides_plain.obj";
    const std::filesystem::path guidedOut = tmp / "retopo_cli_guides_guided.obj";
    const std::filesystem::path guidedErr = tmp / "retopo_cli_guides_stderr.txt";
    const std::filesystem::path batchDir = tmp / "retopo_cli_guides_batch";
    std::error_code ec;
    for (const auto& p : {helpOut, errOut, cubeIn, loopFile, badFile, plainOut, guidedOut, guidedErr})
        std::filesystem::remove(p, ec);
    std::filesystem::remove_all(batchDir, ec);

    writeFile(cubeIn.string(), kCubeObj);
    writeFile(loopFile.string(), kLoopGuides);
    writeFile(badFile.string(), "1 2\n");

    // 1. Help lists the flag.
    CHECK(0 == std::system(("\"" + binary + "\" --help > \"" + helpOut.string() + "\"").c_str()));
    CHECK(readFile(helpOut.string()).find("--guides") != std::string::npos);

    // 2. Missing file fails loudly.
    const std::string missingPath = (tmp / "retopo_cli_guides_missing_xyz.txt").string();
    std::filesystem::remove(missingPath, ec);
    CHECK(0 != std::system(("\"" + binary + "\" --input \"" + cubeIn.string() + "\" --output \""
        + plainOut.string() + "\" --guides \"" + missingPath + "\" > \"" + errOut.string()
        + "\" 2>&1").c_str()));
    CHECK(readFile(errOut.string()).find("cannot open --guides file") != std::string::npos);

    // 3. Malformed content fails loudly.
    CHECK(0 != std::system(("\"" + binary + "\" --input \"" + cubeIn.string() + "\" --output \""
        + plainOut.string() + "\" --guides \"" + badFile.string() + "\" > \"" + errOut.string()
        + "\" 2>&1").c_str()));
    CHECK(readFile(errOut.string()).find("expects 'x y z'") != std::string::npos);

    // 4. Batch + guides is rejected.
    std::filesystem::create_directories(batchDir, ec);
    std::filesystem::copy_file(cubeIn, batchDir / "cube.obj", ec);
    CHECK(0 != std::system(("\"" + binary + "\" --input \"" + batchDir.string() + "\" --output \""
        + (tmp / "retopo_cli_guides_batch_out").string() + "\" --guides \"" + loopFile.string()
        + "\" > \"" + errOut.string() + "\" 2>&1").c_str()));
    CHECK(readFile(errOut.string()).find("not a batch directory") != std::string::npos);

    // 5. Guided run succeeds and differs from the unguided run.
    CHECK(0 == std::system(("\"" + binary + "\" --input \"" + cubeIn.string() + "\" --output \""
        + plainOut.string() + "\" --target-quads 200").c_str()));
    CHECK(std::filesystem::exists(plainOut));
    CHECK(0 == std::system(("\"" + binary + "\" --input \"" + cubeIn.string() + "\" --output \""
        + guidedOut.string() + "\" --target-quads 200 --guides \"" + loopFile.string()
        + "\" 2> \"" + guidedErr.string() + "\"").c_str()));
    CHECK(std::filesystem::exists(guidedOut));
    CHECK(readFile(guidedErr.string()).find("Guide polylines: 1") != std::string::npos);

    std::vector<Vec3> plainVerts;
    std::vector<Vec3> guidedVerts;
    if (readObjVertices(plainOut.string(), &plainVerts)
        && readObjVertices(guidedOut.string(), &guidedVerts)) {
        CHECK(!plainVerts.empty());
        CHECK(!guidedVerts.empty());
        bool differ = plainVerts.size() != guidedVerts.size();
        for (size_t i = 0; !differ && i < plainVerts.size() && i < guidedVerts.size(); ++i) {
            const double dx = plainVerts[i].x - guidedVerts[i].x;
            const double dy = plainVerts[i].y - guidedVerts[i].y;
            const double dz = plainVerts[i].z - guidedVerts[i].z;
            if (std::sqrt(dx * dx + dy * dy + dz * dz) > 1e-9)
                differ = true;
        }
        std::printf("cube: plain %zu verts, guided %zu verts, differ=%d\n",
            plainVerts.size(), guidedVerts.size(), differ ? 1 : 0);
        CHECK(differ);
    }

    std::filesystem::remove_all(batchDir, ec);
    if (g_failures == 0)
        std::printf("test_cli_guides: all checks passed\n");
    return g_failures == 0 ? 0 : 1;
}
