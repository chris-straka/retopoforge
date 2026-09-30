// CLI --features test: runs the BUILT retopo binary (from the build tree,
// never from PATH) and checks the sharp/feature-file wiring end to end:
//   1. --help lists --features.
//   2. A missing feature file fails loudly (exit != 0).
//   3. A malformed feature file fails loudly (exit != 0).
//   4. --features with a batch directory input is rejected.
//   5. A cube remeshed with its 12-edge cage exits 0 with a
//      "Feature polylines: 12" line and output vertices that DIFFER from
//      the unmarked run (proving the flag reaches the engine, not just
//      the parser — crispness itself is test_sharp's job).
//   6. --features works in --lods mode (single mesh, two rungs).
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

// The cube's 12 edges in input-mesh coordinates: same polyline file
// format as --guides (one 'x y z' per line, blank line per polyline).
constexpr const char* kCageFeatures = R"(# verticals
-1 -1 -1
-1 1 -1

1 -1 -1
1 1 -1

-1 -1 1
-1 1 1

1 -1 1
1 1 1

# top rim (y=1)
-1 1 -1
1 1 -1

1 1 -1
1 1 1

1 1 1
-1 1 1

-1 1 1
-1 1 -1

# bottom rim (y=-1)
-1 -1 -1
1 -1 -1

1 -1 -1
1 -1 1

1 -1 1
-1 -1 1

-1 -1 1
-1 -1 -1
)";

} // namespace

int main()
{
    const std::string binary = RETOPO_BINARY;

    if (!std::filesystem::exists(binary)) {
        std::printf("FAIL %s:%d: retopo binary not found: %s\n", __FILE__, __LINE__, binary.c_str());
        return 1;
    }

    const std::filesystem::path tmp = std::filesystem::temp_directory_path();
    const std::filesystem::path helpOut = tmp / "retopo_cli_features_help.txt";
    const std::filesystem::path errOut = tmp / "retopo_cli_features_err.txt";
    const std::filesystem::path cubeIn = tmp / "retopo_cli_features_cube.obj";
    const std::filesystem::path cageFile = tmp / "retopo_cli_features_cage.txt";
    const std::filesystem::path badFile = tmp / "retopo_cli_features_bad.txt";
    const std::filesystem::path plainOut = tmp / "retopo_cli_features_plain.obj";
    const std::filesystem::path markedOut = tmp / "retopo_cli_features_marked.obj";
    const std::filesystem::path markedErr = tmp / "retopo_cli_features_stderr.txt";
    const std::filesystem::path batchDir = tmp / "retopo_cli_features_batch";
    const std::filesystem::path lodBase = tmp / "retopo_cli_features_lod.obj";
    std::error_code ec;
    for (const auto& p : {helpOut, errOut, cubeIn, cageFile, badFile, plainOut, markedOut, markedErr})
        std::filesystem::remove(p, ec);
    std::filesystem::remove_all(batchDir, ec);

    writeFile(cubeIn.string(), kCubeObj);
    writeFile(cageFile.string(), kCageFeatures);
    writeFile(badFile.string(), "1 2\n");

    // 1. Help lists the flag.
    CHECK(0 == std::system(("\"" + binary + "\" --help > \"" + helpOut.string() + "\"").c_str()));
    CHECK(readFile(helpOut.string()).find("--features") != std::string::npos);

    // 2. Missing file fails loudly.
    const std::string missingPath = (tmp / "retopo_cli_features_missing_xyz.txt").string();
    std::filesystem::remove(missingPath, ec);
    CHECK(0 != std::system(("\"" + binary + "\" --input \"" + cubeIn.string() + "\" --output \""
        + plainOut.string() + "\" --features \"" + missingPath + "\" > \"" + errOut.string()
        + "\" 2>&1").c_str()));
    CHECK(readFile(errOut.string()).find("cannot open --features file") != std::string::npos);

    // 3. Malformed content fails loudly.
    CHECK(0 != std::system(("\"" + binary + "\" --input \"" + cubeIn.string() + "\" --output \""
        + plainOut.string() + "\" --features \"" + badFile.string() + "\" > \"" + errOut.string()
        + "\" 2>&1").c_str()));
    CHECK(readFile(errOut.string()).find("expects 'x y z'") != std::string::npos);

    // 4. Batch + features is rejected.
    std::filesystem::create_directories(batchDir, ec);
    std::filesystem::copy_file(cubeIn, batchDir / "cube.obj", ec);
    CHECK(0 != std::system(("\"" + binary + "\" --input \"" + batchDir.string() + "\" --output \""
        + (tmp / "retopo_cli_features_batch_out").string() + "\" --features \"" + cageFile.string()
        + "\" > \"" + errOut.string() + "\" 2>&1").c_str()));
    CHECK(readFile(errOut.string()).find("not a batch directory") != std::string::npos);

    // 5. Marked run succeeds and differs from the unmarked run.
    CHECK(0 == std::system(("\"" + binary + "\" --input \"" + cubeIn.string() + "\" --output \""
        + plainOut.string() + "\" --target-quads 200").c_str()));
    CHECK(std::filesystem::exists(plainOut));
    CHECK(0 == std::system(("\"" + binary + "\" --input \"" + cubeIn.string() + "\" --output \""
        + markedOut.string() + "\" --target-quads 200 --features \"" + cageFile.string()
        + "\" 2> \"" + markedErr.string() + "\"").c_str()));
    CHECK(std::filesystem::exists(markedOut));
    CHECK(readFile(markedErr.string()).find("Feature polylines: 12") != std::string::npos);

    std::vector<Vec3> plainVerts;
    std::vector<Vec3> markedVerts;
    if (readObjVertices(plainOut.string(), &plainVerts)
        && readObjVertices(markedOut.string(), &markedVerts)) {
        CHECK(!plainVerts.empty());
        CHECK(!markedVerts.empty());
        bool differ = plainVerts.size() != markedVerts.size();
        for (size_t i = 0; !differ && i < plainVerts.size() && i < markedVerts.size(); ++i) {
            const double dx = plainVerts[i].x - markedVerts[i].x;
            const double dy = plainVerts[i].y - markedVerts[i].y;
            const double dz = plainVerts[i].z - markedVerts[i].z;
            if (std::sqrt(dx * dx + dy * dy + dz * dz) > 1e-9)
                differ = true;
        }
        std::printf("cube: plain %zu verts, marked %zu verts, differ=%d\n",
            plainVerts.size(), markedVerts.size(), differ ? 1 : 0);
        CHECK(differ);
    }

    // 6. --lods + --features: two rungs, both emitted.
    const std::filesystem::path lod0 = tmp / "retopo_cli_features_lod_lod0.obj";
    const std::filesystem::path lod1 = tmp / "retopo_cli_features_lod_lod1.obj";
    std::filesystem::remove(lod0, ec);
    std::filesystem::remove(lod1, ec);
    CHECK(0 == std::system(("\"" + binary + "\" --input \"" + cubeIn.string() + "\" --output \""
        + lodBase.string() + "\" --lods 200,100 --features \"" + cageFile.string()
        + "\" > \"" + errOut.string() + "\" 2>&1").c_str()));
    CHECK(std::filesystem::exists(lod0));
    CHECK(std::filesystem::exists(lod1));

    std::filesystem::remove_all(batchDir, ec);
    if (g_failures == 0)
        std::printf("test_cli_features: all checks passed\n");
    return g_failures == 0 ? 0 : 1;
}
