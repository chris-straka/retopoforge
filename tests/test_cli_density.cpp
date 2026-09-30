// CLI --density test: runs the BUILT retopo binary (from the build tree,
// never from PATH) and checks the density-file wiring end to end:
//   1. --help lists --density.
//   2. A missing density file fails loudly (exit != 0).
//   3. A malformed density file fails loudly (exit != 0).
//   4. A multiplier count that does not match the input vertex count
//      fails loudly (exit != 0).
//   5. --density with a batch directory input is rejected.
//   6. A cube remeshed with a 4x mask on one face exits 0 with a
//      "Density multipliers: 8" line and output vertices that DIFFER
//      from the unmasked run (proving the flag reaches the engine —
//      contrast numbers themselves are test_density's job).
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

// 4x on the +X face corners (v-lines 2,3,6,7), 1.0 elsewhere.
constexpr const char* kMask = R"(1.0
4.0
4.0
1.0
1.0
4.0
4.0
1.0
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
    const std::filesystem::path helpOut = tmp / "retopo_cli_dens_help.txt";
    const std::filesystem::path errOut = tmp / "retopo_cli_dens_err.txt";
    const std::filesystem::path cubeIn = tmp / "retopo_cli_dens_cube.obj";
    const std::filesystem::path maskFile = tmp / "retopo_cli_dens_mask.txt";
    const std::filesystem::path badFile = tmp / "retopo_cli_dens_bad.txt";
    const std::filesystem::path shortFile = tmp / "retopo_cli_dens_short.txt";
    const std::filesystem::path plainOut = tmp / "retopo_cli_dens_plain.obj";
    const std::filesystem::path maskedOut = tmp / "retopo_cli_dens_masked.obj";
    const std::filesystem::path maskedErr = tmp / "retopo_cli_dens_stderr.txt";
    const std::filesystem::path batchDir = tmp / "retopo_cli_dens_batch";
    std::error_code ec;
    for (const auto& p : {helpOut, errOut, cubeIn, maskFile, badFile, shortFile,
            plainOut, maskedOut, maskedErr})
        std::filesystem::remove(p, ec);
    std::filesystem::remove_all(batchDir, ec);

    writeFile(cubeIn.string(), kCubeObj);
    writeFile(maskFile.string(), kMask);
    writeFile(badFile.string(), "abc\n");
    writeFile(shortFile.string(), "1.0\n1.0\n");

    // 1. Help lists the flag.
    CHECK(0 == std::system(("\"" + binary + "\" --help > \"" + helpOut.string() + "\"").c_str()));
    CHECK(readFile(helpOut.string()).find("--density") != std::string::npos);

    // 2. Missing file fails loudly.
    const std::string missingPath = (tmp / "retopo_cli_dens_missing_xyz.txt").string();
    std::filesystem::remove(missingPath, ec);
    CHECK(0 != std::system(("\"" + binary + "\" --input \"" + cubeIn.string() + "\" --output \""
        + plainOut.string() + "\" --density \"" + missingPath + "\" > \"" + errOut.string()
        + "\" 2>&1").c_str()));
    CHECK(readFile(errOut.string()).find("cannot open --density file") != std::string::npos);

    // 3. Malformed content fails loudly.
    CHECK(0 != std::system(("\"" + binary + "\" --input \"" + cubeIn.string() + "\" --output \""
        + plainOut.string() + "\" --density \"" + badFile.string() + "\" > \"" + errOut.string()
        + "\" 2>&1").c_str()));
    CHECK(readFile(errOut.string()).find("expects a number") != std::string::npos);

    // 4. Count mismatch fails loudly.
    CHECK(0 != std::system(("\"" + binary + "\" --input \"" + cubeIn.string() + "\" --output \""
        + plainOut.string() + "\" --density \"" + shortFile.string() + "\" > \"" + errOut.string()
        + "\" 2>&1").c_str()));
    CHECK(readFile(errOut.string()).find("2 multipliers, input has 8 vertices")
        != std::string::npos);

    // 5. Batch + density is rejected.
    std::filesystem::create_directories(batchDir, ec);
    std::filesystem::copy_file(cubeIn, batchDir / "cube.obj", ec);
    CHECK(0 != std::system(("\"" + binary + "\" --input \"" + batchDir.string() + "\" --output \""
        + (tmp / "retopo_cli_dens_batch_out").string() + "\" --density \"" + maskFile.string()
        + "\" > \"" + errOut.string() + "\" 2>&1").c_str()));
    CHECK(readFile(errOut.string()).find("not a batch directory") != std::string::npos);

    // 6. Masked run succeeds and differs from the plain run.
    CHECK(0 == std::system(("\"" + binary + "\" --input \"" + cubeIn.string() + "\" --output \""
        + plainOut.string() + "\" --target-quads 200").c_str()));
    CHECK(std::filesystem::exists(plainOut));
    CHECK(0 == std::system(("\"" + binary + "\" --input \"" + cubeIn.string() + "\" --output \""
        + maskedOut.string() + "\" --target-quads 200 --density \"" + maskFile.string()
        + "\" 2> \"" + maskedErr.string() + "\"").c_str()));
    CHECK(std::filesystem::exists(maskedOut));
    CHECK(readFile(maskedErr.string()).find("Density multipliers: 8") != std::string::npos);

    std::vector<Vec3> plainVerts;
    std::vector<Vec3> maskedVerts;
    if (readObjVertices(plainOut.string(), &plainVerts)
        && readObjVertices(maskedOut.string(), &maskedVerts)) {
        CHECK(!plainVerts.empty());
        CHECK(!maskedVerts.empty());
        bool differ = plainVerts.size() != maskedVerts.size();
        for (size_t i = 0; !differ && i < plainVerts.size() && i < maskedVerts.size(); ++i) {
            const double dx = plainVerts[i].x - maskedVerts[i].x;
            const double dy = plainVerts[i].y - maskedVerts[i].y;
            const double dz = plainVerts[i].z - maskedVerts[i].z;
            if (std::sqrt(dx * dx + dy * dy + dz * dz) > 1e-9)
                differ = true;
        }
        std::printf("cube: plain %zu verts, masked %zu verts, differ=%d\n",
            plainVerts.size(), maskedVerts.size(), differ ? 1 : 0);
        CHECK(differ);
    }

    std::filesystem::remove_all(batchDir, ec);
    if (g_failures == 0)
        std::printf("test_cli_density: all checks passed\n");
    return g_failures == 0 ? 0 : 1;
}
