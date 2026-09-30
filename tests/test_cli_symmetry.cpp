// CLI --symmetry test: runs the BUILT retopo binary (from the build tree,
// never from PATH) and checks the flag wiring end to end:
//   1. --help lists --symmetry.
//   2. --symmetry bogus fails with a usage error.
//   3. An asymmetric input (bench/models/fandisk.obj) with --symmetry auto
//      exits 0 with valid output and reports the below-threshold fallback
//      on stderr ("Symmetry skipped").
//   4. A symmetric input (an inline unit cube, Y-symmetric among others)
//      with --symmetry auto exits 0 with NO fallback message, and the
//      output vertices are mirror-symmetric about Y through the bbox
//      center within 1e-6 * diagonal.
// Plain assert-style main, no framework.
// NOTE: CHECK instead of <cassert> assert(), which is a no-op under NDEBUG.
//
// Paths arrive as compile definitions from tests/CMakeLists.txt:
//   RETOPO_BINARY          absolute path to the built retopo binary
//   RETOPO_MODEL_FANDISK   absolute path to bench/models/fandisk.obj
#ifndef RETOPO_BINARY
#error "RETOPO_BINARY must be defined (absolute path to the built retopo binary)"
#endif
#ifndef RETOPO_MODEL_FANDISK
#error "RETOPO_MODEL_FANDISK must be defined (absolute path to bench/models/fandisk.obj)"
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

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

// Minimal OBJ vertex reader: collects "v x y z" lines, ignores the rest.
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

// Max over vertices of the distance to the nearest vertex at the mirrored
// position (mirror of p about the Y plane y=offset is (x, 2*offset-y, z);
// a vertex on the plane is its own partner). O(n^2) is fine for test sizes.
double maxMirrorDeviation(const std::vector<Vec3>& verts, double offset)
{
    double worst = 0.0;
    for (const Vec3& p : verts) {
        const double my = 2.0 * offset - p.y;
        double best = 1e300;
        for (const Vec3& q : verts) {
            const double dx = p.x - q.x;
            const double dy = my - q.y;
            const double dz = p.z - q.z;
            const double d = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (d < best)
                best = d;
        }
        if (best > worst)
            worst = best;
    }
    return worst;
}

// Unit cube, side 2, centered on the origin: symmetric about X, Y and Z.
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

} // namespace

int main()
{
    const std::string binary = RETOPO_BINARY;
    const std::string fandisk = RETOPO_MODEL_FANDISK;

    if (!std::filesystem::exists(binary)) {
        std::printf("FAIL %s:%d: retopo binary not found: %s\n", __FILE__, __LINE__, binary.c_str());
        return 1;
    }
    if (!std::filesystem::exists(fandisk)) {
        std::printf("FAIL %s:%d: model not found: %s (run bench/fetch_models.sh)\n",
            __FILE__, __LINE__, fandisk.c_str());
        return 1;
    }

    const std::filesystem::path tmp = std::filesystem::temp_directory_path();
    const std::filesystem::path helpOut = tmp / "retopo_cli_sym_help.txt";
    const std::filesystem::path badOut = tmp / "retopo_cli_sym_bad.txt";
    const std::filesystem::path asymOut = tmp / "retopo_cli_sym_asym.obj";
    const std::filesystem::path asymErr = tmp / "retopo_cli_sym_asym_stderr.txt";
    const std::filesystem::path cubeIn = tmp / "retopo_cli_sym_cube.obj";
    const std::filesystem::path cubeOut = tmp / "retopo_cli_sym_cube_out.obj";
    const std::filesystem::path cubeErr = tmp / "retopo_cli_sym_cube_stderr.txt";
    std::error_code ec;
    for (const auto& p : {helpOut, badOut, asymOut, asymErr, cubeIn, cubeOut, cubeErr})
        std::filesystem::remove(p, ec);

    // 1. Help lists the flag.
    CHECK(0 == std::system(("\"" + binary + "\" --help > \"" + helpOut.string() + "\"").c_str()));
    CHECK(readFile(helpOut.string()).find("--symmetry") != std::string::npos);

    // 2. Bogus value fails loudly.
    const int badRet = std::system(
        ("\"" + binary + "\" --symmetry bogus > \"" + badOut.string() + "\" 2>&1").c_str());
    CHECK(badRet != 0);
    CHECK(readFile(badOut.string()).find("--symmetry expects") != std::string::npos);

    // 3. Asymmetric input: exit 0, valid output, fallback reported on stderr.
    const std::string asymCmd = "\"" + binary + "\" --input \"" + fandisk + "\" --output \""
        + asymOut.string() + "\" --target-quads 1000 --symmetry auto 2> \"" + asymErr.string()
        + "\"";
    CHECK(0 == std::system(asymCmd.c_str()));
    CHECK(std::filesystem::exists(asymOut));
    {
        const std::string err = readFile(asymErr.string());
        if (err.find("Symmetry skipped") == std::string::npos)
            std::printf("asymmetric stderr was:\n%s\n", err.c_str());
        CHECK(err.find("Symmetry skipped") != std::string::npos);
    }
    {
        std::vector<Vec3> verts;
        CHECK(readObjVertices(asymOut.string(), &verts));
        CHECK(!verts.empty());
    }

    // 4. Symmetric input: no fallback, output mirror-symmetric about Y.
    {
        std::ofstream out(cubeIn.string(), std::ios::out | std::ios::binary);
        out << kCubeObj;
    }
    const std::string cubeCmd = "\"" + binary + "\" --input \"" + cubeIn.string()
        + "\" --output \"" + cubeOut.string() + "\" --target-quads 200 --symmetry auto 2> \""
        + cubeErr.string() + "\"";
    CHECK(0 == std::system(cubeCmd.c_str()));
    CHECK(std::filesystem::exists(cubeOut));
    {
        const std::string err = readFile(cubeErr.string());
        if (err.find("Symmetry skipped") != std::string::npos)
            std::printf("cube stderr was:\n%s\n", err.c_str());
        CHECK(err.find("Symmetry skipped") == std::string::npos);
    }
    {
        std::vector<Vec3> verts;
        if (readObjVertices(cubeOut.string(), &verts)) {
            CHECK(!verts.empty());
            if (!verts.empty()) {
                double minY = verts[0].y;
                double maxY = verts[0].y;
                double minX = verts[0].x;
                double maxX = verts[0].x;
                double minZ = verts[0].z;
                double maxZ = verts[0].z;
                for (const Vec3& v : verts) {
                    if (v.y < minY) minY = v.y;
                    if (v.y > maxY) maxY = v.y;
                    if (v.x < minX) minX = v.x;
                    if (v.x > maxX) maxX = v.x;
                    if (v.z < minZ) minZ = v.z;
                    if (v.z > maxZ) maxZ = v.z;
                }
                const double dx = maxX - minX;
                const double dy = maxY - minY;
                const double dz = maxZ - minZ;
                const double diag = std::sqrt(dx * dx + dy * dy + dz * dz);
                const double deviation = maxMirrorDeviation(verts, 0.5 * (minY + maxY));
                std::printf("cube run: %zu verts, max Y-mirror deviation %g (diag %g)\n",
                    verts.size(), deviation, diag);
                CHECK(deviation < 1e-6 * diag);
            }
        }
    }

    if (g_failures == 0)
        std::printf("test_cli_symmetry: all checks passed\n");
    return g_failures == 0 ? 0 : 1;
}
