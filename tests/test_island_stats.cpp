// Engine per-island output accounting (AutoRemesher::islandOutputQuadCounts)
// plus the CLI island reporting that reads it. Plain assert-style main,
// no third-party framework.
// NOTE: CHECK instead of <cassert> assert(), which is a no-op under NDEBUG.
//
// Paths arrive as compile definitions from tests/CMakeLists.txt:
//   RETOPO_BINARY  absolute path to the built retopo binary
#ifndef RETOPO_BINARY
#error "RETOPO_BINARY must be defined (absolute path to the built retopo binary)"
#endif

import retopo.core.auto_remesher;
import retopo.core.mesh_separator;
import retopo.core.vector3;
import retopo.core.mesh_separator;
import retopo.core.vector3;

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>

static int g_failures = 0;

#define CHECK(cond)                                                                                \
    do {                                                                                           \
        if (!(cond)) {                                                                             \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                             \
            ++g_failures;                                                                           \
        }                                                                                          \
    } while (0)

namespace {

using Remesher = AutoRemesher::AutoRemesher;
using MeshSeparator = AutoRemesher::MeshSeparator;
using Vector3 = AutoRemesher::Vector3;

// Closed box, outward winding. Offset shifts the whole box along +X so two
// boxes form two well-separated islands.
void buildBox(double offset, std::vector<Vector3>* vertices,
    std::vector<std::vector<size_t>>* triangles)
{
    *vertices = {
        Vector3(-1.0 + offset, -1.0, -1.0),
        Vector3(1.0 + offset, -1.0, -1.0),
        Vector3(1.0 + offset, 1.0, -1.0),
        Vector3(-1.0 + offset, 1.0, -1.0),
        Vector3(-1.0 + offset, -1.0, 1.0),
        Vector3(1.0 + offset, -1.0, 1.0),
        Vector3(1.0 + offset, 1.0, 1.0),
        Vector3(-1.0 + offset, 1.0, 1.0),
    };
    *triangles = {
        { 0, 3, 2 }, { 0, 2, 1 },
        { 4, 5, 6 }, { 4, 6, 7 },
        { 0, 1, 5 }, { 0, 5, 4 },
        { 3, 7, 6 }, { 3, 6, 2 },
        { 0, 4, 7 }, { 0, 7, 3 },
        { 1, 2, 6 }, { 1, 6, 5 },
    };
}

void appendBox(double offset, std::vector<Vector3>* vertices,
    std::vector<std::vector<size_t>>* triangles)
{
    std::vector<Vector3> boxVertices;
    std::vector<std::vector<size_t>> boxTriangles;
    buildBox(offset, &boxVertices, &boxTriangles);
    const size_t base = vertices->size();
    vertices->insert(vertices->end(), boxVertices.begin(), boxVertices.end());
    for (auto triangle : boxTriangles) {
        for (size_t& index : triangle)
            index += base;
        triangles->push_back(triangle);
    }
}

std::string readFile(const std::filesystem::path& path)
{
    std::ifstream in(path, std::ios::in | std::ios::binary);
    std::ostringstream out;
    out << in.rdbuf();
    return out.str();
}

bool contains(const std::string& text, const std::string& needle)
{
    return text.find(needle) != std::string::npos;
}

} // namespace

int main()
{
    // Engine contract: two separated boxes remesh into two counter entries,
    // both productive, summing to exactly the merged face count.
    {
        std::vector<Vector3> vertices;
        std::vector<std::vector<size_t>> triangles;
        appendBox(0.0, &vertices, &triangles);
        appendBox(5.0, &vertices, &triangles);
        std::vector<std::vector<std::vector<size_t>>> islands;
        MeshSeparator::splitToIslands(triangles, islands);
        CHECK(islands.size() == 2);

        Remesher remesher(vertices, triangles);
        remesher.setTargetTriangleCount(800);
        CHECK(remesher.remesh());
        const std::vector<size_t>& counts = remesher.islandOutputQuadCounts();
        size_t total = 0;
        for (size_t c : counts)
            total += c;
        std::printf("two boxes: islands=%zu counts=[%s] total=%zu merged=%zu\n",
            counts.size(),
            (std::to_string(counts.size() > 0 ? counts[0] : 0) + ","
                + std::to_string(counts.size() > 1 ? counts[1] : 0))
                .c_str(),
            total, remesher.remeshedQuads().size());
        CHECK(counts.size() == 2);
        CHECK(counts.size() == 2 && counts[0] > 0 && counts[1] > 0);
        CHECK(total == remesher.remeshedQuads().size());
    }

    // Engine contract: one box, one entry holding the whole output.
    {
        std::vector<Vector3> vertices;
        std::vector<std::vector<size_t>> triangles;
        buildBox(0.0, &vertices, &triangles);
        Remesher remesher(vertices, triangles);
        remesher.setTargetTriangleCount(800);
        CHECK(remesher.remesh());
        const std::vector<size_t>& counts = remesher.islandOutputQuadCounts();
        std::printf("one box: islands=%zu counts=[%s]\n", counts.size(),
            counts.empty() ? "" : std::to_string(counts[0]).c_str());
        CHECK(counts.size() == 1);
        CHECK(!counts.empty() && counts[0] == remesher.remeshedQuads().size());
    }

    // CLI surfacing: single-file mode reports the engine's island totals.
    const std::string binary = RETOPO_BINARY;
    if (!std::filesystem::exists(binary)) {
        std::printf("FAIL %s:%d: retopo binary not found: %s\n", __FILE__, __LINE__, binary.c_str());
        return 1;
    }
    std::error_code ec;
    const std::filesystem::path tmpdir = std::filesystem::temp_directory_path()
        / ("retopo_island_stats_" + std::to_string(::getpid()));
    std::filesystem::remove_all(tmpdir, ec);
    std::filesystem::create_directories(tmpdir, ec);
    const std::filesystem::path inputPath = tmpdir / "two_boxes.obj";
    const std::filesystem::path outputPath = tmpdir / "two_boxes.out.obj";
    const std::filesystem::path stdoutPath = tmpdir / "run.out";
    const std::filesystem::path stderrPath = tmpdir / "run.err";
    {
        std::vector<Vector3> vertices;
        std::vector<std::vector<size_t>> triangles;
        appendBox(0.0, &vertices, &triangles);
        appendBox(5.0, &vertices, &triangles);
        std::ofstream obj(inputPath);
        CHECK(obj.is_open());
        for (const auto& v : vertices)
            obj << "v " << v.x() << " " << v.y() << " " << v.z() << "\n";
        for (const auto& t : triangles)
            obj << "f " << (t[0] + 1) << " " << (t[1] + 1) << " " << (t[2] + 1) << "\n";
    }
    {
        const std::string command = "\"" + binary + "\" --input \"" + inputPath.string()
            + "\" --output \"" + outputPath.string() + "\" --target-quads 400"
            + " > \"" + stdoutPath.string() + "\" 2> \"" + stderrPath.string() + "\"";
        const int status = std::system(command.c_str());
        CHECK(0 != WIFEXITED(status) && 0 == WEXITSTATUS(status));
        const std::string stdoutText = readFile(stdoutPath);
        std::printf("cli report:\n%s\n", stdoutText.c_str());
        CHECK(contains(stdoutText, "Islands: 2"));
        CHECK(contains(stdoutText, "Failed islands: 0"));
    }

    std::filesystem::remove_all(tmpdir, ec);
    if (g_failures == 0)
        std::printf("PASS test_island_stats\n");
    return g_failures == 0 ? 0 : 1;
}
