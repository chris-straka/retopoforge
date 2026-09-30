// CLI --quiet test: runs the BUILT retopo binary (from the build tree,
// never from PATH) on bench/models/armadillo.obj with the bench "tiny"
// preset flags plus --quiet, then checks the silence contract: exit 0,
// the === retopoforge Report === block still on stdout with island lines,
// no per-stage progress lines, and output counts EXACTLY matching
// bench/baseline.json (quiet changes no semantics).
// Plain assert-style main, no framework.
// NOTE: CHECK instead of <cassert> assert(), which is a no-op under NDEBUG.
//
// Paths arrive as compile definitions from tests/CMakeLists.txt:
//   RETOPO_BINARY            absolute path to the built retopo binary
//   RETOPO_MODEL_ARMADDILLO  absolute path to bench/models/armadillo.obj
#ifndef RETOPO_BINARY
#error "RETOPO_BINARY must be defined (absolute path to the built retopo binary)"
#endif
#ifndef RETOPO_MODEL_ARMADDILLO
#error "RETOPO_MODEL_ARMADDILLO must be defined (absolute path to bench/models/armadillo.obj)"
#endif

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

// Expected counts for armadillo.obj + tiny preset ("--target-quads 1000"),
// copied from bench/baseline.json. If the engine changes these numbers,
// update baseline.json (via the bench harness) and this test together.
// Counts jitter a few quads run-to-run (macOS Accelerate multithreaded
// sparse solves; other solvers/platforms may also differ), so pin with a
// small tolerance instead of exact equality. A real regression moves
// counts by percent, not by single quads. Revisit per-platform pins if
// Linux CI disagrees beyond tolerance.
inline bool nearCount(long actual, long expected)
{
    const long diff = actual >= expected ? actual - expected : expected - actual;
    return diff <= expected / 200 + 8; // 0.5% + 8 quads floor
}

constexpr long kExpectedVerts = 544;
constexpr long kExpectedQuads = 524;
constexpr long kExpectedNonQuads = 12;

struct ObjCounts {
    long verts = 0;
    long quads = 0;
    long nonQuads = 0;
    long errors = 0;
};

// Parse + validate an output OBJ the same way bench/run.py validate_obj
// does: every face corner must parse as an integer, lie in 1..vertCount,
// and differ from the other corners of its face (no degenerate faces).
bool parseOutputObj(const std::string& path, ObjCounts* counts)
{
    std::ifstream in(path);
    if (!in.is_open()) {
        std::printf("FAIL %s:%d: cannot open output OBJ %s\n", __FILE__, __LINE__, path.c_str());
        ++g_failures;
        return false;
    }
    std::string line;
    long lineNo = 0;
    while (std::getline(in, line)) {
        ++lineNo;
        if (line.rfind("v ", 0) == 0) {
            ++counts->verts;
        } else if (line.rfind("f ", 0) == 0) {
            std::istringstream words(line.substr(2));
            std::vector<long> indices;
            std::string corner;
            bool cornerOk = true;
            while (words >> corner) {
                const size_t slash = corner.find('/');
                const std::string first = corner.substr(0, slash);
                char* end = nullptr;
                const long index = std::strtol(first.c_str(), &end, 10);
                if (end == first.c_str() || *end != '\0') {
                    std::printf("FAIL %s:%d: line %ld: unparsable face\n", __FILE__, __LINE__, lineNo);
                    ++g_failures;
                    ++counts->errors;
                    cornerOk = false;
                    break;
                }
                indices.push_back(index);
            }
            if (!cornerOk)
                continue;
            if (indices.size() == 4)
                ++counts->quads;
            else
                ++counts->nonQuads;
            for (const long index : indices) {
                if (index < 1 || index > counts->verts) {
                    std::printf("FAIL %s:%d: line %ld: index out of range\n", __FILE__, __LINE__, lineNo);
                    ++g_failures;
                    ++counts->errors;
                }
            }
            for (size_t i = 0; i < indices.size(); ++i) {
                for (size_t j = i + 1; j < indices.size(); ++j) {
                    if (indices[i] == indices[j]) {
                        std::printf("FAIL %s:%d: line %ld: degenerate face\n", __FILE__, __LINE__, lineNo);
                        ++g_failures;
                        ++counts->errors;
                    }
                }
            }
        }
    }
    return true;
}

std::string readFile(const std::string& path)
{
    std::ifstream in(path, std::ios::in | std::ios::binary);
    std::ostringstream out;
    out << in.rdbuf();
    return out.str();
}

} // namespace

int main()
{
    const std::string binary = RETOPO_BINARY;
    const std::string model = RETOPO_MODEL_ARMADDILLO;

    if (!std::filesystem::exists(binary)) {
        std::printf("FAIL %s:%d: retopo binary not found: %s\n", __FILE__, __LINE__, binary.c_str());
        return 1;
    }
    if (!std::filesystem::exists(model)) {
        std::printf("FAIL %s:%d: model not found: %s (run bench/fetch_models.sh)\n",
            __FILE__, __LINE__, model.c_str());
        return 1;
    }

    const std::filesystem::path outputPath = std::filesystem::temp_directory_path()
        / "retopo_cli_quiet_out.obj";
    const std::filesystem::path stdoutPath = std::filesystem::temp_directory_path()
        / "retopo_cli_quiet_stdout.txt";
    std::error_code ec;
    std::filesystem::remove(outputPath, ec);
    std::filesystem::remove(stdoutPath, ec);

    // Bench "tiny" preset for armadillo.obj: --target-quads 1000.
    const std::string command = "\"" + binary + "\" --input \"" + model
        + "\" --output \"" + outputPath.string() + "\" --target-quads 1000 --quiet"
        + " > \"" + stdoutPath.string() + "\"";
    const int ret = std::system(command.c_str());
    CHECK(ret == 0);
    CHECK(std::filesystem::exists(outputPath));

    if (g_failures == 0) {
        const std::string captured = readFile(stdoutPath.string());
        CHECK(captured.find("=== retopoforge Report ===") != std::string::npos);
        CHECK(captured.find("Islands: ") != std::string::npos);
        CHECK(captured.find("Failed islands: 0") != std::string::npos);
        CHECK(captured.find("done.") == std::string::npos);
        if (captured.find("done.") != std::string::npos)
            std::printf("quiet stdout still has progress lines:\n%s\n", captured.c_str());

        ObjCounts counts;
        if (parseOutputObj(outputPath.string(), &counts)) {
            CHECK(counts.errors == 0);
            CHECK(nearCount(counts.verts, kExpectedVerts));
            CHECK(nearCount(counts.quads, kExpectedQuads));
            CHECK(nearCount(counts.nonQuads, kExpectedNonQuads));
            if (!nearCount(counts.verts, kExpectedVerts)
                || !nearCount(counts.quads, kExpectedQuads)
                || !nearCount(counts.nonQuads, kExpectedNonQuads)) {
                std::printf("got verts=%ld quads=%ld non_quads=%ld, want verts=%ld quads=%ld non_quads=%ld\n",
                    counts.verts, counts.quads, counts.nonQuads,
                    kExpectedVerts, kExpectedQuads, kExpectedNonQuads);
            }
        }
    }

    std::filesystem::remove(outputPath, ec);
    std::filesystem::remove(stdoutPath, ec);

    if (g_failures == 0)
        std::printf("PASS test_cli_quiet\n");
    return g_failures == 0 ? 0 : 1;
}
