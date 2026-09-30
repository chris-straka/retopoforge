// CLI round-trip test: runs the BUILT retopo binary (from the build tree,
// never from PATH) on bench/models/armadillo.obj with the bench "small"
// preset flags, then validates the output OBJ.
// Asserts: exit 0, a parseable output mesh, and counts EXACTLY matching
// bench/baseline.json (the engine is deterministic).
// Plain assert-style main, no framework.
// NOTE: CHECK instead of <cassert> assert(), which is a no-op under NDEBUG.
//
// Paths arrive as compile definitions from tests/CMakeLists.txt:
//   RETOPO_BINARY          absolute path to the built retopo binary
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

// Expected counts for armadillo.obj + small preset ("--target-quads 5000"),
// copied from bench/baseline.json. If the engine changes these numbers,
// update baseline.json (via the bench harness) and this test together.
constexpr long kExpectedVerts = 4595;
constexpr long kExpectedQuads = 4566;
constexpr long kExpectedNonQuads = 20;

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
        / "retopo_cli_roundtrip_out.obj";
    std::error_code ec;
    std::filesystem::remove(outputPath, ec);

    // Bench "small" preset for armadillo.obj: --target-quads 5000.
    const std::string command = "\"" + binary + "\" --input \"" + model
        + "\" --output \"" + outputPath.string() + "\" --target-quads 5000";
    const int ret = std::system(command.c_str());
    CHECK(ret == 0);
    CHECK(std::filesystem::exists(outputPath));

    if (g_failures == 0) {
        ObjCounts counts;
        if (parseOutputObj(outputPath.string(), &counts)) {
            CHECK(counts.errors == 0);
            CHECK(counts.verts == kExpectedVerts);
            CHECK(counts.quads == kExpectedQuads);
            CHECK(counts.nonQuads == kExpectedNonQuads);
            if (counts.verts != kExpectedVerts || counts.quads != kExpectedQuads
                || counts.nonQuads != kExpectedNonQuads) {
                std::printf("got verts=%ld quads=%ld non_quads=%ld, want verts=%ld quads=%ld non_quads=%ld\n",
                    counts.verts, counts.quads, counts.nonQuads,
                    kExpectedVerts, kExpectedQuads, kExpectedNonQuads);
            }
        }
    }

    std::filesystem::remove(outputPath, ec);

    if (g_failures == 0)
        std::printf("PASS test_cli_roundtrip\n");
    return g_failures == 0 ? 0 : 1;
}
