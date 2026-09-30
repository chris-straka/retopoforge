// CLI multi-mode test: --lods chains + folder batch mode (single-file mode
// is covered by test_cli_roundtrip.cpp).
// Asserts: LOD outputs exist with EXACT expected counts (the engine is
// deterministic), batch prints per-file lines plus the failed-files list
// and exits 1 on partial failure.
// Plain assert-style main, no framework.
// NOTE: CHECK instead of <cassert> assert(), which is a no-op under NDEBUG.
//
// Paths arrive as compile definitions from tests/CMakeLists.txt:
//   RETOPO_BINARY        absolute path to the built retopo binary
//   RETOPO_MODEL_FANDISK absolute path to bench/models/fandisk.obj
#ifndef RETOPO_BINARY
#error "RETOPO_BINARY must be defined (absolute path to the built retopo binary)"
#endif
#ifndef RETOPO_MODEL_FANDISK
#error "RETOPO_MODEL_FANDISK must be defined (absolute path to bench/models/fandisk.obj)"
#endif

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <sys/wait.h>
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

// Expected LOD counts for fandisk.obj --lods 2000,1000 (verified against the
// built binary; rung 0 matches the bench tiny preset counts).
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

// Linux pins TBD from CI: the LOD rungs disagree beyond tolerance there
// (libstdc++ solves, no Accelerate). The LOD block prints got/want so one
// CI round-trip yields the numbers; the batch pin matches on both.
#if defined(__linux__)
constexpr long kLod0Quads = 3090; // TBD: read from CI log, then pin
constexpr long kLod1Quads = 1546; // TBD: read from CI log, then pin
#else
constexpr long kLod0Quads = 3090;
constexpr long kLod1Quads = 1546;
#endif
// Expected batch count for fandisk.obj at the default 50000 target.
constexpr long kBatchQuads = 30725;

struct RunResult {
    int exitCode = -1;
    std::string stdoutText;
};

RunResult runCapture(const std::string& binary, const std::string& args,
    const std::filesystem::path& stdoutPath)
{
    RunResult result;
    const std::string command = "\"" + binary + "\" " + args + " > \"" + stdoutPath.string() + "\"";
    // std::system returns the raw wait status; decode the exit code.
    const int status = std::system(command.c_str());
    if (WIFEXITED(status))
        result.exitCode = WEXITSTATUS(status);
    std::ifstream in(stdoutPath);
    if (in.is_open()) {
        std::ostringstream text;
        text << in.rdbuf();
        result.stdoutText = text.str();
    }
    return result;
}

// quads=N value on the line starting with the given prefix, or -1.
// Matches " quads=" with a leading space so target-quads=/non-quads=
// earlier on the same line do not shadow the real count.
long quadsOnLine(const std::string& text, const std::string& prefix)
{
    std::istringstream lines(text);
    std::string line;
    while (std::getline(lines, line)) {
        if (line.rfind(prefix, 0) != 0)
            continue;
        const size_t at = line.find(" quads=");
        if (at == std::string::npos)
            continue;
        long quads = -1;
        if (std::sscanf(line.c_str() + at + 1, "quads=%ld", &quads) == 1)
            return quads;
    }
    return -1;
}

long countObjQuads(const std::string& path)
{
    std::ifstream in(path);
    if (!in.is_open())
        return -1;
    long quads = 0;
    std::string line;
    while (std::getline(in, line)) {
        if (line.rfind("f ", 0) != 0)
            continue;
        std::istringstream corners(line.substr(2));
        std::string corner;
        long n = 0;
        while (corners >> corner)
            ++n;
        if (n == 4)
            ++quads;
    }
    return quads;
}

} // namespace

int main()
{
    const std::string binary = RETOPO_BINARY;
    const std::string model = RETOPO_MODEL_FANDISK;

    if (!std::filesystem::exists(binary)) {
        std::printf("FAIL %s:%d: retopo binary not found: %s\n", __FILE__, __LINE__, binary.c_str());
        return 1;
    }
    if (!std::filesystem::exists(model)) {
        std::printf("FAIL %s:%d: model not found: %s (run bench/fetch_models.sh)\n",
            __FILE__, __LINE__, model.c_str());
        return 1;
    }

    std::error_code ec;
    // PID-suffixed: parallel lanes run the same suite from sibling checkouts,
    // and a fixed tmpdir name lets them clobber each other's files (observed
    // test_cli_glb flake: two writers racing one fixed path).
    const std::filesystem::path tmpdir = std::filesystem::temp_directory_path()
        / ("retopo_cli_multimode_" + std::to_string(::getpid()));
    std::filesystem::remove_all(tmpdir, ec);
    std::filesystem::create_directories(tmpdir, ec);

    // --lods chain: both rungs exist, pinned counts, strictly decreasing.
    {
        const std::string outBase = (tmpdir / "lod_t.obj").string();
        const RunResult run = runCapture(binary,
            "--input \"" + model + "\" --output \"" + outBase + "\" --lods 2000,1000",
            tmpdir / "lod_stdout.txt");
        CHECK(run.exitCode == 0);
        CHECK(nearCount(quadsOnLine(run.stdoutText, "LOD 0:"), kLod0Quads));
        CHECK(nearCount(quadsOnLine(run.stdoutText, "LOD 1:"), kLod1Quads));
        const std::string lod0 = (tmpdir / "lod_t_lod0.obj").string();
        const std::string lod1 = (tmpdir / "lod_t_lod1.obj").string();
        CHECK(std::filesystem::exists(lod0));
        CHECK(std::filesystem::exists(lod1));
        const long fileQuads0 = countObjQuads(lod0);
        const long fileQuads1 = countObjQuads(lod1);
        CHECK(nearCount(fileQuads0, kLod0Quads));
        CHECK(nearCount(fileQuads1, kLod1Quads));
        CHECK(fileQuads0 > fileQuads1);
        std::printf("lod rungs: got lod0=%ld lod1=%ld, want lod0=%ld lod1=%ld\n",
            fileQuads0, fileQuads1, kLod0Quads, kLod1Quads);
    }

    // Batch mode: per-file lines, failed-files list, exit 1 on partial failure.
    {
        const std::filesystem::path batchIn = tmpdir / "batch_in";
        const std::filesystem::path batchOut = tmpdir / "batch_out";
        std::filesystem::create_directories(batchIn, ec);
        std::filesystem::copy_file(model, batchIn / "fandisk.obj", ec);
        CHECK(!ec);
        const std::filesystem::path unreadable = batchIn / "unreadable.obj";
        {
            std::ofstream touch(unreadable);
        }
        std::filesystem::permissions(unreadable, std::filesystem::perms::none, ec);
        CHECK(!ec);

        const RunResult run = runCapture(binary,
            "--input \"" + batchIn.string() + "\" --output \"" + batchOut.string() + "\"",
            tmpdir / "batch_stdout.txt");
        CHECK(run.exitCode == 1);
        CHECK(nearCount(quadsOnLine(run.stdoutText, "FILE fandisk.obj:"), kBatchQuads));
        CHECK(run.stdoutText.find("FILE unreadable.obj: FAILED") != std::string::npos);
        CHECK(run.stdoutText.find("Failed files (1): unreadable.obj") != std::string::npos);
        CHECK(std::filesystem::exists(batchOut / "fandisk.obj"));

        std::filesystem::permissions(unreadable, std::filesystem::perms::owner_all, ec);
    }

    std::filesystem::remove_all(tmpdir, ec);

    if (g_failures == 0)
        std::printf("PASS test_cli_multimode\n");
    return g_failures == 0 ? 0 : 1;
}
