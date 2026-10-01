// Differential oracle dump for the obj_reader Rust port (rs-objreader lane).
// Generates seeded random OBJ documents (plus a fixed numeric/structural edge
// battery and the committed nasty-*.obj corpus, embedded), loads each with the
// C++ implementation, and prints inputs + outputs in a format that
// rust/core/tests/obj_reader_diff.rs replays. Positions print as f32 bit
// patterns (%08x), so the replay compares bitwise. Also emits randomized
// direct-weld cases and times large inputs for the runtime ratio.
//
// Build-only helper: not registered with ctest. Run it with the fixtures dir
// and redirect stdout to tests/fixtures/objreader_diff.txt, then commit the
// fixture:
//   ./build-<id>/tests/objreader_diff_dump tests/fixtures > tests/fixtures/objreader_diff.txt
import retopo.core.obj_reader;

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <unistd.h>
#include <vector>

static std::uint64_t g_state = 0x0B3EC7D1A5510B3Bull;

static std::uint64_t nextU64()
{
    std::uint64_t z = (g_state += 0x9E3779B97F4A7C15ull);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
}

static std::uint64_t below(std::uint64_t n)
{
    return nextU64() % n;
}

static const std::string& pick(const std::vector<std::string>& v)
{
    return v[below(v.size())];
}

// Escape arbitrary bytes for one fixture line: printable ASCII (except
// backslash) raw, backslash/CR/LF/NUL short escapes, everything else \xNN.
static std::string escapeBytes(const std::string& s)
{
    std::string out;
    for (unsigned char c : s) {
        if (c == '\\')
            out += "\\\\";
        else if (c == '\r')
            out += "\\r";
        else if (c == '\n')
            out += "\\n";
        else if (c == '\0')
            out += "\\0";
        else if (c >= 0x20 && c <= 0x7E)
            out += (char)c;
        else {
            char buf[8];
            std::snprintf(buf, sizeof(buf), "\\x%02x", c);
            out += buf;
        }
    }
    return out;
}

static int g_tempCounter = 0;

static std::string writeTemp(const std::string& bytes)
{
    static int pid = 0;
    if (!pid)
        pid = (int)::getpid();
    std::filesystem::path path = std::filesystem::temp_directory_path()
        / ("retopo_objdiff_" + std::to_string(pid) + "_" + std::to_string(g_tempCounter++) + ".obj");
    std::ofstream out(path, std::ios::out | std::ios::binary);
    out.write(bytes.data(), (std::streamsize)bytes.size());
    out.close();
    return path.string();
}

static void dumpPosLine(const std::vector<float>& positions)
{
    std::printf("POS");
    for (float f : positions) {
        std::uint32_t bits;
        std::memcpy(&bits, &f, 4);
        std::printf(" %08x", bits);
    }
    std::printf("\n");
}

static void dumpTris(const std::vector<std::vector<size_t>>& tris, const char* tag)
{
    for (const auto& face : tris) {
        std::printf("%s %zu", tag, face.size());
        for (size_t c : face)
            std::printf(" %zu", c);
        std::printf("\n");
    }
}

// Sentinel pre-fill: on ok=0 the loader must leave outputs untouched, so both
// sides pre-fill with these and the fixture pins the post-call contents.
static std::vector<float> sentinelPositions()
{
    return std::vector<float> { 1.5f, -2.5f };
}

static std::vector<std::vector<size_t>> sentinelTriangles()
{
    return std::vector<std::vector<size_t>> { { 99 }, { 7, 8 } };
}

static void dumpLoadCase(const std::string& objBytes, const std::string& header)
{
    // Split into lines for the fixture (joining with \n reproduces the bytes).
    std::vector<std::string> lines;
    {
        size_t start = 0;
        for (size_t i = 0; i <= objBytes.size(); ++i) {
            if (i == objBytes.size() || objBytes[i] == '\n') {
                lines.push_back(objBytes.substr(start, i - start));
                start = i + 1;
            }
        }
    }
    std::printf("%s LINES %zu\n", header.c_str(), lines.size());
    for (const std::string& line : lines)
        std::printf("%s\n", escapeBytes(line).c_str());

    std::string path = writeTemp(objBytes);
    std::vector<float> positions = sentinelPositions();
    std::vector<std::vector<size_t>> triangles = sentinelTriangles();
    std::string warn, err;
    bool ok = AutoRemesher::loadObjPositionsAndTriangles(
        path.c_str(), &positions, &triangles, &warn, &err);
    std::filesystem::remove(path);

    std::printf("R ok=%d npos=%zu ntri=%zu\n", ok ? 1 : 0, positions.size(), triangles.size());
    dumpPosLine(positions);
    dumpTris(triangles, "TRI");
    std::printf("W %s\n", escapeBytes(warn).c_str());
    std::printf("E %s\n", escapeBytes(err).c_str());

    // Weld-after-load on a copy (load path only emits triangles, so k is
    // always 3 here; the direct WELDCASE section covers general faces).
    if (ok) {
        std::vector<float> wpos = positions;
        std::vector<std::vector<size_t>> wtri = triangles;
        AutoRemesher::WeldStats stats;
        AutoRemesher::weldPositionsAndTriangles(&wpos, &wtri, &stats);
        std::printf("WELD npos=%zu ntri=%zu deg=%zu nonfin=%zu\n",
            wpos.size(), wtri.size(), stats.degenerateDropped, stats.nonFiniteDropped);
        dumpPosLine(wpos);
        dumpTris(wtri, "TRI");
    }
}

// ---------------------------------------------------------------------------
// Seeded generators
// ---------------------------------------------------------------------------

static std::string genCoordToken()
{
    // Weighted classes: plain ints, decimals, exponents, specials, hex,
    // tricky partials, garbage (which ends the coordinate list with zeros).
    switch (below(20)) {
    case 0:
    case 1:
    case 2:
    case 3: {
        static const std::vector<std::string> plain { "0", "1", "-1", "+2", "007", "-0", "12", "-34", "5", "100" };
        return pick(plain);
    }
    case 4:
    case 5:
    case 6: {
        static const std::vector<std::string> dec { "2.5", "-.75", "+.5", "5.", "0.1", "00.5", "3.125", "-0.30000000000000004", "9.9.9", "1_000" };
        return pick(dec);
    }
    case 7:
    case 8: {
        static const std::vector<std::string> exp { "1e3", "1E-3", "2e+5", "1E5", "12e+007", "1.5e-2", "-2E2", "0e5", "1d5" };
        return pick(exp);
    }
    case 9: {
        static const std::vector<std::string> spec { "inf", "-INF", "nan", "NAN", "-nan", "1e999", "-1e999", "1e-999", "in", "nanx" };
        return pick(spec);
    }
    case 10: {
        static const std::vector<std::string> nano { "nan(1)", "nan()", "nan(abc)", "nan(0x10)", "nan(010)", "nan(", "nan(99999999999999999999999)", "nan(1_2)" };
        return pick(nano);
    }
    case 11: {
        static const std::vector<std::string> hex { "0x1p3", "0x.8", "0XABCDEFp-4", "0x1", "0x", "0x1.", "0x1p", "0x.8p1", "0xABCDEF", "-0x0", "0xg" };
        return pick(hex);
    }
    case 12: {
        static const std::vector<std::string> tricky { "1e", "1e+", ".", "+.", "5.e3", "e5", "--1", "+-1", "+", "-", "007x", "12 34" };
        return pick(tricky);
    }
    default: {
        // Small exact decimals (typical mesh data).
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%d.%d", (int)below(21) - 10, (int)below(1000));
        return buf;
    }
    }
}

static std::string genSpaces()
{
    switch (below(6)) {
    case 0:
        return " ";
    case 1:
        return "  ";
    case 2:
        return "\t";
    case 3:
        return " \t ";
    case 4:
        return "   ";
    default:
        return " ";
    }
}

static std::string genVertexLine()
{
    std::string line;
    if (below(10) == 0)
        line += genSpaces(); // leading whitespace
    line += "v";
    line += genSpaces();
    size_t ncoords = below(6); // 0..5 (short pads zeros, long ignores extras)
    for (size_t i = 0; i < ncoords; ++i) {
        if (i > 0)
            line += genSpaces();
        line += genCoordToken();
    }
    int tail = (int)below(10);
    if (tail == 0)
        line += " # vertex comment";
    else if (tail == 1)
        line += " extra junk 1 2";
    else if (tail == 2)
        line += "\r"; // lone CR truncates the rest of the line
    if (below(20) == 0)
        line += "\r"; // CRLF ending
    return line;
}

// A face corner: leading index plus optional /vt/vn parts.
static std::string genCorner(size_t vertexCount, bool malformed)
{
    std::string idx;
    if (malformed) {
        static const std::vector<std::string> bad { "0", "-0", "99999", "99999999999999999999999",
            "2147483647", "2147483648", "-2147483649", "x", "+", "-", "+-1", "--1", "1.5x", "" };
        idx = pick(bad);
        if (idx.empty())
            return below(2) ? "/2" : "//"; // empty vertex part
        if (below(3) == 0 && vertexCount > 0) {
            // Forward reference: valid number, vertex defined later (or never).
            idx = std::to_string(vertexCount + 1 + below(3));
        }
    } else if (vertexCount == 0) {
        static const std::vector<std::string> bad { "0", "1", "-1", "x" };
        return pick(bad);
    } else if (below(4) == 0) {
        // Negative relative index, in range.
        idx = std::to_string(-(1 + (long)below(vertexCount)));
    } else {
        idx = std::to_string(1 + below(vertexCount));
        if (below(10) == 0)
            idx = "+" + idx;
        else if (below(10) == 0)
            idx = "0" + idx;
    }
    switch (below(8)) {
    case 0:
        return idx;
    case 1:
        return idx + "/" + std::to_string(1 + below(9));
    case 2:
        return idx + "//" + std::to_string(1 + below(9));
    case 3:
        return idx + "/" + std::to_string(1 + below(9)) + "/" + std::to_string(1 + below(9));
    case 4:
        return idx + "/";
    case 5:
        return idx + "//";
    case 6:
        return idx + "/x";
    default:
        return idx + "/" + std::to_string(1 + below(99)) + "/" + std::to_string(1 + below(99));
    }
}

static std::string genFaceLine(size_t vertexCount)
{
    std::string line;
    if (below(10) == 0)
        line += genSpaces();
    line += "f";
    line += genSpaces();
    bool malformed = below(10) == 0;
    size_t n = below(9); // 0..8 corners (n-gons stress the ear clipper)
    if (malformed && n == 0)
        n = 3;
    for (size_t i = 0; i < n; ++i) {
        if (i > 0)
            line += genSpaces();
        // One bad corner fails the whole face.
        line += genCorner(vertexCount, malformed && (i == n / 2 || n == 1));
    }
    int tail = (int)below(10);
    if (tail == 0)
        line += " # face comment";
    else if (tail == 1)
        line += "\r";
    if (below(20) == 0)
        line += "\r";
    return line;
}

static std::string genNoiseLine()
{
    switch (below(16)) {
    case 0:
        return "";
    case 1:
        return "   ";
    case 2:
        return "# a comment";
    case 3:
        return "  # indented comment";
    case 4:
        return "vn 0 0 1";
    case 5:
        return "vt 0.5 0.5";
    case 6:
        return "vp 0.5";
    case 7:
        return "o ObjectName";
    case 8:
        return "g group1 group2";
    case 9:
        return "usemtl mat1";
    case 10:
        return "s 1";
    case 11:
        return "mtllib test.mtl";
    case 12:
        return "v1 2 3"; // no space: ignored
    case 13:
        return "V 1 2 3"; // uppercase: ignored
    case 14:
        return "f1 2 3"; // no space: ignored
    default:
        return "garbage line here";
    }
}

static std::string genCase()
{
    std::string obj;
    if (below(20) == 0)
        obj += "\xEF\xBB\xBF"; // BOM at file start
    bool facesFirst = below(10) == 0;
    size_t nverts = below(25);
    size_t nfaces = below(11);
    // Pre-generate vertices so faces-first can forward-reference them.
    std::vector<std::string> verts;
    for (size_t i = 0; i < nverts; ++i)
        verts.push_back(genVertexLine());
    // vertexCount as the face parser sees it depends on emission order.
    auto emitFaces = [&](size_t seen) {
        std::string s;
        for (size_t i = 0; i < nfaces; ++i)
            s += genFaceLine(seen) + "\n";
        return s;
    };
    if (facesFirst) {
        obj += emitFaces(0);
        for (const auto& v : verts)
            obj += v + "\n";
    } else {
        // Interleave: mostly verts then faces, sometimes a face mid-verts.
        size_t emitted = 0;
        for (const auto& v : verts) {
            obj += v + "\n";
            ++emitted;
            if (nfaces > 0 && below(8) == 0) {
                obj += genFaceLine(emitted) + "\n";
                --nfaces;
            }
        }
        obj += emitFaces(emitted);
    }
    // Noise lines sprinkled through (appended; order vs faces is irrelevant).
    for (size_t i = 0, m = below(5); i < m; ++i) {
        // Insert at a random line boundary.
        std::string noise = genNoiseLine() + "\n";
        if (!obj.empty() && below(2)) {
            size_t pos = below(obj.size());
            size_t nl = obj.find('\n', pos);
            size_t at = (nl == std::string::npos) ? obj.size() : nl + 1;
            obj.insert(at, noise);
        } else {
            obj += noise;
        }
    }
    // Byte-level noise: rare NUL truncation / high bytes / lone CR.
    if (below(12) == 0 && !obj.empty()) {
        size_t pos = below(obj.size());
        int kind = (int)below(4);
        if (kind == 0)
            obj.insert(pos, 1, '\0');
        else if (kind == 1)
            obj.insert(pos, 1, (char)(0x80 + below(0x80)));
        else if (kind == 2)
            obj.insert(pos, 1, '\r');
        else
            obj.insert(pos, 1, (char)(below(2) ? '\x0b' : '\x0c'));
    }
    return obj;
}

// ---------------------------------------------------------------------------
// Direct weld cases
// ---------------------------------------------------------------------------

static float bitsToFloat(std::uint32_t bits)
{
    float f;
    std::memcpy(&f, &bits, 4);
    return f;
}

static void dumpWeldCase(int id)
{
    // Positions from a small pool (forces duplicates/welding) with occasional
    // NaN/inf payloads, -0.0 (bitwise distinct from 0.0: must NOT weld), and
    // subnormals.
    static const std::uint32_t pool[] = {
        0x00000000, 0x80000000, 0x3f800000, 0xbf800000, 0x40000000, 0x40490fdb,
        0x7f800000, 0xff800000, 0x7fc00000, 0xffc00000, 0x7fc00001, 0x00000001,
        0x7fffffff, 0x007fffff, 0x00800000, 0x3dcccccd,
    };
    size_t nverts = below(13);
    std::vector<float> positions;
    for (size_t v = 0; v < nverts; ++v)
        for (int c = 0; c < 3; ++c)
            positions.push_back(bitsToFloat(pool[below(sizeof(pool) / sizeof(pool[0]))]));
    if (below(10) == 0 && !positions.empty()) {
        // Non-multiple-of-3 positions.
        positions.resize(positions.size() - (1 + below(2)));
    }
    size_t nfaces = below(8);
    std::vector<std::vector<size_t>> tris;
    size_t vcount = positions.size() / 3;
    for (size_t f = 0; f < nfaces; ++f) {
        int kind = (int)below(10);
        if (kind < 5 || vcount == 0) {
            // Triangle, in range (sometimes degenerate by index).
            size_t a = vcount ? below(vcount) : 0;
            size_t b = vcount ? below(vcount) : 0;
            size_t c = vcount ? below(vcount) : 0;
            if (below(4) == 0)
                b = a;
            tris.push_back({ a, b, c });
        } else if (kind == 5) {
            // Triangle with an out-of-range corner.
            size_t a = vcount ? below(vcount) : 0;
            tris.push_back({ a, vcount + below(3), vcount ? below(vcount) : 0 });
        } else if (kind == 6) {
            // Non-triangle face (passes through / bails the remap).
            size_t k = below(6);
            if (k == 3)
                k = 4;
            std::vector<size_t> face;
            for (size_t i = 0; i < k; ++i)
                face.push_back(vcount ? below(vcount) : 0);
            tris.push_back(face);
        } else if (kind == 7) {
            // Triangle citing a vertex that may hold NaN/inf (pool covers it).
            size_t a = vcount ? below(vcount) : 0;
            size_t b = vcount ? below(vcount) : 0;
            size_t c = vcount ? below(vcount) : 0;
            tris.push_back({ a, b, c });
        } else {
            // Duplicate of an earlier face (same indices).
            if (!tris.empty())
                tris.push_back(tris[below(tris.size())]);
            else
                tris.push_back({ 0, 0, 0 });
        }
    }

    std::printf("WELDCASE %d npos=%zu ntri=%zu\n", id, positions.size(), tris.size());
    std::printf("INPOS");
    for (float f : positions) {
        std::uint32_t bits;
        std::memcpy(&bits, &f, 4);
        std::printf(" %08x", bits);
    }
    std::printf("\n");
    dumpTris(tris, "INTRI");

    AutoRemesher::WeldStats stats;
    AutoRemesher::weldPositionsAndTriangles(&positions, &tris, &stats);
    std::printf("WELDOUT npos=%zu ntri=%zu deg=%zu nonfin=%zu\n",
        positions.size(), tris.size(), stats.degenerateDropped, stats.nonFiniteDropped);
    dumpPosLine(positions);
    dumpTris(tris, "WTRI");
}

// ---------------------------------------------------------------------------
// Timing inputs (regenerated byte-identically by the Rust timing test)
// ---------------------------------------------------------------------------

static std::uint64_t fnv1a(const std::string& s)
{
    std::uint64_t h = 0xcbf29ce484222325ull;
    for (unsigned char c : s) {
        h ^= c;
        h *= 0x100000001b3ull;
    }
    return h;
}

// GxG grid, indexed: G*G vertices + 2*(G-1)^2 triangles.
static std::string timingIndexed(int G)
{
    std::string obj;
    obj.reserve((size_t)G * G * 16 + (size_t)G * G * 2 * 20);
    char buf[64];
    for (int y = 0; y < G; ++y)
        for (int x = 0; x < G; ++x) {
            int n = std::snprintf(buf, sizeof(buf), "v %d %d 0\n", x, y);
            obj.append(buf, (size_t)n);
        }
    for (int y = 0; y < G - 1; ++y)
        for (int x = 0; x < G - 1; ++x) {
            int a = y * G + x + 1;
            int b = a + 1;
            int c = a + G;
            int d = c + 1;
            int n = std::snprintf(buf, sizeof(buf), "f %d %d %d\n", a, c, b);
            obj.append(buf, (size_t)n);
            n = std::snprintf(buf, sizeof(buf), "f %d %d %d\n", b, c, d);
            obj.append(buf, (size_t)n);
        }
    return obj;
}

// Same triangles as non-indexed soup (weld stress).
static std::string timingSoup(int G)
{
    std::string obj;
    obj.reserve((size_t)G * G * 2 * 3 * 16 + (size_t)G * G * 2 * 24);
    char buf[96];
    int vi = 0;
    for (int y = 0; y < G - 1; ++y)
        for (int x = 0; x < G - 1; ++x) {
            int xs[6] = { x, x, x + 1, x + 1, x, x + 1 };
            int ys[6] = { y, y + 1, y, y, y + 1, y + 1 };
            for (int k = 0; k < 6; ++k) {
                int n = std::snprintf(buf, sizeof(buf), "v %d %d 0\n", xs[k], ys[k]);
                obj.append(buf, (size_t)n);
            }
            int n = std::snprintf(buf, sizeof(buf), "f %d %d %d\n", vi + 1, vi + 2, vi + 3);
            obj.append(buf, (size_t)n);
            n = std::snprintf(buf, sizeof(buf), "f %d %d %d\n", vi + 4, vi + 5, vi + 6);
            obj.append(buf, (size_t)n);
            vi += 6;
        }
    return obj;
}

static void timeInput(const char* name, const std::string& obj)
{
    std::string path = writeTemp(obj);
    for (int sample = 0; sample < 3; ++sample) {
        std::vector<float> positions;
        std::vector<std::vector<size_t>> triangles;
        std::string warn, err;
        auto t0 = std::chrono::steady_clock::now();
        bool ok = AutoRemesher::loadObjPositionsAndTriangles(
            path.c_str(), &positions, &triangles, &warn, &err);
        auto t1 = std::chrono::steady_clock::now();
        AutoRemesher::WeldStats stats;
        AutoRemesher::weldPositionsAndTriangles(&positions, &triangles, &stats);
        auto t2 = std::chrono::steady_clock::now();
        double loadMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
        double weldMs = std::chrono::duration<double, std::milli>(t2 - t1).count();
        std::printf("T obj_%s ok=%d npos=%zu ntri=%zu load_ms=%.3f weld_ms=%.3f hash=%016llx\n",
            name, ok ? 1 : 0, positions.size(), triangles.size(), loadMs, weldMs,
            (unsigned long long)fnv1a(obj));
    }
    std::filesystem::remove(path);
}

static std::string readFile(const std::string& path)
{
    std::ifstream in(path, std::ios::in | std::ios::binary);
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

int main(int argc, char** argv)
{
    if (argc != 2) {
        std::fprintf(stderr, "usage: %s <tests/fixtures dir>\n", argv[0]);
        return 2;
    }
    const std::string fixtures = argv[1];

    std::printf("OBJDIFF1\n");
    const int kGenCases = 220;
    for (int i = 0; i < kGenCases; ++i) {
        char header[32];
        std::snprintf(header, sizeof(header), "CASE %d", i);
        dumpLoadCase(genCase(), header);
    }

    // Fixed edge battery: every probed numeric token as a vertex coordinate,
    // plus structural edges (order fixed, unseeded).
    static const std::vector<std::string> numToks {
        "0", "1", "-0", "+.5", ".5", "5.", "1e3", "1E-3", "1e", "1e+", "1e-",
        ".", "+.", "-", "+", "e5", "0x", "0x1", "0X1P3", "0x1p3", "0x1p",
        "0x1p+", "0xp3", "0x.8p1", "0x.8", "inf", "INF", "Infinity",
        "infinitesimal", "in", "i", "nan", "NAN", "Nan", "nan(", "nan()",
        "nan(1)", "nan(0x1)", "nanx", "1e999", "-1e999", "1e-999", "2.5",
        "12 34", "0.30000000000000004", "3.141592653589793",
        "1.1754943508222875e-38", "1.1754942106974411e-38",
        "340282346638528859811704183484516925440",
        "340282356779733661637539395458142568448", "00.5", "--1", "+-1",
        "0b1", "007", "9.9.9", "1_000", "1,000", "nan(abc)", "NAN(ABC)", "0d5",
        "nan(2)", "nan(16)", "nan(0x10)", "nan(010)", "nan(17)",
        "nan(18446744073709551615)", "nan(18446744073709551616)", "nan(-1)",
        "nan(+2)", "nan(1.5)", "nan(1e3)", "nan(a1)", "nan(1a)", "nan(_)",
        "nan(0)", "nan(00)", "nan(0x)", "nan(0XABCDEF)", "nan( 1)", "nan(1 )",
        "0x1.fffffffffffffp1023", "0x1.0000000000001p0", "0x1fffffffffffff",
        "0x10000000000000000", "0x0.0000000000001p-1022", "0x1p-1074",
        "0x1p-1075", "0x1p1024", "0x1.fffffffffffff8p-1",
        "0x1.0000000000000800000001p0", "0xffffffffffffffffffffffffffffffp1000",
        "0x1p-2000", "0x00000000000000000000000001p0", "0XABCDEFp-4", "0xap0",
        "0xAp0", "0.1", "2.2250738585072014e-308", "2.2250738585072009e-308",
        "5e-324", "4e-324", "3e-324", "2.4703282292062327e-324",
        "1.7976931348623157e308", "1.7976931348623159e308", "9007199254740993",
        "9007199254740994", "-5e-324", "0x1.921fb54442d18p+1",
        "0x3.243f6a8885a3p-1", "nan(1_2)", "nan(0b1)", "nan(09)",
        "nan(0xABCDEF)", "nan(00x10)", "nan(8)", "nan(0X10)", "nan(0xx10)",
        "nan(00012)", "nan(12_34)", "0x.p1", "0x.", "0x.0", "5.e3", ".5e2",
        "0e5", ".e5", "00e2", "0X.8P1", "0xABCDEF", "0xabcdef", "0xg", "0x1g5",
        "12e+007", "0INF", "0NAN", "iNf", "nAn", "INFINITYx", "infinity",
        "INFINITY", "+inf", "-INF", "+nan", "-nan", "-nan(5)", "inf0", "nan2",
        "1d5", "1q5", "1E5", "1P5", "0b101", "0o17",
        "nan(0xFFFFFFFFFFFFFFFF)", "nan(0x10000000000000000)",
        "nan(07777777777777777777777)", "nan(01000000000000000000000)",
        "nan(99999999999999999999999)", "nan(0xABCDEF123456789ABCDEF)",
        "nan(foo)", "nan(a)", "nan(Z9_)", "0x1p999999999999",
        "0x1p-999999999999", "1e999999999",
        "0x10000000000000800000000000000001p-60", "-0x0p1", "-0x0",
        "0x0p99999", "0x1.", "0x1.p3", "0x.8p", "0x1.8p", "5.e", ".5e+",
        "0x1P+3", "0x1p3.", "0xABC.", "0x.p0", "0x.0p", "0x0.", "nan((x))",
        "nan(1", "nan)", "007x", "0.5e", "1E+0002",
    };
    int fixedId = 0;
    for (const std::string& tok : numToks) {
        char header[32];
        std::snprintf(header, sizeof(header), "FIXED %d", fixedId++);
        // Token in each coordinate slot plus a face, so parse failures,
        // zeros and triangulations all show.
        std::string obj = "v " + tok + " 0 0\nv 0 " + tok + " 0\nv 0 0 " + tok
            + "\nv 1 1 1\nf 1 2 3\nf 1 2 3 4\n";
        dumpLoadCase(obj, header);
    }
    static const std::vector<std::string> structCases {
        "",
        "\n",
        "   \n\t\n",
        "# only a comment\n",
        "v\n",
        "v \n",
        "v 1\n",
        "v 1 2\n",
        "v 1 2 3 4 5\n",
        "v 1 2 3 # comment\n",
        "v 1 2 # then junk\n",
        "v # no coords\n",
        "f\n",
        "f \n",
        "f 1\n",
        "v 0 0 0\nf 1\n",
        std::string("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3") + std::string(1, '\0') + " garbage\n",
        std::string("v 1 2") + std::string(1, '\0') + "3 4 5\n",
        std::string("f 1 2") + std::string(1, '\0') + "3\nv 0 0 0\nv 1 0 0\nv 0 1 0\n",
        "v 0 0 0\r\nv 1 0 0\r\nv 0 1 0\r\nf 1 2 3\r\n",
        "v 0 0 0\rv 1 0 0\nv 0 1 0\nf 1 2 3\n",
        "v\t1\t2\t3\nf\t1\t1\t1\n",
        "v \x0b" "1 2 3\n",
        "v \x0c" "1 2 3\n",
        "f 1\x0b" "2 3\nv 0 0 0\nv 1 0 0\nv 0 1 0\n",
        "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1\x0b" "2 3\n",
        "\xEF\xBB\xBFv 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n",
        "v 0 0 0\nf 1 1 1\nf 1\nf\nf # comment\n",
        "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 3 2 1\nf -1 -2 -3\nf +1 +2 +3\n",
        "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 01 02 03\n",
        "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1/ 2/ 3/\n",
        "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1// 2// 3//\n",
        "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1/2/3/4 2/3/4/5 3/4/5/6\n",
        "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1.5 2.5 3.5\n",
        "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1/x 2/y 3/z\n",
        "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1//x 2//x 3//x\n",
        "v 0 0 0\nv 1 0 0\nv 0 1 0\nf /2 /2 /2\n",
        "v 0 0 0\nv 1 0 0\nv 0 1 0\nf // // //\n",
        "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2\n",
        "f 1 2 3\n",
        "f 1 2 3\nv 0 0 0\nv 1 0 0\nv 0 1 0\n",
        "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 4\n",
        "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 -4\n",
        "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 4 2 3\n",
        "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 2147483647 2 3\n",
        "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 2147483648 2 3\n",
        "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 9223372036854775807 2 3\n",
        "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 99999999999999999999999 2 3\n",
        "v 0 0 0\nv 1 0 0\nv 0 1 0\nf -2147483648 2 3\n",
        "v 0 0 0\nv 1 0 0\nv 0 1 0\nf -0 2 3\n",
        "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3 # trailing comment\n",
        "v 0 0 0\nv 1 0 0\nv 0 1 0\nf # no corners\n",
        "  v 0 0 0\n\tv 1 0 0\n  f 1 2 2\n",
        "vn 0 0 1\nvt 1 2\nvp 3\nv 0 0 0\nv 1 0 0\nv 0 1 0\nf 1/1/1 2/2/2 3/3/3\n",
        "o name\ng group\nusemtl m\ns 1\nmtllib m\nv 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n",
        "V 0 0 0\nF 1 2 3\nv 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n",
        "v\x0b" "0 0 0\n",
        "f\x0c" "1 2 3\n",
        "v 0 0 0 \x80\xff\n",
        "v 1e999 0 0\nv 0 1 0\nv 0 0 1\nf 1 2 3\n",
        "v nan nan nan\nv 1 0 0\nv 0 1 0\nf 1 2 3\n",
    };
    for (const std::string& obj : structCases) {
        char header[32];
        std::snprintf(header, sizeof(header), "FIXED %d", fixedId++);
        dumpLoadCase(obj, header);
    }

    // Embedded nasty corpus (malformed-input classes, committed fixtures).
    static const std::vector<std::string> nasty {
        "nasty-all-degenerate.obj", "nasty-all-zeroarea.obj", "nasty-degenerate.obj",
        "nasty-empty.obj", "nasty-floaters.obj", "nasty-multicomp.obj", "nasty-nan.obj",
        "nasty-nonmanifold.obj", "nasty-oor.obj", "nasty-single-tetra.obj", "nasty-soup.obj",
    };
    for (const std::string& name : nasty) {
        dumpLoadCase(readFile(fixtures + "/" + name), "FILE " + name);
    }

    for (int i = 0; i < 80; ++i)
        dumpWeldCase(i);

    timeInput("indexed", timingIndexed(200));
    timeInput("soup", timingSoup(200));
    return 0;
}
