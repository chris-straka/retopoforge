// Differential oracle dump for the glb Rust port (rs-glb lane).
// Generates a .glb corpus programmatically (fixed edge battery + seeded
// randomized cases), loads each with the C++ implementation, and prints
// inputs (.glb bytes as hex) + outputs in a format that
// rust/core/tests/glb_diff.rs replays. Positions print as f32 bit patterns
// (%08x), so exact cases compare bitwise; INEXACT=1 cases carry general
// transforms and replay within tolerance (C++ -O3 fuses the transform
// multiply-adds, so inexact intermediates have no stable bitwise contract).
// Writer cases print the input mesh plus the C++-written bytes for a
// byte-identical replay, and the timing section prints Release ms + hashes
// for the Rust-side regeneration check.
//
// Build-only helper: not registered with ctest. Run it with the fixtures
// dir and redirect stdout to tests/fixtures/glb_diff.txt, then commit:
//   ./build-<id>/tests/glb_diff_dump tests/fixtures > tests/fixtures/glb_diff.txt
#include "../cli/glb.h"

import retopo.core.vector2;
import retopo.core.vector3;

#include <chrono>
#include <cmath>
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

static std::uint64_t g_state = 0x6C622D72732D677ull; // "glb-rs-g"

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

static std::string hexEncode(const std::string& s)
{
    static const char* digits = "0123456789abcdef";
    std::string out;
    out.reserve(s.size() * 2);
    for (unsigned char c : s) {
        out.push_back(digits[c >> 4]);
        out.push_back(digits[c & 15]);
    }
    return out;
}

static std::string replaceAll(std::string s, const std::string& from, const std::string& to)
{
    if (from.empty())
        return s;
    size_t at = 0;
    while ((at = s.find(from, at)) != std::string::npos) {
        s.replace(at, from.size(), to);
        at += to.size();
    }
    return s;
}

static int g_tempCounter = 0;

static std::string writeTemp(const std::string& bytes, const std::string& suffix)
{
    static int pid = 0;
    if (!pid)
        pid = (int)::getpid();
    std::filesystem::path path = std::filesystem::temp_directory_path()
        / ("retopo_glbdiff_" + std::to_string(pid) + "_" + std::to_string(g_tempCounter++) + suffix);
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

static std::uint64_t fnv1a(const std::string& s)
{
    std::uint64_t h = 0xcbf29ce484222325ull;
    for (unsigned char c : s) {
        h ^= c;
        h *= 0x100000001b3ull;
    }
    return h;
}

// ---------------------------------------------------------------------------
// .glb builder: BIN blob + JSON fragment arrays, assembled on build().
// ---------------------------------------------------------------------------

struct GlbBuilder {
    std::string bin; // byte blob (std::string as a byte vector)
    std::vector<std::string> accessors;
    std::vector<std::string> views;
    std::vector<std::string> meshes;
    std::vector<std::string> nodes;
    std::vector<std::string> scenes;
    std::string buffersOverride; // "" = default single buffer over bin
    std::string topScene; // "" = omit, else raw JSON value
    std::string extraTop; // raw `"key":value,` members (with trailing comma)

    size_t binSize() const { return bin.size(); }
    void pushBytes(const void* p, size_t n)
    {
        bin.append((const char*)p, n);
    }
    void pushF32(float v) { pushBytes(&v, 4); }
    void pushU32(std::uint32_t v)
    {
        unsigned char b[4] = {
            (unsigned char)(v & 0xff), (unsigned char)((v >> 8) & 0xff),
            (unsigned char)((v >> 16) & 0xff), (unsigned char)((v >> 24) & 0xff)
        };
        pushBytes(b, 4);
    }
    void pushU16(std::uint16_t v)
    {
        unsigned char b[2] = { (unsigned char)(v & 0xff), (unsigned char)((v >> 8) & 0xff) };
        pushBytes(b, 2);
    }
    void pushU8(std::uint8_t v) { bin.push_back((char)v); }
    void pad4()
    {
        while (bin.size() % 4 != 0)
            bin.push_back('\0');
    }

    int addView(size_t offset, size_t length, size_t stride = 0, const char* buffer = "0")
    {
        std::ostringstream s;
        s << "{\"buffer\":" << buffer << ",\"byteOffset\":" << offset
          << ",\"byteLength\":" << length;
        if (stride != 0)
            s << ",\"byteStride\":" << stride;
        s << "}";
        views.push_back(s.str());
        return (int)views.size() - 1;
    }
    int addAccessor(const char* type, int component, size_t count, const char* view /*raw or ""=none*/,
        size_t byteOffset = 0, bool normalized = false, const char* sparse = nullptr)
    {
        std::ostringstream s;
        s << "{\"componentType\":" << component << ",\"count\":" << count
          << ",\"type\":\"" << type << "\"";
        if (view[0] != '\0')
            s << ",\"bufferView\":" << view;
        if (byteOffset != 0)
            s << ",\"byteOffset\":" << byteOffset;
        if (normalized)
            s << ",\"normalized\":true";
        if (sparse)
            s << ",\"sparse\":" << sparse;
        s << "}";
        accessors.push_back(s.str());
        return (int)accessors.size() - 1;
    }
    int addMesh(const std::string& primsJson)
    {
        meshes.push_back("{\"primitives\":" + primsJson + "}");
        return (int)meshes.size() - 1;
    }
    int addNode(const std::string& nodeJson)
    {
        nodes.push_back(nodeJson);
        return (int)nodes.size() - 1;
    }
    int addScene(const std::string& rootsJson)
    {
        scenes.push_back("{\"nodes\":" + rootsJson + "}");
        return (int)scenes.size() - 1;
    }

    std::string build(bool withBin = true)
    {
        std::ostringstream json;
        json << "{\"asset\":{\"version\":\"2.0\",\"generator\":\"glb_diff_dump\"}";
        if (!topScene.empty())
            json << ",\"scene\":" << topScene;
        json << extraTop;
        json << ",\"scenes\":[";
        for (size_t i = 0; i < scenes.size(); ++i)
            json << (i ? "," : "") << scenes[i];
        json << "]";
        json << ",\"nodes\":[";
        for (size_t i = 0; i < nodes.size(); ++i)
            json << (i ? "," : "") << nodes[i];
        json << "]";
        json << ",\"meshes\":[";
        for (size_t i = 0; i < meshes.size(); ++i)
            json << (i ? "," : "") << meshes[i];
        json << "]";
        json << ",\"accessors\":[";
        for (size_t i = 0; i < accessors.size(); ++i)
            json << (i ? "," : "") << accessors[i];
        json << "]";
        json << ",\"bufferViews\":[";
        for (size_t i = 0; i < views.size(); ++i)
            json << (i ? "," : "") << views[i];
        json << "]";
        if (!buffersOverride.empty())
            json << ",\"buffers\":" << buffersOverride;
        else {
            json << ",\"buffers\":[{\"byteLength\":" << bin.size() << "}]";
        }
        json << "}";
        std::string text = json.str();
        while (text.size() % 4 != 0)
            text.push_back(' ');

        std::string blob;
        blob.reserve(12 + 8 + text.size() + (withBin ? 8 + bin.size() : 0));
        blob.append("glTF", 4);
        auto putU32 = [&](std::uint32_t v) {
            blob.push_back((char)(v & 0xff));
            blob.push_back((char)((v >> 8) & 0xff));
            blob.push_back((char)((v >> 16) & 0xff));
            blob.push_back((char)((v >> 24) & 0xff));
        };
        putU32(2);
        putU32((std::uint32_t)(12 + 8 + text.size() + (withBin ? 8 + bin.size() : 0)));
        putU32((std::uint32_t)text.size());
        putU32(0x4E4F534A);
        blob.append(text);
        if (withBin) {
            putU32((std::uint32_t)bin.size());
            putU32(0x004E4942);
            blob.append(bin);
        }
        return blob;
    }
};

// Primitive JSON: attrs/idx/mode/targets are raw JSON fragments ("" omits
// the key), so invalid values (-1, "x", null) can be emitted verbatim.
static std::string primJson(
    const std::string& attrs, const std::string& idx, const std::string& mode, const std::string& targets = "")
{
    std::string s = "{";
    bool first = true;
    auto field = [&](const char* key, const std::string& raw) {
        if (raw.empty())
            return;
        if (!first)
            s += ",";
        first = false;
        s += std::string("\"") + key + "\":" + raw;
    };
    field("attributes", attrs);
    field("indices", idx);
    field("mode", mode);
    field("targets", targets);
    s += "}";
    return s;
}

static std::string attrsPos(int posAcc, const std::string& extra = "")
{
    std::ostringstream s;
    s << "{\"POSITION\":" << posAcc;
    if (!extra.empty())
        s << "," << extra;
    s << "}";
    return s.str();
}

// Node JSON: mesh/children raw fragments ("" omits), xform raw members.
static std::string nodeJson(const std::string& mesh, const std::string& children, const std::string& xform = "")
{
    std::string s = "{";
    bool first = true;
    auto field = [&](const char* key, const std::string& raw) {
        if (raw.empty())
            return;
        if (!first)
            s += ",";
        first = false;
        s += std::string("\"") + key + "\":" + raw;
    };
    field("mesh", mesh);
    field("children", children);
    if (!xform.empty()) {
        if (!first)
            s += ",";
        s += xform;
    }
    s += "}";
    return s;
}

static std::string fmtF(float v)
{
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.9g", v);
    return buf;
}

static std::string xformTRS(const float t[3], const float q[4], const float s[3])
{
    std::ostringstream o;
    o << "\"translation\":[" << fmtF(t[0]) << "," << fmtF(t[1]) << "," << fmtF(t[2]) << "]";
    o << ",\"rotation\":[" << fmtF(q[0]) << "," << fmtF(q[1]) << "," << fmtF(q[2]) << "," << fmtF(q[3]) << "]";
    o << ",\"scale\":[" << fmtF(s[0]) << "," << fmtF(s[1]) << "," << fmtF(s[2]) << "]";
    return o.str();
}

static std::string xformMatrix(const float m[16])
{
    std::ostringstream o;
    o << "\"matrix\":[";
    for (int i = 0; i < 16; ++i)
        o << (i ? "," : "") << fmtF(m[i]);
    o << "]";
    return o.str();
}

// ---------------------------------------------------------------------------
// Case dumpers. Sentinel pre-fills pin the no-clear contract: positions and
// triangles are cleared at entry (never sentinel post-call), while warn/err
// keep stale contents unless C++ assigns them.
// ---------------------------------------------------------------------------

static int g_failures = 0;

#define CHECK(cond)                                                                                  \
    do {                                                                                             \
        if (!(cond)) {                                                                               \
            std::fprintf(stderr, "DUMPFAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                  \
            ++g_failures;                                                                            \
        }                                                                                            \
    } while (0)

static void dumpReadCase(int id, int inexact, const std::string& glbBytes,
    const std::string& suffix, int expectOk, const std::string& fixedPath = "")
{
    std::printf("READCASE %d INEXACT=%d SUFFIX=%s HEXLEN=%zu\n", id, inexact,
        suffix == ".GLB" ? "GLB" : "glb", glbBytes.size());
    std::printf("HEX %s\n", hexEncode(glbBytes).c_str());

    std::string path = fixedPath.empty() ? writeTemp(glbBytes, suffix) : fixedPath;
    std::vector<float> positions { 1.5f, -2.5f };
    std::vector<std::vector<size_t>> triangles { { 99 }, { 7, 8 } };
    std::string warn = "W", err = "E";
    bool ok = GlbIo::loadGlbPositionsAndTriangles(
        path.c_str(), &positions, &triangles, &warn, &err);
    if (fixedPath.empty())
        std::filesystem::remove(path);
    if (expectOk >= 0 && (ok ? 1 : 0) != expectOk) {
        std::fprintf(stderr, "DUMPFAIL case %d: ok=%d expectOk=%d err=%s\n", id, ok ? 1 : 0,
            expectOk, err.c_str());
        ++g_failures;
    }

    std::printf("R ok=%d npos=%zu ntri=%zu\n", ok ? 1 : 0, positions.size(), triangles.size());
    dumpPosLine(positions);
    dumpTris(triangles, "TRI");
    // Normalize the dump-time temp path (the replay substitutes its own).
    std::printf("W %s\n", escapeBytes(replaceAll(warn, path, "@F@")).c_str());
    std::printf("E %s\n", escapeBytes(replaceAll(err, path, "@F@")).c_str());
}

static void dumpWriteCase(int id, const std::string& name, int withUvs,
    const std::vector<AutoRemesher::Vector3>& verts,
    const std::vector<std::vector<size_t>>& faces,
    const std::vector<AutoRemesher::Vector2>& uvs, int badPath,
    const char* generator = "glb_diff_dump")
{
    std::printf("WRITECASE %d name=%s uvs=%d nverts=%zu nfaces=%zu badpath=%d\n",
        id, name.c_str(), withUvs, verts.size(), faces.size(), badPath);
    std::printf("GEN %s\n", escapeBytes(generator).c_str());
    for (const auto& v : verts) {
        std::uint64_t xb, yb, zb;
        double x = v.x(), y = v.y(), z = v.z();
        std::memcpy(&xb, &x, 8);
        std::memcpy(&yb, &y, 8);
        std::memcpy(&zb, &z, 8);
        std::printf("VERT %016llx %016llx %016llx\n", (unsigned long long)xb,
            (unsigned long long)yb, (unsigned long long)zb);
    }
    for (const auto& face : faces) {
        std::printf("FACE %zu", face.size());
        for (size_t c : face)
            std::printf(" %zu", c);
        std::printf("\n");
    }
    if (withUvs) {
        for (const auto& uv : uvs) {
            std::uint64_t xb, yb;
            double x = uv.x(), y = uv.y();
            std::memcpy(&xb, &x, 8);
            std::memcpy(&yb, &y, 8);
            std::printf("UV %016llx %016llx\n", (unsigned long long)xb, (unsigned long long)yb);
        }
    }
    std::string path;
    if (badPath) {
        path = "/nonexistent-dir-rs-glb-9f3/nope.glb";
    } else {
        static int pid = 0;
        if (!pid)
            pid = (int)::getpid();
        path = (std::filesystem::temp_directory_path()
            / ("retopo_glbwrite_" + std::to_string(pid) + "_" + std::to_string(id) + ".glb"))
                   .string();
    }
    bool ok;
    if (withUvs)
        ok = GlbIo::saveGlb(path.c_str(), generator, verts, faces, uvs);
    else
        ok = GlbIo::saveGlb(path.c_str(), generator, verts, faces);
    std::string bytes;
    if (ok) {
        std::ifstream in(path, std::ios::binary);
        bytes.assign((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        std::filesystem::remove(path);
    }
    std::printf("ROUT ok=%d nbytes=%zu\n", ok ? 1 : 0, bytes.size());
    std::printf("BYTES %s\n", hexEncode(bytes).c_str());
}

static void dumpExtCase(int id, const std::string& path)
{
    std::printf("EXTCASE %d\n", id);
    std::printf("P %s\n", escapeBytes(path).c_str());
    std::printf("R glb=%d sup=%d\n",
        GlbIo::hasGlbExtension(path) ? 1 : 0,
        GlbIo::isSupportedInputExtension(path) ? 1 : 0);
}

// One indexed triangle (float VEC3 + u32 indices) under one node; the
// workhorse valid file. Returns the builder for case-specific tweaks.
static GlbBuilder basicTriBuilder()
{
    GlbBuilder b;
    size_t po = b.binSize();
    float pos[9] = { 0, 0, 0, 1, 0, 0, 0, 1, 0 };
    b.pushBytes(pos, sizeof(pos));
    size_t io = b.binSize();
    std::uint32_t idx[3] = { 0, 1, 2 };
    b.pushBytes(idx, sizeof(idx));
    int pv = b.addView(po, 36);
    int iv = b.addView(io, 12);
    int pa = b.addAccessor("VEC3", 5126, 3, "0");
    int ia = b.addAccessor("SCALAR", 5125, 3, "1");
    (void)pv;
    (void)iv;
    b.addMesh("[" + primJson(attrsPos(pa), std::to_string(ia), "4") + "]");
    b.addNode(nodeJson("0", ""));
    b.addScene("[0]");
    b.topScene = "0";
    return b;
}

// ---------------------------------------------------------------------------
// Fixed reader battery: every exercised subset path, each with the C++
// outcome pinned at generation time (a builder bug aborts the dump).
// ---------------------------------------------------------------------------

static int runFixedReads()
{
    int id = 0;
    // R1: minimal indexed triangle.
    { GlbBuilder b = basicTriBuilder(); dumpReadCase(id++, 0, b.build(), ".glb", 1); }
    // R2: same bytes under a .GLB (uppercase) name.
    { GlbBuilder b = basicTriBuilder(); dumpReadCase(id++, 0, b.build(), ".GLB", 1); }
    // R3: non-indexed soup (2 triangles).
    {
        GlbBuilder b;
        size_t po = b.binSize();
        float pos[18] = { 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 1 };
        b.pushBytes(pos, sizeof(pos));
        b.addView(po, sizeof(pos));
        int pa = b.addAccessor("VEC3", 5126, 6, "0");
        b.addMesh("[" + primJson(attrsPos(pa), "", "4") + "]");
        b.addNode(nodeJson("0", ""));
        b.addScene("[0]");
        b.topScene = "0";
        dumpReadCase(id++, 0, b.build(), ".glb", 1);
    }
    // R4: three primitives in one mesh (indexed + soup + indexed).
    {
        GlbBuilder b;
        size_t p1 = b.binSize();
        float a[9] = { 0, 0, 0, 1, 0, 0, 0, 1, 0 };
        b.pushBytes(a, sizeof(a));
        size_t i1 = b.binSize();
        std::uint32_t ia1[3] = { 0, 1, 2 };
        b.pushBytes(ia1, sizeof(ia1));
        size_t p2 = b.binSize();
        float c[9] = { 5, 5, 5, 6, 5, 5, 5, 6, 5 };
        b.pushBytes(c, sizeof(c));
        size_t p3 = b.binSize();
        float d[12] = { 9, 0, 0, 9, 1, 0, 9, 0, 1, 9, 1, 1 };
        b.pushBytes(d, sizeof(d));
        size_t i3 = b.binSize();
        std::uint16_t ia3[6] = { 0, 1, 2, 1, 3, 2 };
        b.pushBytes(ia3, sizeof(ia3));
        b.pad4();
        b.addView(p1, 36);
        b.addView(i1, 12);
        b.addView(p2, 36);
        b.addView(p3, 48);
        b.addView(i3, 12);
        int pa1 = b.addAccessor("VEC3", 5126, 3, "0");
        int ia1a = b.addAccessor("SCALAR", 5125, 3, "1");
        int pa2 = b.addAccessor("VEC3", 5126, 3, "2");
        int pa3 = b.addAccessor("VEC3", 5126, 4, "3");
        int ia3a = b.addAccessor("SCALAR", 5123, 6, "4");
        std::string prims = "[" + primJson(attrsPos(pa1), std::to_string(ia1a), "4") + ","
            + primJson(attrsPos(pa2), "", "4") + ","
            + primJson(attrsPos(pa3), std::to_string(ia3a), "4") + "]";
        b.addMesh(prims);
        b.addNode(nodeJson("0", ""));
        b.addScene("[0]");
        b.topScene = "0";
        dumpReadCase(id++, 0, b.build(), ".glb", 1);
    }
    // R5: two meshes + instancing (nodes 0,2 share mesh 0 with transforms).
    {
        GlbBuilder b = basicTriBuilder();
        // Second mesh: translated soup triangle.
        size_t p2 = b.binSize();
        float e[9] = { 2, 2, 2, 3, 2, 2, 2, 3, 2 };
        b.pushBytes(e, sizeof(e));
        int v2 = b.addView(p2, 36);
        int pa2 = b.addAccessor("VEC3", 5126, 3, std::to_string(v2).c_str());
        b.addMesh("[" + primJson(attrsPos(pa2), "", "4") + "]");
        // Nodes: 0 (mesh 0, child 2), 1 (mesh 1), 2 (mesh 0 again, scaled).
        float t0[3] = { 10, 0, 0 }, q0[4] = { 0, 0, 0, 1 }, s0[3] = { 1, 1, 1 };
        float t2[3] = { 0, 0, 0 }, q2[4] = { 0, 0, 0, 1 }, s2[3] = { 2, 2, 2 };
        b.nodes[0] = nodeJson("0", "[2]", xformTRS(t0, q0, s0));
        b.addNode(nodeJson("1", ""));
        b.addNode(nodeJson("0", "", xformTRS(t2, q2, s2)));
        b.scenes[0] = "{\"nodes\":[0,1]}";
        dumpReadCase(id++, 0, b.build(), ".glb", 1);
    }
    // R6: node-less mesh (no nodes at all) -> identity fallback.
    {
        GlbBuilder b = basicTriBuilder();
        b.nodes.clear();
        b.scenes.clear();
        b.topScene.clear();
        dumpReadCase(id++, 0, b.build(), ".glb", 1);
    }
    // R7: mesh-less node + unreferenced mesh -> fallback.
    {
        GlbBuilder b = basicTriBuilder();
        b.nodes[0] = nodeJson("", "");
        dumpReadCase(id++, 0, b.build(), ".glb", 1);
    }
    // R8: skip battery (7 non-triangle modes + 2 position-less, 1 tri loads).
    {
        GlbBuilder b;
        size_t po = b.binSize();
        float pos[9] = { 0, 0, 0, 1, 0, 0, 0, 1, 0 };
        b.pushBytes(pos, sizeof(pos));
        size_t io = b.binSize();
        std::uint32_t idx[3] = { 0, 1, 2 };
        b.pushBytes(idx, sizeof(idx));
        b.addView(po, 36);
        b.addView(io, 12);
        int pa = b.addAccessor("VEC3", 5126, 3, "0");
        int ia = b.addAccessor("SCALAR", 5125, 3, "1");
        int empty = b.addAccessor("VEC3", 5126, 0, "0");
        std::string prims = "[";
        const char* modes[] = { "0", "1", "2", "3", "5", "6", "99" };
        for (auto m : modes)
            prims += primJson(attrsPos(pa), std::to_string(ia), m) + ",";
        prims += primJson("{\"NORMAL\":" + std::to_string(pa) + "}", "", "4") + ",";
        prims += primJson(attrsPos(empty), "", "4") + ",";
        prims += primJson(attrsPos(pa), std::to_string(ia), "") + "]"; // missing mode = triangles
        b.addMesh(prims);
        b.addNode(nodeJson("0", ""));
        b.addScene("[0]");
        b.topScene = "0";
        dumpReadCase(id++, 0, b.build(), ".glb", 1);
    }
    // R9: nested TRS chain (translate + scale + 180-degree Z rotation).
    {
        GlbBuilder b = basicTriBuilder();
        float t0[3] = { 1, 2, 3 }, q0[4] = { 0, 0, 0, 1 }, s0[3] = { 1, 1, 1 };
        float t1[3] = { 0, 0, 0 }, q1[4] = { 0, 0, 1, 0 }, s1[3] = { 1, 1, 1 };
        float t2[3] = { 0, 0, 0 }, q2[4] = { 0, 0, 0, 1 }, s2[3] = { 2, 0.5f, -1 };
        b.nodes[0] = nodeJson("0", "[1]", xformTRS(t0, q0, s0));
        b.addNode(nodeJson("0", "[2]", xformTRS(t1, q1, s1)));
        b.addNode(nodeJson("0", "", xformTRS(t2, q2, s2)));
        dumpReadCase(id++, 0, b.build(), ".glb", 1);
    }
    // R10: nested matrix nodes (exact 90-degree rotation + translation).
    {
        GlbBuilder b = basicTriBuilder();
        float m0[16] = { 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1, 0, 5, 6, 7, 1 };
        float m1[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, -1, 2, -4, 1 };
        b.nodes[0] = nodeJson("0", "[1]", xformMatrix(m0));
        b.addNode(nodeJson("0", "", xformMatrix(m1)));
        dumpReadCase(id++, 0, b.build(), ".glb", 1);
    }
    // R11: matrix wins over TRS when both are present.
    {
        GlbBuilder b = basicTriBuilder();
        float m[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 9, 9, 9, 1 };
        float t[3] = { 100, 100, 100 }, q[4] = { 0, 0, 0, 1 }, s[3] = { 1, 1, 1 };
        b.nodes[0] = nodeJson("0", "", xformMatrix(m) + "," + xformTRS(t, q, s));
        dumpReadCase(id++, 0, b.build(), ".glb", 1);
    }
    // R12: u8 indices. R13: u16 indices.
    for (int variant = 0; variant < 2; ++variant) {
        GlbBuilder b;
        size_t po = b.binSize();
        float pos[12] = { 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1 };
        b.pushBytes(pos, sizeof(pos));
        size_t io = b.binSize();
        int comp;
        if (variant == 0) {
            std::uint8_t idx[6] = { 0, 1, 2, 0, 2, 3 };
            b.pushBytes(idx, sizeof(idx));
            comp = 5121;
        } else {
            std::uint16_t idx[6] = { 0, 1, 2, 0, 2, 3 };
            b.pushBytes(idx, sizeof(idx));
            comp = 5123;
        }
        b.pad4();
        b.addView(po, 48);
        b.addView(io, variant == 0 ? 6 : 12);
        int pa = b.addAccessor("VEC3", 5126, 4, "0");
        int ia = b.addAccessor("SCALAR", comp, 6, "1");
        b.addMesh("[" + primJson(attrsPos(pa), std::to_string(ia), "4") + "]");
        b.addNode(nodeJson("0", ""));
        b.addScene("[0]");
        b.topScene = "0";
        dumpReadCase(id++, 0, b.build(), ".glb", 1);
    }
    // R14: i16 indices, all positive (ok).
    {
        GlbBuilder b;
        size_t po = b.binSize();
        float pos[9] = { 0, 0, 0, 1, 0, 0, 0, 1, 0 };
        b.pushBytes(pos, sizeof(pos));
        size_t io = b.binSize();
        std::int16_t idx[3] = { 0, 1, 2 };
        b.pushBytes(idx, sizeof(idx));
        b.pad4();
        b.addView(po, 36);
        b.addView(io, 6);
        int pa = b.addAccessor("VEC3", 5126, 3, "0");
        int ia = b.addAccessor("SCALAR", 5122, 3, "1");
        b.addMesh("[" + primJson(attrsPos(pa), std::to_string(ia), "4") + "]");
        b.addNode(nodeJson("0", ""));
        b.addScene("[0]");
        b.topScene = "0";
        dumpReadCase(id++, 0, b.build(), ".glb", 1);
    }
    // R15: i8 index -1 sign-extends to huge -> out-of-range fatal.
    {
        GlbBuilder b;
        size_t po = b.binSize();
        float pos[9] = { 0, 0, 0, 1, 0, 0, 0, 1, 0 };
        b.pushBytes(pos, sizeof(pos));
        size_t io = b.binSize();
        std::int8_t idx[3] = { 0, 1, -1 };
        b.pushBytes(idx, sizeof(idx));
        b.pad4();
        b.addView(po, 36);
        b.addView(io, 3);
        int pa = b.addAccessor("VEC3", 5126, 3, "0");
        int ia = b.addAccessor("SCALAR", 5120, 3, "1");
        b.addMesh("[" + primJson(attrsPos(pa), std::to_string(ia), "4") + "]");
        b.addNode(nodeJson("0", ""));
        b.addScene("[0]");
        b.topScene = "0";
        dumpReadCase(id++, 0, b.build(), ".glb", 0);
    }
    // R16: float indices read 0 -> degenerate tris at vertex 0 (ok).
    {
        GlbBuilder b;
        size_t po = b.binSize();
        float pos[9] = { 0, 0, 0, 1, 0, 0, 0, 1, 0 };
        b.pushBytes(pos, sizeof(pos));
        size_t io = b.binSize();
        float idx[3] = { 2, 1, 0 };
        b.pushBytes(idx, sizeof(idx));
        b.addView(po, 36);
        b.addView(io, 12);
        int pa = b.addAccessor("VEC3", 5126, 3, "0");
        int ia = b.addAccessor("SCALAR", 5126, 3, "1");
        b.addMesh("[" + primJson(attrsPos(pa), std::to_string(ia), "4") + "]");
        b.addNode(nodeJson("0", ""));
        b.addScene("[0]");
        b.topScene = "0";
        dumpReadCase(id++, 0, b.build(), ".glb", 1);
    }
    // R17: VEC2 indices -> "failed to read index accessor".
    {
        GlbBuilder b = basicTriBuilder();
        b.accessors[1] = "{\"componentType\":5125,\"count\":3,\"type\":\"VEC2\",\"bufferView\":1}";
        dumpReadCase(id++, 0, b.build(), ".glb", 0);
    }
    // R18: index count not a multiple of 3.
    {
        GlbBuilder b;
        size_t po = b.binSize();
        float pos[12] = { 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1 };
        b.pushBytes(pos, sizeof(pos));
        size_t io = b.binSize();
        std::uint32_t idx[4] = { 0, 1, 2, 3 };
        b.pushBytes(idx, sizeof(idx));
        b.addView(po, 48);
        b.addView(io, 16);
        int pa = b.addAccessor("VEC3", 5126, 4, "0");
        int ia = b.addAccessor("SCALAR", 5125, 4, "1");
        b.addMesh("[" + primJson(attrsPos(pa), std::to_string(ia), "4") + "]");
        b.addNode(nodeJson("0", ""));
        b.addScene("[0]");
        b.topScene = "0";
        dumpReadCase(id++, 0, b.build(), ".glb", 0);
    }
    // R19: non-indexed vertex count not a multiple of 3.
    {
        GlbBuilder b;
        size_t po = b.binSize();
        float pos[12] = { 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1 };
        b.pushBytes(pos, sizeof(pos));
        b.addView(po, 48);
        int pa = b.addAccessor("VEC3", 5126, 4, "0");
        b.addMesh("[" + primJson(attrsPos(pa), "", "4") + "]");
        b.addNode(nodeJson("0", ""));
        b.addScene("[0]");
        b.topScene = "0";
        dumpReadCase(id++, 0, b.build(), ".glb", 0);
    }
    // R20: out-of-range u32 at absolute index 4 (pins the i+k spelling).
    {
        GlbBuilder b;
        size_t po = b.binSize();
        float pos[9] = { 0, 0, 0, 1, 0, 0, 0, 1, 0 };
        b.pushBytes(pos, sizeof(pos));
        size_t io = b.binSize();
        std::uint32_t idx[6] = { 0, 1, 2, 0, 99, 1 };
        b.pushBytes(idx, sizeof(idx));
        b.addView(po, 36);
        b.addView(io, 24);
        int pa = b.addAccessor("VEC3", 5126, 3, "0");
        int ia = b.addAccessor("SCALAR", 5125, 6, "1");
        b.addMesh("[" + primJson(attrsPos(pa), std::to_string(ia), "4") + "]");
        b.addNode(nodeJson("0", ""));
        b.addScene("[0]");
        b.topScene = "0";
        dumpReadCase(id++, 0, b.build(), ".glb", 0);
    }
    // R21: VEC2 POSITION (z stays 0). R22: SCALAR POSITION ([x,0,0]).
    for (int variant = 0; variant < 2; ++variant) {
        GlbBuilder b;
        size_t po = b.binSize();
        if (variant == 0) {
            float pos[6] = { 0, 0, 1, 0, 0, 1 };
            b.pushBytes(pos, sizeof(pos));
        } else {
            float pos[3] = { 4, 5, 6 };
            b.pushBytes(pos, sizeof(pos));
        }
        b.pad4();
        b.addView(po, variant == 0 ? 24 : 12);
        int pa = b.addAccessor(variant == 0 ? "VEC2" : "SCALAR", 5126, 3, "0");
        b.addMesh("[" + primJson(attrsPos(pa), "", "4") + "]");
        b.addNode(nodeJson("0", ""));
        b.addScene("[0]");
        b.topScene = "0";
        dumpReadCase(id++, 0, b.build(), ".glb", 1);
    }
    // R23: VEC4 POSITION -> "failed to read POSITION accessor".
    {
        GlbBuilder b = basicTriBuilder();
        b.accessors[0] = "{\"componentType\":5126,\"count\":3,\"type\":\"VEC4\",\"bufferView\":0}";
        dumpReadCase(id++, 0, b.build(), ".glb", 0);
    }
    // R24: u8 normalized POSITION. R25: i16 normalized POSITION.
    // R26: u16 raw POSITION. R27: u32 raw POSITION.
    for (int variant = 0; variant < 4; ++variant) {
        GlbBuilder b;
        size_t po = b.binSize();
        int comp;
        bool norm = false;
        if (variant == 0) {
            std::uint8_t v[9] = { 0, 0, 0, 255, 0, 0, 0, 255, 0 };
            b.pushBytes(v, sizeof(v));
            comp = 5121;
            norm = true;
        } else if (variant == 1) {
            std::int16_t v[9] = { 0, 0, 0, 32767, 0, 0, 0, -32767, 0 };
            b.pushBytes(v, sizeof(v));
            comp = 5122;
            norm = true;
        } else if (variant == 2) {
            std::uint16_t v[9] = { 0, 0, 0, 7, 0, 0, 0, 9, 0 };
            b.pushBytes(v, sizeof(v));
            comp = 5123;
        } else {
            std::uint32_t v[9] = { 0, 0, 0, 7, 0, 0, 0, 9, 0 };
            b.pushBytes(v, sizeof(v));
            comp = 5125;
        }
        b.pad4();
        size_t len = variant == 0 ? 9 : variant == 1 ? 18 : variant == 2 ? 18 : 36;
        b.addView(po, len);
        int pa = b.addAccessor("VEC3", comp, 3, "0", 0, norm);
        b.addMesh("[" + primJson(attrsPos(pa), "", "4") + "]");
        b.addNode(nodeJson("0", ""));
        b.addScene("[0]");
        b.topScene = "0";
        dumpReadCase(id++, 0, b.build(), ".glb", 1);
    }
    // R28: bogus componentType -> zeros (ok). R29: bogus accessor type ->
    // 1 component (ok). R30: missing componentType -> zeros (ok).
    {
        GlbBuilder b = basicTriBuilder();
        b.accessors[0] = "{\"componentType\":5199,\"count\":3,\"type\":\"VEC3\",\"bufferView\":0}";
        dumpReadCase(id++, 0, b.build(), ".glb", 1);
    }
    {
        GlbBuilder b = basicTriBuilder();
        b.accessors[0] = "{\"componentType\":5126,\"count\":3,\"type\":\"BOGUS\",\"bufferView\":0}";
        dumpReadCase(id++, 0, b.build(), ".glb", 1);
    }
    {
        GlbBuilder b = basicTriBuilder();
        b.accessors[0] = "{\"count\":3,\"type\":\"VEC3\",\"bufferView\":0}";
        dumpReadCase(id++, 0, b.build(), ".glb", 1);
    }
    // R31: sparse POSITION -> "failed to read POSITION accessor".
    {
        GlbBuilder b = basicTriBuilder();
        b.accessors[0] = "{\"componentType\":5126,\"count\":3,\"type\":\"VEC3\",\"bufferView\":0,"
                         "\"sparse\":{\"count\":1,\"indices\":{\"bufferView\":1,\"componentType\":5123,\"byteOffset\":0},"
                         "\"values\":{\"bufferView\":1,\"byteOffset\":0}}}";
        dumpReadCase(id++, 0, b.build(), ".glb", 0);
    }
    // R32: POSITION without bufferView -> zeros (ok). R33: indices without
    // bufferView -> zeros -> tris at vertex 0 (ok).
    {
        GlbBuilder b = basicTriBuilder();
        b.accessors[0] = "{\"componentType\":5126,\"count\":3,\"type\":\"VEC3\"}";
        dumpReadCase(id++, 0, b.build(), ".glb", 1);
    }
    {
        GlbBuilder b = basicTriBuilder();
        b.accessors[1] = "{\"componentType\":5125,\"count\":3,\"type\":\"SCALAR\"}";
        dumpReadCase(id++, 0, b.build(), ".glb", 1);
    }
    // R34: interleaved POSITION+NORMAL (stride 24, POSITION at offset 0).
    {
        GlbBuilder b;
        size_t vo = b.binSize();
        for (int v = 0; v < 3; ++v) {
            float p[3] = { (float)v, (float)(v + 10), (float)(v + 20) };
            float n[3] = { 0, 0, 1 };
            b.pushBytes(p, sizeof(p));
            b.pushBytes(n, sizeof(n));
        }
        size_t io = b.binSize();
        std::uint32_t idx[3] = { 0, 1, 2 };
        b.pushBytes(idx, sizeof(idx));
        b.addView(vo, 72, 24);
        b.addView(io, 12);
        int pa = b.addAccessor("VEC3", 5126, 3, "0");
        int ia = b.addAccessor("SCALAR", 5125, 3, "1");
        b.addMesh("[" + primJson(attrsPos(pa), std::to_string(ia), "4") + "]");
        b.addNode(nodeJson("0", ""));
        b.addScene("[0]");
        b.topScene = "0";
        dumpReadCase(id++, 0, b.build(), ".glb", 1);
    }
    // R35: strided u16 indices (stride 4).
    {
        GlbBuilder b;
        size_t po = b.binSize();
        float pos[9] = { 0, 0, 0, 1, 0, 0, 0, 1, 0 };
        b.pushBytes(pos, sizeof(pos));
        size_t io = b.binSize();
        std::uint16_t idx[6] = { 0, 0xbeef, 1, 0xbeef, 2, 0xbeef };
        b.pushBytes(idx, sizeof(idx));
        b.addView(po, 36);
        b.addView(io, 12, 4);
        int pa = b.addAccessor("VEC3", 5126, 3, "0");
        int ia = b.addAccessor("SCALAR", 5123, 3, "1");
        b.addMesh("[" + primJson(attrsPos(pa), std::to_string(ia), "4") + "]");
        b.addNode(nodeJson("0", ""));
        b.addScene("[0]");
        b.topScene = "0";
        dumpReadCase(id++, 0, b.build(), ".glb", 1);
    }
    // R36: buffers[0].byteLength beyond BIN -> load-buffers fatal.
    {
        GlbBuilder b = basicTriBuilder();
        b.buffersOverride = "[{\"byteLength\":999999}]";
        dumpReadCase(id++, 0, b.build(), ".glb", 0);
    }
    // R37: no BIN chunk, POSITION needs data -> read fatal.
    {
        GlbBuilder b = basicTriBuilder();
        dumpReadCase(id++, 0, b.build(false), ".glb", 0);
    }
    // R38: no BIN chunk, all position-less -> no-triangle fatal.
    {
        GlbBuilder b = basicTriBuilder();
        b.meshes[0] = "{\"primitives\":[" + primJson("{\"NORMAL\":0}", "", "4") + "]}";
        dumpReadCase(id++, 0, b.build(false), ".glb", 0);
    }
    // R39: no meshes at all.
    {
        GlbBuilder b = basicTriBuilder();
        b.meshes.clear();
        b.nodes[0] = nodeJson("", "");
        dumpReadCase(id++, 0, b.build(), ".glb", 0);
    }
    // R40: meshes exist but every primitive skips -> no-triangle fatal.
    {
        GlbBuilder b = basicTriBuilder();
        b.meshes[0] = "{\"primitives\":[" + primJson(attrsPos(0), "1", "0") + ","
            + primJson("{\"NORMAL\":0}", "", "4") + "]}";
        dumpReadCase(id++, 0, b.build(), ".glb", 0);
    }
    // R41-R47: dangling indices -> parse fatal (mesh, children, POSITION,
    // indices, view buffer, scene roots, top scene).
    {
        GlbBuilder b = basicTriBuilder();
        b.nodes[0] = nodeJson("7", "");
        dumpReadCase(id++, 0, b.build(), ".glb", 0);
    }
    {
        GlbBuilder b = basicTriBuilder();
        b.nodes[0] = nodeJson("0", "[9]");
        dumpReadCase(id++, 0, b.build(), ".glb", 0);
    }
    {
        GlbBuilder b = basicTriBuilder();
        b.meshes[0] = "{\"primitives\":[" + primJson(attrsPos(9), "1", "4") + "]}";
        dumpReadCase(id++, 0, b.build(), ".glb", 0);
    }
    {
        GlbBuilder b = basicTriBuilder();
        b.meshes[0] = "{\"primitives\":[" + primJson(attrsPos(0), "9", "4") + "]}";
        dumpReadCase(id++, 0, b.build(), ".glb", 0);
    }
    {
        GlbBuilder b = basicTriBuilder();
        b.views[0] = "{\"buffer\":5,\"byteOffset\":0,\"byteLength\":36}";
        dumpReadCase(id++, 0, b.build(), ".glb", 0);
    }
    {
        GlbBuilder b = basicTriBuilder();
        b.scenes[0] = "{\"nodes\":[4]}";
        dumpReadCase(id++, 0, b.build(), ".glb", 0);
    }
    {
        GlbBuilder b = basicTriBuilder();
        b.topScene = "4";
        dumpReadCase(id++, 0, b.build(), ".glb", 0);
    }
    // R48: double-parented node -> parse fatal. R49: scene lists a
    // non-root -> parse fatal.
    {
        GlbBuilder b = basicTriBuilder();
        b.addNode(nodeJson("", ""));
        b.nodes[0] = nodeJson("0", "[1]");
        b.addNode(nodeJson("", "[1]"));
        dumpReadCase(id++, 0, b.build(), ".glb", 0);
    }
    {
        GlbBuilder b = basicTriBuilder();
        b.addNode(nodeJson("0", ""));
        b.nodes[0] = nodeJson("0", "[1]");
        b.scenes[0] = "{\"nodes\":[0,1]}";
        dumpReadCase(id++, 0, b.build(), ".glb", 0);
    }
    // R50: truncated file (first 100 bytes). R51: garbage (>= 12 bytes).
    // R52: short file (5 bytes). R53: empty file.
    {
        GlbBuilder b = basicTriBuilder();
        std::string full = b.build();
        dumpReadCase(id++, 0, full.substr(0, 100), ".glb", 0);
    }
    {
        dumpReadCase(id++, 0, "this is not a glb file, merely text", ".glb", 0);
    }
    {
        dumpReadCase(id++, 0, "short", ".glb", 0);
    }
    {
        dumpReadCase(id++, 0, "", ".glb", 0);
    }
    // R54: missing file (fixed nonexistent path, no temp file).
    {
        std::printf("READCASE %d INEXACT=0 SUFFIX=glb HEXLEN=0\n", id);
        std::printf("HEX \n");
        const std::string path = "/nonexistent-dir-rs-glb-9f3/missing.glb";
        std::vector<float> positions { 1.5f, -2.5f };
        std::vector<std::vector<size_t>> triangles { { 99 }, { 7, 8 } };
        std::string warn = "W", err = "E";
        bool ok = GlbIo::loadGlbPositionsAndTriangles(
            path.c_str(), &positions, &triangles, &warn, &err);
        CHECK(!ok);
        std::printf("R ok=%d npos=%zu ntri=%zu\n", ok ? 1 : 0, positions.size(), triangles.size());
        dumpPosLine(positions);
        dumpTris(triangles, "TRI");
        std::printf("W %s\n", escapeBytes(replaceAll(warn, path, "@F@")).c_str());
        std::printf("E %s\n", escapeBytes(replaceAll(err, path, "@F@")).c_str());
        ++id;
    }
    // R55-R58: container corruptions (version, JSON type, BIN type,
    // declared total beyond EOF).
    {
        GlbBuilder b = basicTriBuilder();
        std::string blob = b.build();
        blob[4] = 1; // version 1
        dumpReadCase(id++, 0, blob, ".glb", 0);
    }
    {
        GlbBuilder b = basicTriBuilder();
        std::string blob = b.build();
        blob[16] = 'X'; // JSON chunk type
        dumpReadCase(id++, 0, blob, ".glb", 0);
    }
    {
        GlbBuilder b = basicTriBuilder();
        std::string blob = b.build();
        // Second chunk header: find it past the JSON chunk.
        std::uint32_t jl = 0;
        std::memcpy(&jl, blob.data() + 12, 4);
        blob[20 + jl + 4] = 'X'; // BIN chunk type
        dumpReadCase(id++, 0, blob, ".glb", 0);
    }
    {
        GlbBuilder b = basicTriBuilder();
        std::string blob = b.build();
        std::uint32_t big = (std::uint32_t)blob.size() + 100;
        std::memcpy(blob.data() + 8, &big, 4); // total length beyond EOF
        dumpReadCase(id++, 0, blob, ".glb", 0);
    }
    // R59: trailing garbage past the declared total is ignored (ok).
    {
        GlbBuilder b = basicTriBuilder();
        std::string blob = b.build() + "TRAILING-JUNK-BYTES";
        dumpReadCase(id++, 0, blob, ".glb", 1);
    }
    // R60: JSON chunk with trailing spaces (writer padding) still parses.
    {
        GlbBuilder b = basicTriBuilder();
        dumpReadCase(id++, 0, b.build(), ".glb", 1);
    }
    // R61: custom + standard extra attributes are ignored (ok).
    {
        GlbBuilder b = basicTriBuilder();
        b.meshes[0] = "{\"primitives\":["
            + primJson(attrsPos(0, "\"NORMAL\":0,\"TEXCOORD_0\":0,\"_CUSTOM\":0,\"COLOR_0\":0"), "1", "4") + "]}";
        dumpReadCase(id++, 0, b.build(), ".glb", 1);
    }
    // R62: custom attribute with a dangling index -> parse fatal.
    {
        GlbBuilder b = basicTriBuilder();
        b.meshes[0] = "{\"primitives\":["
            + primJson(attrsPos(0, "\"_CUSTOM\":9"), "1", "4") + "]}";
        dumpReadCase(id++, 0, b.build(), ".glb", 0);
    }
    // R63: benign morph targets are ignored (ok). R64: dangling target
    // index -> parse fatal.
    {
        GlbBuilder b = basicTriBuilder();
        b.meshes[0] = "{\"primitives\":["
            + primJson(attrsPos(0), "1", "4", "[{\"POSITION\":0}]") + "]}";
        dumpReadCase(id++, 0, b.build(), ".glb", 1);
    }
    {
        GlbBuilder b = basicTriBuilder();
        b.meshes[0] = "{\"primitives\":["
            + primJson(attrsPos(0), "1", "4", "[{\"POSITION\":9}]") + "]}";
        dumpReadCase(id++, 0, b.build(), ".glb", 0);
    }
    // R65: extras/extensions junk is ignored (ok).
    {
        GlbBuilder b = basicTriBuilder();
        b.extraTop = ",\"extras\":{\"anything\":[1,2,{\"x\":null}]},\"extensions\":{\"EXT_foo\":{\"y\":true}}";
        b.meshes[0] = "{\"primitives\":[" + primJson(attrsPos(0), "1", "4")
            + "],\"extras\":[1],\"extensions\":{\"E\":{}}}";
        dumpReadCase(id++, 0, b.build(), ".glb", 1);
    }
    // R66: string "mode" -> invalid -> skip-count (ok with warn).
    {
        GlbBuilder b = basicTriBuilder();
        b.meshes[0] = "{\"primitives\":[" + primJson(attrsPos(0), "1", "\"x\"") + ","
            + primJson(attrsPos(0), "1", "4") + "]}";
        dumpReadCase(id++, 0, b.build(), ".glb", 1);
    }
    // R67: "indices": -1 -> non-indexed path (ok).
    {
        GlbBuilder b = basicTriBuilder();
        b.meshes[0] = "{\"primitives\":[" + primJson(attrsPos(0), "-1", "4") + "]}";
        dumpReadCase(id++, 0, b.build(), ".glb", 1);
    }
    // R68: "mesh": -1 -> node mesh-less -> identity fallback (ok).
    {
        GlbBuilder b = basicTriBuilder();
        b.nodes[0] = nodeJson("-1", "");
        dumpReadCase(id++, 0, b.build(), ".glb", 1);
    }
    // R69: "mesh": "x" (explicit type check) -> parse fatal.
    {
        GlbBuilder b = basicTriBuilder();
        b.nodes[0] = nodeJson("\"x\"", "");
        dumpReadCase(id++, 0, b.build(), ".glb", 0);
    }
    // R70: "bufferView": -1 -> zeros (ok).
    {
        GlbBuilder b = basicTriBuilder();
        b.accessors[0] = "{\"componentType\":5126,\"count\":3,\"type\":\"VEC3\",\"bufferView\":-1}";
        dumpReadCase(id++, 0, b.build(), ".glb", 1);
    }
    // R71: buffer with a uri -> load-buffers fatal.
    {
        GlbBuilder b = basicTriBuilder();
        std::ostringstream o;
        o << "[{\"byteLength\":" << b.binSize() << ",\"uri\":\"external.bin\"}]";
        b.buffersOverride = o.str();
        dumpReadCase(id++, 0, b.build(), ".glb", 0);
    }
    // R72: translation array of wrong length -> parse fatal. R73: matrix
    // of 15 entries -> parse fatal.
    {
        GlbBuilder b = basicTriBuilder();
        b.nodes[0] = nodeJson("0", "", "\"translation\":[1,2]");
        dumpReadCase(id++, 0, b.build(), ".glb", 0);
    }
    {
        GlbBuilder b = basicTriBuilder();
        b.nodes[0] = nodeJson("0", "", "\"matrix\":[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0]");
        dumpReadCase(id++, 0, b.build(), ".glb", 0);
    }
    // R74: exponent count "1e3" reads as 1 (atoi prefix) -> 1 vertex,
    // non-indexed 1 % 3 -> fatal.
    {
        GlbBuilder b = basicTriBuilder();
        b.accessors[0] = "{\"componentType\":5126,\"count\":1e3,\"type\":\"VEC3\",\"bufferView\":0}";
        b.meshes[0] = "{\"primitives\":[" + primJson(attrsPos(0), "", "4") + "]}";
        dumpReadCase(id++, 0, b.build(), ".glb", 0);
    }
    // R75: bool count reads as 0 -> position-less skip; with no other
    // geometry -> no-triangle fatal.
    {
        GlbBuilder b = basicTriBuilder();
        b.accessors[0] = "{\"componentType\":5126,\"count\":true,\"type\":\"VEC3\",\"bufferView\":0}";
        dumpReadCase(id++, 0, b.build(), ".glb", 0);
    }
    // R76: raw JSON glTF without the container (cgltf fallback) parses,
    // then reads fail on the unbound buffer data.
    {
        GlbBuilder b = basicTriBuilder();
        std::string blob = b.build();
        std::uint32_t jl = 0;
        std::memcpy(&jl, blob.data() + 12, 4);
        std::string json = blob.substr(20, jl);
        dumpReadCase(id++, 0, json, ".glb", 0);
    }
    // R76b: raw JSON with view-less accessors loads zeros through the
    // same fallback path (ok).
    {
        GlbBuilder b = basicTriBuilder();
        b.accessors[0] = "{\"componentType\":5126,\"count\":3,\"type\":\"VEC3\"}";
        b.accessors[1] = "{\"componentType\":5125,\"count\":3,\"type\":\"SCALAR\"}";
        std::string blob = b.build();
        std::uint32_t jl = 0;
        std::memcpy(&jl, blob.data() + 12, 4);
        dumpReadCase(id++, 0, blob.substr(20, jl), ".glb", 1);
    }
    // R77: raw JSON garbage -> parse fatal.
    {
        dumpReadCase(id++, 0, "{\"meshes\": [oops", ".glb", 0);
    }
    // R78: inf POSITION data under identity (bitwise).
    {
        GlbBuilder b;
        size_t po = b.binSize();
        float inf = 1.0f / 0.0f;
        float pos[9] = { inf, 1, 2, 3, 4, 5, 6, 7, 8 };
        b.pushBytes(pos, sizeof(pos));
        b.addView(po, 36);
        int pa = b.addAccessor("VEC3", 5126, 3, "0");
        b.addMesh("[" + primJson(attrsPos(pa), "", "4") + "]");
        b.addNode(nodeJson("0", ""));
        b.addScene("[0]");
        b.topScene = "0";
        dumpReadCase(id++, 0, b.build(), ".glb", 1);
    }
    // R79: count-0 index accessor contributes nothing; the soup prim loads.
    {
        GlbBuilder b;
        size_t po = b.binSize();
        float pos[9] = { 0, 0, 0, 1, 0, 0, 0, 1, 0 };
        b.pushBytes(pos, sizeof(pos));
        size_t io = b.binSize();
        std::uint32_t idx[3] = { 0, 1, 2 };
        b.pushBytes(idx, sizeof(idx));
        b.addView(po, 36);
        b.addView(io, 12);
        int pa = b.addAccessor("VEC3", 5126, 3, "0");
        int ia = b.addAccessor("SCALAR", 5125, 0, "1");
        std::string prims = "[" + primJson(attrsPos(pa), std::to_string(ia), "4") + ","
            + primJson(attrsPos(pa), "", "4") + "]";
        b.addMesh(prims);
        b.addNode(nodeJson("0", ""));
        b.addScene("[0]");
        b.topScene = "0";
        dumpReadCase(id++, 0, b.build(), ".glb", 1);
    }
    // R80: escaped attribute name "POS\\u0049TION" matches nothing
    // (cgltf compares raw names), so the prim is position-less -> fatal.
    {
        GlbBuilder b = basicTriBuilder();
        b.meshes[0] = "{\"primitives\":[" + primJson("{\"POS\\u0049TION\":0}", "1", "4") + "]}";
        dumpReadCase(id++, 0, b.build(), ".glb", 0);
    }
    return id;
}

// ---------------------------------------------------------------------------
// Seeded randomized reader cases (combinations of the subset paths).
// ---------------------------------------------------------------------------

static float randFiniteBits()
{
    std::uint32_t bits = (std::uint32_t)nextU64();
    float f;
    std::memcpy(&f, &bits, 4);
    if (!std::isfinite(f))
        return 0.0f;
    return f;
}

// Moderate magnitudes for INEXACT cases: general rotations must stay far
// from overflow so the tolerance pins logic, not FMA overflow boundaries.
static float randModerate()
{
    for (;;) {
        float f = randFiniteBits();
        if (std::fabs(f) <= 1e6f)
            return f;
    }
}

static float randExact()
{
    // Small integers and halves: every transform intermediate is exact.
    static const float pool[] = { 0, 1, -1, 2, -2, 3, -3, 4, 0.5f, -0.5f, 1.5f, 8, -8, 0.25f, 10 };
    return pool[below(sizeof(pool) / sizeof(pool[0]))];
}

static std::string genXformExact()
{
    switch (below(10)) {
    case 0:
    case 1:
    case 2:
        return ""; // identity
    case 3:
    case 4: {
        float t[3] = { randExact(), randExact(), randExact() };
        float q[4] = { 0, 0, 0, 1 };
        float s[3] = { 1, 1, 1 };
        return xformTRS(t, q, s);
    }
    case 5: {
        float t[3] = { 0, 0, 0 };
        float q[4] = { 0, 0, 0, 1 };
        float s[3] = { randExact(), randExact(), randExact() };
        if (s[0] == 0)
            s[0] = 1;
        if (s[1] == 0)
            s[1] = 1;
        if (s[2] == 0)
            s[2] = 1;
        return xformTRS(t, q, s);
    }
    case 6: {
        // 180-degree rotation about a random axis (exact quaternion).
        float t[3] = { randExact(), randExact(), 0 };
        float q[4] = { 0, 0, 0, 0 };
        q[below(3)] = 1;
        float s[3] = { 1, 2, 0.5f };
        return xformTRS(t, q, s);
    }
    case 7: {
        // Exact axis-permutation matrix + translation.
        float m[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };
        int perm[3] = { 0, 1, 2 };
        for (int i = 2; i > 0; --i) {
            int j = (int)below(i + 1);
            int t = perm[i];
            perm[i] = perm[j];
            perm[j] = t;
        }
        for (int c = 0; c < 3; ++c) {
            m[c * 4 + perm[c]] = below(2) ? 1.0f : -1.0f;
        }
        m[12] = randExact();
        m[13] = randExact();
        m[14] = randExact();
        return xformMatrix(m);
    }
    default: {
        float t[3] = { randExact(), randExact(), randExact() };
        float q[4] = { 0, 0, 0, 1 };
        float s[3] = { 2, 0.5f, -1 };
        return xformTRS(t, q, s);
    }
    }
}

static std::string genXformInexact()
{
    // General TRS (normalized random quaternion): replay is approximate.
    float t[3] = { randModerate(), randModerate(), randModerate() };
    float q[4] = { randFiniteBits(), randFiniteBits(), randFiniteBits(), randFiniteBits() };
    float len = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (!(len > 0) || !std::isfinite(len)) {
        q[0] = q[1] = q[2] = 0;
        q[3] = 1;
    } else {
        q[0] /= len;
        q[1] /= len;
        q[2] /= len;
        q[3] /= len;
    }
    float s[3] = { 0.5f + (float)below(100) / 50.0f, 0.5f + (float)below(100) / 50.0f,
        0.5f + (float)below(100) / 50.0f };
    return xformTRS(t, q, s);
}

static int runGenReads(int id, int count)
{
    for (int c = 0; c < count; ++c) {
        GlbBuilder b;
        int inexact = (below(10) == 0) ? 1 : 0;
        int nmeshes = 1 + (int)below(2);
        std::vector<int> meshIds;
        for (int m = 0; m < nmeshes; ++m) {
            int nprims = 1 + (int)below(3);
            std::string prims = "[";
            for (int p = 0; p < nprims; ++p) {
                // Mode.
                std::string mode;
                std::uint64_t mr = below(10);
                if (mr < 7)
                    mode = "4";
                else if (mr == 7) {
                    static const char* ms[] = { "0", "1", "2", "3", "5", "6" };
                    mode = ms[below(6)];
                } else if (mr == 8)
                    mode = "99";
                // Position accessor kind.
                size_t vcount = 3 + below(7);
                std::string attrs;
                std::uint64_t pr = below(20);
                std::uint64_t irPeek = below(12);
                bool soupPeek = irPeek < 3;
                if (soupPeek && vcount % 3 != 0 && below(5) != 0)
                    vcount += 3 - vcount % 3; // usually-valid soup
                if (pr == 12) {
                    attrs = (below(2) == 0) ? "" : "{\"NORMAL\":0}";
                    // Ensure accessor 0 exists for the NORMAL-only form.
                    if (!attrs.empty() && b.accessors.empty()) {
                        size_t po = b.binSize();
                        float z[3] = { 0, 0, 0 };
                        b.pushBytes(z, sizeof(z));
                        b.pad4();
                        b.addView(po, 12);
                        b.addAccessor("VEC3", 5126, 1, "0");
                    }
                    if (!attrs.empty() && attrs != "") {
                        // NORMAL cites accessor 0 (exists by now).
                    }
                } else {
                    int comp = 5126;
                    const char* type = "VEC3";
                    bool norm = false;
                    size_t compBytes = 4;
                    if (pr == 13)
                        vcount = 0; // empty POSITION
                    else if (pr == 14)
                        type = "VEC2";
                    else if (pr == 15) {
                        comp = 5121;
                        norm = true;
                        compBytes = 1;
                    } else if (pr == 16) {
                        comp = 5122;
                        norm = true;
                        compBytes = 2;
                    } else if (pr == 17) {
                        comp = 5123;
                        compBytes = 2;
                    } else if (pr == 19)
                        type = "VEC4"; // fatal read
                    bool strided = below(7) == 0 && compBytes == 4
                        && std::string(type) == "VEC3" && vcount > 0;
                    size_t stride = strided ? 24 : 0;
                    size_t accOffset = (below(5) == 0 && vcount > 0) ? 12 : 0;
                    size_t po = b.binSize();
                    if (accOffset)
                        b.pushBytes("\0\0\0\0\0\0\0\0\0\0\0\0", 12);
                    size_t ncomp = 3;
                    if (std::strcmp(type, "VEC2") == 0)
                        ncomp = 2;
                    else if (std::strcmp(type, "VEC4") == 0)
                        ncomp = 4;
                    for (size_t v = 0; v < vcount; ++v) {
                        for (size_t k = 0; k < ncomp; ++k) {
                            if (comp == 5126)
                                b.pushF32(inexact ? randModerate() : randFiniteBits());
                            else if (comp == 5121)
                                b.pushU8((std::uint8_t)below(256));
                            else if (comp == 5122)
                                b.pushU16((std::uint16_t)nextU64());
                            else if (comp == 5123)
                                b.pushU16((std::uint16_t)below(64));
                        }
                        if (strided) {
                            float pad[3] = { 0, 0, 0 };
                            b.pushBytes(pad, sizeof(pad));
                        }
                    }
                    b.pad4();
                    size_t vlen = b.binSize() - po;
                    int view = b.addView(po, vlen, stride);
                    int pa;
                    if (pr == 18) {
                        // View-less POSITION (zeros).
                        b.views.pop_back();
                        pa = b.addAccessor(type, comp, vcount, "");
                    } else {
                        pa = b.addAccessor(type, comp, vcount, std::to_string(view).c_str(),
                            accOffset, norm);
                    }
                    std::ostringstream an;
                    an << "{\"POSITION\":" << pa;
                    if (below(3) == 0)
                        an << ",\"NORMAL\":" << pa;
                    if (below(4) == 0)
                        an << ",\"TEXCOORD_0\":" << pa;
                    an << "}";
                    attrs = an.str();
                }
                // Indices.
                std::string idx;
                std::uint64_t ir = irPeek;
                bool fatalOor = false;
                if (soupPeek || attrs.empty()) {
                    // Non-indexed (a bad soup count stays fatal: coverage).
                } else {
                    int icomp = 5125;
                    size_t iwidth = 4;
                    if (ir == 3 || ir == 4) {
                        icomp = 5121;
                        iwidth = 1;
                    } else if (ir == 5 || ir == 6) {
                        icomp = 5123;
                        iwidth = 2;
                    } else if (ir == 11) {
                        icomp = 5122;
                        iwidth = 2;
                    }
                    size_t icount = vcount == 0 ? 0 : 3 * (1 + below(3));
                    if (ir == 9 && icount > 0)
                        icount -= 1; // bad count (fatal)
                    if (ir == 10 && vcount > 0) {
                        fatalOor = true; // one corner out of range
                    }
                    size_t io = b.binSize();
                    size_t istride = (below(6) == 0 && iwidth < 4) ? 4 : 0;
                    for (size_t i = 0; i < icount; ++i) {
                        std::uint32_t v = 0;
                        if (vcount > 0) {
                            v = (std::uint32_t)below(vcount);
                            if (fatalOor && i == icount - 1)
                                v = (std::uint32_t)(vcount + below(3));
                        }
                        if (iwidth == 1)
                            b.pushU8((std::uint8_t)v);
                        else if (iwidth == 2)
                            b.pushU16((std::uint16_t)v);
                        else
                            b.pushU32(v);
                        if (istride) {
                            std::uint16_t pad = 0;
                            b.pushBytes(&pad, istride - iwidth);
                        }
                    }
                    b.pad4();
                    size_t ilen = b.binSize() - io;
                    int iview = b.addView(io, ilen, istride);
                    int ia = b.addAccessor(
                        "SCALAR", icomp, icount, std::to_string(iview).c_str());
                    idx = std::to_string(ia);
                }
                std::string targets;
                if (below(10) == 0 && !b.accessors.empty())
                    targets = "[{\"POSITION\":0}]";
                prims += (p ? "," : "") + primJson(attrs, idx, mode, targets);
            }
            prims += "]";
            meshIds.push_back(b.addMesh(prims));
        }
        // Nodes (instancing: nodes may share meshes; some mesh-less).
        int nnodes = nmeshes + (int)below(2);
        std::vector<int> hasParent(nnodes, 0);
        std::vector<std::string> nodeMesh(nnodes), nodeKids(nnodes), nodeXf(nnodes);
        for (int n = 0; n < nnodes; ++n) {
            if (below(5) == 0)
                nodeMesh[n] = "";
            else
                nodeMesh[n] = std::to_string(meshIds[below(meshIds.size())]);
            nodeXf[n] = inexact ? genXformInexact() : genXformExact();
        }
        // Random chains (node n+1 under node n, sometimes).
        for (int n = 0; n + 1 < nnodes; ++n) {
            if (below(2) == 0) {
                nodeKids[n] = "[" + std::to_string(n + 1) + "]";
                hasParent[n + 1] = 1;
            }
        }
        for (int n = 0; n < nnodes; ++n)
            b.addNode(nodeJson(nodeMesh[n], nodeKids[n], nodeXf[n]));
        std::string roots = "[";
        bool first = true;
        for (int n = 0; n < nnodes; ++n) {
            if (!hasParent[n]) {
                if (!first)
                    roots += ",";
                first = false;
                roots += std::to_string(n);
            }
        }
        roots += "]";
        b.addScene(roots);
        if (below(5) != 0)
            b.topScene = "0";
        std::string suffix = (below(10) == 0) ? ".GLB" : ".glb";
        dumpReadCase(id++, inexact, b.build(), suffix, -1);
    }
    return id;
}

// ---------------------------------------------------------------------------
// Committed-file case, extension battery, writer battery, timing.
// ---------------------------------------------------------------------------

static void runFileCase(const std::string& fixturesDir)
{
    const std::string path = fixturesDir + "/tetra.glb";
    std::printf("FILE tetra.glb\n");
    std::vector<float> positions { 1.5f, -2.5f };
    std::vector<std::vector<size_t>> triangles { { 99 }, { 7, 8 } };
    std::string warn = "W", err = "E";
    bool ok = GlbIo::loadGlbPositionsAndTriangles(
        path.c_str(), &positions, &triangles, &warn, &err);
    CHECK(ok);
    std::printf("R ok=%d npos=%zu ntri=%zu\n", ok ? 1 : 0, positions.size(), triangles.size());
    dumpPosLine(positions);
    dumpTris(triangles, "TRI");
    std::printf("W %s\n", escapeBytes(replaceAll(warn, path, "@F@")).c_str());
    std::printf("E %s\n", escapeBytes(replaceAll(err, path, "@F@")).c_str());
}

static void runExtCases()
{
    static const char* paths[] = {
        "model.glb", "model.GLB", "model.GlB", "model.obj", "model.OBJ",
        "model.txt", "model", "model.", ".glb", ".obj", ".GLB",
        "dir.glb/file", "dir/file.GLB", "a\\b.glb", "a.b\\c",
        "/tmp/x/y.obj", "archive.tar.glb", "", "GLB", "x.Obj", "x.GLB2",
    };
    for (size_t i = 0; i < sizeof(paths) / sizeof(paths[0]); ++i)
        dumpExtCase((int)i, paths[i]);
}

static void runWriteCases()
{
    using V3 = AutoRemesher::Vector3;
    using V2 = AutoRemesher::Vector2;
    int id = 0;
    const std::vector<V2> noUvs;
    // W1: single triangle.
    dumpWriteCase(id++, "tri", 0,
        { V3(0, 0, 0), V3(1, 0, 0), V3(0, 1, 0) }, { { 0, 1, 2 } }, noUvs, 0);
    // W2: quad fan. W3: pentagon fan.
    dumpWriteCase(id++, "quad", 0,
        { V3(0, 0, 0), V3(1, 0, 0), V3(1, 1, 0), V3(0, 1, 0) }, { { 0, 1, 2, 3 } }, noUvs, 0);
    dumpWriteCase(id++, "pentagon", 0,
        { V3(0, 0, 0), V3(3, 0, 0), V3(4, 2, 0), V3(1.5, 4, 0), V3(-1, 2, 0) },
        { { 0, 1, 2, 3, 4 } }, noUvs, 0);
    // W4: mixed faces + degenerate skips (2-vert, 1-vert, empty).
    dumpWriteCase(id++, "mixed-degen", 0,
        { V3(0, 0, 0), V3(1, 0, 0), V3(1, 1, 0), V3(0, 1, 0), V3(0, 0, 1) },
        { { 0, 1 }, { 2 }, {}, { 0, 1, 2 }, { 0, 1, 2, 3 }, { 0, 1, 2, 3, 4 } },
        noUvs, 0);
    // W5: empty verts. W6: empty faces. W7: all-degenerate faces.
    dumpWriteCase(id++, "empty-verts", 0, {}, {}, noUvs, 0);
    dumpWriteCase(id++, "empty-faces", 0,
        { V3(0, 0, 0), V3(1, 0, 0), V3(0, 1, 0) }, {}, noUvs, 0);
    dumpWriteCase(id++, "all-degen", 0,
        { V3(0, 0, 0), V3(1, 0, 0) }, { { 0, 1 }, { 0 } }, noUvs, 0);
    // W8: out-of-range index. W9: out-of-range in the second face.
    dumpWriteCase(id++, "oor", 0,
        { V3(0, 0, 0), V3(1, 0, 0), V3(0, 1, 0) }, { { 0, 1, 9 } }, noUvs, 0);
    dumpWriteCase(id++, "oor-second", 0,
        { V3(0, 0, 0), V3(1, 0, 0), V3(0, 1, 0) },
        { { 0, 1, 2 }, { 0, 1, 7 } }, noUvs, 0);
    // W10: uvs overload (quad + tri). W11: uvs length mismatch.
    // W12: empty verts + empty uvs (size check passes, write fails empty).
    dumpWriteCase(id++, "uvs", 1,
        { V3(0, 0, 0), V3(1, 0, 0), V3(1, 1, 0), V3(0, 1, 0) },
        { { 0, 1, 2 }, { 0, 1, 2, 3 } },
        { V2(0, 0), V2(1, 0), V2(1, 1), V2(0, 1) }, 0);
    dumpWriteCase(id++, "uvs-mismatch", 1,
        { V3(0, 0, 0), V3(1, 0, 0), V3(0, 1, 0) }, { { 0, 1, 2 } },
        { V2(0, 0), V2(1, 1) }, 0);
    dumpWriteCase(id++, "uvs-empty", 1, {}, {}, {}, 0);
    // W13: %g stress coordinates (fixed/scientific boundary cases).
    dumpWriteCase(id++, "g-stress", 0,
        { V3(0.1, 1e-5, 1e7), V3(123456.789, -0.00125, 999999.5),
          V3(100000.5, 0.0001, 1234567.0), V3(-2.5, 3.14159265358979, 1e-4) },
        { { 0, 1, 2 }, { 0, 2, 3 } }, noUvs, 0);
    // W14: infinite coordinates. W15: NaN first vertex. W16: -0.0 first.
    {
        double inf = 1.0 / 0.0;
        dumpWriteCase(id++, "inf", 0,
            { V3(0, 0, 0), V3(inf, 1, 1), V3(0, 1, -inf) }, { { 0, 1, 2 } }, noUvs, 0);
    }
    {
        double nan = 0.0 / 0.0;
        dumpWriteCase(id++, "nan-first", 0,
            { V3(nan, 0, 0), V3(1, 0, 0), V3(0, 1, 0) }, { { 0, 1, 2 } }, noUvs, 0);
    }
    dumpWriteCase(id++, "negzero-first", 0,
        { V3(-0.0, 0, 0), V3(1, 0, 0), V3(0, 1, 0) }, { { 0, 1, 2 } }, noUvs, 0);
    // W17: subnormal + huge coordinates.
    dumpWriteCase(id++, "sub-huge", 0,
        { V3(1e-40, 0, 0), V3(1e38, -1e38, 2.5), V3(0, 1, 0) }, { { 0, 1, 2 } }, noUvs, 0);
    // W18: UTF-8 generator. W19: empty generator.
    dumpWriteCase(id++, "utf8-gen", 0,
        { V3(0, 0, 0), V3(1, 0, 0), V3(0, 1, 0) }, { { 0, 1, 2 } }, noUvs, 0,
        "retopo\xC3\xA9\xE2\x80\x94"
        "forge");
    dumpWriteCase(id++, "empty-gen", 0,
        { V3(0, 0, 0), V3(1, 0, 0), V3(0, 1, 0) }, { { 0, 1, 2 } }, noUvs, 0, "");
    // W20: unwritable path.
    dumpWriteCase(id++, "badpath", 0,
        { V3(0, 0, 0), V3(1, 0, 0), V3(0, 1, 0) }, { { 0, 1, 2 } }, noUvs, 1);
}

// Grid mesh (G x G verts, 2 tris per cell) shared with the Rust timing
// test: same loops, verified by hash before timing.
static void timingGrid(int g, std::vector<AutoRemesher::Vector3>* verts,
    std::vector<std::vector<size_t>>* faces)
{
    for (int y = 0; y < g; ++y) {
        for (int x = 0; x < g; ++x)
            verts->push_back(AutoRemesher::Vector3(x, y, 0));
    }
    for (int y = 0; y < g - 1; ++y) {
        for (int x = 0; x < g - 1; ++x) {
            size_t a = (size_t)y * g + x;
            size_t b = a + 1;
            size_t c = a + g;
            size_t d = c + 1;
            faces->push_back({ a, c, b });
            faces->push_back({ b, c, d });
        }
    }
}

static void runTiming()
{
    const int g = 200;
    std::vector<AutoRemesher::Vector3> verts;
    std::vector<std::vector<size_t>> faces;
    timingGrid(g, &verts, &faces);
    static int pid = 0;
    if (!pid)
        pid = (int)::getpid();
    std::string path = (std::filesystem::temp_directory_path()
        / ("retopo_glbtime_" + std::to_string(pid) + ".glb"))
                           .string();
    for (int i = 0; i < 3; ++i) {
        auto t0 = std::chrono::steady_clock::now();
        bool ok = GlbIo::saveGlb(path.c_str(), "glb_diff_dump", verts, faces);
        auto t1 = std::chrono::steady_clock::now();
        CHECK(ok);
        std::ifstream in(path, std::ios::binary);
        std::string bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        std::printf("T save_glb_grid i=%d ok=%d nbytes=%zu save_ms=%.3f hash=%016llx\n",
            i, ok ? 1 : 0, bytes.size(), ms, (unsigned long long)fnv1a(bytes));
    }
    for (int i = 0; i < 3; ++i) {
        std::vector<float> positions;
        std::vector<std::vector<size_t>> triangles;
        std::string warn, err;
        auto t0 = std::chrono::steady_clock::now();
        bool ok = GlbIo::loadGlbPositionsAndTriangles(
            path.c_str(), &positions, &triangles, &warn, &err);
        auto t1 = std::chrono::steady_clock::now();
        CHECK(ok);
        double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        std::ifstream in(path, std::ios::binary);
        std::string bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        std::printf("T load_glb_grid i=%d ok=%d npos=%zu ntri=%zu load_ms=%.3f hash=%016llx\n",
            i, ok ? 1 : 0, positions.size(), triangles.size(), ms,
            (unsigned long long)fnv1a(bytes));
    }
    std::filesystem::remove(path);
}

int main(int argc, char** argv)
{
    if (argc != 2) {
        std::fprintf(stderr, "usage: glb_diff_dump <fixtures-dir>\n");
        return 2;
    }
    std::printf("GLBDIFF1\n");
    int id = runFixedReads();
    id = runGenReads(id, 150);
    runFileCase(argv[1]);
    runExtCases();
    runWriteCases();
    runTiming();
    if (g_failures == 0)
        std::fprintf(stderr, "glb_diff_dump: %d read cases ok\n", id);
    return g_failures == 0 ? 0 : 1;
}
