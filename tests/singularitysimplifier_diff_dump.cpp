// Differential oracle dump for the singularitysimplifier Rust port
// (wave 3, rs-singularitysimplifier lane). Builds mesh + face-field cases
// (planar/relief/ridge grids, tetra/octa/cube, degenerate variants;
// uniform/vortex/dipole/random/tiny-rotated fields), runs the C++
// SingularitySimplifier, and prints inputs + outputs in a token format
// that rust/core/tests/singularity_simplifier_diff.rs replays.
//
// Classes: 'E' (exact: few input singularities, < 24 candidates per
// round, so the libc++ insertion-sort regime matches Rust's stable sort
// bit-for-bit) vs 'S' (structural-only: dense singularity clusters where
// the unspecified std::sort tie order may legitimately diverge). The tool
// proposes E for structured/small cases and S for stress meshes, then
// DEMOTES to S whenever the C++ output shows before > 7 or
// after > before; the replay re-checks the E preconditions on the Rust
// side, so generator drift fails loudly instead of silently weakening.
//
// Build-only helper: not registered with ctest. Run it and redirect
// stdout to tests/fixtures/singularitysimplifier_diff.txt, then commit
// the fixture.
import retopo.core.singularity_simplifier;
import retopo.core.surface_mesh;
import retopo.core.vector3;

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

using AutoRemesher::SingularitySimplifier;
using AutoRemesher::SurfaceMesh;
using AutoRemesher::Vector3;

static std::uint64_t g_state = 0x51DE5EDD1EULL;

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

static double unitRand()
{
    return static_cast<double>(nextU64() >> 11) * (1.0 / 9007199254740992.0);
}

static void printDouble(double x)
{
    std::printf(" %.17g", x);
}

struct Mesh {
    std::vector<Vector3> positions;
    std::vector<std::vector<size_t>> triangles;
};

// Triangulated w x h grid over (w+1) x (h+1) vertices, CCW from +z.
// relief: 0 = planar, 1 = integer relief (0.1 steps), 2 = sharp ridge.
static Mesh makeGrid(size_t w, size_t h, int relief)
{
    Mesh mesh;
    auto id = [w](size_t x, size_t y) { return y * (w + 1) + x; };
    for (size_t y = 0; y <= h; ++y) {
        for (size_t x = 0; x <= w; ++x) {
            double z = 0.0;
            if (relief == 1)
                z = 0.1 * static_cast<double>((x * 7 + y * 13) % 5);
            else if (relief == 2)
                z = std::fabs(static_cast<double>(x) - static_cast<double>(w) / 2.0);
            mesh.positions.emplace_back(static_cast<double>(x), static_cast<double>(y), z);
        }
    }
    for (size_t y = 0; y < h; ++y) {
        for (size_t x = 0; x < w; ++x) {
            const size_t a = id(x, y), b = id(x + 1, y);
            const size_t c = id(x + 1, y + 1), d = id(x, y + 1);
            mesh.triangles.push_back({ a, b, c });
            mesh.triangles.push_back({ a, c, d });
        }
    }
    return mesh;
}

static Mesh makeTetra()
{
    Mesh mesh;
    mesh.positions = { Vector3(0, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0), Vector3(0, 0, 1) };
    mesh.triangles = { { 0, 2, 1 }, { 0, 1, 3 }, { 0, 3, 2 }, { 1, 2, 3 } };
    return mesh;
}

static Mesh makeOcta()
{
    Mesh mesh;
    mesh.positions = { Vector3(1, 0, 0), Vector3(-1, 0, 0), Vector3(0, 1, 0),
        Vector3(0, -1, 0), Vector3(0, 0, 1), Vector3(0, 0, -1) };
    mesh.triangles = { { 4, 0, 2 }, { 4, 2, 1 }, { 4, 1, 3 }, { 4, 3, 0 },
        { 5, 2, 0 }, { 5, 1, 2 }, { 5, 3, 1 }, { 5, 0, 3 } };
    return mesh;
}

static Mesh makeCube()
{
    Mesh mesh;
    mesh.positions = { Vector3(0, 0, 0), Vector3(1, 0, 0), Vector3(1, 1, 0), Vector3(0, 1, 0),
        Vector3(0, 0, 1), Vector3(1, 0, 1), Vector3(1, 1, 1), Vector3(0, 1, 1) };
    mesh.triangles = { { 0, 3, 2 }, { 0, 2, 1 }, { 4, 5, 6 }, { 4, 6, 7 }, { 0, 1, 5 },
        { 0, 5, 4 }, { 3, 7, 6 }, { 3, 6, 2 }, { 0, 4, 7 }, { 0, 7, 3 }, { 1, 2, 6 },
        { 1, 6, 5 } };
    return mesh;
}

static Vector3 centroidOf(const SurfaceMesh& mesh, size_t face)
{
    const auto& tri = mesh.triangle(face);
    const Vector3& a = mesh.position(tri[0]);
    const Vector3& b = mesh.position(tri[1]);
    const Vector3& c = mesh.position(tri[2]);
    return Vector3((a.x() + b.x() + c.x()) / 3.0, (a.y() + b.y() + c.y()) / 3.0,
        (a.z() + b.z() + c.z()) / 3.0);
}

// Tangential swirl around `center` (+1) or its reverse (-1).
static Vector3 swirl(const SurfaceMesh& mesh, size_t face, const Vector3& center, double chirality)
{
    const Vector3 normal = mesh.faceNormal(face);
    const Vector3 d = centroidOf(mesh, face) - center;
    Vector3 t = Vector3::crossProduct(normal, d);
    if (t.length() < 1e-12)
        t = mesh.edgeVector(3 * face);
    if (t.length() < 1e-12)
        t = Vector3(1, 0, 0);
    return chirality * t.normalized();
}

static std::vector<Vector3> uniformField(size_t faces, const Vector3& v)
{
    return std::vector<Vector3>(faces, v);
}

static std::vector<Vector3> vortexField(
    const SurfaceMesh& mesh, const Vector3& center, double chirality)
{
    std::vector<Vector3> field;
    for (size_t f = 0; f < mesh.faceCount(); ++f)
        field.push_back(swirl(mesh, f, center, chirality));
    return field;
}

// Fractional-winding field: theta(f) = sum_k s_k * arg(centroid - c_k)/4
// over (center, strength) poles. A strength-+1 pole is a clean +1
// cross-field singularity (a full vector vortex would be charge 0 mod 4);
// a (+1,-1) pole pair is a cancellable dipole, (1+3)%4 == 0.
static std::vector<Vector3> windingField(
    const SurfaceMesh& mesh, const std::vector<std::pair<Vector3, double>>& poles)
{
    std::vector<Vector3> field;
    for (size_t f = 0; f < mesh.faceCount(); ++f) {
        const Vector3 m = centroidOf(mesh, f);
        double theta = 0.0;
        for (const auto& [c, s] : poles) {
            const double dx = m.x() - c.x(), dy = m.y() - c.y();
            if (dx == 0.0 && dy == 0.0)
                continue;
            theta += s * std::atan2(dy, dx) / 4.0;
        }
        field.emplace_back(std::cos(theta), std::sin(theta), 0.0);
    }
    return field;
}

static std::vector<Vector3> dipoleWinding(
    const SurfaceMesh& mesh, const Vector3& c0, const Vector3& c1)
{
    return windingField(mesh, { { c0, 1.0 }, { c1, -1.0 } });
}

// Single-face defect: uniform field with one face rotated 45 degrees
// about its normal. (90 degrees would be cross-symmetric, hence
// invisible; 45 degrees is a genuine local twist.) An adjacent
// opposite-charge pair on any mesh with an interior face.
static std::vector<Vector3> defectField(
    const SurfaceMesh& mesh, size_t defectFace, const Vector3& base)
{
    std::vector<Vector3> field(mesh.faceCount(), base);
    if (defectFace < field.size()) {
        const Vector3 n = mesh.faceNormal(defectFace);
        Vector3 perp = Vector3::crossProduct(n, base);
        if (perp.length() < 1e-12)
            perp = Vector3::crossProduct(n, Vector3(0, 1, 0));
        const double c = std::cos(M_PI / 4.0), s = std::sin(M_PI / 4.0);
        field[defectFace] = c * base + s * perp.normalized() * base.length();
    }
    return field;
}

static std::vector<Vector3> randomField(size_t faces)
{
    std::vector<Vector3> field;
    for (size_t f = 0; f < faces; ++f) {
        const std::uint64_t pick = below(100);
        if (pick < 5) {
            field.emplace_back(0, 0, 0);
        } else if (pick < 10) {
            field.emplace_back(1e-13 * (unitRand() - 0.5), 1e-13 * (unitRand() - 0.5),
                1e-13 * (unitRand() - 0.5));
        } else {
            field.emplace_back(2.0 * unitRand() - 1.0, 2.0 * unitRand() - 1.0,
                2.0 * unitRand() - 1.0);
        }
    }
    return field;
}

// Adversarial near-degenerate input: rotate K random faces' vectors by a
// tiny angle around the face normal, jittering field angles across the
// lround quarter-turn boundaries to catch unfused-FP divergences.
static void tinyRotate(
    const SurfaceMesh& mesh, std::vector<Vector3>& field, double delta, size_t count)
{
    for (size_t i = 0; i < count && !field.empty(); ++i) {
        const size_t f = static_cast<size_t>(below(field.size()));
        const Vector3 n = mesh.faceNormal(f);
        const Vector3 perp = Vector3::crossProduct(n, field[f]);
        field[f] = std::cos(delta) * field[f] + std::sin(delta) * perp;
    }
}

static size_t g_exact = 0, g_struct = 0, g_cancelled = 0;
static size_t g_histBefore[16] = { 0 };
static size_t g_histAfter[16] = { 0 };

int main()
{
    std::printf("SINGSIMP1\n");
    size_t id = 0;
    // The input field is snapshotted before simplify() mutates it; the F
    // section prints that snapshot, FO prints the simplified output.
    auto run = [&](const Mesh& mesh, std::vector<Vector3> field, size_t maxPair,
                   double sharp, char cls) {
        // Re-dump inputs faithfully: stash a copy for the F section.
        const std::vector<Vector3> input = field;
        const SurfaceMesh probe(mesh.positions, mesh.triangles);
        std::vector<Vector3> work = input;
        if (work.size() != probe.faceCount())
            work.assign(probe.faceCount(), Vector3(1, 0, 0));
        SingularitySimplifier simp(probe, &work);
        simp.setMaximumPairDistance(maxPair);
        simp.setSharpEdgeDegrees(sharp);
        const std::vector<int> chargesBefore = simp.vertexCharges();
        simp.simplify();
        const size_t before = simp.singularityCountBefore();
        const size_t after = simp.singularityCountAfter();
        const size_t cancelled = simp.cancelledPairCount();
        const std::vector<int> chargesAfter = simp.vertexCharges();
        char c = cls;
        if (before > 7 || after > before)
            c = 'S';
        if (c == 'E')
            ++g_exact;
        else
            ++g_struct;
        g_cancelled += cancelled;
        if (before < 16)
            ++g_histBefore[before];
        if (after < 16)
            ++g_histAfter[after];
        std::printf("CASE %zu %zu %zu %zu %.17g %c\n", id++, mesh.positions.size(),
            mesh.triangles.size(), maxPair, sharp, c);
        for (const auto& p : mesh.positions) {
            std::printf("v");
            printDouble(p.x());
            printDouble(p.y());
            printDouble(p.z());
            std::printf("\n");
        }
        for (const auto& t : mesh.triangles) {
            std::printf("t %zu", t.size());
            for (size_t v : t)
                std::printf(" %zu", v);
            std::printf("\n");
        }
        std::printf("F %zu", input.size());
        for (const auto& v : input) {
            printDouble(v.x());
            printDouble(v.y());
            printDouble(v.z());
        }
        std::printf("\n");
        std::printf("R %zu %zu %zu\n", before, after, cancelled);
        std::printf("CHI %zu", chargesBefore.size());
        for (int cc : chargesBefore)
            std::printf(" %d", cc);
        std::printf("\n");
        std::printf("CH %zu", chargesAfter.size());
        for (int cc : chargesAfter)
            std::printf(" %d", cc);
        std::printf("\n");
        std::printf("FO %zu", work.size());
        for (const auto& v : work) {
            printDouble(v.x());
            printDouble(v.y());
            printDouble(v.z());
        }
        std::printf("\n");
    };

    // E1: uniform fields (expect identity: no singularities on clean grids).
    {
        const Mesh g4 = makeGrid(4, 4, 0);
        const SurfaceMesh m4(g4.positions, g4.triangles);
        run(g4, uniformField(m4.faceCount(), Vector3(1, 0, 0)), 6, 90.0, 'E');
        const Mesh g6r = makeGrid(6, 6, 1);
        const SurfaceMesh m6r(g6r.positions, g6r.triangles);
        run(g6r, uniformField(m6r.faceCount(), Vector3(0, 1, 0)), 6, 90.0, 'E');
        run(g6r, uniformField(m6r.faceCount(), Vector3(1, 1, 1)), 2, 30.0, 'E');
        const Mesh ridge = makeGrid(6, 6, 2);
        const SurfaceMesh mr(ridge.positions, ridge.triangles);
        run(ridge, uniformField(mr.faceCount(), Vector3(1, 0, 0)), 6, 90.0, 'E');
        run(ridge, uniformField(mr.faceCount(), Vector3(1, 0, 0)), 6, 150.0, 'E');
        const Mesh octa = makeOcta();
        const SurfaceMesh mo(octa.positions, octa.triangles);
        run(octa, uniformField(mo.faceCount(), Vector3(1, 0, 0)), 6, 90.0, 'E');
    }
    // E2: single fractional vortices (one +1 pole: no pair, identity),
    // double-strength winding dipoles, single-face 45-degree defects on
    // closed meshes, and legacy vector swirls (fragmented charges: the
    // no-candidate path).
    {
        const size_t sizes[] = { 4, 6 };
        for (size_t s : sizes) {
            const Mesh grid = makeGrid(s, s, 0);
            const SurfaceMesh mesh(grid.positions, grid.triangles);
            auto at = [s](size_t x, size_t y) { return y * (s + 1) + x; };
            const Vector3 center = mesh.position(at(s / 2, s / 2));
            const Vector3 off = mesh.position(at(1, 1));
            run(grid, windingField(mesh, { { center, 1.0 } }), 6, 90.0, 'E');
            run(grid, windingField(mesh, { { center, -1.0 } }), 6, 90.0, 'E');
            run(grid, windingField(mesh, { { off, 1.0 } }), 2, 90.0, 'E');
            run(grid, windingField(mesh, { { off, -1.0 } }), 6, 30.0, 'E');
            run(grid, vortexField(mesh, center, 1.0), 6, 90.0, 'E');
            // Double-strength winding dipole (empirically a {1, 3} pair:
            // the close poles partially unwind each other; a different
            // field construction from E3 over the same charge pattern).
            run(grid,
                windingField(mesh, { { mesh.position(at(1, 2)), 2.0 },
                                       { mesh.position(at(3, 2)), -2.0 } }),
                6, 90.0, 'E');
            run(grid,
                windingField(mesh, { { mesh.position(at(1, 2)), 2.0 },
                                       { mesh.position(at(3, 2)), -2.0 } }),
                0, 90.0, 'E');
        }
        const Mesh octa = makeOcta();
        const SurfaceMesh mo(octa.positions, octa.triangles);
        run(octa, defectField(mo, 0, Vector3(1, 0, 0)), 6, 90.0, 'E');
        run(octa, defectField(mo, 3, Vector3(0, 1, 0)), 0, 90.0, 'E');
        run(octa, vortexField(mo, mo.position(4), 1.0), 6, 90.0, 'E');
        const Mesh tetra = makeTetra();
        const SurfaceMesh mt(tetra.positions, tetra.triangles);
        run(tetra, defectField(mt, 1, Vector3(1, 0, 0)), 6, 90.0, 'E');
    }
    // E3: fractional-dipole sweep (THE cancel exercise: separation x
    // maxPair matrix; near pairs cancel, far pairs and mp=0 do not).
    {
        const size_t sizes[] = { 4, 6, 8 };
        const size_t gaps[][2] = { { 1, 0 }, { 2, 0 }, { 3, 0 }, { 2, 2 } };
        const size_t pairs[] = { 0, 2, 6 };
        for (size_t s : sizes) {
            const Mesh grid = makeGrid(s, s, 0);
            const SurfaceMesh mesh(grid.positions, grid.triangles);
            auto at = [s](size_t x, size_t y) { return y * (s + 1) + x; };
            for (const auto& gap : gaps) {
                const size_t x0 = 1, y0 = s / 2;
                const size_t x1 = x0 + gap[0], y1 = y0 + gap[1];
                if (x1 >= s || y1 >= s)
                    continue;
                const Vector3 c0 = mesh.position(at(x0, y0));
                const Vector3 c1 = mesh.position(at(x1, y1));
                for (size_t mp : pairs)
                    run(grid, dipoleWinding(mesh, c0, c1), mp, 90.0, 'E');
            }
        }
        // Relief-grid dipoles + extreme distances on 6x6.
        const Mesh grid = makeGrid(6, 6, 0);
        const SurfaceMesh mesh(grid.positions, grid.triangles);
        auto at = [](size_t x, size_t y) { return y * 7 + x; };
        for (size_t mp : { size_t(1), size_t(8) }) {
            run(grid,
                dipoleWinding(mesh, mesh.position(at(1, 3)), mesh.position(at(4, 3))), mp, 90.0,
                'E');
            run(grid,
                dipoleWinding(mesh, mesh.position(at(1, 1)), mesh.position(at(5, 5))), mp, 90.0,
                'E');
        }
        const Mesh rel = makeGrid(6, 6, 1);
        const SurfaceMesh mrel(rel.positions, rel.triangles);
        run(rel, dipoleWinding(mrel, mrel.position(at(1, 3)), mrel.position(at(3, 3))), 6, 90.0,
            'E');
        run(rel, dipoleWinding(mrel, mrel.position(at(1, 3)), mrel.position(at(3, 3))), 0, 90.0,
            'E');
        // Quadrupole: two dipoles (tests multi-pair rounds).
        run(grid,
            windingField(mesh,
                { { mesh.position(at(1, 2)), 1.0 }, { mesh.position(at(2, 2)), -1.0 },
                    { mesh.position(at(4, 4)), 1.0 }, { mesh.position(at(5, 4)), -1.0 } }),
            6, 90.0, 'E');
        run(grid,
            windingField(mesh,
                { { mesh.position(at(1, 2)), 1.0 }, { mesh.position(at(2, 2)), -1.0 },
                    { mesh.position(at(4, 4)), 1.0 }, { mesh.position(at(5, 4)), -1.0 } }),
            0, 90.0, 'E');
    }
    // E4: cube + ridge sharp-edge matrices (sharp freezing vs sharpDegrees).
    {
        const Mesh cube = makeCube();
        const SurfaceMesh mc(cube.positions, cube.triangles);
        run(cube, uniformField(mc.faceCount(), Vector3(1, 0, 0)), 6, 30.0, 'E');
        run(cube, uniformField(mc.faceCount(), Vector3(1, 0, 0)), 6, 90.0, 'E');
        run(cube, uniformField(mc.faceCount(), Vector3(1, 0, 0)), 6, 150.0, 'E');
        run(cube, defectField(mc, 0, Vector3(1, 0, 0)), 6, 30.0, 'E');
        run(cube, defectField(mc, 0, Vector3(1, 0, 0)), 6, 90.0, 'E');
        run(cube, defectField(mc, 0, Vector3(1, 0, 0)), 6, 150.0, 'E');
        const Mesh ridge = makeGrid(6, 6, 2);
        const SurfaceMesh mr(ridge.positions, ridge.triangles);
        auto at = [](size_t x, size_t y) { return y * 7 + x; };
        run(ridge, dipoleWinding(mr, mr.position(at(1, 3)), mr.position(at(5, 3))), 6, 30.0,
            'E');
        run(ridge, dipoleWinding(mr, mr.position(at(1, 3)), mr.position(at(5, 3))), 6, 90.0,
            'E');
    }
    // E5: adversarial tiny rotations (lround-boundary jitter, FMA catchers).
    {
        const double deltas[] = { 1e-13, 1e-11, 1e-9, 1e-7 };
        const Mesh grid = makeGrid(6, 6, 0);
        const SurfaceMesh mesh(grid.positions, grid.triangles);
        auto at = [](size_t x, size_t y) { return y * 7 + x; };
        for (double d : deltas) {
            for (int seed = 0; seed < 3; ++seed) {
                std::vector<Vector3> f =
                    dipoleWinding(mesh, mesh.position(at(2, 3)), mesh.position(at(4, 3)));
                tinyRotate(mesh, f, d, 2);
                run(grid, f, 6, 90.0, 'E');
            }
        }
        const Mesh octa = makeOcta();
        const SurfaceMesh mo(octa.positions, octa.triangles);
        for (double d : deltas) {
            for (int seed = 0; seed < 3; ++seed) {
                std::vector<Vector3> f = randomField(mo.faceCount());
                tinyRotate(mo, f, d, 2);
                run(octa, f, 6, 90.0, 'E');
            }
        }
    }
    // E6: degenerate geometry + zero fields.
    {
        Mesh snap = makeGrid(4, 4, 0);
        for (size_t i = 0; i < snap.positions.size(); i += 7)
            snap.positions[i] = snap.positions[0];
        const SurfaceMesh ms(snap.positions, snap.triangles);
        run(snap, uniformField(ms.faceCount(), Vector3(1, 0, 0)), 6, 90.0, 'E');
        run(snap, randomField(ms.faceCount()), 6, 90.0, 'E');
        const Mesh grid = makeGrid(4, 4, 0);
        const SurfaceMesh mg(grid.positions, grid.triangles);
        run(grid, uniformField(mg.faceCount(), Vector3(0, 0, 0)), 6, 90.0, 'E');
        run(grid, uniformField(mg.faceCount(), Vector3(0, 0, 0)), 0, 30.0, 'E');
        const Mesh octa = makeOcta();
        const SurfaceMesh mo(octa.positions, octa.triangles);
        run(octa, uniformField(mo.faceCount(), Vector3(0, 0, 0)), 6, 90.0, 'E');
        // Collapsed faces: repeated-vertex triangles (zero area).
        Mesh collapsed = makeGrid(3, 3, 0);
        collapsed.triangles[0] = { 0, 0, 1 };
        collapsed.triangles[3] = { 5, 5, 5 };
        const SurfaceMesh mc(collapsed.positions, collapsed.triangles);
        run(collapsed, uniformField(mc.faceCount(), Vector3(1, 0, 0)), 6, 90.0, 'E');
        run(collapsed, randomField(mc.faceCount()), 2, 90.0, 'E');
        // Empty meshes: no faces (empty field), no vertices at all.
        const Mesh nofaces = makeGrid(0, 0, 0);
        run(nofaces, {}, 6, 90.0, 'E');
        run(Mesh{}, {}, 6, 90.0, 'E');
    }
    // E7: seeded random small meshes (few interior verts => few
    // singularities; demotion backstops any surprise).
    {
        const double sharps[] = { 30.0, 90.0, 150.0 };
        for (int i = 0; i < 30; ++i) {
            const Mesh grid = makeGrid(2, 2, 0);
            const SurfaceMesh mesh(grid.positions, grid.triangles);
            run(grid, randomField(mesh.faceCount()), below(9), sharps[below(3)], 'E');
        }
        for (int i = 0; i < 40; ++i) {
            const Mesh grid = makeGrid(3, 3, below(2) ? 0 : 1);
            const SurfaceMesh mesh(grid.positions, grid.triangles);
            run(grid, randomField(mesh.faceCount()), below(9), sharps[below(3)], 'E');
        }
        for (int i = 0; i < 15; ++i) {
            const Mesh octa = makeOcta();
            const SurfaceMesh mesh(octa.positions, octa.triangles);
            run(octa, randomField(mesh.faceCount()), below(9), sharps[below(3)], 'E');
        }
        for (int i = 0; i < 15; ++i) {
            const Mesh tetra = makeTetra();
            const SurfaceMesh mesh(tetra.positions, tetra.triangles);
            run(tetra, randomField(mesh.faceCount()), below(9), sharps[below(3)], 'E');
        }
    }
    // S1: stress meshes (dense random-field singularity clusters;
    // structural asserts only).
    {
        const double sharps[] = { 30.0, 90.0, 150.0 };
        const size_t sizes[][2] = { { 8, 8 }, { 10, 10 }, { 12, 12 }, { 8, 12 } };
        for (int i = 0; i < 20; ++i) {
            const auto& sz = sizes[below(4)];
            const Mesh grid = makeGrid(sz[0], sz[1], below(2) ? 0 : 1);
            const SurfaceMesh mesh(grid.positions, grid.triangles);
            run(grid, randomField(mesh.faceCount()), 2 + below(7), sharps[below(3)], 'S');
        }
    }
    // Timing: 40x40 relief grid + two fractional dipoles (4
    // singularities, 2 deterministic cancels), rebuilt from formula on
    // both sides.
    {
        const Mesh grid = makeGrid(40, 40, 1);
        const SurfaceMesh mesh(grid.positions, grid.triangles);
        auto at = [](size_t x, size_t y) { return y * 41 + x; };
        for (int sample = 0; sample < 3; ++sample) {
            std::vector<Vector3> field = windingField(mesh,
                { { mesh.position(at(10, 20)), 1.0 }, { mesh.position(at(14, 20)), -1.0 },
                    { mesh.position(at(26, 22)), 1.0 }, { mesh.position(at(30, 22)), -1.0 } });
            auto t0 = std::chrono::steady_clock::now();
            SingularitySimplifier simp(mesh, &field);
            simp.simplify();
            auto t1 = std::chrono::steady_clock::now();
            double xsum = 0.0;
            for (const auto& v : field)
                xsum += v.x();
            double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
            std::printf("T singsimp faces=%zu before=%zu after=%zu cancelled=%zu xsum=%.17g ms=%.3f\n",
                mesh.faceCount(), simp.singularityCountBefore(), simp.singularityCountAfter(),
                simp.cancelledPairCount(), xsum, ms);
        }
    }
    std::fprintf(stderr,
        "singsimp dump: %zu cases (%zu exact, %zu structural), %zu cancelled pairs\n",
        id, g_exact, g_struct, g_cancelled);
    std::fprintf(stderr, "before histogram:");
    for (int i = 0; i < 16; ++i)
        std::fprintf(stderr, " %d:%zu", i, g_histBefore[i]);
    std::fprintf(stderr, "\nafter histogram:");
    for (int i = 0; i < 16; ++i)
        std::fprintf(stderr, " %d:%zu", i, g_histAfter[i]);
    std::fprintf(stderr, "\n");
    return 0;
}
