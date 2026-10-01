// Differential oracle dump for the framefield Rust port.
// Generates seeded random + adversarial near-degenerate frame-field cases,
// solves them with the C++ implementation, and prints inputs + outputs in a
// token format that rust/core/tests/frame_field_diff.rs replays.
// Doubles print with %.17g (exact round-trip). Also times a cover-sized case
// for the runtime ratio (informational T lines; the Rust timing harness
// regenerates the identical mesh analytically and prints its own ms).
//
// Build-only helper: not registered with ctest. Run it and redirect stdout
// to tests/fixtures/framefield_diff.txt, then commit the fixture.
import retopo.core.constrained_least_squares;
import retopo.core.frame_field;
import retopo.core.guides;
import retopo.core.surface_mesh;
import retopo.core.vector3;

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <string>
#include <utility>
#include <vector>

using AutoRemesher::ConstrainedLeastSquares;
using AutoRemesher::FrameField;
using AutoRemesher::Guides;
using AutoRemesher::SurfaceMesh;
using AutoRemesher::Vector3;

// FFX threshold: a face whose smoothed periodic magnitude drops below this
// has its normalized direction defined by backend noise (Cholesky roundoff
// ~1e-15 amplified by the projection to the unit circle), so its final
// field is backend-defined and no port can match it. Normal faces solve to
// magnitudes O(0.1..1); observed cancellations read ~1e-16: six orders of
// margin on each side.
static constexpr double kFfxMagnitude = 1e-6;

// splitmix64 (fixed seed: the fixture is committed, not regenerated per run).
static std::uint64_t g_state = 0xF1E1D04CE9u;

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

static void printDouble(double v)
{
    std::printf("%.17g", v);
}

static void printVec3(const Vector3& v)
{
    printDouble(v.x());
    std::printf(" ");
    printDouble(v.y());
    std::printf(" ");
    printDouble(v.z());
}

struct Mesh {
    std::vector<Vector3> vertices;
    std::vector<std::vector<size_t>> triangles;
};

static double randSym(double amplitude)
{
    return (static_cast<int>(below(2001)) - 1000) / 1000.0 * amplitude;
}

static Mesh makeGrid(size_t w, size_t h, double jitter)
{
    Mesh mesh;
    for (size_t j = 0; j <= h; ++j) {
        for (size_t i = 0; i <= w; ++i) {
            mesh.vertices.push_back(Vector3(
                static_cast<double>(i) + (jitter > 0.0 ? randSym(jitter) : 0.0),
                static_cast<double>(j) + (jitter > 0.0 ? randSym(jitter) : 0.0),
                jitter > 0.0 ? randSym(jitter) : 0.0));
        }
    }
    for (size_t j = 0; j < h; ++j) {
        for (size_t i = 0; i < w; ++i) {
            const size_t a = j * (w + 1) + i, b = a + 1, c = a + w + 1, d = c + 1;
            mesh.triangles.push_back({ a, b, d });
            mesh.triangles.push_back({ a, d, c });
        }
    }
    return mesh;
}

static Mesh makeBox(double jitter)
{
    Mesh mesh;
    const double c[8][3] = { { -1, -1, -1 }, { 1, -1, -1 }, { 1, 1, -1 }, { -1, 1, -1 },
        { -1, -1, 1 }, { 1, -1, 1 }, { 1, 1, 1 }, { -1, -1, 1 } };
    for (const auto& p : c)
        mesh.vertices.push_back(Vector3(p[0] + (jitter > 0.0 ? randSym(jitter) : 0.0),
            p[1] + (jitter > 0.0 ? randSym(jitter) : 0.0),
            p[2] + (jitter > 0.0 ? randSym(jitter) : 0.0)));
    // Outward winding (verified face by face).
    const size_t f[12][3] = { { 0, 2, 1 }, { 0, 3, 2 }, { 4, 5, 6 }, { 4, 6, 7 },
        { 0, 1, 5 }, { 0, 5, 4 }, { 2, 3, 7 }, { 2, 7, 6 }, { 0, 4, 7 }, { 0, 7, 3 },
        { 1, 2, 6 }, { 1, 6, 5 } };
    for (const auto& t : f)
        mesh.triangles.push_back({ t[0], t[1], t[2] });
    return mesh;
}

static Mesh makeTetra(double jitter)
{
    Mesh mesh;
    const double c[4][3] = { { 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
    for (const auto& p : c)
        mesh.vertices.push_back(Vector3(p[0] + (jitter > 0.0 ? randSym(jitter) : 0.0),
            p[1] + (jitter > 0.0 ? randSym(jitter) : 0.0),
            p[2] + (jitter > 0.0 ? randSym(jitter) : 0.0)));
    const size_t f[4][3] = { { 0, 2, 1 }, { 0, 1, 3 }, { 0, 3, 2 }, { 1, 2, 3 } };
    for (const auto& t : f)
        mesh.triangles.push_back({ t[0], t[1], t[2] });
    return mesh;
}

static Mesh makeOcta(double jitter)
{
    Mesh mesh;
    const double c[6][3]
        = { { 1, 0, 0 }, { -1, 0, 0 }, { 0, 1, 0 }, { 0, -1, 0 }, { 0, 0, 1 }, { 0, 0, -1 } };
    for (const auto& p : c)
        mesh.vertices.push_back(Vector3(p[0] + (jitter > 0.0 ? randSym(jitter) : 0.0),
            p[1] + (jitter > 0.0 ? randSym(jitter) : 0.0),
            p[2] + (jitter > 0.0 ? randSym(jitter) : 0.0)));
    const size_t f[8][3] = { { 4, 0, 2 }, { 4, 2, 1 }, { 4, 1, 3 }, { 4, 3, 0 },
        { 5, 2, 0 }, { 5, 1, 2 }, { 5, 3, 1 }, { 5, 0, 3 } };
    for (const auto& t : f)
        mesh.triangles.push_back({ t[0], t[1], t[2] });
    return mesh;
}

static Mesh makeSphere(size_t nlat, size_t nlon, double jitter)
{
    Mesh mesh;
    mesh.vertices.push_back(Vector3(0, 0, 1));
    for (size_t j = 1; j < nlat; ++j) {
        const double theta = M_PI * static_cast<double>(j) / static_cast<double>(nlat);
        for (size_t i = 0; i < nlon; ++i) {
            const double phi = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(nlon);
            mesh.vertices.push_back(Vector3(std::sin(theta) * std::cos(phi)
                        + (jitter > 0.0 ? randSym(jitter) : 0.0),
                std::sin(theta) * std::sin(phi) + (jitter > 0.0 ? randSym(jitter) : 0.0),
                std::cos(theta) + (jitter > 0.0 ? randSym(jitter) : 0.0)));
        }
    }
    mesh.vertices.push_back(Vector3(0, 0, -1));
    const size_t south = mesh.vertices.size() - 1;
    for (size_t i = 0; i < nlon; ++i) {
        const size_t a = 1 + i, b = 1 + (i + 1) % nlon;
        mesh.triangles.push_back({ 0, b, a });
        const size_t ring = 1 + (nlat - 2) * nlon;
        mesh.triangles.push_back({ south, ring + i, ring + (i + 1) % nlon });
    }
    for (size_t j = 0; j + 2 < nlat; ++j) {
        for (size_t i = 0; i < nlon; ++i) {
            const size_t a = 1 + j * nlon + i, b = 1 + j * nlon + (i + 1) % nlon;
            const size_t c = a + nlon, d = b + nlon;
            mesh.triangles.push_back({ a, b, d });
            mesh.triangles.push_back({ a, d, c });
        }
    }
    return mesh;
}

// Analytic saddle z = (x^2 - y^2)/2: indefinite curvature tensors
// (opposite-sign eigenvalue-tie probe for the Jacobi-vs-Eigen sort).
static Mesh makeSaddle(size_t w, size_t h)
{
    Mesh mesh;
    for (size_t j = 0; j <= h; ++j) {
        for (size_t i = 0; i <= w; ++i) {
            const double x = static_cast<double>(i) - static_cast<double>(w) / 2.0;
            const double y = static_cast<double>(j) - static_cast<double>(h) / 2.0;
            mesh.vertices.push_back(Vector3(x, y, (x * x - y * y) / 2.0));
        }
    }
    for (size_t j = 0; j < h; ++j) {
        for (size_t i = 0; i < w; ++i) {
            const size_t a = j * (w + 1) + i, b = a + 1, c = a + w + 1, d = c + 1;
            mesh.triangles.push_back({ a, b, d });
            mesh.triangles.push_back({ a, d, c });
        }
    }
    return mesh;
}

static Mesh makeStrip(size_t n, double jitter)
{
    Mesh mesh;
    for (size_t i = 0; i <= n; ++i) {
        mesh.vertices.push_back(Vector3(static_cast<double>(i)
                    + (jitter > 0.0 ? randSym(jitter) : 0.0),
            (jitter > 0.0 ? randSym(jitter) : 0.0), (jitter > 0.0 ? randSym(jitter) : 0.0)));
        mesh.vertices.push_back(Vector3(static_cast<double>(i)
                    + (jitter > 0.0 ? randSym(jitter) : 0.0),
            1.0 + (jitter > 0.0 ? randSym(jitter) : 0.0),
            (jitter > 0.0 ? randSym(jitter) : 0.0)));
    }
    for (size_t i = 0; i < n; ++i)
        mesh.triangles.push_back({ 2 * i, 2 * i + 1, 2 * i + 2 });
    return mesh;
}

static Mesh makeFan(size_t n, double jitter)
{
    Mesh mesh;
    mesh.vertices.push_back(Vector3(0, 0, (jitter > 0.0 ? randSym(jitter) : 0.0)));
    for (size_t i = 0; i < n; ++i) {
        const double a = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(n);
        mesh.vertices.push_back(Vector3(std::cos(a) + (jitter > 0.0 ? randSym(jitter) : 0.0),
            std::sin(a) + (jitter > 0.0 ? randSym(jitter) : 0.0),
            (jitter > 0.0 ? randSym(jitter) : 0.0)));
    }
    for (size_t i = 0; i < n; ++i)
        mesh.triangles.push_back({ 0, 1 + i, 1 + (i + 1) % n });
    return mesh;
}

static Mesh makeSoup(size_t nv, size_t nt, bool allowDegenerate)
{
    Mesh mesh;
    for (size_t i = 0; i < nv; ++i)
        mesh.vertices.push_back(Vector3(randSym(2.0), randSym(2.0), randSym(2.0)));
    for (size_t t = 0; t < nt; ++t) {
        size_t a = below(nv), b = below(nv), c = below(nv);
        if (!allowDegenerate) {
            if (b == a)
                b = (b + 1) % nv;
            if (c == a || c == b)
                c = (c + 2) % nv;
            if (c == a || c == b)
                c = (c + 1) % nv;
        }
        mesh.triangles.push_back({ a, b, c });
    }
    return mesh;
}

// Two triangles sharing the edge (0,0,0)-(1,0,0), the second folded so the
// measured dihedral is `angleDegrees` (up to libm rounding).
static Mesh makeFold(double angleDegrees)
{
    const double alpha = M_PI - angleDegrees * M_PI / 180.0;
    Mesh mesh;
    mesh.vertices.push_back(Vector3(0, 0, 0));
    mesh.vertices.push_back(Vector3(1, 0, 0));
    mesh.vertices.push_back(Vector3(0, 1, 0));
    mesh.vertices.push_back(Vector3(0, std::cos(alpha), std::sin(alpha)));
    mesh.triangles.push_back({ 0, 1, 2 });
    mesh.triangles.push_back({ 1, 0, 3 });
    return mesh;
}

// Triangular bipyramid: highly symmetric closed 6-face mesh (cancellation
// probe like the tetra).
static Mesh makeBipyramid()
{
    Mesh mesh;
    mesh.vertices.push_back(Vector3(1, 0, 0));
    mesh.vertices.push_back(Vector3(-0.5, 0.8660254037844386, 0));
    mesh.vertices.push_back(Vector3(-0.5, -0.8660254037844386, 0));
    mesh.vertices.push_back(Vector3(0, 0, 1));
    mesh.vertices.push_back(Vector3(0, 0, -1));
    mesh.triangles.push_back({ 0, 1, 3 });
    mesh.triangles.push_back({ 1, 2, 3 });
    mesh.triangles.push_back({ 2, 0, 3 });
    mesh.triangles.push_back({ 1, 0, 4 });
    mesh.triangles.push_back({ 2, 1, 4 });
    mesh.triangles.push_back({ 0, 2, 4 });
    return mesh;
}

static Mesh makeFlatQuad()
{
    Mesh mesh;
    mesh.vertices.push_back(Vector3(0, 0, 0));
    mesh.vertices.push_back(Vector3(1, 0, 0));
    mesh.vertices.push_back(Vector3(1, 1, 0));
    mesh.vertices.push_back(Vector3(0, 1, 0));
    mesh.triangles.push_back({ 0, 1, 2 });
    mesh.triangles.push_back({ 0, 2, 3 });
    return mesh;
}

// Polylines snapped to actual mesh edges (distance 0: guaranteed hits)
// unless `far` (offset by +10: guaranteed misses).
static std::vector<std::vector<Vector3>> randomEdgeLines(const Mesh& mesh, size_t k, bool far)
{
    std::vector<std::vector<Vector3>> lines;
    const double off = far ? 10.0 : 0.0;
    for (size_t i = 0; i < k; ++i) {
        if (mesh.triangles.empty())
            break;
        const auto& t = mesh.triangles[below(mesh.triangles.size())];
        if (t.size() != 3 || t[0] >= mesh.vertices.size() || t[1] >= mesh.vertices.size())
            continue;
        std::vector<Vector3> line;
        line.push_back(mesh.vertices[t[0]] + Vector3(off, off, off));
        line.push_back(mesh.vertices[t[1]] + Vector3(off, off, off));
        lines.push_back(line);
    }
    return lines;
}

// A 3-point polyline through the mesh bounding box: usually hits something.
static std::vector<std::vector<Vector3>> randomBoxLines(const Mesh& mesh, size_t k)
{
    std::vector<std::vector<Vector3>> lines;
    if (mesh.vertices.empty())
        return lines;
    Vector3 lo = mesh.vertices[0], hi = mesh.vertices[0];
    for (const Vector3& v : mesh.vertices) {
        lo = Vector3(std::min(lo.x(), v.x()), std::min(lo.y(), v.y()), std::min(lo.z(), v.z()));
        hi = Vector3(std::max(hi.x(), v.x()), std::max(hi.y(), v.y()), std::max(hi.z(), v.z()));
    }
    for (size_t i = 0; i < k; ++i) {
        const double t = (below(101) + 25) / 150.0;
        std::vector<Vector3> line;
        line.push_back(Vector3(lo.x(), lo.y() + t * (hi.y() - lo.y()), lo.z() + t * (hi.z() - lo.z())));
        line.push_back(Vector3((lo.x() + hi.x()) / 2.0, lo.y() + (1.0 - t) * (hi.y() - lo.y()),
            hi.z() - t * (hi.z() - lo.z())));
        line.push_back(Vector3(hi.x(), hi.y() - t * (hi.y() - lo.y()), lo.z() + (1.0 - t) * (hi.z() - lo.z())));
        lines.push_back(line);
    }
    return lines;
}

static std::vector<std::vector<Vector3>> junkLines()
{
    return {
        {},
        { Vector3(1.0, 0.0, 0.0) },
        { Vector3(0.0, 0.0, 1.0), Vector3(0.0, 0.0, 1.0) },
        { Vector3(100.0, 0.0, 0.0), Vector3(101.0, 0.0, 0.0) },
    };
}

// Replicates FrameField::create's seeding + smoothing loop to observe the
// minimum smoothed periodic magnitude over faces x iterations (the FFX
// detector). Returns +inf when the solve fails (then RES=0 and no values
// are asserted anyway). This is a verification aid, not a second oracle:
// the TAG it implies is printed as MAG and the replay asserts every tag is
// consistent with it, so the segregation is auditable, not cherry-picked.
static double minSmoothedMagnitude(const SurfaceMesh& mesh, double sharpEdgeDegrees,
    const std::vector<std::vector<Vector3>>& guides,
    const std::vector<std::vector<Vector3>>& sharps)
{
    if (mesh.faceCount() == 0)
        return std::numeric_limits<double>::infinity();
    constexpr double kSymmetry = 4.0;
    struct Basis {
        Vector3 tangent, perpendicularTangent, normal;
    };
    const auto normalizedOrFallback = [](const Vector3& v, const Vector3& fb) {
        return v.length() <= 1e-12 ? fb.normalized() : v.normalized();
    };
    const auto tangentAngle = [](const Vector3& v, const Basis& b) {
        return std::atan2(Vector3::dotProduct(v, b.perpendicularTangent),
            Vector3::dotProduct(v, b.tangent));
    };
    const size_t faces = mesh.faceCount();
    std::vector<Basis> bases(faces);
    for (size_t f = 0; f < faces; ++f) {
        const Vector3 normal = normalizedOrFallback(mesh.faceNormal(f), Vector3(0, 0, 1));
        Vector3 tangent = mesh.edgeVector(3 * f);
        tangent = tangent - Vector3::dotProduct(tangent, normal) * normal;
        if (tangent.length() <= 1e-12) {
            tangent = std::fabs(normal.x()) < .9 ? Vector3(1, 0, 0) : Vector3(0, 1, 0);
            tangent = tangent - Vector3::dotProduct(tangent, normal) * normal;
        }
        tangent = normalizedOrFallback(tangent, Vector3(1, 0, 0));
        bases[f] = { tangent, Vector3::crossProduct(normal, tangent), normal };
    }
    std::vector<double> periodic(2 * faces, 0.0), certainty(faces, 0.0);
    std::vector<char> locked(faces, 0);
    const double sharpRadians = sharpEdgeDegrees * M_PI / 180.0;
    for (size_t f = 0; f < faces; ++f)
        for (size_t c = 3 * f; c < 3 * f + 3; ++c) {
            if (mesh.oppositeCorner(c) != SurfaceMesh::npos
                && std::fabs(mesh.normalAngle(c)) <= sharpRadians)
                continue;
            const double fa = kSymmetry * tangentAngle(mesh.edgeVector(c), bases[f]);
            periodic[2 * f] = std::cos(fa);
            periodic[2 * f + 1] = std::sin(fa);
            locked[f] = 1;
        }
    const auto lockLines = [&](const std::vector<std::vector<Vector3>>& lines, double radius) {
        for (size_t f = 0; f < faces; ++f) {
            if (locked[f])
                continue;
            const auto& t = mesh.triangle(f);
            const Vector3 centroid = (mesh.position(t[0]) + mesh.position(t[1]) + mesh.position(t[2])) / 3.0;
            const Vector3 tangent = Guides::tangentNear(lines, centroid, bases[f].normal, radius);
            if (tangent.length() <= 1e-12)
                continue;
            const double fa = kSymmetry * tangentAngle(tangent, bases[f]);
            periodic[2 * f] = std::cos(fa);
            periodic[2 * f + 1] = std::sin(fa);
            locked[f] = 1;
        }
    };
    if (!sharps.empty())
        lockLines(sharps, 2.0 * mesh.averageEdgeLength());
    if (!guides.empty())
        lockLines(guides, Guides::influenceRadius(mesh));
    std::vector<std::array<double, 6>> vertexTensor(mesh.vertexCount());
    for (auto& t : vertexTensor)
        t.fill(0.0);
    for (size_t c = 0; c < mesh.cornerCount(); ++c) {
        const size_t o = mesh.oppositeCorner(c);
        if (o == SurfaceMesh::npos || o < c)
            continue;
        const Vector3 edge = mesh.edgeVector(c);
        const double dihedral = mesh.normalAngle(c);
        for (size_t v : { mesh.cornerVertex(c), mesh.cornerVertex(mesh.nextCorner(c)) }) {
            const Vector3 ue = normalizedOrFallback(edge, Vector3(1, 0, 0));
            const double w = edge.length() * dihedral;
            vertexTensor[v][0] += w * ue.x() * ue.x();
            vertexTensor[v][1] += w * ue.x() * ue.y();
            vertexTensor[v][2] += w * ue.y() * ue.y();
            vertexTensor[v][3] += w * ue.x() * ue.z();
            vertexTensor[v][4] += w * ue.y() * ue.z();
            vertexTensor[v][5] += w * ue.z() * ue.z();
        }
    }
    for (size_t f = 0; f < faces; ++f) {
        if (locked[f])
            continue;
        std::array<double, 6> total {};
        for (size_t c = 3 * f; c < 3 * f + 3; ++c)
            for (size_t k = 0; k < 6; ++k)
                total[k] += vertexTensor[mesh.cornerVertex(c)][k];
        Eigen::Matrix3d m;
        m << total[0], total[1], total[3], total[1], total[2], total[4], total[3],
            total[4], total[5];
        const double trace = m(0, 0) + m(1, 1) + m(2, 2);
        const double reg = trace == 0.0 ? 1e-6 : 1e-6 * trace;
        m(0, 0) += reg;
        m(1, 1) += reg;
        m(2, 2) += reg;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(m);
        if (eig.info() != Eigen::Success)
            continue;
        std::array<int, 3> ord = { 0, 1, 2 };
        std::sort(ord.begin(), ord.end(), [&](int a, int b) {
            return std::fabs(eig.eigenvalues()[a]) > std::fabs(eig.eigenvalues()[b]);
        });
        const Eigen::Vector3d dir = eig.eigenvectors().col(ord[0]);
        const double fa = kSymmetry * tangentAngle(Vector3(dir.x(), dir.y(), dir.z()), bases[f]);
        periodic[2 * f] = std::cos(fa);
        periodic[2 * f + 1] = std::sin(fa);
        certainty[f] = std::fabs(eig.eigenvalues()[ord[0]] - eig.eigenvalues()[ord[1]]);
    }
    double maximumCertainty = 0.0;
    for (double v : certainty)
        maximumCertainty = std::max(maximumCertainty, v);
    if (maximumCertainty > 0.0)
        for (double& v : certainty)
            v /= maximumCertainty;
    const auto normalizePeriodic = [&]() {
        for (size_t f = 0; f < faces; ++f) {
            const double L = std::hypot(periodic[2 * f], periodic[2 * f + 1]);
            if (L > 1e-30) {
                periodic[2 * f] /= L;
                periodic[2 * f + 1] /= L;
            }
        }
    };
    normalizePeriodic();
    ConstrainedLeastSquares system(2 * faces);
    for (size_t f = 0; f < faces; ++f)
        if (locked[f]) {
            system.addConstraint({ { 2 * f, 1.0 } }, periodic[2 * f]);
            system.addConstraint({ { 2 * f + 1, 1.0 } }, periodic[2 * f + 1]);
        }
    for (size_t c = 0; c < mesh.cornerCount(); ++c) {
        const size_t o = mesh.oppositeCorner(c);
        if (o == SurfaceMesh::npos)
            continue;
        const size_t f = mesh.cornerFace(c), g = mesh.cornerFace(o);
        if (f < g)
            continue;
        const double tr = -kSymmetry
            * (tangentAngle(mesh.edgeVector(c), bases[g]) - tangentAngle(mesh.edgeVector(c), bases[f]));
        const double co = std::cos(tr), si = std::sin(tr);
        system.addEnergy({ { 2 * f, co }, { 2 * f + 1, si }, { 2 * g, -1.0 } }, 0.0);
        system.addEnergy({ { 2 * f, -si }, { 2 * f + 1, co }, { 2 * g + 1, -1.0 } }, 0.0);
    }
    std::vector<std::pair<size_t, size_t>> certaintyRows;
    for (size_t f = 0; f < faces; ++f)
        if (certainty[f] > 0.0) {
            const double w = certainty[f] * certainty[f];
            certaintyRows.push_back({ system.addEnergy({ { 2 * f, 1.0 } }, periodic[2 * f], w),
                system.addEnergy({ { 2 * f + 1, 1.0 } }, periodic[2 * f + 1], w) });
        }
    double minMagnitude = std::numeric_limits<double>::infinity();
    std::vector<double> solved;
    for (int it = 0; it < 5; ++it) {
        size_t ri = 0;
        for (size_t f = 0; f < faces; ++f)
            if (certainty[f] > 0.0) {
                system.setEnergyRightHandSide(certaintyRows[ri].first, periodic[2 * f]);
                system.setEnergyRightHandSide(certaintyRows[ri].second, periodic[2 * f + 1]);
                ++ri;
            }
        if (!system.solve(&solved))
            return std::numeric_limits<double>::infinity();
        for (size_t f = 0; f < faces; ++f)
            minMagnitude = std::min(minMagnitude, std::hypot(solved[2 * f], solved[2 * f + 1]));
        periodic.swap(solved);
        normalizePeriodic();
    }
    return minMagnitude;
}

static void dumpCase(int id, const char* kind, const Mesh& mesh, double hard,
    const std::vector<std::vector<Vector3>>& guides,
    const std::vector<std::vector<Vector3>>& sharps)
{
    const SurfaceMesh surface(mesh.vertices, mesh.triangles);
    std::vector<Vector3> field;
    const bool ok = FrameField::create(surface, hard, &field, guides, sharps);
    // FFX (like the solvers' CLSX / quadparameterizer's QPX):
    // robustness-only. The port must match the ok flag, the face count,
    // and unit outputs; values are reported, not asserted. Two listed
    // stragglers (quadparameterizer-37/109/174 precedent), each with a
    // demonstrated backend-noise mechanism, plus the magnitude auto-rule
    // as forward protection:
    // - id 56 (tetra, NV 4 NT 4 HARD 180, no lines): cancellation. Face 3
    //   solves to (-2.3e-16, -1.6e-17), bit-identical with and without its
    //   2e-31-weight certainty rows, so normalizePeriodic projects pure
    //   backend Cholesky noise onto the unit circle; Eigen's and faer's
    //   noise point different ways. Unreachable by any port.
    // - id 290 (tiny, NV 4 NT 4 HARD 90, no lines): atan2 branch-cut
    //   y-sign. Face 3's y-channel cancels to -5.8e-16 at iteration 0
    //   (backend-defined sign), the fixed-point iteration locks it to
    //   (-1, -5.1e-15), and the cut turns faer's +branch into a 90-degree
    //   field rotation (the same cross under the 4-symmetry, but a
    //   different vector). Proven from the fields alone: the shared facet
    //   basis maps the C++ field to y < 0 and the Rust field to y > 0.
    //   A C++-side detector provably cannot isolate 290: healthy cases 27
    //   and 282 show the same signature (free face, x < 0, |y| ~ 1e-16)
    //   with deterministically-agreeing signs (verified across TBB thread
    //   counts), so any observation rule either misses 290 or demotes
    //   proven-healthy cases. Hence the explicit listing: the replay
    //   asserts both the listing and each case's identity.
    // - auto-rule 0 < MAG < 1e-6 (also catches 56): cancellation-class
    //   forward protection. Exact 0.0 is carved out: fully-homogeneous
    //   systems (octa180: no locks, no certainty rows) solve to exactly
    //   0.0 on both backends and agree (atan2(0,0) = 0); observed margins
    //   are 2.3e-16 vs 2.3e-2.
    const double minMagnitude = ok
        ? minSmoothedMagnitude(surface, hard, guides, sharps)
        : std::numeric_limits<double>::infinity();
    const bool robust = ok
        && (id == 56 || id == 290
            || (minMagnitude > 0.0 && minMagnitude < kFfxMagnitude));
    std::printf(robust ? "FFX %d KIND %s NV %zu NT %zu HARD " : "CASE %d KIND %s NV %zu NT %zu HARD ", id, kind, mesh.vertices.size(),
        mesh.triangles.size());
    printDouble(hard);
    std::printf(" GUIDES %zu SHARPS %zu\n", guides.size(), sharps.size());
    for (const Vector3& v : mesh.vertices) {
        std::printf("V ");
        printVec3(v);
        std::printf("\n");
    }
    for (const auto& t : mesh.triangles) {
        std::printf("T %zu", t.size());
        for (size_t i : t)
            std::printf(" %zu", i);
        std::printf("\n");
    }
    for (const auto& line : guides) {
        std::printf("GUIDE %zu", line.size());
        for (const Vector3& p : line) {
            std::printf(" ");
            printVec3(p);
        }
        std::printf("\n");
    }
    for (const auto& line : sharps) {
        std::printf("SHARP %zu", line.size());
        for (const Vector3& p : line) {
            std::printf(" ");
            printVec3(p);
        }
        std::printf("\n");
    }

    std::printf("RES %d MAG ", ok ? 1 : 0);
    printDouble(minMagnitude);
    std::printf("\n");
    if (!ok)
        return;
    std::printf("NF %zu\nFIELD", field.size());
    for (const Vector3& f : field) {
        std::printf(" ");
        printVec3(f);
    }
    std::printf("\n");
}

static double pickHard()
{
    static const double kH[] = { 0.0, 10.0, 30.0, 45.0, 60.0, 90.0, 135.0, 180.0 };
    if (below(10) == 0) {
        static const double kOdd[] = { 15.0, 75.0, 100.0 };
        return kOdd[below(3)];
    }
    return kH[below(8)];
}

static std::pair<const char*, Mesh> randomMesh()
{
    const double jitterPick = below(2) == 0 ? 0.0 : (below(2) == 0 ? 0.05 : 0.3);
    switch (below(10)) {
    case 0:
    case 1:
        return { "grid", makeGrid(1 + below(5), 1 + below(5), jitterPick) };
    case 2:
        return { "box", makeBox(jitterPick) };
    case 3:
        return below(2) == 0 ? std::pair<const char*, Mesh> { "tetra", makeTetra(jitterPick) }
                             : std::pair<const char*, Mesh> { "octa", makeOcta(jitterPick) };
    case 4:
        return { "sphere", makeSphere(3 + below(4), 4 + below(6), jitterPick) };
    case 5:
        return { "strip", makeStrip(2 + below(8), jitterPick) };
    case 6:
        return { "fan", makeFan(3 + below(8), jitterPick) };
    case 7:
        return { "soup", makeSoup(4 + below(12), 2 + below(10), false) };
    case 8:
        return { "quad", makeFlatQuad() };
    default:
        return { "soupdeg", makeSoup(4 + below(10), 2 + below(8), true) };
    }
}

static std::vector<std::vector<Vector3>> randomLines(const Mesh& mesh)
{
    switch (below(10)) {
    case 0:
    case 1:
    case 2:
        return {};
    case 3:
    case 4:
        return randomEdgeLines(mesh, 1 + below(2), false);
    case 5:
        return randomEdgeLines(mesh, 1, true);
    case 6:
        return randomBoxLines(mesh, 1);
    case 7:
        return junkLines();
    case 8:
        return randomEdgeLines(mesh, 2 + below(3), false);
    default:
        return randomBoxLines(mesh, 2);
    }
}

static void dumpSeededCase(int id)
{
    auto [meshKind, mesh] = randomMesh();
    const double hard = pickHard();
    dumpCase(id, meshKind, mesh, hard, randomLines(mesh), randomLines(mesh));
}

// Adversarial battery: near-degenerate inputs aimed at every FP-decision
// cliff in the pipeline (sharp dihedral gate, guide/sharp radius gates,
// the 1e-12 degenerate-length gates, the 60-degree into-surface gate,
// curvature degeneracy, fallbacks, early-false paths).
static void dumpAdversarial(int& id)
{
    const double kDeltas[] = { 0.0, 1e-12, -1e-12, 1e-9, -1e-9, 1e-6, -1e-6 };
    const double kHards[] = { 30.0, 45.0, 90.0 };
    for (double hard : kHards)
        for (double d : kDeltas)
            dumpCase(id++, "fold", makeFold(hard + d), hard, {}, {});
    for (double d : kDeltas)
        dumpCase(id++, "boxhard", makeBox(0.0), 90.0 + d, {}, {});
    // Near-flat folds: the shared-edge tensor contribution shrinks toward
    // the regularizer, probing near-degenerate eigenspaces.
    dumpCase(id++, "flatfold6", makeFold(180.0 - 1e-6), 180.0, {}, {});
    dumpCase(id++, "flatfold9", makeFold(180.0 - 1e-9), 180.0, {}, {});
    // Curvature degeneracy battery (no locks anywhere: hard=180 on
    // dihedral-free or flat inputs; the field comes purely from the
    // curvature seeding + smoothing).
    {
        Mesh quad = makeFlatQuad();
        dumpCase(id++, "flatquad", quad, 180.0, {}, {});
        dumpCase(id++, "flatquad0", quad, 0.0, {}, {});
        dumpCase(id++, "box180", makeBox(0.0), 180.0, {}, {});
        dumpCase(id++, "sphere0", makeSphere(4, 8, 0.0), 180.0, {}, {});
        dumpCase(id++, "sphere90", makeSphere(4, 8, 0.0), 90.0, {}, {});
        dumpCase(id++, "spheretiny", makeSphere(4, 8, 1e-9), 180.0, {}, {});
        dumpCase(id++, "fan12", makeFan(12, 0.0), 180.0, {}, {});
        dumpCase(id++, "fan12j", makeFan(12, 0.05), 180.0, {}, {});
        dumpCase(id++, "octa180", makeOcta(0.0), 180.0, {}, {});
        dumpCase(id++, "bipy180", makeBipyramid(), 180.0, {}, {});
        dumpCase(id++, "saddle33", makeSaddle(3, 3), 180.0, {}, {});
        dumpCase(id++, "saddle55", makeSaddle(5, 5), 180.0, {}, {});
        dumpCase(id++, "saddle90", makeSaddle(3, 3), 90.0, {}, {});
        Mesh single;
        single.vertices.push_back(Vector3(0, 0, 0));
        single.vertices.push_back(Vector3(1, 0, 0));
        single.vertices.push_back(Vector3(0, 1, 0));
        single.triangles.push_back({ 0, 1, 2 });
        dumpCase(id++, "single180", single, 180.0, {}, {});
        dumpCase(id++, "single90", single, 90.0, {}, {});
    }
    // Guide/sharp radius gates: a tangent segment at height r*(1+d) over
    // the flat quad's first-triangle centroid (2/3, 1/3, 0). Guide radius
    // is 6 edge lengths, sharp radius is 2.
    {
        Mesh quad = makeFlatQuad();
        const SurfaceMesh surface(quad.vertices, quad.triangles);
        const double guideR = 6.0 * surface.averageEdgeLength();
        const double sharpR = 2.0 * surface.averageEdgeLength();
        for (double d : kDeltas) {
            const double h = guideR * (1.0 + d);
            std::vector<std::vector<Vector3>> guides = { { Vector3(0.4, 1.0 / 3.0, h),
                Vector3(0.9, 1.0 / 3.0, h) } };
            dumpCase(id++, "guiderad", quad, 180.0, guides, {});
        }
        for (double d : kDeltas) {
            const double h = sharpR * (1.0 + d);
            std::vector<std::vector<Vector3>> sharps = { { Vector3(0.4, 1.0 / 3.0, h),
                Vector3(0.9, 1.0 / 3.0, h) } };
            dumpCase(id++, "sharprad", quad, 180.0, {}, sharps);
        }
        // 60-degree into-surface gate: direction (cos a, 0, sin a) over the
        // flat quad; |projected tangent| = cos a, rejected at <= 0.5.
        const double kTilt[] = { 0.0, 1e-12, -1e-12, 1e-9, -1e-9, 1e-6, -1e-6 };
        for (double d : kTilt) {
            const double a = (60.0 + d) * M_PI / 180.0;
            std::vector<std::vector<Vector3>> guides = { { Vector3(0.4, 1.0 / 3.0, 0.0),
                Vector3(0.4 + std::cos(a), 1.0 / 3.0, std::sin(a)) } };
            dumpCase(id++, "tilt60", quad, 180.0, guides, {});
        }
        // 1e-12 degenerate-length gate: point-sized segments at (0.5, 0, 0).
        const double kLens[] = { 1e-13, 1e-12, 1e-12 * (1.0 - 1e-9), 1e-12 * (1.0 + 1e-9),
            2e-12, 1e-9 };
        for (double len : kLens) {
            std::vector<std::vector<Vector3>> guides = { { Vector3(0.5, 0.0, 0.0),
                Vector3(0.5 + len, 0.0, 0.0) } };
            dumpCase(id++, "shortseg", quad, 180.0, guides, {});
        }
        // Degenerate / rejected line shapes.
        dumpCase(id++, "guideempty", quad, 180.0, { {} }, {});
        dumpCase(id++, "guidepoint", quad, 180.0, { { Vector3(0.5, 0.0, 0.0) } }, {});
        dumpCase(id++, "guidezero", quad, 180.0,
            { { Vector3(0.5, 0.0, 0.0), Vector3(0.5, 0.0, 0.0) } }, {});
        dumpCase(id++, "guidefar", quad, 180.0,
            { { Vector3(100.0, 0.0, 0.0), Vector3(101.0, 0.0, 0.0) } }, {});
        dumpCase(id++, "guidenormal", quad, 180.0,
            { { Vector3(0.5, 1.0 / 3.0, 0.0), Vector3(0.5, 1.0 / 3.0, 1.0) } }, {});
        dumpCase(id++, "sharpempty", quad, 180.0, {}, { {} });
        dumpCase(id++, "sharpnormal", quad, 180.0, {},
            { { Vector3(0.5, 1.0 / 3.0, 0.0), Vector3(0.5, 1.0 / 3.0, 1.0) } });
        dumpCase(id++, "junkboth", quad, 90.0, junkLines(), junkLines());
    }
    // Sharp-wins-tie + guide-lock mirrors of the C++ golden groups.
    {
        Mesh box = makeBox(0.0);
        const std::vector<std::vector<Vector3>> verticals = {
            { Vector3(-1.0, -1.0, -1.0), Vector3(-1.0, 1.0, -1.0) },
            { Vector3(1.0, -1.0, -1.0), Vector3(1.0, 1.0, -1.0) },
            { Vector3(-1.0, -1.0, 1.0), Vector3(-1.0, 1.0, 1.0) },
            { Vector3(1.0, -1.0, 1.0), Vector3(1.0, 1.0, 1.0) },
        };
        const std::vector<std::vector<Vector3>> diagonal
            = { { Vector3(2.0, -1.0, -1.0), Vector3(2.0, 1.0, 1.0) } };
        dumpCase(id++, "tieboth", box, 90.0, diagonal, verticals);
        dumpCase(id++, "tieguide", box, 90.0, diagonal, {});
        dumpCase(id++, "tiesharp", box, 90.0, {}, verticals);
        dumpCase(id++, "tieplain", box, 90.0, {}, {});
    }
    // Degenerate geometry.
    {
        Mesh zeroArea = makeFlatQuad();
        zeroArea.vertices.push_back(Vector3(0, 0, 0));
        zeroArea.vertices.push_back(Vector3(0, 0, 0));
        zeroArea.vertices.push_back(Vector3(0, 0, 0));
        const size_t n = zeroArea.vertices.size();
        zeroArea.triangles.push_back({ n - 3, n - 2, n - 1 });
        dumpCase(id++, "zeroarea", zeroArea, 90.0, {}, {});
        Mesh dupVert;
        dupVert.vertices.push_back(Vector3(0, 0, 0));
        dupVert.vertices.push_back(Vector3(0, 0, 0));
        dupVert.vertices.push_back(Vector3(1, 0, 0));
        dupVert.vertices.push_back(Vector3(0, 1, 0));
        dupVert.triangles.push_back({ 0, 1, 2 });
        dupVert.triangles.push_back({ 0, 2, 3 });
        dumpCase(id++, "dupvert", dupVert, 90.0, {}, {});
        Mesh single;
        single.vertices.push_back(Vector3(0, 0, 0));
        single.vertices.push_back(Vector3(1, 0, 0));
        single.vertices.push_back(Vector3(0, 1, 0));
        single.triangles.push_back({ 0, 1, 2 });
        dumpCase(id++, "single", single, 90.0, {}, {});
        Mesh disjoint = single;
        disjoint.vertices.push_back(Vector3(5, 5, 5));
        disjoint.vertices.push_back(Vector3(6, 5, 5));
        disjoint.vertices.push_back(Vector3(5, 6, 5));
        disjoint.triangles.push_back({ 3, 4, 5 });
        dumpCase(id++, "disjoint", disjoint, 90.0, {}, {});
        Mesh nonmanifold;
        nonmanifold.vertices.push_back(Vector3(0, 0, 0));
        nonmanifold.vertices.push_back(Vector3(1, 0, 0));
        nonmanifold.vertices.push_back(Vector3(0, 1, 0));
        nonmanifold.vertices.push_back(Vector3(0, 0, 1));
        nonmanifold.vertices.push_back(Vector3(0, -1, 0));
        nonmanifold.triangles.push_back({ 0, 1, 2 });
        nonmanifold.triangles.push_back({ 1, 0, 3 });
        nonmanifold.triangles.push_back({ 0, 1, 4 });
        dumpCase(id++, "nonmanifold", nonmanifold, 90.0, {}, {});
        Mesh coincident;
        coincident.vertices.push_back(Vector3(0, 0, 0));
        coincident.vertices.push_back(Vector3(1, 0, 0));
        coincident.vertices.push_back(Vector3(0, 1, 0));
        coincident.triangles.push_back({ 0, 1, 2 });
        coincident.triangles.push_back({ 0, 2, 1 });
        dumpCase(id++, "coincident", coincident, 90.0, {}, {});
        Mesh tiny = makeTetra(0.0);
        for (Vector3& v : tiny.vertices)
            v = Vector3(v.x() * 1e-9, v.y() * 1e-9, v.z() * 1e-9);
        dumpCase(id++, "tiny", tiny, 90.0, {}, {});
        Mesh huge = makeTetra(0.0);
        for (Vector3& v : huge.vertices)
            v = Vector3(v.x() * 1e9, v.y() * 1e9, v.z() * 1e9);
        dumpCase(id++, "huge", huge, 90.0, {}, {});
    }
    // Early-false paths.
    {
        Mesh empty;
        dumpCase(id++, "empty", empty, 90.0, {}, {});
        Mesh noTris;
        noTris.vertices.push_back(Vector3(0, 0, 0));
        dumpCase(id++, "notris", noTris, 90.0, {}, {});
        Mesh badTri = makeFlatQuad();
        badTri.triangles.push_back({ 0, 1 });
        dumpCase(id++, "badtri2", badTri, 90.0, {}, {});
        Mesh badTri4 = makeFlatQuad();
        badTri4.triangles.push_back({ 0, 1, 2, 3 });
        dumpCase(id++, "badtri4", badTri4, 90.0, {}, {});
    }
}

// Timing mesh, analytic so the Rust harness regenerates it bit-identically:
// 64x64 grid, z = 0.1 * sin(i) * cos(j) (same mesh the quadparameterizer
// lane times, so the two lanes' ratios are comparable).
static Mesh makeTimingMesh()
{
    const size_t w = 64, h = 64;
    Mesh mesh;
    for (size_t j = 0; j <= h; ++j)
        for (size_t i = 0; i <= w; ++i)
            mesh.vertices.push_back(Vector3(static_cast<double>(i), static_cast<double>(j),
                0.1 * std::sin(static_cast<double>(i)) * std::cos(static_cast<double>(j))));
    for (size_t j = 0; j < h; ++j)
        for (size_t i = 0; i < w; ++i) {
            const size_t a = j * (w + 1) + i, b = a + 1, c = a + w + 1, d = c + 1;
            mesh.triangles.push_back({ a, b, d });
            mesh.triangles.push_back({ a, d, c });
        }
    return mesh;
}

static void timeField()
{
    const Mesh mesh = makeTimingMesh();
    const SurfaceMesh surface(mesh.vertices, mesh.triangles);
    // Warmup (unprinted): page faults, TBB pool spin-up, allocator caches.
    {
        std::vector<Vector3> field;
        FrameField::create(surface, 90.0, &field);
    }
    for (int sample = 0; sample < 3; ++sample) {
        std::vector<Vector3> field;
        const auto t0 = std::chrono::steady_clock::now();
        const bool ok = FrameField::create(surface, 90.0, &field);
        const auto t1 = std::chrono::steady_clock::now();
        const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        double checksum = 0.0;
        for (const Vector3& f : field)
            checksum += f.x() + f.y() + f.z();
        std::printf("T ff ok=%d ms=%.3f faces=%zu checksum=", ok ? 1 : 0, ms, field.size());
        printDouble(checksum);
        std::printf("\n");
    }
}

int main()
{
    std::printf("FFDIFF1\n");
    int id = 0;
    for (int i = 0; i < 200; ++i)
        dumpSeededCase(id++);
    dumpAdversarial(id);
    std::printf("CASES %d\n", id);
    timeField();
    return 0;
}
