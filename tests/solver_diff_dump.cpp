// Differential oracle dump for the Rust solvers calibration experiment.
// Generates seeded random CLS + MILS systems, solves them with the C++
// implementation, and prints inputs + outputs in a line format that
// rust/solvers/tests/differential.rs replays. Doubles print with %.17g
// (exact round-trip). Also times cover-sized systems for the runtime ratio.
//
// Build-only helper: not registered with ctest. Run it and redirect stdout
// to tests/fixtures/solver_diff.txt, then commit the fixture.
import retopo.core.constrained_least_squares;
import retopo.core.mixed_integer_least_squares;

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <utility>
#include <vector>

// splitmix64: identical 5-line implementations exist in the Rust replay test,
// so the timing case below is generated bit-identically on both sides.
static std::uint64_t g_state = 0x123456789ABCDEFull;

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

// Small exact-representable coefficient in {-2..2} \ {0}, quarter steps.
static double nzCoeff()
{
    for (;;) {
        int v = static_cast<int>(below(17)) - 8;
        if (v != 0)
            return v / 4.0;
    }
}

static double rhsVal()
{
    return (static_cast<int>(below(41)) - 20) / 4.0;
}

static double weightVal()
{
    static const double kW[] = { 0.25, 0.5, 1.0, 2.0, 4.0 };
    return kW[below(5)];
}

static int smallInt()
{
    for (;;) {
        int v = static_cast<int>(below(5)) - 2;
        if (v != 0)
            return v;
    }
}

using AutoRemesher::ConstrainedLeastSquares;
using AutoRemesher::MixedIntegerLeastSquares;
using Row = std::vector<std::pair<size_t, double>>;

static void printDouble(double v)
{
    std::printf("%.17g", v);
}

static Row randomRow(size_t nvars, bool smallInts)
{
    // k must not exceed nvars: the distinct-variable loop below would
    // otherwise reject duplicates forever on 1-2 variable systems.
    size_t k = 1 + below(nvars < 3 ? nvars : 3);
    std::vector<size_t> vars;
    while (vars.size() < k) {
        size_t v = below(nvars);
        bool dup = false;
        for (size_t u : vars)
            if (u == v)
                dup = true;
        if (!dup)
            vars.push_back(v);
    }
    Row row;
    for (size_t v : vars)
        row.push_back({ v, smallInts ? static_cast<double>(smallInt()) : nzCoeff() });
    return row;
}

static Row randomRowWide(size_t nvars, size_t want)
{
    size_t k = want < nvars ? want : nvars;
    std::vector<size_t> vars;
    while (vars.size() < k) {
        size_t v = below(nvars);
        bool dup = false;
        for (size_t u : vars)
            if (u == v)
                dup = true;
        if (!dup)
            vars.push_back(v);
    }
    Row row;
    for (size_t v : vars)
        row.push_back({ v, nzCoeff() });
    return row;
}

static void printRow(const Row& row)
{
    std::printf("%zu", row.size());
    for (const auto& [i, c] : row) {
        std::printf(" %zu ", i);
        printDouble(c);
    }
}

static void dumpClsCase(bool consistent)
{
    size_t nvars = 1 + below(8);
    // Overdetermined like the downstream cover systems (far more corner
    // equations than variables): underdetermined systems sit in the
    // ridge regime (cond ~ 1e10) where the two Cholesky backends differ
    // by FP noise, which is not a logic signal.
    size_t nE = nvars + below(8);
    size_t nC = below(4);
    // Consistent cases pin their constraints to a known target: any maximal
    // independent subset then yields the same values, so the SPQR+COLAMD
    // (C++) vs ColPiv (Rust) selection difference is unobservable. CLSX cases
    // keep random right-hand sides to quantify that known gap (robustness
    // only: the port must solve everything C++ solves, values may differ).
    std::vector<double> target(nvars);
    for (size_t v = 0; v < nvars; ++v)
        target[v] = rhsVal();
    ConstrainedLeastSquares s(nvars);
    struct Energy {
        Row row;
        double rhs;
        double w;
    };
    std::vector<Energy> energies;
    for (size_t e = 0; e < nE; ++e)
        energies.push_back({ randomRow(nvars, false), rhsVal(), weightVal() });
    // Coverage: every variable appears in at least one energy, as in the
    // downstream cover systems. Without this the normal matrix is
    // rank-deficient by construction and the two Cholesky backends differ
    // by ridge-regime FP noise (cond ~ 1e10), which is not a logic signal.
    {
        std::vector<bool> covered(nvars, false);
        for (const Energy& energy : energies)
            for (const auto& [v, a] : energy.row)
                covered[v] = true;
        for (size_t v = 0; v < nvars; ++v)
            if (!covered[v])
                energies.push_back({ Row { { v, nzCoeff() } }, rhsVal(), weightVal() });
    }
    nE = energies.size();
    std::printf(consistent ? "CLS %zu %zu %zu\n" : "CLSX %zu %zu %zu\n", nvars, nE, nC);
    for (const Energy& energy : energies) {
        std::printf("e ");
        printRow(energy.row);
        std::printf(" ");
        printDouble(energy.rhs);
        std::printf(" ");
        printDouble(energy.w);
        std::printf("\n");
        s.addEnergy(energy.row, energy.rhs, energy.w);
    }
    for (size_t c = 0; c < nC; ++c) {
        // A quarter of the rows are wide (3-4 vars) so the Lagrange path
        // (3+ roots after reduction) gets covered, not just substitution.
        Row row = randomRow(nvars, false);
        if (nvars >= 3 && below(4) == 0) {
            size_t want = 3 + below(nvars < 4 ? nvars - 2 : 2);
            row = randomRowWide(nvars, want);
        }
        double rhs;
        if (consistent) {
            rhs = 0.0;
            for (const auto& [v, a] : row)
                rhs += a * target[v];
        } else {
            rhs = rhsVal();
        }
        std::printf("c ");
        printRow(row);
        std::printf(" ");
        printDouble(rhs);
        std::printf("\n");
        s.addConstraint(row, rhs);
    }
    std::vector<double> x;
    bool ok = s.solve(&x);
    std::printf("s %d", ok ? 1 : 0);
    for (double v : x) {
        std::printf(" ");
        printDouble(v);
    }
    std::printf("\n");
    // Cache path: update one energy RHS and re-solve (matrix not rebuilt).
    if (below(10) < 3) {
        size_t idx = below(nE);
        double rhs = rhsVal();
        std::printf("u %zu ", idx);
        printDouble(rhs);
        std::printf("\n");
        s.setEnergyRightHandSide(idx, rhs);
        std::vector<double> x2;
        bool ok2 = s.solve(&x2);
        std::printf("s2 %d", ok2 ? 1 : 0);
        for (double v : x2) {
            std::printf(" ");
            printDouble(v);
        }
        std::printf("\n");
    }
    // Clear path: drop constraints and re-solve energy-only.
    if (nC > 0 && below(10) < 2) {
        std::printf("clr\n");
        s.clearConstraints();
        std::vector<double> x3;
        bool ok3 = s.solve(&x3);
        std::printf("s3 %d", ok3 ? 1 : 0);
        for (double v : x3) {
            std::printf(" ");
            printDouble(v);
        }
        std::printf("\n");
    }
}

static void dumpMilsCase()
{
    size_t nvars = 1 + below(6);
    size_t nEq = below(5);
    size_t nEn = nvars + below(4);
    bool useBuilder = below(10) < 5 && nEq > 0;
    // Generate everything first: coverage can grow the energy list, and the
    // header must carry the final count.
    std::vector<std::pair<size_t, int>> periods;
    for (size_t v = 0; v < nvars; ++v)
        if (below(10) < 5)
            periods.push_back({ v, 1 + static_cast<int>(below(3)) });
    std::vector<Row> equations;
    for (size_t q = 0; q < nEq; ++q)
        equations.push_back(randomRow(nvars, true));
    struct Energy {
        Row row;
        double rhs;
        double w;
    };
    std::vector<Energy> energies;
    for (size_t e = 0; e < nEn; ++e)
        energies.push_back({ randomRow(nvars, false), rhsVal(), weightVal() });
    {
        std::vector<bool> covered(nvars, false);
        for (const Energy& energy : energies)
            for (const auto& [v, a] : energy.row)
                covered[v] = true;
        for (size_t v = 0; v < nvars; ++v)
            if (!covered[v])
                energies.push_back({ Row { { v, nzCoeff() } }, rhsVal(), weightVal() });
    }
    nEn = energies.size();
    std::printf("MILS %zu %zu %zu %d\n", nvars, nEq, nEn, useBuilder ? 1 : 0);
    MixedIntegerLeastSquares s(nvars);
    for (const auto& [v, period] : periods) {
        std::printf("p %zu %d\n", v, period);
        s.setVariablePeriod(v, period);
    }
    bool builderOk = true;
    for (size_t q = 0; q < nEq; ++q) {
        const Row& row = equations[q];
        std::printf("q ");
        printRow(row);
        std::printf("\n");
        if (useBuilder && q + 1 == nEq) {
            s.beginConstraint();
            for (const auto& [i, c] : row)
                s.addConstraintCoefficient(i, c);
            builderOk = s.endConstraint();
        } else {
            s.addConstraint(row);
        }
    }
    std::printf("b %d\n", builderOk ? 1 : 0);
    for (const Energy& energy : energies) {
        std::printf("e ");
        printRow(energy.row);
        std::printf(" ");
        printDouble(energy.rhs);
        std::printf(" ");
        printDouble(energy.w);
        std::printf("\n");
        s.addEnergy(energy.row, energy.rhs, energy.w);
    }
    s.finalizeConstraints();
    bool ok = true;
    size_t iters = 0;
    for (; iters < 100; ++iters) {
        if (!s.solveIteration()) {
            ok = false;
            break;
        }
        if (s.converged())
            break;
    }
    std::printf("s %d %d %zu %zu %zu %zu", ok ? 1 : 0, s.converged() ? 1 : 0, iters,
        s.kernelSize(), s.integerKernelVariableCount(), s.originalIntegerVariableCount());
    for (size_t v = 0; v < nvars; ++v) {
        std::printf(" ");
        printDouble(s.value(v));
    }
    std::printf("\n");
}

// Cover-sized timing case. The Rust side regenerates the identical system
// from the same splitmix64 stream (reset to kTimingSeed) and times its solve;
// only the elapsed ms are compared, never across implementations here.
static constexpr std::uint64_t kTimingSeed = 0xC10C4C0FFEEull;

static double timeClsCover()
{
    g_state = kTimingSeed;
    const size_t nvars = 2000;
    const size_t nE = 4000;
    const size_t nC = 200;
    ConstrainedLeastSquares s(nvars);
    for (size_t e = 0; e < nE; ++e)
        s.addEnergy(randomRow(nvars, false), rhsVal(), weightVal());
    for (size_t c = 0; c < nC; ++c)
        s.addConstraint(randomRow(nvars, false), rhsVal());
    std::vector<double> x;
    auto t0 = std::chrono::steady_clock::now();
    bool ok = s.solve(&x);
    auto t1 = std::chrono::steady_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    std::printf("T cls ok=%d ms=%.3f\n", ok ? 1 : 0, ms);
    return ms;
}

static double timeMilsCover()
{
    g_state = kTimingSeed ^ 0xF00DBABEull;
    const size_t nvars = 500;
    const size_t nEq = 300;
    const size_t nEn = 600;
    MixedIntegerLeastSquares s(nvars);
    for (size_t v = 0; v < nvars; ++v)
        if (below(10) < 5)
            s.setVariablePeriod(v, 1);
    for (size_t q = 0; q < nEq; ++q) {
        // Downstream shape: difference constraints (a,1),(b,-1), plus
        // occasional general small-int rows.
        Row row;
        if (below(10) < 7) {
            size_t a = below(nvars);
            size_t b = below(nvars);
            if (a == b)
                b = (b + 1) % nvars;
            row = { { a, 1.0 }, { b, -1.0 } };
        } else {
            row = randomRow(nvars, true);
        }
        s.addConstraint(row);
    }
    for (size_t e = 0; e < nEn; ++e)
        s.addEnergy(randomRow(nvars, false), rhsVal(), weightVal());
    auto t0 = std::chrono::steady_clock::now();
    s.finalizeConstraints();
    auto t1 = std::chrono::steady_clock::now();
    bool ok = true;
    size_t iters = 0;
    for (; iters < 100; ++iters) {
        if (!s.solveIteration()) {
            ok = false;
            break;
        }
        if (s.converged())
            break;
    }
    auto t2 = std::chrono::steady_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t2 - t0).count();
    double fin = std::chrono::duration<double, std::milli>(t1 - t0).count();
    double loop = std::chrono::duration<double, std::milli>(t2 - t1).count();
    std::printf("T mils ok=%d ms=%.3f conv=%d iters=%zu fin=%.3f loop=%.3f\n", ok ? 1 : 0, ms,
        s.converged() ? 1 : 0, iters, fin, loop);
    return ms;
}

int main()
{
    std::printf("DIFF1\n");
    for (int i = 0; i < 200; ++i)
        dumpClsCase(true);
    for (int i = 0; i < 40; ++i)
        dumpClsCase(false);
    for (int i = 0; i < 200; ++i)
        dumpMilsCase();
    timeClsCover();
    timeMilsCover();
    return 0;
}
