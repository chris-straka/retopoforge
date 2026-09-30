// Unit tests for retopo.core.mixed_integer_least_squares
// (core/mixedintegerleastsquares.cppm). Solver goldens on small
// deterministic systems: homogeneous constraints define the kernel,
// weighted energies are least-squares rows over it, periodic variables
// round to integer multiples across solveIteration passes.
// Plain assert-style main, no third-party framework.
// NOTE: CHECK instead of <cassert> assert(), which is a no-op under NDEBUG.
import retopo.core.mixed_integer_least_squares;

#include <cmath>
#include <cstdio>
#include <utility>
#include <vector>

static int g_failures = 0;

#define CHECK(cond)                                                                                  \
    do {                                                                                             \
        if (!(cond)) {                                                                               \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                               \
            ++g_failures;                                                                            \
        }                                                                                            \
    } while (0)

static bool near(double a, double b, double tol = 1e-6)
{
    return std::fabs(a - b) <= tol;
}

int main()
{
    using AutoRemesher::MixedIntegerLeastSquares;
    using Row = std::vector<std::pair<size_t, double>>;

    // Unconstrained: energies pass through the identity kernel untouched.
    {
        MixedIntegerLeastSquares s(2);
        s.addEnergy(Row { { 0, 1.0 } }, 3.0);
        s.addEnergy(Row { { 1, 1.0 } }, -1.0);
        s.finalizeConstraints();
        CHECK(s.kernelSize() == 2);
        CHECK(s.solveIteration());
        CHECK(s.converged());
        CHECK(near(s.value(0), 3.0));
        CHECK(near(s.value(1), -1.0));
    }

    // Equality constraint x0 - x1 = 0 merges both variables into one kernel
    // variable; the two-variable energy x0 + x1 = 6 then solves to 3 and 3.
    {
        MixedIntegerLeastSquares s(2);
        s.addConstraint(0, 1.0, 1, -1.0);
        s.addEnergy(0, 1.0, 1, 1.0, 6.0);
        s.finalizeConstraints();
        CHECK(s.kernelSize() == 1);
        CHECK(s.solveIteration());
        CHECK(s.converged());
        CHECK(near(s.value(0), 3.0));
        CHECK(near(s.value(1), 3.0));
    }

    // Single-term constraint anchors x0 to zero: its energy row drops out of
    // the reduced system while x1 still solves to its target.
    {
        MixedIntegerLeastSquares s(2);
        s.addConstraint(0, 1.0);
        s.addEnergy(Row { { 0, 1.0 } }, 5.0);
        s.addEnergy(Row { { 1, 1.0 } }, 7.0);
        s.finalizeConstraints();
        CHECK(s.kernelSize() == 1);
        CHECK(s.solveIteration());
        CHECK(s.converged());
        CHECK(near(s.value(0), 0.0));
        CHECK(near(s.value(1), 7.0));
    }

    // Periodic variable rounds to the nearest integer multiple: 2.3 with
    // period 1 converges to 2 within the iteration cap.
    {
        MixedIntegerLeastSquares s(1);
        s.setVariablePeriod(0, 1);
        s.addEnergy(Row { { 0, 1.0 } }, 2.3);
        s.finalizeConstraints();
        CHECK(s.kernelSize() == 1);
        CHECK(s.originalIntegerVariableCount() == 1);
        CHECK(s.integerKernelVariableCount() == 1);
        bool solved = false;
        for (size_t iteration = 0; iteration < 100; ++iteration) {
            solved = s.solveIteration();
            if (!solved || s.converged())
                break;
        }
        CHECK(solved);
        CHECK(s.converged());
        CHECK(near(s.value(0), 2.0));
    }

    // Degenerate: an empty system has no kernel to solve, and reading past
    // the variable count yields zero.
    {
        MixedIntegerLeastSquares s(0);
        s.finalizeConstraints();
        CHECK(s.kernelSize() == 0);
        CHECK(!s.solveIteration());
        MixedIntegerLeastSquares t(1);
        t.addEnergy(Row { { 0, 1.0 } }, 4.0);
        t.finalizeConstraints();
        CHECK(t.solveIteration());
        CHECK(t.value(99) == 0.0);
    }

    if (g_failures == 0)
        std::printf("PASS test_mixedintegerleastsquares\n");
    return g_failures == 0 ? 0 : 1;
}
