// Unit test for DrivetrainController's angular (turning-scrub) feedforward.
//
// drivetrain_controller.cpp's only PROS dependency is pros::millis(), which is
// stubbed below, so this links standalone on the host. Run from the repo root:
//
//   g++ -std=gnu++20 -Iinclude tests/cpp/test_angular_feedforward.cpp \
//       src/controllers/drivetrain_controller.cpp -o /tmp/test_angff && /tmp/test_angff
//
// The property that matters: KS/KV/KA are fitted to straight driving and applied per
// wheel, so they cannot see the yaw-resisting scrub torque that only exists while
// turning. It drags the outer wheel back and pushes the inner one forward, and since
// rotation is the DIFFERENCE of two large wheel speeds, that small per-wheel error
// destroys a large share of the turn -- 21% of the commanded differential in
// tests/logs/07132026-4.txt. The angular term must reinforce the commanded
// differential, do so symmetrically in both turn directions, and leave forward speed
// alone.
#include "controllers/drivetrain_controller.hpp"
#include <cstdio>
#include <cmath>

// PROS stub: the controller only reads millis() to derive dt for the I and D terms.
// Every test below runs with kI = kD = 0, so the value is irrelevant. pros::millis()
// is the C API re-exported into the namespace, so it has C linkage.
extern "C" uint32_t millis(void) { return 0; }

static int failures = 0;

static void check(bool ok, const char* what) {
    std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) failures++;
}

static bool close(double a, double b, double tol = 1.01) {
    return std::abs(a - b) < tol;  // voltages are truncated to int, so allow 1 unit
}

// Same gains as Config, but with the angular term switched off, to get a baseline.
static DrivetrainController plain() {
    return DrivetrainController(Config::KS, Config::KV, Config::KA,
                                Config::KP, 0.0, 0.0, 0.0, 0.0);
}

static DrivetrainController withAngular() {
    return DrivetrainController(Config::KS, Config::KV, Config::KA,
                                Config::KP, 0.0, 0.0,
                                Config::KS_ANGULAR, Config::KV_ANGULAR);
}

// Drive both controllers with actual == setpoint, so feedback is zero and we are
// looking purely at the feedforward.
struct Result { double baseDiff, angDiff, baseSum, angSum; };

static Result run(double left, double right) {
    auto p = plain();
    auto a = withAngular();
    auto vp = p.calculateVoltages(left, right, left, right, 0.0, 0.0);
    auto va = a.calculateVoltages(left, right, left, right, 0.0, 0.0);
    return {double(vp.right - vp.left), double(va.right - va.left),
            double(vp.right + vp.left), double(va.right + va.left)};
}

int main() {
    const double W = Config::WHEEL_BASE_WIDTH;
    std::printf("Angular feedforward (KV_ANGULAR = %.2f, deadband %.2f rad/s)\n",
                Config::KV_ANGULAR, Config::ANGULAR_FF_DEADBAND);

    // 1. Straight driving: no scrub torque, so the term must not fire at all.
    {
        Result r = run(30.0, 30.0);
        check(close(r.angDiff, r.baseDiff, 1e-9) && close(r.angSum, r.baseSum, 1e-9),
              "straight (w = 0): angular term is inert");
    }

    // 2. Left turn (right wheel outer, w > 0): must INCREASE the differential.
    {
        Result r = run(15.0, 25.0);
        double w = (25.0 - 15.0) / W;
        // Both halves of the term, not just the velocity one: KS_ANGULAR became
        // non-zero on 2026-09-18 (0 -> 5), and a formula that omits it silently
        // asserts the old shape of the model rather than the current one.
        double expected = 2.0 * (Config::KS_ANGULAR + Config::KV_ANGULAR * w);
        check(r.angDiff > r.baseDiff, "left turn: differential is reinforced");
        check(close(r.angDiff - r.baseDiff, expected, 2.01),
              "left turn: differential gain is 2 * (KS_ANGULAR + KV_ANGULAR * w)");
    }

    // 3. Right turn (left wheel outer, w < 0): must increase the differential's
    //    MAGNITUDE, i.e. drive it further negative. A sign error here would cancel
    //    the turn instead of reinforcing it, which is the whole bug being fixed.
    {
        Result r = run(25.0, 15.0);
        check(r.angDiff < r.baseDiff, "right turn: differential is reinforced");
        check(std::abs(r.angDiff) > std::abs(r.baseDiff),
              "right turn: differential MAGNITUDE grows (no sign inversion)");
    }

    // 4. Symmetry: equal and opposite turns get equal and opposite corrections.
    {
        Result l = run(15.0, 25.0);
        Result r = run(25.0, 15.0);
        check(close((l.angDiff - l.baseDiff), -(r.angDiff - r.baseDiff), 2.01),
              "correction is symmetric across turn direction");
    }

    // 5. Antisymmetry: the term moves the differential ONLY. Forward speed, which is
    //    the SUM of the wheel voltages, must be untouched.
    {
        Result r = run(15.0, 25.0);
        check(close(r.angSum, r.baseSum, 2.01),
              "forward speed (voltage sum) is unchanged");
    }

    // 6. Deadband: a whisker of commanded rotation must not trip the term.
    {
        double tiny = 0.5 * Config::ANGULAR_FF_DEADBAND * W;  // w = half the deadband
        Result r = run(30.0 - tiny / 2, 30.0 + tiny / 2);
        check(close(r.angDiff, r.baseDiff, 1e-9), "inside deadband: term is inert");
    }

    // 7. Magnitude at the scrub measured on steady arcs, 2026-10-09 (double traction, boots
    //    1001/1002/1004): at ~1.7 rad/s each wheel needed ~17.5 V beyond the straight-line
    //    feedforward (outer +14, inner -21). The previous reference, a 2.4 in/s outer-wheel
    //    shortfall in 07132026-4.txt (12 motor units at 2 rad/s), predates KS_ANGULAR and
    //    the rebuilds; 12 / 12 supplied 300% of it, so this check had been failing.
    //
    //    Assert on the VOLTAGE, not on the in/s it buys back. Injecting A motor units
    //    moves the wheel by A / (KP + KV) in/s, because the velocity PID sees the
    //    resulting speed change and backs off by KP times it -- so the same term "buys"
    //    half as much velocity at KP = 8 as it did at KP = 3. That is not the term
    //    getting weaker. Scrub is a disturbance, and the residual it leaves is
    //    (D - A) / (KP + KV): the feedforward's share and the leftover error are divided
    //    by the SAME factor, so raising KP strictly reduces the under-rotation.
    {
        constexpr double ARC_RATE = 1.7;          ///< rad/s, steady-arc mean
        constexpr double SCRUB_VOLTS = 17.5;      ///< motor units per wheel at ARC_RATE

        double w = ARC_RATE;
        double delta = w * W / 2.0;
        Result r = run(30.0 - delta, 30.0 + delta);
        double perWheel = (r.angDiff - r.baseDiff) / 2.0;
        double coverage = perWheel / SCRUB_VOLTS;
        double residual = (SCRUB_VOLTS - perWheel) / (Config::KP + Config::KV);
        std::printf("      w=%.1f rad/s -> %.1f of %.1f motor units of scrub (%.0f%%); "
                    "residual %.2f in/s at KP=%.1f\n",
                    w, perWheel, SCRUB_VOLTS, coverage * 100.0, residual, Config::KP);
        check(coverage > 0.5 && coverage <= 1.1,
              "arc rate cancels 50-110% of the measured scrub voltage (over 100% over-turns)");
        check(std::abs(residual) <= SCRUB_VOLTS / (Config::KP + Config::KV),
              "residual never exceeds the uncompensated shortfall");
    }

    std::printf("%s (%d failure%s)\n", failures ? "FAILED" : "OK", failures,
                failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
