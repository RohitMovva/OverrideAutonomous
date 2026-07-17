// Unit test for DrivetrainController's asymmetric acceleration feedforward (KA/KA_DECEL).
//
// drivetrain_controller.cpp's only PROS dependency is pros::millis(), stubbed below, so
// this links standalone on the host. Run from the repo root:
//
//   g++ -std=gnu++20 -Iinclude tests/cpp/test_feedforward_asymmetry.cpp \
//       src/controllers/drivetrain_controller.cpp -o /tmp/test_ffa && /tmp/test_ffa
//
// Friction opposes the motors while accelerating and ASSISTS them while braking, so the
// same commanded |acceleration| costs less voltage to brake than to speed up. Measured on
// the straight, unsaturated ticks of 07132026-{2,4,5,6}.txt, the robot decelerated 17%
// harder than commanded (while accelerating tracked to within 10%), and so stopped short
// of its goal every run.
//
// The case that matters most here is DRIVING BACKWARDS: at v < 0 it is a *positive*
// acceleration that slows the robot down, and that is still braking. Testing sign(a)
// instead of sign(v*a) would get this exactly backwards -- and reverse nodes make it a
// routine case, not an exotic one.
#include "controllers/drivetrain_controller.hpp"
#include <cstdio>
#include <cmath>

// pros::millis() is the C API re-exported into the namespace, so it has C linkage. Only
// used to derive dt for the I and D terms; every test here runs with kI = kD = 0.
extern "C" uint32_t millis(void) { return 0; }

static int failures = 0;

static void check(bool ok, const char* what) {
    std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) failures++;
}

static bool close(double a, double b, double tol = 1.01) {
    return std::abs(a - b) < tol;  // voltages truncate to int, so allow one unit
}

static DrivetrainController make(double kADecel) {
    return DrivetrainController(Config::KS, Config::KV, Config::KA,
                                Config::KP, 0.0, 0.0, 0.0, 0.0, kADecel);
}

// Feedforward only: drive with actual == setpoint so the feedback term is zero.
static double ff(DrivetrainController& c, double v, double a) {
    return double(c.calculateVoltages(v, v, v, v, a, a).left);
}

int main() {
    std::printf("Asymmetric accel feedforward (KA = %.2f, KA_DECEL = %.2f)\n",
                Config::KA, Config::KA_DECEL);

    auto c = make(Config::KA_DECEL);
    auto sym = make(Config::KA);  // old symmetric behaviour, for comparison

    // 1. Speeding up forwards (v > 0, a > 0) must be untouched by the change.
    check(close(ff(c, 30.0, 100.0), ff(sym, 30.0, 100.0), 1e-9),
          "forward + accelerating: unchanged (still uses KA)");

    // 2. Braking forwards (v > 0, a < 0): must brake LESS hard than before, i.e. command a
    //    LESS negative voltage. This is the bug being fixed.
    {
        double now = ff(c, 30.0, -100.0), before = ff(sym, 30.0, -100.0);
        double expected = (Config::KA - Config::KA_DECEL) * 100.0;  // less braking effort
        std::printf("      v=+30 a=-100 -> %.0f now vs %.0f before (%+.0f)\n",
                    now, before, now - before);
        check(now > before, "forward + braking: brakes less hard than before");
        check(close(now - before, expected, 2.01),
              "forward + braking: relief is exactly (KA - KA_DECEL) * |a|");
    }

    // 3. Driving BACKWARDS and speeding up (v < 0, a < 0): still accelerating, so KA.
    check(close(ff(c, -30.0, -100.0), ff(sym, -30.0, -100.0), 1e-9),
          "reverse + accelerating: unchanged (still uses KA)");

    // 4. Driving BACKWARDS and braking (v < 0, a > 0): this is braking, so KA_DECEL. The
    //    voltage is negative here, and braking means driving it toward zero -- so the
    //    corrected command must be LESS positive than the symmetric one.
    {
        double now = ff(c, -30.0, 100.0), before = ff(sym, -30.0, 100.0);
        std::printf("      v=-30 a=+100 -> %.0f now vs %.0f before (%+.0f)\n",
                    now, before, now - before);
        check(now < before, "reverse + braking: brakes less hard (sign(v*a), not sign(a))");
        check(close(before - now, (Config::KA - Config::KA_DECEL) * 100.0, 2.01),
              "reverse + braking: same relief magnitude as forward braking");
    }

    // 5. Symmetry: the correction must not depend on which way the robot is pointing.
    {
        double fwd = ff(sym, 30.0, -100.0) - ff(c, 30.0, -100.0);
        double rev = ff(c, -30.0, 100.0) - ff(sym, -30.0, 100.0);
        check(close(fwd, rev, 2.01), "braking relief is direction-independent");
    }

    // 6. Coasting (a = 0): nothing to choose between, both gains multiply zero.
    check(close(ff(c, 30.0, 0.0), ff(sym, 30.0, 0.0), 1e-9), "a = 0: unchanged");

    // 7. Back-compat: a controller built without the new arg must behave exactly as before.
    {
        DrivetrainController old(Config::KS, Config::KV, Config::KA, Config::KP, 0.0, 0.0);
        check(close(ff(old, 30.0, -100.0), ff(sym, 30.0, -100.0), 1e-9),
              "omitting kADecel falls back to kA (old callers unchanged)");
    }

    // 8. The measured fix: a 17% over-brake should be nulled. The plant's true braking
    //    coefficient is ~0.341; commanding KA_DECEL against it should land near 1.0x.
    {
        const double kATrueBraking = 0.341;
        double ratio = Config::KA_DECEL / kATrueBraking;
        std::printf("      achieved/commanded decel: %.2fx before -> %.2fx now\n",
                    Config::KA / kATrueBraking, ratio);
        check(ratio > 0.92 && ratio < 1.08,
              "commanded deceleration is now delivered within 8%");
    }

    std::printf("%s (%d failure%s)\n", failures ? "FAILED" : "OK", failures,
                failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
