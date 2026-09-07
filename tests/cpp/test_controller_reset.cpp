// Unit test for DrivetrainController::reset() and the dt clamp that backs it.
//
// drivetrain_controller.cpp's only PROS dependency is pros::millis(), stubbed below with
// a settable clock so a test can put the controller at a chosen wall-clock time. Run from
// the repo root:
//
//   g++ -std=gnu++20 -Iinclude tests/cpp/test_controller_reset.cpp \
//       src/controllers/drivetrain_controller.cpp -o /tmp/test_reset && /tmp/test_reset
//
// The controller is CONSTRUCTED in initialize() but first USED when autonomous() starts,
// seconds later. prevTime is constructed to 0, so the opening calculateVoltages() call
// measured dt as the entire time since program start: 2.294 s in tests/logs/08012026-3
// and 2.110 s in 08012026-4. That weights the first tick's integral by ~230x, by an
// amount that varies with however long initialize() happened to take that day. It was
// inert while KI was 0 and became a live run-to-run variable the moment KI was not.
//
// Two separate defects, two separate fixes, and the tests below keep them apart:
//   - the STALE CLOCK is handled by the dt clamp, which rejects any interval past 0.1 s
//     on a 10 ms loop as a bad clock rather than a real gap;
//   - the CARRIED STATE (integrals and previous errors surviving from a prior segment,
//     or a prior run without a reboot) is handled by reset().
// Case 4 is the one that matters: the same tick sequence must produce the same voltages
// no matter what the controller did beforehand or what the clock said when it started.
#include "controllers/drivetrain_controller.hpp"
#include <cstdio>
#include <cmath>
#include <vector>

// pros::millis() is the C API re-exported into the namespace, so it has C linkage.
static uint32_t g_now = 0;
extern "C" uint32_t millis(void) { return g_now; }

static int failures = 0;

static void check(bool ok, const char* what) {
    std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) failures++;
}

static DrivetrainController make() {
    return DrivetrainController(Config::KS, Config::KV, Config::KA,
                                Config::KP, Config::KI, Config::KD,
                                Config::KS_ANGULAR, Config::KV_ANGULAR,
                                Config::KA_DECEL);
}

// Equal left/right setpoints, so the angular scrub term stays inert and the numbers below
// are purely feedforward + PID.
static int tick(DrivetrainController& c, double setpoint, double actual) {
    g_now += static_cast<uint32_t>(Config::DT);
    return c.calculateVoltages(setpoint, setpoint, actual, actual, 0.0, 0.0).left;
}

int main() {
    std::printf("DrivetrainController::reset() (KP = %.1f, KI = %.2f)\n",
                Config::KP, Config::KI);

    const double SP = 10.0;    // in/s commanded
    const double ACT = 0.0;    // in/s measured -> a sustained 10 in/s error
    const double NOMINAL_DT = Units::millisecondsToSeconds(Config::DT);

    // 1. A stale clock on the first call must be treated as one nominal tick, not as the
    //    whole of initialize(). Reference: an identical controller whose clock is sane.
    int stale, sane;
    {
        g_now = 0;
        DrivetrainController c = make();       // prevTime = 0
        g_now = 2294;                          // 08012026-3's autonomous() start
        stale = c.calculateVoltages(SP, SP, ACT, ACT, 0.0, 0.0).left;

        g_now = 0;
        DrivetrainController ref = make();
        ref.reset();                           // prevTime = 0, clock also 0
        sane = tick(ref, SP, ACT);
    }
    std::printf("      first tick: stale clock -> %d, sane clock -> %d motor units\n",
                stale, sane);
    check(stale == sane, "stale first-tick clock is clamped to one nominal tick");

    // 2. ...and the clamp is what makes that true, not luck. Unclamped, dt = 2.294 s puts
    //    err * dt = 22.9 into an accumulator limited to 5.0, i.e. straight to the clamp
    //    and KI * 5.0 = 25 motor units of integral on tick one. Clamped it is
    //    err * 0.01 = 0.1, worth KI * 0.1.
    {
        const double windup = SP * NOMINAL_DT;
        const double bogus = SP * 2.294;
        std::printf("      accumulator on tick 1: %.2f clamped vs %.2f unclamped "
                    "(limit 5.00) -> %.1f vs %.1f motor units of integral\n",
                    windup, bogus, Config::KI * windup, Config::KI * 5.0);
        check(Config::KI * 5.0 - Config::KI * windup > 10.0,
              "the bug this guards was worth >10 motor units on the opening tick");
    }

    // 3. reset() clears a wound-up integrator. Drive a sustained error long enough to load
    //    the accumulator, then reset and confirm the output falls back to the first-tick
    //    value -- i.e. no memory of the previous segment survives.
    {
        g_now = 0;
        DrivetrainController c = make();
        c.reset();
        int first = tick(c, SP, ACT);
        for (int i = 0; i < 200; i++) tick(c, SP, ACT);
        int wound = tick(c, SP, ACT);

        c.reset();
        int after = tick(c, SP, ACT);

        std::printf("      sustained error: first tick %d -> wound up %d -> after reset %d\n",
                    first, wound, after);
        check(std::abs(wound) > std::abs(first),
              "a sustained error really does wind the integrator up");
        check(after == first, "reset() returns the controller to its first-tick output");
    }

    // 4. The property the whole change exists for: identical input sequences produce
    //    identical outputs regardless of what the controller did first or what the clock
    //    read when the run started. ctrlA has a previous segment behind it and starts at
    //    08012026-3's 2.294 s; ctrlB is fresh and starts at -4's 2.110 s.
    {
        std::vector<int> a, b;

        g_now = 0;
        DrivetrainController ctrlA = make();
        for (int i = 0; i < 150; i++) tick(ctrlA, 25.0, 18.0);   // a previous segment
        g_now = 2294;
        ctrlA.reset();
        for (int i = 0; i < 40; i++) a.push_back(tick(ctrlA, SP, ACT));

        g_now = 0;
        DrivetrainController ctrlB = make();
        g_now = 2110;
        ctrlB.reset();
        for (int i = 0; i < 40; i++) b.push_back(tick(ctrlB, SP, ACT));

        bool same = a == b;
        std::printf("      40-tick sequence: A[0..2] = %d,%d,%d   B[0..2] = %d,%d,%d\n",
                    a[0], a[1], a[2], b[0], b[1], b[2]);
        check(same, "same inputs -> same voltages, whatever ran before");
    }

    // 5. A mid-run stall must not be integrated either. delay_until() pins the loop to
    //    Config::DT, so a gap that large is a stalled task, not a real interval, and
    //    charging the integrator for it would dump a step into the motors on recovery.
    {
        g_now = 0;
        DrivetrainController c = make();
        c.reset();
        int steady = 0;
        for (int i = 0; i < 5; i++) steady = tick(c, SP, ACT);

        g_now += 2000;   // 2 s stall, nowhere near a 10 ms tick
        int afterStall = c.calculateVoltages(SP, SP, ACT, ACT, 0.0, 0.0).left;

        g_now = 0;
        DrivetrainController ref = make();
        ref.reset();
        int refOut = 0;
        for (int i = 0; i < 6; i++) refOut = tick(ref, SP, ACT);

        std::printf("      after a 2 s stall: %d vs %d with no stall\n", afterStall, refOut);
        check(afterStall == refOut, "a stalled tick is clamped, not integrated");
    }

    std::printf("%s (%d failure%s)\n", failures ? "FAILED" : "OK", failures,
                failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
