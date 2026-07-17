// Unit test for VelocityObserver, the model-based wheel-velocity estimator.
//
// velocity_observer.cpp has no PROS dependencies, so this links standalone on the
// host -- no brain, no toolchain. Run from the repo root:
//
//   g++ -std=gnu++20 -Iinclude tests/cpp/test_velocity_observer.cpp \
//       src/filters/velocity_observer.cpp -o /tmp/test_obs && /tmp/test_obs
//
// The property that matters is in case 4: the observer PREDICTS from the applied voltage
// rather than smoothing the encoder, so it tracks a hard acceleration with almost no lag.
// The SlewRateLimiter(200, 200) it replaced could only move the measured velocity 2.0 in/s
// per 10 ms tick -- less than the 2.64 in/s the drivetrain reaches near stall -- so it
// clipped real acceleration and reported a wheel speed several in/s stale. At kP = 3.0
// that stale reading becomes a double-digit motor-unit command the robot never asked for.
#include "filters/velocity_observer.hpp"
#include <cstdio>
#include <cmath>
#include <algorithm>

// Matches Config:: KS / KV / KA / VELOCITY_OBSERVER_GAIN and DT.
static constexpr double kS = 2.55;
static constexpr double kV = 2.0;
static constexpr double kA = 0.4;
static constexpr double GAIN = 0.2;
static constexpr double DGAIN = 0.15;
static constexpr double DT = 0.01;

// One encoder count is ~0.0217 in, so a 10 ms position delta quantizes to ~2.17 in/s.
static constexpr double COUNT_IN_PER_SEC = 0.0217 / DT;

static int failures = 0;

static void check(bool ok, const char* what) {
    std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) failures++;
}

static double quantize(double v) {
    return std::round(v / COUNT_IN_PER_SEC) * COUNT_IN_PER_SEC;
}

// The plant the gains describe: V = kS*sgn(v) + kV*v + kA*a.
//
// kVeff lets a test make the REAL drivetrain differ from the model the observer carries.
// That is not hypothetical: turning scrubs the wheels sideways, so the robot genuinely
// needs more volts per in/s mid-turn than the straight-line kV says.
static double plantStep(double v, double voltage, double kVeff = kV) {
    const double friction = (std::abs(v) > 0.25)
        ? std::copysign(kS, v)
        : std::copysign(std::min(std::abs(voltage), kS), voltage);
    return v + ((voltage - friction - kVeff * v) / kA) * DT;
}

int main() {
    std::printf("VelocityObserver\n");

    // 1. At rest with no voltage it must stay at rest -- in particular the friction term
    //    must not push the estimate off zero when there is nothing driving it.
    {
        VelocityObserver obs(kS, kV, kA, GAIN, DGAIN);
        for (int i = 0; i < 50; i++) obs.update(0.0, 0.0, DT);
        check(std::abs(obs.get()) < 1e-9, "at rest, zero voltage -> estimate stays zero");
    }

    // 2. A small voltage that cannot break static friction must not make the model predict
    //    motion, and must never predict motion BACKWARDS (which is what an unconditional
    //    copysign(kS, v_hat) would do at v_hat = 0).
    {
        VelocityObserver obs(kS, kV, kA, GAIN, DGAIN);
        for (int i = 0; i < 50; i++) obs.update(0.0, 1.0, DT);
        check(obs.get() >= -1e-9, "sub-friction voltage never predicts reverse motion");
    }

    // 3. gain = 1.0 discards the model and returns the raw measurement.
    {
        VelocityObserver obs(kS, kV, kA, 1.0, 0.0);
        const double out = obs.update(31.7, 60.0, DT);
        check(std::abs(out - 31.7) < 1e-9, "gain = 1.0 passes the measurement through");
    }

    // 4. THE POINT. Track a full-send 0 -> top-speed step and compare against the old
    //    SlewRateLimiter(200, 200). Both see the same quantized encoder.
    {
        VelocityObserver obs(kS, kV, kA, GAIN, DGAIN);
        double vTrue = 0.0, vSlew = 0.0;
        double peakObsErr = 0.0, peakSlewErr = 0.0;

        for (int k = 0; k < 100; k++) {
            const double V = 127.0;
            vTrue = plantStep(vTrue, V);
            const double meas = quantize(vTrue);

            const double vObs = obs.update(meas, V, DT);

            double d = std::clamp(meas - vSlew, -200.0 * DT, 200.0 * DT);
            vSlew += d;

            peakObsErr = std::max(peakObsErr, std::abs(vObs - vTrue));
            peakSlewErr = std::max(peakSlewErr, std::abs(vSlew - vTrue));
        }

        std::printf("    peak error: observer %.2f in/s, slew limiter %.2f in/s\n",
                    peakObsErr, peakSlewErr);
        check(peakObsErr < 1.5, "observer tracks a full-send step within 1.5 in/s");
        check(peakSlewErr > 3.0, "slew limiter lags the same step by over 3 in/s (regression witness)");
        check(peakObsErr < peakSlewErr / 3.0, "observer's transient error is <1/3 of the limiter's");
    }

    // 5. It must converge to the truth in steady state, not sit at a biased offset --
    //    the encoder correction is what guarantees this even if the model is imperfect.
    {
        VelocityObserver obs(kS, kV, kA, GAIN, DGAIN);
        double vTrue = 0.0;
        for (int k = 0; k < 500; k++) {
            const double V = 80.0;
            vTrue = plantStep(vTrue, V);
            obs.update(quantize(vTrue), V, DT);
        }
        check(std::abs(obs.get() - vTrue) < 1.0, "converges to truth at steady state");
    }

    // 6. A model that disagrees with reality (a defender, a wall) must be dragged back to
    //    the encoder rather than running away on the prediction. Full voltage, wheel
    //    physically locked, encoder reads zero.
    //
    //    With disturbanceGain = 0 this settles at 10.4 in/s of velocity that does not
    //    exist, because a proportional correction can only ever cancel `gain` of the error
    //    the model re-injects every tick. The disturbance state is what drives it to zero.
    {
        VelocityObserver obs(kS, kV, kA, GAIN, DGAIN);
        for (int k = 0; k < 400; k++) obs.update(0.0, 127.0, DT);
        check(std::abs(obs.get()) < 1.0, "stalled wheel: estimate goes to zero, not 10 in/s");

        VelocityObserver noDist(kS, kV, kA, GAIN, 0.0);
        for (int k = 0; k < 400; k++) noDist.update(0.0, 127.0, DT);
        check(std::abs(noDist.get()) > 5.0, "...and without it, the bias is real (regression witness)");
    }

    // 6b. THE ONE THAT PROTECTS THE TURN. Scrub makes the real drivetrain need ~40% more
    //     volts per in/s than the straight-line kV the observer models. If the observer
    //     over-reports velocity here, the PID concludes the wheels are already up to speed
    //     and under-drives them -- worsening exactly the under-rotation this work exists to
    //     fix. The disturbance state must absorb the mismatch.
    {
        VelocityObserver obs(kS, kV, kA, GAIN, DGAIN);
        constexpr double kVscrub = 2.8;
        double vTrue = 0.0;
        for (int k = 0; k < 1200; k++) {
            const double V = 80.0;
            vTrue = plantStep(vTrue, V, kVscrub);
            obs.update(quantize(vTrue), V, DT);
        }
        const double bias = std::abs(obs.get() - vTrue);
        std::printf("    scrub bias: %.2f in/s (%.0f%% of true %.1f in/s)\n",
                    bias, 100.0 * bias / vTrue, vTrue);
        check(bias < 0.05 * vTrue, "40% scrub mismatch -> under 5% velocity bias");
    }

    // 7. A bad sample must not poison the estimate. NaN is sticky through +=, and a NaN
    //    velocity reaches static_cast<int>(voltage), which is undefined behaviour.
    {
        VelocityObserver obs(kS, kV, kA, GAIN, DGAIN);
        for (int k = 0; k < 20; k++) obs.update(30.0, 60.0, DT);
        const double before = obs.get();

        obs.update(std::nan(""), 60.0, DT);
        check(std::isfinite(obs.get()) && obs.get() == before, "NaN measurement is rejected");

        obs.update(30.0, INFINITY, DT);
        check(std::isfinite(obs.get()), "infinite voltage is rejected");

        obs.update(30.0, 60.0, 0.0);
        check(std::isfinite(obs.get()), "zero dt is rejected");
    }

    // 8. kA <= 0 would divide by zero; the observer must degrade to a plain EMA instead.
    {
        VelocityObserver obs(kS, kV, 0.0, 0.5, DGAIN);
        const double out = obs.update(10.0, 127.0, DT);
        check(std::isfinite(out) && std::abs(out - 5.0) < 1e-9, "kA = 0 degrades to an EMA, no div-by-zero");
    }

    // 9. reset() must actually clear the estimate.
    {
        VelocityObserver obs(kS, kV, kA, GAIN, DGAIN);
        for (int k = 0; k < 50; k++) obs.update(40.0, 100.0, DT);
        obs.reset();
        check(obs.get() == 0.0, "reset() clears the estimate");
    }

    std::printf("%s (%d failure%s)\n", failures ? "FAILED" : "ALL PASSED",
                failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
