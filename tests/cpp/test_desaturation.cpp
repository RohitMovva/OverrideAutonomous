// Unit test for RamseteController::calculate_wheel_velocities desaturation.
//
// ramsete_controller.cpp has no PROS dependencies, so this links standalone on the
// host -- no brain, no toolchain. Run from the repo root:
//
//   g++ -std=gnu++20 -Iinclude tests/cpp/test_desaturation.cpp \
//       src/controllers/ramsete_controller.cpp -o /tmp/test_desat && /tmp/test_desat
//
// The property that matters is in case 3: when a wheel is over budget, scaling both
// wheels preserves the ARC (w/v), whereas clamping each wheel independently shrinks
// the differential and flattens the turn. That flattening is what made the robot
// under-rotate and run wide for an entire run in tests/logs/initial_tuning.txt.
#include "controllers/ramsete_controller.hpp"
#include <cstdio>
#include <cmath>

static int failures = 0;

static void check(bool ok, const char* what) {
    std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) failures++;
}

static bool close(double a, double b, double tol = 1e-9) {
    return std::abs(a - b) < tol;
}

int main() {
    RamseteController rc(2.4, 0.4, 0.0254000508);
    const double W = 15.0;
    const double VMAX = 55.276;   // Config::MAX_WHEEL_VELOCITY

    std::printf("track width %.1f in, wheel budget %.3f in/s\n\n", W, VMAX);

    // 1. Under budget -> untouched.
    {
        auto v = rc.calculate_wheel_velocities(30.0, 1.0, W, VMAX);
        check(close(v[0], 30.0 - 7.5) && close(v[1], 30.0 + 7.5),
              "under budget: passes through unscaled (22.5, 37.5)");
    }

    // 2. Over budget -> scaled to exactly the budget, curvature preserved.
    {
        // v=48, w=2.94 rad/s  -> diff/2 = 22.05 -> wheels (25.95, 70.05): way over.
        double v_in = 48.0, w_in = 2.94;
        auto v = rc.calculate_wheel_velocities(v_in, w_in, W, VMAX);
        double lo = v[0], hi = v[1];
        check(close(std::max(std::abs(lo), std::abs(hi)), VMAX, 1e-6),
              "over budget: fastest wheel lands exactly on the budget");

        // curvature = w/v must be unchanged: recover v,w from the wheels
        double v_out = (lo + hi) / 2.0;
        double w_out = (hi - lo) / W;
        check(close(w_out / v_out, w_in / v_in, 1e-9),
              "over budget: curvature (w/v) preserved exactly");
        std::printf("        in : v=%.2f w=%.3f -> wheels(%.2f, %.2f)\n",
                    v_in, w_in, v_in - w_in * W / 2, v_in + w_in * W / 2);
        std::printf("        out: v=%.2f w=%.3f -> wheels(%.2f, %.2f)\n",
                    v_out, w_out, lo, hi);
    }

    // 3. THE REGRESSION THIS EXISTS TO PREVENT: independent clamping would
    //    flatten the turn. Show that scaling does not.
    {
        double v_in = 48.0, w_in = 2.94;
        double raw_l = v_in - w_in * W / 2, raw_r = v_in + w_in * W / 2;
        // what a naive per-wheel clamp would have produced:
        double clamp_l = std::fmax(-VMAX, std::fmin(raw_l, VMAX));
        double clamp_r = std::fmax(-VMAX, std::fmin(raw_r, VMAX));
        double clamp_w = (clamp_r - clamp_l) / W;

        auto v = rc.calculate_wheel_velocities(v_in, w_in, W, VMAX);
        double scale_w = (v[1] - v[0]) / W;

        double raw_w = (raw_r - raw_l) / W;
        std::printf("        commanded omega      : %.3f rad/s\n", raw_w);
        std::printf("        per-wheel CLAMP omega: %.3f rad/s  (%.0f%% of commanded)\n",
                    clamp_w, 100 * clamp_w / raw_w);
        std::printf("        desaturate    omega  : %.3f rad/s  (%.0f%% of commanded)\n",
                    scale_w, 100 * scale_w / raw_w);
        check(clamp_w < raw_w * 0.85,
              "a naive per-wheel clamp really does destroy the turn (<85% of omega)");
        check(scale_w < raw_w,
              "desaturation also reduces omega (it must -- the robot is slower)...");
        check(close(scale_w / ((v[0] + v[1]) / 2.0), raw_w / v_in, 1e-9),
              "...but it reduces v by the SAME factor, so the ARC is unchanged");
    }

    // 4. Reverse / negative velocities handled by magnitude.
    {
        auto v = rc.calculate_wheel_velocities(-48.0, 2.94, W, VMAX);
        check(std::max(std::abs(v[0]), std::abs(v[1])) <= VMAX + 1e-6,
              "reverse driving: still bounded by the budget");
    }

    // 5. Disabled when budget <= 0.
    {
        auto v = rc.calculate_wheel_velocities(100.0, 0.0, W, 0.0);
        check(close(v[0], 100.0) && close(v[1], 100.0),
              "budget <= 0 disables desaturation (escape hatch)");
    }

    std::printf("\n%s (%d failure(s))\n", failures ? "FAILED" : "ALL PASSED", failures);
    return failures ? 1 : 0;
}
