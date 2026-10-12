// Unit test for trackingArc(), the two-tracking-wheel arc odometry step that
// Odometry::update applies every tick.
//
// tracking_arc.hpp is header-only and PROS-free. Run from the repo root:
//
//   g++ -std=gnu++20 -Iinclude tests/cpp/test_tracking_arc.cpp -o /tmp/test_tracking_arc && /tmp/test_tracking_arc
//
// Each case simulates a known rigid-body motion, computes what each wheel at its offset
// would actually roll, and checks that trackingArc recovers the tracking centre's motion.
// The property that matters most: a pure turn with the lateral wheel 3 in in front of the
// centre rolls that wheel sideways, and none of that may come out as translation.
#include "navigation/tracking_arc.hpp"
#include <cstdio>
#include <cmath>

static int failures = 0;

static void check(bool ok, const char* what) {
    std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) failures++;
}

static bool close(double a, double b, double tol = 1e-9) { return std::abs(a - b) < tol; }

// Wheel readings for a constant-curvature arc of the tracking centre: forward speed v,
// sideways speed vy, turn rate w, over time 1 (so the arguments are the totals). Each
// wheel is integrated finely so the "sensor" does not share trackingArc's own formula.
struct Readings { double dVertical, dLateral; double x, y; };

static Readings simulate(double v, double vy, double w, double verticalOffset, double lateralOffset) {
    const int steps = 200000;
    const double dt = 1.0 / steps;
    double x = 0, y = 0, th = 0, dv = 0, dl = 0;
    for (int i = 0; i < steps; ++i) {
        // Point velocity in the robot frame: v_c + w x r
        dv += (v - w * verticalOffset) * dt;   // vertical wheel at (0, verticalOffset)
        dl += (vy + w * lateralOffset) * dt;   // lateral wheel at (lateralOffset, 0)
        const double mid = th + w * dt / 2;
        x += (v * std::cos(mid) - vy * std::sin(mid)) * dt;
        y += (v * std::sin(mid) + vy * std::cos(mid)) * dt;
        th += w * dt;
    }
    return {dv, dl, x, y};
}

// trackingArc gives the displacement in the robot frame at the START heading (0 here)
// only after rotating by dTheta/2; do that rotation the way Odometry::update does.
static void field(const LocalDisplacement& l, double dTheta, double& x, double& y) {
    const double m = dTheta / 2;
    x = l.x * std::cos(m) - l.y * std::sin(m);
    y = l.x * std::sin(m) + l.y * std::cos(m);
}

static void runCase(const char* name, double v, double vy, double w, double vOff, double lOff) {
    const Readings r = simulate(v, vy, w, vOff, lOff);
    double x, y;
    field(trackingArc(r.dVertical, r.dLateral, w, vOff, lOff), w, x, y);
    char msg[160];
    std::snprintf(msg, sizeof msg, "%s: got (%.6f, %.6f) want (%.6f, %.6f)", name, x, y, r.x, r.y);
    check(close(x, r.x, 1e-6) && close(y, r.y, 1e-6), msg);
}

int main() {
    std::printf("tracking arc\n");

    // The robot's real geometry: vertical wheel on the centre line, lateral 3 in in front
    runCase("straight", 0.6, 0.0, 0.0, 0.0, 3.0);
    runCase("pure CCW turn", 0.0, 0.0, 0.07, 0.0, 3.0);
    runCase("pure CW turn", 0.0, 0.0, -0.07, 0.0, 3.0);
    runCase("forward arc", 0.6, 0.0, 0.05, 0.0, 3.0);
    runCase("arc with sideslip", 0.6, 0.02, -0.05, 0.0, 3.0);
    runCase("reverse arc", -0.5, 0.0, 0.04, 0.0, 3.0);
    // Offset vertical wheel too, to cover the general formula
    runCase("offset vertical wheel", 0.6, 0.01, 0.05, 1.5, -2.0);

    // Pure turn in place: the lateral wheel rolls offset*dTheta, all of it rotation
    {
        const double dTheta = 0.07;
        const LocalDisplacement l = trackingArc(0.0, 3.0 * dTheta, dTheta, 0.0, 3.0);
        check(close(l.x, 0.0) && close(l.y, 0.0), "turn in place yields no translation");
    }
    // Small-angle branch agrees with the full arc formula at an angle just under the switch
    {
        const double dTheta = 0.9e-5;
        const LocalDisplacement a = trackingArc(0.5, 0.01, dTheta, 0.0, 3.0);
        const double chord = 2.0 * std::sin(dTheta / 2.0);
        check(close(a.x, chord * (0.5 / dTheta), 1e-9) &&
              close(a.y, chord * (0.01 / dTheta - 3.0), 1e-9),
              "small-angle branch matches the arc formula");
    }

    std::printf("%s (%d failure%s)\n", failures ? "FAILED" : "OK", failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
