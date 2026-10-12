#ifndef TRACKING_ARC_H
#define TRACKING_ARC_H

#include <cmath>

/// Robot-frame displacement over one odometry tick (inches; x forward, y left)
struct LocalDisplacement
{
    double x;
    double y;
};

/**
 * @brief Arc odometry for two perpendicular tracking wheels
 *
 * Assumes the robot moved along a constant-curvature arc during the tick. A wheel
 * offset from the tracking centre also rolls when the robot only turns, so that part is
 * removed before the chord is taken.
 *
 * Kept free of PROS so it can be host-tested (tests/cpp/test_tracking_arc.cpp).
 *
 * @param dVertical Vertical (forward) wheel travel this tick, inches
 * @param dLateral Lateral (sideways, +left) wheel travel this tick, inches
 * @param dTheta Heading change this tick, radians CCW+
 * @param verticalOffset Vertical wheel's sideways offset from the tracking centre, +left
 * @param lateralOffset Lateral wheel's fore/aft offset from the tracking centre, +forward
 */
inline LocalDisplacement trackingArc(double dVertical, double dLateral, double dTheta,
                                     double verticalOffset, double lateralOffset)
{
    // A pure CCW turn moves a wheel at +left offset backwards (reads -offset*dTheta) and
    // a wheel at +forward offset to the left (reads +offset*dTheta).
    if (std::abs(dTheta) < 1e-5)
    {
        return {dVertical + verticalOffset * dTheta, dLateral - lateralOffset * dTheta};
    }
    const double chord = 2.0 * std::sin(dTheta / 2.0);
    return {chord * (dVertical / dTheta + verticalOffset),
            chord * (dLateral / dTheta - lateralOffset)};
}

#endif // TRACKING_ARC_H
