#include "filters/velocity_observer.hpp"

#include <algorithm>
#include <cmath>

VelocityObserver::VelocityObserver(double kS, double kV, double kA, double gain,
                                   double disturbanceGain)
    : kS(kS)
    , kV(kV)
    , kA(kA)
    , gain(std::clamp(gain, 0.0, 1.0))
    , disturbanceGain(std::max(disturbanceGain, 0.0))
    , modelUsable(kA > 0.0)
    , vHat(0.0)
    , dHat(0.0)
{
}

double VelocityObserver::update(double measuredVelocity, double appliedVoltage, double dt)
{
    // A bad sample must not be able to poison the estimate: NaN is sticky through +=, and
    // a NaN velocity propagates through RAMSETE into static_cast<int>(voltage), which is
    // undefined behaviour. Hold the last good estimate instead.
    if (!std::isfinite(measuredVelocity) || !std::isfinite(appliedVoltage) || !(dt > 0.0))
    {
        return vHat;
    }

    if (modelUsable)
    {
        // Friction opposes motion. At a standstill it has no direction of its own, so it
        // can only cancel the applied voltage, never reverse it
        const double friction =
            (std::abs(vHat) > STATIC_VELOCITY_EPSILON)
                ? std::copysign(kS, vHat)
                : std::copysign(std::min(std::abs(appliedVoltage), kS), appliedVoltage);

        // dHat is the voltage the model is missing: scrub, sag, a driving into a goal, a bad kV.
        const double effectiveVoltage = appliedVoltage + dHat;
        const double predictedAccel = (effectiveVoltage - friction - kV * vHat) / kA;
        vHat += predictedAccel * dt;
    }

    // If constant error is detected don't keep offsetting it, assume it's actually real
    const double residual = measuredVelocity - vHat;
    vHat += gain * residual;

    if (modelUsable)
    {
        dHat = std::clamp(dHat + disturbanceGain * residual, -MAX_DISTURBANCE, MAX_DISTURBANCE);
    }

    return vHat;
}

void VelocityObserver::reset(double velocity)
{
    vHat = std::isfinite(velocity) ? velocity : 0.0;
    dHat = 0.0;
}
