#include "filters/velocity_observer.hpp"

#include <algorithm>
#include <cmath>

VelocityObserver::VelocityObserver(double kS, double kV, double kA, double gain)
    : kS(kS)
    , kV(kV)
    , kA(kA)
    , gain(std::clamp(gain, 0.0, 1.0))
    , modelUsable(kA > 0.0)
    , vHat(0.0)
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
        // can only cancel the applied voltage, never reverse it -- otherwise the model
        // would predict the robot accelerating backwards out of every stop.
        const double friction =
            (std::abs(vHat) > STATIC_VELOCITY_EPSILON)
                ? std::copysign(kS, vHat)
                : std::copysign(std::min(std::abs(appliedVoltage), kS), appliedVoltage);

        const double predictedAccel = (appliedVoltage - friction - kV * vHat) / kA;
        vHat += predictedAccel * dt;
    }

    // Correct towards the encoder. With modelUsable == false this is all that runs, and
    // the observer is exactly an exponential moving average with alpha = gain.
    vHat += gain * (measuredVelocity - vHat);

    return vHat;
}

void VelocityObserver::reset(double velocity)
{
    vHat = std::isfinite(velocity) ? velocity : 0.0;
}
