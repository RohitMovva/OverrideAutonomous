#ifndef VELOCITY_OBSERVER_H
#define VELOCITY_OBSERVER_H

/**
 * @brief Wheel-velocity estimator that fuses the drivetrain model with the encoder.
 *
 * Use existing kS/kV/kA model from dt controller:
 *
 *     V = kS * sgn(v) + kV * v + kA * a    =>    a = (V - kS * sgn(v) - kV * v) / kA
 *
 *
 * That is a complementary filter: model on the high frequencies, encoder on the low.
 * Equivalently, a steady-state 1-D Kalman filter with the voltage as its control input.
 *
 * There is a second state: an estimated DISTURBANCE, in motor units, standing for
 * whatever voltage the model fails to account for. Without it, a persistent model error
 * leaves a permanent velocity bias, because a proportional correction only ever cancels
 * `gain` of the error the model re-injects each tick. 
 */
class VelocityObserver
{
public:
    /**
     * @param kS   Static friction compensation (motor units)
     * @param kV   Velocity feedforward gain (motor units)/(in/s)
     * @param kA   Acceleration feedforward gain (motor units)/(in/s^2)
     * @param gain Correction gain applied to the encoder residual each tick, in (0, 1].
     *             The estimate converges on the encoder with a time constant of dt/gain,
     *             and passes through sqrt(gain / (2 - gain)) of the measurement noise.
     *             1.0 discards the model and returns the raw measurement.
     * @param disturbanceGain Integral gain on the residual, in (motor units)/(in/s) per
     *             tick. Corrects steady-state model error. 0 disables the second state
     *             and reduces this to a proportional-only observer.
     */
    VelocityObserver(double kS, double kV, double kA, double gain, double disturbanceGain);

    /**
     * @brief Advance the estimate one tick.
     *
     * @param measuredVelocity Velocity from the encoder delta this tick (in/s)
     * @param appliedVoltage   Voltage actually sent to the motors LAST tick, in the same
     *                         motor units as kS/kV/kA (i.e. the +/-127 command scale),
     *                         after desaturation.
     * @param dt               Time step in seconds
     * @return The fused velocity estimate (in/s)
     */
    double update(double measuredVelocity, double appliedVoltage, double dt);

    /// Reset the estimate. Call whenever the drivetrain state is discontinuous.
    void reset(double velocity = 0.0);

    double get() const { return vHat; }

    /// Estimated unmodelled voltage (motor units). Useful for tuning: a disturbance that
    /// sits well away from zero while driving straight means kS/kV are miscalibrated.
    double getDisturbance() const { return dHat; }

private:
    /// Below this speed the wheel is treated as stationary for the friction term, so the
    /// model does not chatter kS back and forth across a sign change at a standstill.
    static constexpr double STATIC_VELOCITY_EPSILON = 0.25; // in/s

    /// The disturbance cannot exceed the full command range; clamping keeps a transient
    /// (a wheel briefly off the ground) from winding it up into a lasting bias.
    static constexpr double MAX_DISTURBANCE = 127.0; // motor units

    const double kS;
    const double kV;
    const double kA;
    const double gain;
    const double disturbanceGain;
    const bool modelUsable; ///< false if kA <= 0; the observer degrades to a plain EMA

    double vHat;
    double dHat;
};

#endif // VELOCITY_OBSERVER_H
