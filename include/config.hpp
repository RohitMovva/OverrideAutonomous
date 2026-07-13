#ifndef CONFIG_H
#define CONFIG_H

#include "utilities/math/units.hpp"

namespace Config
{
    // Robot config
    constexpr double WHEEL_DIAMETER = Units::Constants::WHEEL_2_75_INCH;                      ///< Wheel diameter in inches
    constexpr double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * M_PI;                             ///< Wheel circumference in inches
    constexpr double TICKS_PER_ROTATION = Units::Constants::V5_BLUE_MOTOR_TICKS_PER_ROTATION; ///< Encoder ticks per rotation for blue cartridges
    constexpr double GEAR_RATIO = 36.0 / 48.0;                                                ///< Gear ratio of the drivetrain
    constexpr double WHEEL_BASE_WIDTH = 15.0;                                                 ///< Effective track width in inches. Empirically ~15 from measured wheel velocities vs IMU heading in tuning logs (larger than the physical measurement due to wheel scrub); smaller values under-drive the wheel-speed differential and make the robot under-rotate on curves. Re-measure from a fresh log after any wheel/geometry change.
    constexpr double LATERAL_WHEEL_OFFSET = 2.419;                                            ///< Distance from tracking center to lateral wheel in inches

    // Velocity Controller Constants

    constexpr double KS = 2.55; ///< Static friction compensation (motor units)
    constexpr double KV = 2.0;  ///< Velocity feedforward gain (motor units)/(in/s)
    constexpr double KA = 0.4;  ///< Acceleration feedforward gain (motor units)/(in/s^2)
    constexpr double KP = 3.0;  ///< Proportional gain for velocity control
    constexpr double KI = 0.00; ///< Integral gain for velocity control
    constexpr double KD = 0.0;  ///< Derivative gain for velocity control

    // Ramsete Controller Constants
    constexpr double RAMSETE_B = 2.4;                     ///< Ramsete B gain (convergence)
    constexpr double RAMSETE_ZETA = 0.4;                  ///< Ramsete zeta (damping)
    constexpr double RAMSETE_SCALE_FACTOR = 0.0254000508; ///< Scale factor for inputs and outputs (in relation to meters)

    // Drivetrain speed budget
    //
    // The old RAMSETE_MAX_VELOCITY / RAMSETE_MAX_ANGULAR_VELOCITY pair is gone. They
    // clamped the CHASSIS (v and w independently), which is the same mistake as
    // clamping each wheel voltage on its own: it changes the v:w ratio and so
    // flattens the commanded curvature. They were also never actually applied.
    //
    // The real constraint is on WHEEL speed, because that is what the motors limit.
    // A wheel cap subsumes a chassis cap (|v| <= max(|v_l|, |v_r|) for any w), so
    // one number is enough.
    constexpr double MOTOR_FREE_SPEED_RPM = 600.0; ///< V5 blue cartridge free speed
    constexpr double FREE_WHEEL_VELOCITY =
        MOTOR_FREE_SPEED_RPM * GEAR_RATIO * WHEEL_DIAMETER * M_PI / 60.0; ///< ~65.0 in/s, no load

    /// Fraction of free speed the drivetrain can actually hold under load. Measured
    /// from tests/logs/initial_tuning.txt: while pinned at +/-127 the wheels topped
    /// out near 56 in/s, i.e. ~0.86 of free speed. 0.85 is that, rounded down.
    constexpr double USABLE_SPEED_FRACTION = 0.85;

    /// Wheel-speed budget for desaturation (~55 in/s). Commanding above this cannot
    /// be delivered: the motor saturates, the wheel falls behind its setpoint, and
    /// since rotation is the DIFFERENCE of two large wheel speeds, a few in/s of
    /// shortfall on the outside wheel becomes a large rotation error. That is how
    /// initial_tuning.txt under-rotated by 31% and ran wide off the path all run.
    constexpr double MAX_WHEEL_VELOCITY = FREE_WHEEL_VELOCITY * USABLE_SPEED_FRACTION;

    // Miscellaneous constants
    constexpr double DT = 10;              ///< Global default time step in milliseconds
    constexpr bool VELOCITY_FILTER = true; ///< Enable velocity filter
    constexpr bool HEADING_FILTER = false; ///< Enable heading filter

    /// Encoder-correction gain for the wheel-velocity observer (VelocityObserver).
    /// Basically a GH filter that uses the feedforward model we use for velo control
    constexpr double VELOCITY_OBSERVER_GAIN = 0.2;

    /// Integral gain on the observer's residual, in (motor units)/(in/s) per tick. Learns
    /// the voltage the kS/kV/kA model is missing, so a constant model error does not leave
    /// a constant velocity bias.
    constexpr double VELOCITY_OBSERVER_DISTURBANCE_GAIN = 0.15;
}

#endif
