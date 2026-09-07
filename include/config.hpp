#ifndef CONFIG_H
#define CONFIG_H

#include "utilities/math/units.hpp"

namespace Config
{
    // Robot config
    /// Effective wheel diameter, measured from two straight line tests
    constexpr double WHEEL_DIAMETER = 2.65;                                                   ///< Effective wheel diameter in inches (calibrated; nominal is WHEEL_2_75_INCH = 2.76)
    constexpr double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * M_PI;                             ///< Wheel circumference in inches
    constexpr double TICKS_PER_ROTATION = Units::Constants::V5_BLUE_MOTOR_TICKS_PER_ROTATION; ///< Encoder ticks per rotation for blue cartridges
    constexpr double GEAR_RATIO = 36.0 / 48.0;                                                ///< Gear ratio of the drivetrain
    constexpr double WHEEL_BASE_WIDTH = 13.5;                                                 ///< Effective track width in inches. Empirically ~15 from measured wheel velocities vs IMU heading in tuning logs (larger than the physical measurement due to wheel scrub); smaller values under-drive the wheel-speed differential and make the robot under-rotate on curves. Re-measure from a fresh log after any wheel/geometry change.
    constexpr double LATERAL_WHEEL_OFFSET = 2.419;                                            ///< Distance from tracking center to lateral wheel in inches

    // Odometry encoder selection
    constexpr int LEFT_ODOM_MOTOR_INDEX = -1; ///< Left pair agrees to 0.03%; average both
    constexpr int RIGHT_ODOM_MOTOR_INDEX = 0; ///< Port 15 only; port 9's encoder is faulty

    // Velocity Controller Constants

    constexpr double KS = 8.0; ///< Static friction compensation (motor units)
    constexpr double KV = 2.0;  ///< Velocity feedforward gain (motor units)/(in/s)
    constexpr double KA = 0.4;  ///< Acceleration feedforward gain (motor units)/(in/s^2)

    /// Asymetrical decel constant to improve tracking since friction + back EMF means we can brake quicker than accel
    constexpr double KA_DECEL = 0.34; ///< Acceleration feedforward while braking

    constexpr double KP = 8.0;  ///< Proportional gain for velocity control
    constexpr double KI = 5.00; ///< Integral gain for velocity control
    constexpr double KD = 0.0;  ///< Derivative gain for velocity control

    // Angular (turning-scrub) feedforward
    // Prevents underturning from wheel scrub + inertia from turning
    constexpr double KV_ANGULAR = 4.0; ///< Angular velocity feedforward (motor units)/(rad/s)

    /// Coulomb (breakaway) component of the same scrub torque. Unfinished
    constexpr double KS_ANGULAR = 0.0; ///< Angular static friction (motor units)

    /// Below this commanded |w| the robot is driving straight and the scrub term is
    /// not applied -- keeps the copysign KS_ANGULAR term from dithering at w ~ 0.
    constexpr double ANGULAR_FF_DEADBAND = 0.05; ///< rad/s

    // Ramsete Controller Constants
    constexpr double RAMSETE_B = 6;                     ///< Ramsete B gain (convergence)
    constexpr double RAMSETE_ZETA = 0.7;                  ///< Ramsete zeta (damping)
    constexpr double RAMSETE_SCALE_FACTOR = 0.0254000508; ///< Scale factor for inputs and outputs (in relation to meters)

    // Drivetrain speed budget
    constexpr double MOTOR_FREE_SPEED_RPM = 600.0; ///< V5 blue cartridge free speed
    constexpr double FREE_WHEEL_VELOCITY =
        MOTOR_FREE_SPEED_RPM * GEAR_RATIO * WHEEL_DIAMETER * M_PI / 60.0; ///< ~65.0 in/s, no load

    /// Fraction of free speed the drivetrain can actually hold under load. Measured
    /// from tests/logs/initial_tuning.txt: while pinned at +/-127 the wheels topped
    /// out near 56 in/s, i.e. ~0.86 of free speed. 0.85 is that, rounded down.
    constexpr double USABLE_SPEED_FRACTION = 0.72;

    /// Wheel-speed budget for desaturation (~55 in/s)
    constexpr double MAX_WHEEL_VELOCITY = FREE_WHEEL_VELOCITY * USABLE_SPEED_FRACTION;

    // Miscellaneous constants
    constexpr double DT = 10;              ///< Global default time step in milliseconds
    constexpr bool VELOCITY_FILTER = true; ///< Enable velocity filter
    constexpr bool HEADING_FILTER = false; ///< Enable heading filter

    /// IMU gyro scale correction. Measured from spinning the robot 10 turns counterclockwise in place 
    /// the IMU reported 3568.83 (9*360 + 328.83) so this term counteracts that
    constexpr double IMU_HEADING_SCALE = 3600.0 / 3568.83; ///< ~1.00873

    /// Encoder-correction gain for the wheel-velocity observer (VelocityObserver).
    /// Basically a GH filter that uses the feedforward model we use for velo control
    constexpr double VELOCITY_OBSERVER_GAIN = 0.2;

    /// Integral gain on the observer's residual, in (motor units)/(in/s) per tick. Learns
    /// the voltage the kS/kV/kA model is missing, so a constant model error does not leave
    /// a constant velocity bias.
    constexpr double VELOCITY_OBSERVER_DISTURBANCE_GAIN = 0.15;
}

#endif
