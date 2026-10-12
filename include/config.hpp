#ifndef CONFIG_H
#define CONFIG_H

#include "utilities/math/units.hpp"
#include <cstddef>
#include <cstdint>

namespace Config
{
    // Robot config
    /// Effective wheel diameter, measured from two straight line tests
    constexpr double WHEEL_DIAMETER = 2.65;                                                   ///< Effective wheel diameter in inches (calibrated; nominal is WHEEL_2_75_INCH = 2.76)
    constexpr double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * M_PI;                             ///< Wheel circumference in inches
    constexpr double TICKS_PER_ROTATION = Units::Constants::V5_BLUE_MOTOR_TICKS_PER_ROTATION; ///< Encoder ticks per rotation for blue cartridges
    constexpr double GEAR_RATIO = 36.0 / 48.0;                                                ///< Gear ratio of the drivetrain
    // Measured 2026-09-18, not the physical wheel spacing: 17 per-tick regressions of
    // wheel differential against IMU rate (metrics.effective_track_width) across logs
    // 128/130/132 give 11.30 sample-weighted, 11.40 median, 11.39 over the r^2>0.95
    // subset -- the same answer in every session (crawl_R3 returned 11.16 three times
    // at r^2 >= 0.99). Scrub makes the chassis rotate more per unit of wheel
    // differential than the 12.0 in track suggests, so 12.0 over-commanded every turn
    // by 6.2%.
    //
    // Expect this to make the ARCS under-rotate slightly MORE at first: the old 12.0
    // was masking a 12-17% wheel-velocity delivery shortfall, and removing the
    // over-command exposes it. What it fixes is the other end -- crawl_R3 (104%) and
    // vee_reverse (113%), the routes that track well enough to over-rotate. The
    // delivery shortfall is a velocity-loop problem and has to be fixed there.
    constexpr double WHEEL_BASE_WIDTH = 11.35;

    // Tracking wheels (V5 rotation sensors, 1:1 on the wheel). Odometry takes translation
    // from these and heading from the IMU; the drive encoders still supply the per-side
    // velocities the controllers use, and stand in for a tracking wheel that stops reading.
    //
    // Calibrate a diameter by moving the robot a measured distance along the tiles (or
    // sideways, for the lateral wheel) and scaling it by measured / reported distance.
    // Vertical: boot 994 (cruise_8ft, ended at rest) read 97.59 in at 2.016 for 94.25 in
    // tape-measured, so 2.016 * 94.25 / 97.59. The earlier 2.016 came from boot 989, whose
    // log ended with the chassis still sliding, so the tape included travel the wheel never
    // logged. Calibrate only from runs that end stopped. The lateral is nominal.
    constexpr double VERTICAL_TRACKING_WHEEL_DIAMETER = 1.947; ///< Forward/back wheel, inches (nominal 2 in omni)
    constexpr double LATERAL_TRACKING_WHEEL_DIAMETER = Units::Constants::WHEEL_2_75_INCH; ///< Sideways wheel, inches. TUNE
    // Signed offsets from the tracking centre (the drive's centre of rotation, the point
    // the pose, RAMSETE and the distance-sensor offsets all refer to). Measure them with the
    // suite's spin_w* routes (tests: metrics.tracking_wheel_offsets): in a pure spin each
    // wheel rolls offset * radians. They move with the pivot, i.e. with the traction wheels.
    //
    // Lateral: 2.34-2.39 in over spin_w1..w4 in boot 1001 (double traction), both spin
    // directions within 0.14 in. The 3.0 in guess it replaced showed every turn as 0.63 in
    // per radian of sideways motion that did not happen, the goal sat ~0.5 in "ahead" all
    // through a spin, and RAMSETE drove the robot forward ~4 in per revolution to close it.
    // Vertical: about +0.1 in once that creep was fixed (boot 1004); left at 0.
    //
    // Single traction, 2026-10-09: the front traction pair (3.5 in centre to centre from
    // the rear pair) was removed, so the drive now pivots on the rear traction wheels,
    // 1.75 in behind the old pivot. The tracking centre (this point, the pose, RAMSETE and
    // the distance-sensor offsets in main.cpp) moved with it: 2.37 + 1.75. Boot 1007's
    // spins on the old value read 4.03-4.58 in and crept ~2.4 in per radian. Confirm with
    // spin_w2 / spin_w4 on this build. Route nodes placed by contact (goals, loaders) now
    // refer to a point 1.75 in further back on the robot.
    constexpr double VERTICAL_WHEEL_OFFSET = 0.0; ///< Vertical wheel's sideways offset, inches, +left
    constexpr double LATERAL_WHEEL_OFFSET = 4.12; ///< Lateral wheel's fore/aft offset, inches, +forward (sits in front)
    /// A rotation sensor can hold one value across ticks and then report all their travel
    /// at once, so a tracking wheel's implausible-delta gate spans the time since its
    /// reading last changed, capped at this many ticks so a jump after standing still is
    /// still caught. At the default 20 ms data rate every fresh reading at cruise was a
    /// two-tick delta that the one-tick gate rejected (boot 989: pose 69 in, real 116 in).
    constexpr int TRACKING_WHEEL_MAX_STALE_TICKS = 3;

    /// Traction (non-omni) wheels per drive side. Logged on the "Build config" line so the
    /// double vs single traction A/B (tests/traction_ab.py) can tell its logs apart. Change
    /// it in the same build as the rebuild; the pivot, and so the tracking-wheel and
    /// distance-sensor offsets, move with it.
    constexpr int TRACTION_WHEELS_PER_SIDE = 1; ///< Rear pair only since 2026-10-09 (boot 1007 on)

    // Odometry encoder selection
    constexpr int LEFT_ODOM_MOTOR_INDEX = -1; ///< Left pair agrees to 0.03%; average both
    constexpr int RIGHT_ODOM_MOTOR_INDEX = 1; ///< Port 15 only; port 9's encoder is faulty

    // Velocity Controller Constants

    // Measured 2026-09-18 from 172 clean log samples (unsaturated, velocity loop
    // converged, near-constant speed, 4-30 in/s) once KS_ANGULAR/KV_ANGULAR were
    // carrying the turning load: the feedforward still came up ~10 V short, and the
    // shortfall was FLAT against speed (R^2 0.08 for a kV term, 0.00 for a constant
    // -- i.e. no speed structure at all). That is a static term, not a velocity one,
    // so kS absorbs it and KV stays at its free-spin value. The earlier reading of
    // "kV should be 2.7-3.1" was the turning asymmetry, which the angular
    // feedforward now handles; fitting it into kV would have over-driven the inner
    // wheel by as much as it helped the outer.
    constexpr double KS = 18.0; ///< Static friction compensation (motor units)
    constexpr double KV = 2.0;  ///< Velocity feedforward gain (motor units)/(in/s)
    constexpr double KA = 0.4;  ///< Acceleration feedforward gain (motor units)/(in/s^2)

    /// Asymetrical decel constant to improve tracking since friction + back EMF means we can brake quicker than accel
    constexpr double KA_DECEL = 0.34; ///< Acceleration feedforward while braking

    constexpr double KP = 4.0;  ///< Proportional gain for velocity control
    constexpr double KI = 5.00; ///< Integral gain for velocity control
    constexpr double KD = 0.0;  ///< Derivative gain for velocity control

    // Angular (turning-scrub) feedforward
    // Prevents underturning from wheel scrub + inertia from turning
    //
    // Retuned 2026-10-09 (double traction) from 12 / 12, which supplied ~32 V per wheel at
    // 1.7 rad/s. Measured beyond the straight-line feedforward on steady arcs (boots
    // 1001/1002/1004, ~1.7 rad/s): outer wheel +14 V, inner -21 V, so ~17.5 V each way; at
    // 12 / 12 arc_R12 / arc_R24 over-turned 10-16% and the outer wheel sat at 127 V for
    // 16-32% of the arc. 10 + 4 * 1.7 = 16.8 V. Spins need less (~11 V at 2 rad/s,
    // suite spin_w*) and get ~18 V here; feedback trims the difference. The split between
    // the two terms is a guess: every arc ran near the 1.8 rad/s cap, so an arc-rate sweep
    // is needed to separate them. Re-fit after the single-traction rebuild.
    constexpr double KV_ANGULAR = 4.0; ///< Angular velocity feedforward (motor units)/(rad/s)

    /// Coulomb (breakaway) component of the same scrub torque
    constexpr double KS_ANGULAR = 10.0; ///< Angular static friction (motor units)

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
    constexpr double IMU_HEADING_SCALE = 3600.0 / 3573; ///< ~1.00873

    /// Longest Robot::followTrajectory waits for the IMU to finish calibrating before it
    /// starts. Calibration takes ~2 s from imu.reset() in initialize(); runs started sooner
    /// (boots 655, 762, 768, 817) drove blind on a frozen pose. If it times out, the
    /// follower still holds the drive until odometry has a valid baseline.
    constexpr uint32_t IMU_CALIBRATION_TIMEOUT_MS = 3000;

    /// Encoder-correction gain for the wheel-velocity observer (VelocityObserver).
    /// Basically a GH filter that uses the feedforward model we use for velo control.
    ///
    /// Lowered 0.5 -> 0.2 on 2026-09-18. The encoder gives 0.0104 in per count with the
    /// two motors averaged, so at a 10 ms sample the raw velocity is quantised to
    /// 1.04 in/s, and this gain is how much of that quantisation reaches the velocity
    /// loop. At 0.5 it reached a lot: in log 09182026-132, 12-29% of measured velocity
    /// steps were larger than a wheel can physically achieve at full voltage
    /// ((127 - kS)/kA * 0.01 = 2.72 in/s per tick), those steps alternated sign 39-51%
    /// of the time against 2-16% for the setpoints, and kP = 4.0 turned the residue into
    /// 2.2-10.2 V of command built from noise -- 4-10% of the applied voltage, and
    /// 54-72% of all tick-to-tick voltage movement came from feedback rather than
    /// feedforward.
    ///
    /// Trusting the model more is defensible NOW in a way it was not before: the
    /// feedforward it predicts with is accurate to +-2.5 V since kS was corrected to 18
    /// (it was +-10-17 V out), and VELOCITY_OBSERVER_DISTURBANCE_GAIN below already
    /// absorbs whatever steady error is left, so the gain only has to cover model
    /// transients. Raise it again if the estimate visibly lags a real disturbance --
    /// hitting a goal, a stall -- rather than to chase noise.
    ///
    /// Safe for odometry: Odometry::update integrates deltaLeft/deltaRight directly for
    /// pose. This gain only touches the velocity fed to DrivetrainController and the
    /// acceleration derived from it.
    constexpr double VELOCITY_OBSERVER_GAIN = 0.2;

    /// Integral gain on the observer's residual, in (motor units)/(in/s) per tick. Learns
    /// the voltage the kS/kV/kA model is missing, so a constant model error does not leave
    /// a constant velocity bias.
    constexpr double VELOCITY_OBSERVER_DISTURBANCE_GAIN = 0.15;

    // Distance-sensor wall resets (DistanceResetOdometry)
    /// Distance from the field centre to the inside face of each wall, in inches.
    constexpr double DISTANCE_RESET_WALL_POSITION = 70.25;

    /// Largest angle, in degrees, between the sensor beam and the wall's normal that a
    /// reset accepts. Past this the heading error is magnified by d*tan(angle) and the
    /// beam can clip the neighbouring wall. Logs up to 2026-09-28: median 1.9 deg,
    /// 90th percentile 11.3 deg; every reset past 20 deg was one of the bogus jumps.
    constexpr double DISTANCE_RESET_MAX_BEAM_ANGLE_DEG = 20.0;

    /// Largest correction, in inches, one reset may apply to X or Y. A larger jump means
    /// the beam most likely hit a game piece, field element or robot instead of the wall.
    /// TUNE: logs up to 2026-09-28 have ~45 resets above this, including a recurring
    /// 20-30 in Y correction in state.txt that may be genuine drift.
    constexpr double DISTANCE_RESET_MAX_CORRECTION = 12.0;

    /// Readings from the last this-many ms are combined into one median per reset. Every
    /// localization update samples each reset sensor, so this is ~10 ticks. The sensor may
    /// refresh slower than 10 ms, in which case some of those ticks repeat a value.
    constexpr uint32_t DISTANCE_RESET_WINDOW_MS = 100;

    /// Fewest valid (object-seen) readings in the window for a reset to go ahead.
    constexpr std::size_t DISTANCE_RESET_MIN_SAMPLES = 5;

    /// Largest interquartile range of the window's readings, in inches:
    ///   BASE + FRACTION * median reading.
    /// The median already ignores a lone spike; a wide IQR means many readings disagree:
    /// the beam is crossing an edge, hitting something moving, or the robot moved. It
    /// scales with distance because the sensor's error does (spec ~5% past 200 mm).
    /// Was a flat 1.0 in until boot 514, whose front reset at 37.4 in had an IQR of 1.06
    /// while the right sensor at 17.6 in had 0.16. TUNE from the Distance window lines.
    constexpr double DISTANCE_RESET_MAX_SPREAD_BASE = 0.5;
    constexpr double DISTANCE_RESET_MAX_SPREAD_FRACTION = 0.03;

    /// Largest position error (in) a reset may pick up from the robot moving while its
    /// readings were taken. The reset uses the median of the last DISTANCE_RESET_WINDOW_MS,
    /// so it lags by half the window; over that lag, motion toward/away from the wall and
    /// turning (times how strongly the heading moves that sensor's wall distance) become
    /// error. Driving along a wall costs nothing, so a side sensor can reset on the move
    /// (the matchloader). Straight at the wall this allows 3 in/s, the old flat limit.
    /// Wheel-measured motion only: sliding against a goal is invisible to it, which is why
    /// a reset with a wait_lift hold still waits for DISTANCE_RESET_STILL_* instead.
    constexpr double DISTANCE_RESET_MAX_MOTION_ERROR = 0.15;

    /// "Still" for a reset retried during a wait_lift hold: there is time to let contact
    /// with the goal settle, which the wheels cannot measure. Linear in in/s (mean of the
    /// wheel velocities), angular in rad/s (wheel difference over WHEEL_BASE_WIDTH).
    /// Were the only gate (DISTANCE_RESET_MAX_LINEAR/ANGULAR_SPEED) until 2026-09-30.
    constexpr double DISTANCE_RESET_STILL_LINEAR_SPEED = 3.0;
    constexpr double DISTANCE_RESET_STILL_ANGULAR_SPEED = 0.25;
}

#endif
