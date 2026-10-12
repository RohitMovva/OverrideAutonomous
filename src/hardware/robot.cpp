#include "hardware/robot.hpp"
#include "controllers/pid_controller.hpp"
#include "utilities/math/units.hpp"
#include "utilities/math/angle.hpp"
#include "navigation/distance_reset_odometry.hpp"
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <stdexcept>
#include <vector>

namespace
{
    /// Tolerance for matching a route's lift action code. The codes are spaced 0.1 apart at
    /// their closest, and the values arrive as float, so anything between ~1e-6 and 0.05
    /// works; 1e-3 leaves room on both sides.
    constexpr double LIFT_ACTION_EPSILON = 1e-3;

    /// Action-point slots, in the planner's config.yaml robot.actions order. Both repos
    /// have to agree on this; the shared spec is VexAutonomousPlanner/docs/action-slots.md.
    enum ActionSlot : std::size_t
    {
        SLOT_INTAKE = 0,
        SLOT_LIFT_STATE = 1,
        SLOT_RESET_RIGHT = 2, ///< config.yaml right_reset
        SLOT_RESET_LEFT = 3,  ///< config.yaml left_reset
        SLOT_RESET_FRONT = 4,
        SLOT_LIFT_HEIGHT = 5, ///< winch deg above home; see Lift::raise_to
        SLOT_INTAKE_LIFT = 6, ///< intake solenoid; see INTAKE_LIFT_*
        SLOT_ROLLER = 7,      ///< roller mech on/off; see ROLLER_*
        SLOT_WAIT_LIFT = 8,   ///< hold until the lift is done; see LIFT_WAIT_*
    };

    /// lift_state codes. 0 is no command, and the height slot carries a raise instead.
    constexpr double LIFT_ACTION_HOME = -1.0; ///< back to the matchload pose (the only rest pose)
    constexpr double LIFT_ACTION_SCORE = 1.0;
    constexpr double LIFT_ACTION_MATCHLOAD = 2.0;
    constexpr double LIFT_ACTION_GRAB = 3.0; ///< hold the claw closed in the matchload pose, no raise
    /// Was matchload at the vertical-stack clamp width. The clamp has one matchload width,
    /// so it is plain matchload now; kept so routes that carry it still run.
    constexpr double LIFT_ACTION_MATCHLOAD_STACK = 4.0;

    /// intake_lift codes. 0 is no command rather than "down": every point carries every
    /// slot, so a 0-means-down would drop a raised intake at the next action point.
    constexpr double INTAKE_LIFT_UP = 1.0;
    constexpr double INTAKE_LIFT_DOWN = -1.0;

    /// roller codes. 0 is no command, so the mech stays as the last point that set it.
    /// The colour it turns to is RollerMech's target, set in main, not the route's.
    constexpr double ROLLER_ENABLE = 1.0;
    constexpr double ROLLER_DISABLE = -1.0;

    /// wait_lift code. 0 is no wait.
    constexpr double LIFT_WAIT_ON = 1.0;
    /// Longest a wait_lift hold lasts before the route moves on regardless, so a jammed
    /// lift cannot eat the rest of auton. Matches Lift::HEIGHT_TIMEOUT_MS.
    constexpr uint32_t LIFT_WAIT_TIMEOUT_MS = 4000;

    /// Robot::m_pendingResets bits
    constexpr uint8_t RESET_PENDING_LEFT = 1 << 0;
    constexpr uint8_t RESET_PENDING_RIGHT = 1 << 1;
    constexpr uint8_t RESET_PENDING_FRONT = 1 << 2;

    struct PendingReset
    {
        uint8_t bit;
        const char *name;
        DistanceResetOdometry::ResetResult (DistanceResetOdometry::*reset)(bool);
    };
    const PendingReset PENDING_RESETS[] = {
        {RESET_PENDING_LEFT, "left", &DistanceResetOdometry::resetLeft},
        {RESET_PENDING_RIGHT, "right", &DistanceResetOdometry::resetRight},
        {RESET_PENDING_FRONT, "front", &DistanceResetOdometry::resetFront},
    };
    /// in/s. A hold only starts if the sample before the point is slower than this: the
    /// planner stops at every wait_lift node (worst export ~1 in/s from 10 ms sampling),
    /// and holding at speed would slam the drive to a stop.
    constexpr double LIFT_WAIT_MAX_SPEED = 3.0;

    const char *fmtVal(char *buf, std::size_t n, double v, const char *fmt)
    {
        if (n == 0) return "";
        const bool unreadable = !std::isfinite(v) ||
                                std::abs(v) >= static_cast<double>(INT32_MAX);
        if (unreadable)
        {
            std::snprintf(buf, n, "--");
        }
        else
        {
            std::snprintf(buf, n, fmt, v);
        }
        return buf;
    }

    template <typename T>
    double at(const std::vector<T> &v, std::size_t i)
    {
        return i < v.size() ? static_cast<double>(v[i])
                            : std::numeric_limits<double>::quiet_NaN();
    }

    void logSideHealth(const char *context, const char *side, pros::MotorGroup &group)
    {
        const std::vector<double> temp = group.get_temperature_all();
        const std::vector<std::int32_t> cur = group.get_current_draw_all();
        const std::vector<double> pwr = group.get_power_all();
        const std::vector<double> eff = group.get_efficiency_all();
        const std::vector<std::int32_t> ovt = group.is_over_temp_all();
        const std::vector<std::int8_t> ports = group.get_port_all();

        for (std::size_t i = 0; i < ports.size(); ++i)
        {
            char t[12], c[12], w[12], e[12], o[8];
            Logger::getInstance()->log(
                "DRIVEHEALTH %s %s%u port=%d tempC=%s curMA=%s W=%s eff=%s ovt=%s",
                context, side, static_cast<unsigned>(i),
                static_cast<int>(ports[i]),
                fmtVal(t, sizeof(t), at(temp, i), "%.0f"),
                fmtVal(c, sizeof(c), at(cur, i), "%.0f"),
                fmtVal(w, sizeof(w), at(pwr, i), "%.1f"),
                fmtVal(e, sizeof(e), at(eff, i), "%.0f"),
                fmtVal(o, sizeof(o), at(ovt, i), "%.0f"));
        }
    }
}

Robot::Robot()
    : m_leftDrivetrain(nullptr), m_rightDrivetrain(nullptr), m_inertial(nullptr), m_driveController(nullptr), m_ramseteController(nullptr), m_localization(nullptr), m_isFollowingTrajectory(false), m_intake(nullptr), m_lift(nullptr), m_roller(nullptr)
{
    // Default constructor initializes all pointers to null
    // Robot will not be functional until proper initialization
}

Robot::Robot(pros::MotorGroup *leftDrivetrain,
             pros::MotorGroup *rightDrivetrain,
             pros::Imu *inertial,
             pros::Controller *controller,
             DrivetrainController *driveController,
             RamseteController *ramseteController,
             std::unique_ptr<ILocalization> localization,
             Intake *intake,
             Lift *lift,
             RollerMech *roller)
    : m_leftDrivetrain(leftDrivetrain), m_rightDrivetrain(rightDrivetrain), m_controller(controller), m_inertial(inertial), m_driveController(driveController), m_ramseteController(ramseteController), m_localization(std::move(localization)), m_isFollowingTrajectory(false),m_intake(intake), m_lift(lift), m_roller(roller)
{
    // Validate all pointer parameters
    if (!leftDrivetrain)
    {
        throw std::invalid_argument("Left drivetrain pointer cannot be null");
    }
    if (!rightDrivetrain)
    {
        throw std::invalid_argument("Right drivetrain pointer cannot be null");
    }
    if (!inertial)
    {
        throw std::invalid_argument("Inertial sensor pointer cannot be null");
    }
    if (!controller)
    {
        throw std::invalid_argument("Controller pointer cannot be null");
    }
    if (!driveController)
    {
        throw std::invalid_argument("Drive controller pointer cannot be null");
    }
    if (!ramseteController)
    {
        throw std::invalid_argument("Ramsete controller pointer cannot be null");
    }
    if (!m_localization)
    {
        throw std::invalid_argument("Localization object cannot be null");
    }
    if (!lift)
    {
        throw std::invalid_argument("Lift pointer cannot be null");
    }
    m_feedbackEnabled = true;
}

Robot::~Robot()
{
    // Stop any active trajectory following before destruction
    if (isFollowingTrajectory())
    {
        stopTrajectory();
    }

    // Note: We don't delete the raw pointers as this class doesn't own them
    // The localization object is automatically cleaned up via unique_ptr
    // The calling code is responsible for managing the lifetime of other objects
}

bool Robot::isFollowingTrajectory() const
{
    return m_isFollowingTrajectory;
}

void Robot::stopTrajectory()
{
    if (!isInitialized())
    {
        return;
    }

    // This flag is checked in the trajectory following logic
    m_isFollowingTrajectory = false;

    // Stop both drivetrain sides
    if (m_leftDrivetrain)
    {
        m_leftDrivetrain->move_voltage(0);
    }
    if (m_rightDrivetrain)
    {
        m_rightDrivetrain->move_voltage(0);
    }

    // Otherwise the velocity observer keeps predicting forward off the last nonzero
    // voltage it was told about.
    if (m_localization)
    {
        m_localization->setDriveVoltages(0.0, 0.0);
    }
}

auto Robot::getCurrentPosition() const -> decltype(m_localization->getPose())
{
    if (!m_localization)
    {
        throw std::runtime_error("Localization system not initialized");
    }

    return m_localization->getPose();
}

void Robot::resetPosition(const Pose &position)
{
    if (!m_localization)
    {
        throw std::runtime_error("Localization system not initialized");
    }

    // Stop any active trajectory following before resetting position
    if (isFollowingTrajectory())
    {
        stopTrajectory();
    }

    m_localization->setPose(position);
}

bool Robot::isInitialized() const
{
    return (m_leftDrivetrain != nullptr &&
            m_rightDrivetrain != nullptr &&
            m_inertial != nullptr &&
            m_driveController != nullptr &&
            m_ramseteController != nullptr &&
            m_localization != nullptr);
}

void Robot::logDriveHealth(const char *context)
{
    if (m_leftDrivetrain == nullptr || m_rightDrivetrain == nullptr) return;

    static uint32_t last_sample_ms = 0;
    const uint32_t now = pros::millis();
    if (now - last_sample_ms < DRIVE_HEALTH_LOG_MS) return;
    last_sample_ms = now;

    logSideHealth(context, "L", *m_leftDrivetrain);
    logSideHealth(context, "R", *m_rightDrivetrain);

    // Battery, once per sample rather than once per motor: it is a property of the
    // robot, not of a motor, and repeating it on all four lines would pad the log by
    // ~30% for no extra information.
    //
    // Logged because without it no two sessions can be compared. Runs 09182026-128 and
    // -130 were the same routes on the same firmware, and an untouched route (hop_30in)
    // landed 0.17 in from its goal in one and 1.26 in short in the other, with no pose
    // jumps, resets or loop overruns to explain it -- just less peak drive power (13.5
    // -> 10.1 W on arc_R12) for the same commanded work. A depleted pack cannot deliver
    // the feedforward's volts, so kS/kV look wrong when only the battery has moved.
    // Anything tuned against an unlogged battery is tuned against noise.
    char mv[12], ma[12], pct[12], tc[12];
    Logger::getInstance()->log(
        "DRIVEHEALTH %s BATT mV=%s mA=%s pct=%s tempC=%s",
        context,
        fmtVal(mv, sizeof(mv), static_cast<double>(pros::battery::get_voltage()), "%.0f"),
        fmtVal(ma, sizeof(ma), static_cast<double>(pros::battery::get_current()), "%.0f"),
        fmtVal(pct, sizeof(pct), pros::battery::get_capacity(), "%.0f"),
        fmtVal(tc, sizeof(tc), pros::battery::get_temperature(), "%.0f"));
}

void Robot::processTrajectory(const TrajectoryPoint &tp)
{
    // Process trajectory
    Pose current_pose = m_localization->getPose();

    Logger::getInstance()->log("Pose: %f %f %f", current_pose.x, current_pose.y, current_pose.theta);
    Logger::getInstance()->log("Goal: %f %f %f", tp.x, tp.y, tp.theta);

    // Calculate accelerations using next waypoint
    double left_accel = tp.linear_accel - (tp.angular_accel * Config::WHEEL_BASE_WIDTH / 2.0);
    double right_accel = tp.linear_accel + (tp.angular_accel * Config::WHEEL_BASE_WIDTH / 2.0);

    // Get RAMSETE controller output
    auto ramsete_output = m_ramseteController->calculate(
        current_pose.x, current_pose.y, current_pose.theta,
        tp.x, tp.y, tp.theta,
        tp.linear_vel, tp.angular_vel);
    Logger::getInstance()->log("Ramsete Output: %f %f", ramsete_output[0], ramsete_output[1]);
    Logger::getInstance()->log("MP Output: %f %f", tp.linear_vel, tp.angular_vel);

    // Convert RAMSETE output to wheel velocities using configuration
    std::vector<double> wheel_velocities;
    if (m_feedbackEnabled){
      wheel_velocities = m_ramseteController->calculate_wheel_velocities(
            ramsete_output[0],              // linear velocity
            ramsete_output[1],              // angular velocity
            Config::WHEEL_BASE_WIDTH,       // track width from config
            Config::MAX_WHEEL_VELOCITY      // desaturate to what the motors can hold
            );
    } else {
        wheel_velocities = m_ramseteController->calculate_wheel_velocities(
            tp.linear_vel,                  // linear velocity
            tp.angular_vel,                 // angular velocity
            Config::WHEEL_BASE_WIDTH,       // track width from config
            Config::MAX_WHEEL_VELOCITY      // desaturate to what the motors can hold
            );
    }
    
    Logger::getInstance()->log("Left wheel velocities: %f %f", wheel_velocities[0], m_localization->getLeftVelocity().linear);
    Logger::getInstance()->log("Right wheel velocities: %f %f", wheel_velocities[1], m_localization->getRightVelocity().linear);

    // Calculate motor voltages using drivetrain controller

    DrivetrainController::MotorVoltages voltages;
    if (m_feedbackEnabled) {
        voltages = m_driveController->calculateVoltages(
            wheel_velocities[0],                       // left velocity setpoint
            wheel_velocities[1],                       // right velocity setpoint
            m_localization->getLeftVelocity().linear,  // current left velocity
            m_localization->getRightVelocity().linear, // current right velocity
            left_accel,                                // left acceleration
            right_accel                                // right acceleration
        );
    } else {
        voltages = m_driveController->calculateVoltages(
            wheel_velocities[0],                       // left velocity setpoint
            wheel_velocities[1],                       // right velocity setpoint
            wheel_velocities[0],                       // current left velocity
            wheel_velocities[1],                       // current right velocity
            left_accel,                                // left acceleration
            right_accel                                // right acceleration
        );
    }

    // Sometimes robot needs to apply some constant force when stopped (e.g. sitting in a matchloader last year, might be useful this year)
    if (m_min_voltage != 0)
    {
        voltages.left = static_cast<int>(wheel_velocities[0]);
        voltages.right = static_cast<int>(wheel_velocities[1]);
    }
    Logger::getInstance()->log("Voltages: %d %d", voltages.left, voltages.right);

    // Apply voltages to motors
    m_leftDrivetrain->move(voltages.left);
    m_rightDrivetrain->move(voltages.right);

    // Hand the applied voltages to localization. The velocity observer predicts the next
    // tick's wheel speeds from these and the kS/kV/kA model, so it must see what the
    // motors were actually told
    m_localization->setDriveVoltages(voltages.left, voltages.right);
}

void Robot::updateRoller()
{
    if (!m_roller)
        return;
    // Lowest priority on the PTO, as in opcontrol. Ticked after the intake so a stopped
    // intake releasing the motors cannot cut a roller cycle short on the same tick.
    m_roller->set_inhibited((m_lift && m_lift->wants_pto()) ||
                            (m_intake && m_intake->get_state() != 0));
    m_roller->update();
}

void Robot::processAction(const ActionPoint &ap)
{
    Logger::getInstance()->log("Executing action with %zu parameters", ap.actions.size());

    for (const auto &action : ap.actions)
    {
        Logger::getInstance()->log("Action: %f", action);
    }

    // Slots the line does not carry read as 0 (no command), so a trajectory exported
    // before a slot existed still follows.
    const auto slot = [&ap](std::size_t index)
    {
        return index < ap.actions.size() ? static_cast<double>(ap.actions[index]) : 0.0;
    };
    const auto is = [](double value, double code)
    { return std::fabs(value - code) < LIFT_ACTION_EPSILON; };

    m_intake->set_state(static_cast<int>(slot(SLOT_INTAKE)));

    const double intake_lift = slot(SLOT_INTAKE_LIFT);
    if (is(intake_lift, INTAKE_LIFT_UP))
    {
        m_intake->raise_lift();
    }
    else if (is(intake_lift, INTAKE_LIFT_DOWN))
    {
        m_intake->lower_lift();
    }
    else if (intake_lift != 0.0)
    {
        Logger::getInstance()->logError("Intake lift action %f not implemented", intake_lift);
    }

    const double roller = slot(SLOT_ROLLER);
    if (roller != 0.0)
    {
        if (!m_roller)
        {
            Logger::getInstance()->logError("Roller action %f ignored: no roller mech", roller);
        }
        else if (is(roller, ROLLER_ENABLE))
        {
            // Also on a point where the mech is already enabled: it marks where the
            // route wants the PTO in INTAKE mode ahead of the next roller.
            m_roller->set_enabled(true);
            m_roller->prime_pto();
        }
        else if (is(roller, ROLLER_DISABLE))
        {
            m_roller->set_enabled(false);
        }
        else
        {
            Logger::getInstance()->logError("Roller action %f not implemented", roller);
        }
    }

    const double lift_state = slot(SLOT_LIFT_STATE);
    const double lift_height = slot(SLOT_LIFT_HEIGHT);
    if (lift_state != 0.0)
    {
        // A score may carry a height: score there without the tower sensor (Lift::score_at).
        // Any other lift_state owns the point and the height is dropped.
        const bool score_at_height = is(lift_state, LIFT_ACTION_SCORE) && lift_height != 0.0;
        if (lift_height != 0.0 && !score_at_height)
        {
            Logger::getInstance()->logError("lift_height %.1f ignored: lift_state %.1f owns the point",
                                            lift_height, lift_state);
        }

        if (is(lift_state, LIFT_ACTION_HOME))
        {
            m_lift->return_home();
        }
        else if (score_at_height)
        {
            m_lift->score_at(lift_height);
        }
        else if (is(lift_state, LIFT_ACTION_SCORE))
        {
            m_lift->score();
        }
        else if (is(lift_state, LIFT_ACTION_MATCHLOAD) ||
                 is(lift_state, LIFT_ACTION_MATCHLOAD_STACK))
        {
            m_lift->matchload();
        }
        else if (is(lift_state, LIFT_ACTION_GRAB))
        {
            m_lift->grab();
        }
        else
        {
            Logger::getInstance()->logError("Lift action %f not implemented", lift_state);
        }
    }
    else if (lift_height != 0.0)
    {
        m_lift->raise_to(lift_height);
    }

    // After the lift command above, so a point can both start a move and wait on it.
    // The hold itself runs in followTrajectory().
    const double wait_lift = slot(SLOT_WAIT_LIFT);
    if (is(wait_lift, LIFT_WAIT_ON))
    {
        if (!m_lift)
        {
            Logger::getInstance()->logError("wait_lift ignored: no lift attached");
        }
        else if (m_lastTrajectoryPoint &&
                 std::fabs(m_lastTrajectoryPoint->linear_vel) > LIFT_WAIT_MAX_SPEED)
        {
            Logger::getInstance()->logError("wait_lift ignored: moving at %.1f in/s; stop at this point",
                                            m_lastTrajectoryPoint->linear_vel);
        }
        else
        {
            m_liftWaitPending = true;
            m_liftWaitStartMs = pros::millis();
            Logger::getInstance()->log("wait_lift: holding for the lift (%s)",
                                       Lift::state_name(m_lift->state()));
        }
    }
    else if (wait_lift != 0.0)
    {
        Logger::getInstance()->logError("wait_lift action %f not implemented", wait_lift);
    }

    const auto resetOdometry = [this](const PendingReset &which)
    {
        auto *odometry = dynamic_cast<DistanceResetOdometry *>(m_localization.get());
        if (!odometry)
        {
            Logger::getInstance()->logError("Failed to cast to DistanceResetOdometry (%s reset)", which.name);
            return;
        }
        // wait_lift is handled above, so a hold starting at this point is already
        // pending. With a hold there is time: wait for the robot to be still, since
        // arriving (often pivoting onto a goal) moves it in ways the wheels don't see.
        // Without one (a side sensor at a touch-and-go node like the matchloader), try
        // now; the motion gate lets it through when the motion barely changes the reading.
        if (m_liftWaitPending)
        {
            m_pendingResets |= which.bit;
            Logger::getInstance()->log("Distance reset (%s) waits for the robot to be still in the hold", which.name);
            return;
        }
        (odometry->*which.reset)(true);
    };

    if (slot(SLOT_RESET_LEFT) == 1.0)
    {
        resetOdometry(PENDING_RESETS[0]);
    }
    if (slot(SLOT_RESET_RIGHT) == 1.0)
    {
        resetOdometry(PENDING_RESETS[1]);
    }
    if (slot(SLOT_RESET_FRONT) != 0.0)
    {
        resetOdometry(PENDING_RESETS[2]);
    }
}

void Robot::retryPendingResets()
{
    if (m_pendingResets == 0)
    {
        return;
    }
    auto *odometry = dynamic_cast<DistanceResetOdometry *>(m_localization.get());
    // Only attempt once still. Still is the stricter test here, so the reset skips its
    // motion-error gate and every pending reset gets exactly one try.
    if (!odometry || !odometry->isStill())
    {
        return;
    }
    const uint32_t waited_ms = pros::millis() - m_liftWaitStartMs;
    for (const PendingReset &pending : PENDING_RESETS)
    {
        if (!(m_pendingResets & pending.bit))
        {
            continue;
        }
        Logger::getInstance()->log("Distance reset (%s) %u ms into the hold", pending.name, waited_ms);
        (odometry->*pending.reset)(false);
        m_pendingResets &= ~pending.bit;
    }
}

void Robot::dropPendingResets(const char *why)
{
    for (const PendingReset &pending : PENDING_RESETS)
    {
        if (m_pendingResets & pending.bit)
        {
            Logger::getInstance()->logWarning("Distance reset (%s) skipped: robot never still before %s",
                                              pending.name, why);
        }
    }
    m_pendingResets = 0;
}

bool Robot::followTrajectory(Trajectory &trajectory, std::optional<Pose> start_pose)
{
    Logger::getInstance()->log("Starting trajectory following");
    if (!isInitialized())
    {
        Logger::getInstance()->logError("Robot is not initialized, cannot follow trajectory");
        return false;
    }
    
    Logger::getInstance()->log(
        "Build config: wheelBase=%.3f wheelDia=%.3f kS=%.3f kV=%.3f kA=%.3f "
        "kADec=%.3f kP=%.3f kI=%.3f kD=%.3f kSAng=%.3f kVAng=%.3f "
        "ramseteB=%.3f ramseteZeta=%.3f maxWheelVel=%.3f tractionWheels=%d",
        Config::WHEEL_BASE_WIDTH, Config::WHEEL_DIAMETER,
        m_driveController->getKS(), m_driveController->getKV(),
        m_driveController->getKA(), m_driveController->getKADecel(),
        m_driveController->getKP(),
        m_driveController->getKI(), m_driveController->getKD(),
        m_driveController->getKSAngular(), m_driveController->getKVAngular(),
        m_ramseteController->getB(), m_ramseteController->getZeta(),
        Config::MAX_WHEEL_VELOCITY, Config::TRACTION_WHEELS_PER_SIDE);
    if (trajectory.empty())
    {
        Logger::getInstance()->logError("Trajectory is empty, cannot follow");
        return false;
    }
    trajectory.reset();

    m_isFollowingTrajectory = true;
    m_liftWaitPending = false;
    m_pendingResets = 0;
    m_lastTrajectoryPoint.reset();

    Logger::getInstance()->log("Following trajectory with %s localization",
                               m_localization->getTypeName().c_str());

    // imu.reset() in initialize() does not block. Odometry cannot integrate until the IMU
    // has calibrated, and a robot that moves while it calibrates gets a bad gyro bias, so
    // wait here with the drive still. Instant on every run but the first after boot.
    const uint32_t imu_wait_start = pros::millis();
    while (m_inertial->is_calibrating() &&
           pros::millis() - imu_wait_start < Config::IMU_CALIBRATION_TIMEOUT_MS)
    {
        pros::delay(10);
    }
    const uint32_t imu_waited_ms = pros::millis() - imu_wait_start;
    if (m_inertial->is_calibrating())
    {
        Logger::getInstance()->logError("IMU still calibrating after %u ms; starting anyway, "
                                        "drive holds until odometry is valid", imu_waited_ms);
    }
    else if (imu_waited_ms > 0)
    {
        Logger::getInstance()->log("Waited %u ms for IMU calibration", imu_waited_ms);
    }

    uint32_t loop_start_time = pros::millis();

    m_localization->reset();
    // Same reason localization is reset here: the drive PID's integrals, previous errors
    // and dt clock are all carried from construction in initialize(), seconds ago and
    // possibly a whole previous segment ago. Reset alongside it so a run starts from the
    // same controller state every time.
    if (m_driveController)
    {
        m_driveController->reset();
    }
    size_t trajectory_index = 0;
    try
    {
        const TrajectoryPoint &first_trajectory_point = std::get<TrajectoryPoint>(trajectory.getByIndex(1));
        
        Pose initial_pos = {first_trajectory_point.x, first_trajectory_point.y, first_trajectory_point.theta};
        if (start_pose.has_value()) {
            initial_pos = start_pose.value();
        }
        resetPosition(initial_pos);

        
        Logger::getInstance()->log("Initial pose: %f %f %f", initial_pos.x, initial_pos.y, initial_pos.theta);
    }
    catch (const std::bad_variant_access &e)
    {
        Logger::getInstance()->logError("Error: Second point is not a TrajectoryPoint!");
    }

    // Main control loop
    Logger::getInstance()->log("Route size: %d", trajectory.size());
    Logger::getInstance()->log("Has next: %d", trajectory.hasNext());

    bool odom_holding = false;
    uint32_t odom_hold_start_ms = 0;

    while (trajectory.hasNext())
    {
        // Logger::getInstance()->log("Processing trajectory point %zu", trajectory_index);
        if (m_controller->get_digital(DIGITAL_X)){
            stopTrajectory();
            trajectory.reset();
            break;
        }

        logDriveHealth("auton");

        // Update localization
        if (m_localization){
            m_localization->update();
        }
        if (m_intake){
            // The lift owns the PTO while it moves; the commanded intake state is kept
            // underneath and resumes once the lift settles.
            m_intake->set_inhibited(m_lift && m_lift->wants_pto());
            m_intake->update();
        }
        updateRoller();

        if (m_lift)
        {
            m_lift->update();
        }

        // Odometry hold: with no valid sensor baseline the pose is frozen, and following
        // would drive blind while the trajectory clock runs away. Stop the drive and don't
        // consume the sample (or its actions) until the baseline is back.
        if (m_localization && !m_localization->isReliable())
        {
            if (!odom_holding)
            {
                odom_holding = true;
                odom_hold_start_ms = pros::millis();
                Logger::getInstance()->logWarning("Odometry not valid; holding the drive");
            }
            m_leftDrivetrain->move(0);
            m_rightDrivetrain->move(0);
            m_localization->setDriveVoltages(0.0, 0.0);
            pros::Task::delay_until(&loop_start_time, Config::DT);
            continue;
        }
        if (odom_holding)
        {
            odom_holding = false;
            Logger::getInstance()->log("Odometry valid; resuming after %u ms hold",
                                       pros::millis() - odom_hold_start_ms);
        }

        // wait_lift hold: stay on the stop sample, at rest, until the lift is done. Checked
        // after update() so the test reads the state the command has actually resolved to.
        if (m_liftWaitPending)
        {
            // Before the done check, so the tick the hold ends still gets a try
            retryPendingResets();
            const uint32_t waited_ms = pros::millis() - m_liftWaitStartMs;
            if (liftWaitDone())
            {
                m_liftWaitPending = false;
                Logger::getInstance()->log("wait_lift done after %u ms (%s)", waited_ms,
                                           Lift::state_name(m_lift->state()));
                dropPendingResets("the hold ended");
            }
            else if (waited_ms >= LIFT_WAIT_TIMEOUT_MS)
            {
                m_liftWaitPending = false;
                dropPendingResets("the hold timed out");
                Logger::getInstance()->logError("wait_lift TIMED OUT after %u ms (%s pos=%.1f)",
                                                waited_ms, Lift::state_name(m_lift->state()),
                                                m_lift->get_position());
            }
            else
            {
                if (m_lastTrajectoryPoint)
                {
                    TrajectoryPoint hold = *m_lastTrajectoryPoint;
                    hold.linear_vel = hold.angular_vel = 0.0;
                    hold.linear_accel = hold.angular_accel = 0.0;
                    this->processTrajectory(hold);
                }
                pros::Task::delay_until(&loop_start_time, Config::DT);
                continue;
            }
        }

        // Check if the current waypoint is a trajectory point or action point
        const DataPoint *point = trajectory.getNext();
        std::visit([this](const auto &p)
                   {
            if constexpr (std::is_same_v<std::decay_t<decltype(p)>, TrajectoryPoint>) {
                this->processTrajectory(p);
                m_lastTrajectoryPoint = p;
            } else {
                this->processAction(p);
            } }, *point);

        trajectory_index++;

        // Use delta time from configuration
        pros::Task::delay_until(&loop_start_time, Config::DT); // Convert to milliseconds
    }

    // Stop motors
    m_leftDrivetrain->move(0);
    m_rightDrivetrain->move(0);
    m_localization->setDriveVoltages(0.0, 0.0);

    m_isFollowingTrajectory = false;

    Logger::getInstance()->log("Trajectory following completed");
    return true; // Indicate that trajectory following completed successfully
}

bool Robot::followTrajectories(std::vector<Trajectory> &trajectories)
{
    std::optional<Pose> next_start_pose = std::nullopt;

    for (Trajectory& traj : trajectories)
    {
        if (!this->followTrajectory(traj, next_start_pose)) {
            return false;
        }

    next_start_pose = this->m_localization->getPose();
    }
    return true;
}

// Lift
bool Robot::liftBusy() const
{
    return m_lift && m_lift->is_busy();
}

bool Robot::liftSettled() const
{
    return m_lift && m_lift->is_settled();
}

bool Robot::liftWaitDone() const
{
    return !m_lift || m_lift->is_settled() || m_lift->score_released();
}

bool Robot::waitForLift(uint32_t timeout_ms)
{
    if (!m_lift)
    {
        Logger::getInstance()->logError("waitForLift ignored: no lift attached");
        return false;
    }

    uint32_t start_time = pros::millis();
    uint32_t loop_start_time = start_time;

    // Tick the lift ourselves: between trajectories nothing else is driving its
    // update(), so a lift command issued here would otherwise sit un-advanced.
    // Localization is ticked too so the IMU/encoder integration does not go stale
    // across the wait; the drivetrain is deliberately left alone.
    while (true)
    {
        m_lift->update();
        if (m_localization)
        {
            m_localization->update();
        }
        if (m_intake)
        {
            m_intake->set_inhibited(m_lift->wants_pto());
            m_intake->update();
        }
        updateRoller();

        // Checked AFTER the first update() so the settle test reads targets the lift
        // has actually resolved for the command just issued, not the previous pose's.
        if (m_lift->is_settled())
        {
            Logger::getInstance()->log("waitForLift settled after %u ms", pros::millis() - start_time);
            return true;
        }

        if (pros::millis() - start_time >= timeout_ms)
        {
            Logger::getInstance()->logError(
                "waitForLift TIMED OUT after %u ms (busy=%d pos=%.1f)",
                timeout_ms, m_lift->is_busy() ? 1 : 0, m_lift->get_position());
            return false;
        }

        pros::Task::delay_until(&loop_start_time, Config::DT);
    }
}

void Robot::tuneTrackWidth()
{
    PIDController controller = PIDController(200.0, 25.0, 20.0, -127.0, 127.0, 50.0);
    Logger::getInstance()->log("Track width tuning started");

    // Zero encoders
    m_leftDrivetrain->tare_position_all();
    m_rightDrivetrain->tare_position_all();

    double heading = Angles::degreesToRadians(m_inertial->get_heading() * -1);

    double target_heading = Angles::degreesToRadians(heading + 180.0);

    double left_arc_length = 0.0;
    double right_arc_length = 0.0;
    int success_count = 0;
    Logger::getInstance()->log("heading: %f", heading);
    Logger::getInstance()->log("Target heading: %f", target_heading);

    while (success_count < 3)
    {
        Logger::getInstance()->log("Angle difference: %f", Angles::angleDifference(target_heading, heading));
        if (fabs(Angles::angleDifference(target_heading, heading)) > 0.03)
        {
            success_count = 0; // Reset success count if we are not close enough
        }
        else
        {
            success_count++;
        }
        Logger::getInstance()->log("success count: %d", success_count);
        // Get current heading
        heading = Angles::degreesToRadians(m_inertial->get_heading() * -1);
        Logger::getInstance()->log("Current heading: %f", heading);

        // Calculate error
        double error = Angles::angleDifference(target_heading, heading);

        // Calculate control output
        double output = controller.calculate(error);

        // Apply output to motors
        m_leftDrivetrain->move(-output);
        m_rightDrivetrain->move(output);

        pros::delay(10);
    }

    Logger::getInstance()->log("Track width tuning completed");
    m_leftDrivetrain->move(0);
    m_rightDrivetrain->move(0);

    left_arc_length = Units::ticksToDistance(m_leftDrivetrain->get_position(), Config::TICKS_PER_ROTATION, Config::WHEEL_DIAMETER, Config::GEAR_RATIO);
    right_arc_length = Units::ticksToDistance(m_rightDrivetrain->get_position(), Config::TICKS_PER_ROTATION, Config::WHEEL_DIAMETER, Config::GEAR_RATIO);

    Logger::getInstance()->log("Left arc length: %f", left_arc_length);
    Logger::getInstance()->log("Right arc length: %f", right_arc_length);
}
