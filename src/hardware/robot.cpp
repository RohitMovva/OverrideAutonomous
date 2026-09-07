#include "hardware/robot.hpp"
#include "controllers/pid_controller.hpp"
#include "utilities/math/units.hpp"
#include "utilities/math/angle.hpp"
#include "navigation/distance_reset_odometry.hpp"
#include <cmath>
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
    : m_leftDrivetrain(nullptr), m_rightDrivetrain(nullptr), m_inertial(nullptr), m_driveController(nullptr), m_ramseteController(nullptr), m_localization(nullptr), m_isFollowingTrajectory(false), m_intake(nullptr), m_lift(nullptr)
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
             Lift *lift)
    : m_leftDrivetrain(leftDrivetrain), m_rightDrivetrain(rightDrivetrain), m_controller(controller), m_inertial(inertial), m_driveController(driveController), m_ramseteController(ramseteController), m_localization(std::move(localization)), m_isFollowingTrajectory(false),m_intake(intake), m_lift(lift)
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
    // Logger::getInstance()->log("Ramsete Output: %f %f", ramsete_output[0], ramsete_output[1]);
    // Logger::getInstance()->log("MP Output: %f %f", tp.linear_vel, tp.angular_vel);

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

void Robot::processAction(const ActionPoint &ap)
{
    // Process action point
    Logger::getInstance()->log("Executing action with %zu parameters", ap.actions.size());

    // Placeholder
    for (const auto &action : ap.actions)
    {
        Logger::getInstance()->log("Action: %f", action);
    }

    if (ap.actions.size() < 5)
    {
        Logger::getInstance()->logError("Insufficient action parameters, expected at least 5");
        return;
    }

    int intake_action = static_cast<int>(ap.actions[0]); // Convert to int for easier handling
    m_intake->set_state(intake_action); // Set intake state based on action

    if (ap.actions[1] != 0.0)
    {
        // safe comparison with tolerance
        const double lift_action = ap.actions[1];
        const auto is = [lift_action](double code)
        { return std::fabs(lift_action - code) < LIFT_ACTION_EPSILON; };

        if (Lift::is_chain_id(lift_action)){
            this->liftChain(lift_action);
        } else if (is(.1)){
            this->liftState(Lift::Side::BACK, "trigger_roller");
        }
        else if (is(.5)){
            this->liftState(Lift::Side::FRONT, "pickup_stack");
        } else if (is(.51)){
            this->liftState(Lift::Side::FRONT, "pickup_stack_closed");
        }
        else if (is(1)){
            this->liftState(Lift::Side::FRONT, "neutral_one_pin");
        }
        else if (is(6.9)){
            this->liftState(Lift::Side::FRONT, "toggle");
        } 
        else if (is(-.1)){
            this->liftState(Lift::Side::BACK, "move_out");
        }
        else if (is(-.5)){
            this->liftState(Lift::Side::BACK, "pickup_stack");
        } else if (is(-1)){
            this->liftState(Lift::Side::BACK, "neutral_one_pin");
        } else if (is(-2)){
            this->liftState(Lift::Side::BACK, "alliance_empty");
        } else if (is(-2.5)){
            this->liftState(Lift::Side::BACK, "alliance_empty_down");
        } else if (is(-3)){
            this->liftState(Lift::Side::BACK, "load_stack");
        } else if (is(-4)){
            this->liftState(Lift::Side::BACK, "alliance_one_pin");
        } else if (is(-5)){
            this->liftState(Lift::Side::BACK, "alliance_two_pin");
        } else if (is(-5.01)){
            this->liftState(Lift::Side::BACK, "alliance_two_pin_down");
        } else if (is(-5.02)){
            this->liftState(Lift::Side::BACK, "alliance_two_pin_open");
        }
        
        else if (is(-5.3)){
            this->liftState(Lift::Side::BACK, "alliance_three_pin");
        }  else if (is(-5.32)){
            this->liftState(Lift::Side::BACK, "alliance_three_pin_released");
        }
        
        
        else if (is(-5.4)){
            this->liftState(Lift::Side::BACK, "alliance_four_pin");
        } else if (is(-5.41)){
            this->liftState(Lift::Side::BACK, "alliance_four_pin_down");
        } else if (is(-5.42)){
            this->liftState(Lift::Side::BACK, "alliance_four_pin_released");
        }
        
        
        else if (is(-6)){
            this->liftState(Lift::Side::BACK, "flip_clearance");
        } else if (is(-7)){
            this->liftState(Lift::Side::BACK, "flipped_clearance");
        } else if (is(-8)){
            this->liftState(Lift::Side::BACK, "alliance_empty_alt");
        } else if (is(-8.1)){
            this->liftState(Lift::Side::BACK, "alliance_empty_alt_down");
        } else if (is(-8.2)){
            this->liftState(Lift::Side::BACK, "alliance_empty_alt_down_open");
        } else if (is(-9.9)){
            this->liftState(Lift::Side::BACK, "center_one_pin");
        } else if (is(-9.91)){
            this->liftState(Lift::Side::BACK, "center_one_pin_lowered");
        } else if (is(-9.92)){
            this->liftState(Lift::Side::BACK, "center_one_pin_lowered_open");
        } else {
            Logger::getInstance()->logError("Invalid lift action parameter: %f", lift_action);
        }
    }

    if (ap.actions[2] == 1.0)
    {
        if (auto distance_reset_odometry = dynamic_cast<DistanceResetOdometry*>(m_localization.get())) {
            distance_reset_odometry->resetLeft();
        } else {
            Logger::getInstance()->logError("Failed to cast to DistanceResetOdometry");
        }
    }
    if (ap.actions[3] == 1.0)
    {
        if (auto distance_reset_odometry = dynamic_cast<DistanceResetOdometry*>(m_localization.get())) {
            distance_reset_odometry->resetRight();
        } else {
            Logger::getInstance()->logError("Failed to cast to DistanceResetOdometry");
        }
    }

    if (ap.actions[4] != 0.0)
    {
        if (auto distance_reset_odometry = dynamic_cast<DistanceResetOdometry*>(m_localization.get())) {
            distance_reset_odometry->resetFront();
        } else {
            Logger::getInstance()->logError("Failed to cast to DistanceResetOdometry");
        }
    }
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
        "ramseteB=%.3f ramseteZeta=%.3f maxWheelVel=%.3f",
        Config::WHEEL_BASE_WIDTH, Config::WHEEL_DIAMETER,
        m_driveController->getKS(), m_driveController->getKV(),
        m_driveController->getKA(), m_driveController->getKADecel(),
        m_driveController->getKP(),
        m_driveController->getKI(), m_driveController->getKD(),
        m_driveController->getKSAngular(), m_driveController->getKVAngular(),
        m_ramseteController->getB(), m_ramseteController->getZeta(),
        Config::MAX_WHEEL_VELOCITY);
    if (trajectory.empty())
    {
        Logger::getInstance()->logError("Trajectory is empty, cannot follow");
        return false;
    }
    trajectory.reset();

    m_isFollowingTrajectory = true;

    Logger::getInstance()->log("Following trajectory with %s localization",
                               m_localization->getTypeName().c_str());

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
            // Same gentle forward feed the driver gets while a prime pulls a piece out of
            // the intake. An override, so the state the last action point set is untouched.
            m_intake->set_assist_feed(m_lift && m_lift->is_priming_from_rest());
            m_intake->update();
        }

        if (m_lift)
        {
            m_lift->update();
        }

        // Check if the current waypoint is a trajectory point or action point
        const DataPoint *point = trajectory.getNext();
        std::visit([this](const auto &p)
                   {
            if constexpr (std::is_same_v<std::decay_t<decltype(p)>, TrajectoryPoint>) {
                this->processTrajectory(p);
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
bool Robot::liftMacro(Lift::Side side, const char *key)
{
    if (!m_lift)
    {
        Logger::getInstance()->logError("liftMacro('%s') ignored: no lift attached", key ? key : "(null)");
        return false;
    }
    return m_lift->run_macro(side, key);
}

bool Robot::liftState(Lift::Side side, const char *key)
{
    if (!m_lift)
    {
        Logger::getInstance()->logError("liftState('%s') ignored: no lift attached", key ? key : "(null)");
        return false;
    }
    return m_lift->run_lift_state(side, key);
}

bool Robot::liftChain(double id)
{
    if (!m_lift)
    {
        Logger::getInstance()->logError("liftChain(%.2f) ignored: no lift attached", id);
        return false;
    }
    return m_lift->run_lift_chain(id);
}

bool Robot::liftChain(const char *name)
{
    if (!m_lift)
    {
        Logger::getInstance()->logError("liftChain('%s') ignored: no lift attached",
                                        name ? name : "(null)");
        return false;
    }
    return m_lift->run_lift_chain(name);
}

bool Robot::liftBusy() const
{
    return m_lift && m_lift->is_busy();
}

bool Robot::liftSettled() const
{
    return m_lift && m_lift->is_settled();
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
        // The intake for the same reason: a prime waited on here is exactly when its
        // assist feed should be running, and nothing else is ticking it during the wait.
        if (m_intake)
        {
            m_intake->set_assist_feed(m_lift->is_priming_from_rest());
            m_intake->update();
        }

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
                "waitForLift TIMED OUT after %u ms (busy=%d winch=%.1f bar=%.1f ee=%.1f)",
                timeout_ms, m_lift->is_busy() ? 1 : 0,
                m_lift->get_winch_position(), m_lift->get_bar_angle(), m_lift->get_ee_angle());
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
