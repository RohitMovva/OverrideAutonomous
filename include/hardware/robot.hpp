/**
 * @file robot.hpp
 * @brief Robot class encompassing all hardware and control systems
 * @author Rohit Movva
 * @date 2025
 */

#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "main.h"
#include "pros/motors.hpp"
#include "pros/motor_group.hpp"

#include "hardware/intake.hpp"
#include "hardware/lift.hpp"
#include "hardware/roller_mech.hpp"

#include "controllers/ramsete_controller.hpp"
#include "controllers/drivetrain_controller.hpp"
#include "controllers/pid_controller.hpp"

#include "navigation/i_localization.hpp"
#include "navigation/localization_manager.hpp"
#include "navigation/odometry.hpp"
#include "navigation/mcl.hpp"
#include "navigation/trajectory.hpp"

#include "utilities/logger.hpp"

/**
 * @class Robot
 * @brief Main robot class for VEX competition robot using PROS
 *
 * This class provides high-level control for a VEX robot, including
 * drivetrain control, trajectory following, and autonomous motion.
 * It integrates with PROS motion control systems including Ramsete
 * controller and localization for precise autonomous navigation.
 */
class Robot
{
private:
    pros::MotorGroup *m_leftDrivetrain;            ///< Left side drivetrain motor group
    pros::MotorGroup *m_rightDrivetrain;           ///< Right side drivetrain motor group
    pros::Imu *m_inertial;                         ///< Inertial sensor for heading and orientation
    pros::Controller *m_controller;
    DrivetrainController *m_driveController;       ///< Drivetrain velocity controller
    RamseteController *m_ramseteController;        ///< Ramsete controller for trajectory following
    std::unique_ptr<ILocalization> m_localization; ///< Localization manager for managing localization systems
    bool m_isFollowingTrajectory;                  ///< Flag indicating if trajectory following is active
    int m_min_voltage = 0;                          ///< Voltage to keep bot from sliding back from little will

    bool m_feedbackEnabled;

    /// A wait_lift action point is holding followTrajectory() at the current sample
    /// until liftWaitDone() (or LIFT_WAIT_TIMEOUT_MS).
    bool m_liftWaitPending = false;
    uint32_t m_liftWaitStartMs = 0;              ///< pros::millis() when the hold began
    /// Wall resets on a point that starts a wait_lift hold: done during the hold, once
    /// the robot is still (Robot::retryPendingResets); bits are RESET_PENDING_*.
    /// Dropped when the hold ends.
    uint8_t m_pendingResets = 0;
    /// Last sample followTrajectory() drove to; the hold keeps driving to it at rest
    std::optional<TrajectoryPoint> m_lastTrajectoryPoint;


    Intake *m_intake;         ///< Intake class w/ color sorting
    Lift *m_lift;             ///< Lift + end effector (shares the PTO with the intake)
    RollerMech *m_roller;     ///< Field roller spinner (lowest PTO priority); may be null

    /// Tick the roller mech, inhibited while the lift wants the PTO or the intake runs
    void updateRoller();

    /**
     * @brief Process a trajectory point from the trajectory
     * @param tp Trajectory point containing kinematic data
     */
    void processTrajectory(const TrajectoryPoint &tp);

    /**
     * @brief Process an action point from the trajectory
     * @param ap Action point containing actions to execute
     */
    void processAction(const ActionPoint &ap);

    /// Run m_pendingResets once the robot is still; called each tick of a wait_lift hold
    void retryPendingResets();

    /// Log and drop any m_pendingResets left when a hold ends
    void dropPendingResets(const char *why);

public:
    /**
     * @brief Default constructor for Robot class
     */
    Robot();

    /**
     * @brief Constructor with full initialization parameters
     *
     * Initializes the robot with all necessary control systems for
     * autonomous operation including drivetrain, controllers, and localization.
     *
     * @param leftDrivetrain Pointer to left side motor group
     * @param rightDrivetrain Pointer to right side motor group
     * @param inertial Pointer to inertial sensor for heading and orientation
     * @param controller Pointer to master controller
     * @param driveController Pointer to drivetrain velocity controller
     * @param ramseteController Pointer to Ramsete controller for path following
     * @param localization Unique pointer to localization interface
     * @param intake Pointer to the intake subsystem
     * @param lift Pointer to the lift subsystem. Robot ticks its update() during
     *             trajectory following, so the lift is live in autonomous.
     * @param roller Pointer to the roller mech, or null for none. Ticked alongside the
     *             intake; a route turns it on and off with its `roller` action slot.
     *
     * @pre All pointer parameters must be valid and properly initialized
     * @post Robot is fully initialized and ready for autonomous operation
     *
     * @throws std::invalid_argument if any pointer parameter is null
     */
    Robot(pros::MotorGroup *leftDrivetrain,
          pros::MotorGroup *rightDrivetrain,
          pros::Imu *inertial,
          pros::Controller *controller,
          DrivetrainController *driveController,
          RamseteController *ramseteController,
          std::unique_ptr<ILocalization> localization,
          Intake *intake,
          Lift *lift,
          RollerMech *roller = nullptr);

    /**
     * @brief Destructor for Robot class
     *
     * Properly cleans up resources. The localization manager is automatically
     * cleaned up via unique_ptr. Other pointers are not owned by this class.
     */
    ~Robot();

    /**
     * @brief Follow a predefined trajectory using motion profiling
     *
     * Executes autonomous motion along a specified trajectory using
     * the Ramsete controller and motion profiling. The robot will
     * follow the path while maintaining velocity and acceleration
     * constraints defined in the motion profile.
     *
     * @param trajectory The trajectory to follow
     *
     * @return true if trajectory execution started successfully, false otherwise
     *
     * @pre Robot must be properly initialized with all control systems
     * @pre Trajectory must be valid and follow robot constraints
     * @post Robot begins following the specified trajectory
     *
     * @warning Ensure the trajectory is within the robot's physical limits
     *          and field boundaries before execution
     */
    bool followTrajectory(Trajectory &trajectory, std::optional<Pose> start_pose = std::nullopt);

    bool followTrajectories(std::vector<Trajectory> &trajectories);


    /**
     * @brief Check if robot is currently following a trajectory
     *
     * @return true if trajectory following is active, false otherwise
     */
    bool isFollowingTrajectory() const;

    /**
     * @brief Stop current trajectory following
     *
     * Immediately stops any active trajectory following and
     * brings the robot to a controlled stop.
     *
     * @post Robot stops moving and trajectory following is disabled
     */
    void stopTrajectory();

    /**
     * @brief Get current robot position from localization
     *
     * @return Current robot position and heading
     * @pre Localization system must be properly initialized
     */
    auto getCurrentPosition() const -> decltype(m_localization->getPose());

    /**
     * @brief Reset robot localization to specified position
     *
     * @param position New position to set for localization
     * @pre Localization system must be properly initialized
     */
    void resetPosition(const Pose &position);

    /**
     * @brief Check if robot systems are properly initialized
     *
     * @return true if all required systems are initialized, false otherwise
     */
    bool isInitialized() const;

    /**
     * @brief Log drive temperature, current, power, efficiency and over-temp,
     *        ONE LINE PER MOTOR, tagged with its physical port.
     *
     * Self-throttling to DRIVE_HEALTH_LOG_MS, so it is safe to call every tick.
     *
     * Motivated by 08/01: the left side needed 27.1 then 37.1 motor units MORE than the
     * kS+kV model to hold 30-40 in/s (08012026-8 and -9) against 5.8-8.9 in -3..-6, while
     * the right side was unchanged across all seven runs. That was only recoverable by
     * back-calculating PWM-per-in/s from the velocity logs, and only because the fault was
     * one-sided and large.
     *
     * The fields are logged PER MOTOR because the two motors on a side are mechanically
     * coupled: they turn at the same speed under the same command, so the healthy one is
     * the control for the sick one, and no separate baseline run is needed. Read them
     * together -- each alone is ambiguous:
     *
     *   current  the load the motor is actually carrying
     *   power    P/I recovers the volts the motor really saw, so a motor that is not
     *            getting its command (bad connector, damaged wire, failing winding) shows
     *            up as low power for the same command while its partner is normal
     *   eff      mechanical out over electrical in; the direct read on a motor burning
     *            its input as heat (bearings, internal drag). Compare only at MATCHED
     *            speed -- efficiency is near zero at stall for a perfectly good motor
     *   temp/ovt the V5 derates torque in steps as it heats, and nothing else says so
     *
     * Costs two smart-port reads per motor per metric, hence the throttle rather than a
     * per-tick line: the control loop has a 10 ms budget.
     *
     * @param context Short tag identifying the caller, e.g. "auton" or "driver"
     */
    void logDriveHealth(const char *context);

    /// Sampling period for logDriveHealth(). 100 ms gives ~17 samples across the first
    /// trajectory segment -- enough to see a derate develop -- for ~1% more log volume.
    static constexpr uint32_t DRIVE_HEALTH_LOG_MS = 100;

    /**
     * @brief Tune track width for the robot
     */

    void tuneTrackWidth();

    // ---- Lift ---------------------------------------------------------------
    //
    // Autonomous drives the lift from action points (Robot::processAction). Commands are
    // NON-BLOCKING: they arm the move and the lift advances one step per followTrajectory()
    // tick. Use waitForLift() where lift and drive must be sequenced.
    //
    // Between trajectories nothing is ticking the lift, so a command issued there does
    // not start moving until the next followTrajectory() or waitForLift().

    /// True while the lift is moving (raising, slewing to a stop, or returning home)
    bool liftBusy() const;

    /// True when the lift is on its target and nothing is in flight.
    bool liftSettled() const;

    /// True when a wait_lift hold is over: a score once its pose is set
    /// (Lift::score_released), anything else once the lift settles.
    bool liftWaitDone() const;

    /**
     * @brief Block until the lift settles, ticking it at Config::DT.
     *
     * Use between trajectories, where nothing else is driving Lift::update(). The
     * drivetrain is not commanded here -- this only turns the lift.
     *
     * @param timeout_ms give up after this long
     * @return true if the lift settled, false on timeout (logged) or no lift attached
     */
    bool waitForLift(uint32_t timeout_ms);

    // Disable copy constructor and assignment operator
    Robot(const Robot &) = delete;
    Robot &operator=(const Robot &) = delete;
};

#endif
