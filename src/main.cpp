#include "main.h"

// Robot config
bool skills = false;
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({-11, 12, 13});
pros::MotorGroup right_mg({14, -15, -16});
pros::MotorGroup intake_motors({17, -18});
pros::MotorGroup lift_motors({4, -19});

pros::Optical color_sort_os(21);
pros::Distance color_sort_ds(10);
pros::Rotation lift_rotation_sensor(20);
pros::Distance left_distance_sensor(3);
pros::Distance right_distance_sensor(5);
pros::Distance front_distance_sensor(16);

DistanceResetOdometry::DistanceSensor left_sensor(&left_distance_sensor, 5.9272, 0);
DistanceResetOdometry::DistanceSensor right_sensor(&right_distance_sensor, 6.3209, 0);
DistanceResetOdometry::DistanceSensor front_sensor(&front_distance_sensor, 2.8, -4.5);

EnhancedDigitalOut claw_clamp(5, false);
EnhancedDigitalOut claw_angler(6, false);
EnhancedDigitalOut flipper(3, false);
EnhancedDigitalOut intake_widener(7, false);
EnhancedDigitalOut bobby_wheel(6, false);

// Output limits are millivolts (fed straight to move_voltage), so they must be
// ±12000 — the ±127 default would starve the hold and let the lift sag. kI trims
// out the residual droop a pure-P hold would leave against gravity.
PIDController lift_pid(250, 60, 1200, -12000, 12000);
Lift lift(lift_rotation_sensor, lift_motors, claw_angler, claw_clamp, 12000, 12000, 8000, 192000, 200, lift_pid);
Intake intake(intake_motors, intake_widener, color_sort_os, color_sort_ds);

Trajectory trajectory;
Trajectory goal_to_matchloader;
Trajectory low_goal;

pros::Imu imu_sensor(1);

Logger *logger = Logger::getInstance();

pros::Rotation side_encoder(5);

std::string program_type = "autonomous";

// Routes

std::vector<std::vector<double>> route;
// std::string route_name = "";
std::string route_name = "fun_curve";
// std::vector< std::string > routes = std::vector< std::string >{"skills8.1", "skills8.2"};
std::vector<std::string> routes;
std::vector<Trajectory> trajectories;

RamseteController *ramsete_controller;
DrivetrainController *drive_controller;

Robot *robot;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
    pros::lcd::initialize();
    // pros::lcd::clear()

    left_mg.set_encoder_units_all(pros::E_MOTOR_ENCODER_COUNTS);
    right_mg.set_encoder_units_all(pros::E_MOTOR_ENCODER_COUNTS);

    left_mg.tare_position_all();
    right_mg.tare_position_all();

    if (!skills){
        ;
        intake.start_color_sorting("blue");
    }

    imu_sensor.reset();

    side_encoder.set_reversed(true);

    DistanceResetOdometry::DistanceSensors distance_sensors = {front_sensor, nullptr, left_sensor, right_sensor};

    auto localization = LocalizationManager::create(
        LocalizationType::DISTANCE_RESET_ODOMETRY,
        left_mg, right_mg, side_encoder, imu_sensor, distance_sensors,
        false, true
    );

    ramsete_controller = new RamseteController(
        Config::RAMSETE_B, Config::RAMSETE_ZETA,
        Config::RAMSETE_SCALE_FACTOR);
    drive_controller = new DrivetrainController(
        Config::KS, Config::KV, Config::KA,
        Config::KP, Config::KI, Config::KD);

    robot = new Robot(&left_mg, &right_mg, &imu_sensor, &master, drive_controller, ramsete_controller, std::move(localization), &intake);

    pros::delay(5);
    trajectory.loadFromFile("/usd/routes/" + route_name + ".txt");

    if (skills && routes.size() > 0){
        for (auto& route: routes){
            Trajectory traj;
            traj.loadFromFile("/usd/routes/" + route + ".txt");
            trajectories.push_back(traj);
        }
    }

    goal_to_matchloader.loadFromFile("/usd/routes/goal_to_matchloader.txt");
    low_goal.loadFromFile("/usd/routes/low_goal.txt");
    // trajectory.loadFromVector();
    // if (trajectory.empty())
    // {
    //     logger->log("Error: Route file is empty or could not be opened.");
    //     return;
    // }

    logger->log("Robot initialized");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize()
{
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous()
{
    logger->log("Program type: %s", program_type.c_str());
    // trajectory.loadFromFile("/usd/routes/" + route_name + ".txt");

    if (program_type == "autonomous")
    {
        logger->log("Starting autonomous");

        if (trajectories.size() == 0){
            robot->followTrajectory(trajectory);
        } else {
            robot->followTrajectories(trajectories);
        }
    }
    if (route_name == "splitsevenhigh" || route_name == "splitsevenhighcounter"){
        left_mg.move_voltage(3600);
        right_mg.move_voltage(-3600);
        pros::delay(600);
        right_mg.move_voltage(0);
        left_mg.move_voltage(0);
    }
    left_mg.set_brake_mode_all(MOTOR_BRAKE_BRAKE);
    right_mg.set_brake_mode_all(MOTOR_BRAKE_BRAKE);
}

int joystickCurve(int x, double a = 3.0)
{
    return int(((127.0 * std::pow(std::abs(double(x)), std::abs(a))) / (std::pow(127.0, a))) * (double(x) / std::abs(double(x))));
}

int joystickCurveSkills(int x, double a = 3.0)
{
    return int(((std::pow((double(x) / 5.7), 2.0) - std::abs(std::pow((double(x) / 5.7), 3.0) / 6.0) + (std::pow((double(x) / 5.7), 4.0) / 120.0)) * double(x)) / (3.0 * std::abs(double(x))));
}

#include <cmath>
#include <algorithm>
#include <utility>

/**
 * arcade_drive
 *
 * Mixes two joystick axes (throttle + steering) into left/right motor outputs.
 *
 * @param throttle  Raw forward/back axis  [-127, 127]
 * @param steer     Raw left/right axis    [-127, 127]
 * @return          std::pair<int, int>  { leftMotor, rightMotor }  [-127, 127]
 */
// std::pair<int, int> arcade_drive(int throttle, int steer)
// {
//     // -------------------------------------------------------------------------
//     //  TUNING CONSTANTS  (edit these to suit your robot / controller)
//     // -------------------------------------------------------------------------

//     // Deadband: joystick inputs whose absolute value falls at or below this
//     // threshold are treated as zero.  Eliminates controller drift.
//     // Range: [0, 127]  –  typical value: 5–15
//     constexpr int DEADBAND = 10;

//     // Minimum speed: once a joystick clears the deadband, the output is
//     // immediately scaled up to at least this value (avoids a "dead zone" where
//     // the motor stalls because the command is too small to overcome friction).
//     // Range: [0, 127]  –  typical value: 15–30
//     constexpr int MIN_SPEED = 20;

//     // Turning curve exponent: applies a power-curve to the *steering* axis so
//     // small deflections produce finer control while full deflection still
//     // reaches 100 %.  Higher values = gentler at low speeds.
//     // Range: (1.0, ∞)  –  typical value: 1.5–3.0  (1.0 = linear / no curve)
//     constexpr double TURN_CURVE_EXPONENT = 2.5;

//     // Throttle/steer priority: blends throttle vs steering authority when both
//     // axes are active simultaneously.  At 1.0 the robot drives straight at full
//     // speed even with max steering input; at 0.0 steering gets no headroom at
//     // all alongside full throttle.
//     // Range: [0.0, 1.0]  –  typical value: 0.5–0.7
//     constexpr double THROTTLE_PRIORITY = 0.1;

//     // -------------------------------------------------------------------------
//     //  INPUT CLAMPING
//     // -------------------------------------------------------------------------
//     const double rawThrottle = std::clamp(throttle, -127, 127);
//     const double rawSteer    = std::clamp(steer,    -127, 127);

//     // -------------------------------------------------------------------------
//     //  DEADBAND
//     //  Zero out inputs that fall within the dead zone.
//     // -------------------------------------------------------------------------
//     const double filteredThrottle = (std::abs(rawThrottle) <= DEADBAND) ? 0.0 : rawThrottle;
//     const double filteredSteer    = (std::abs(rawSteer)    <= DEADBAND) ? 0.0 : rawSteer;

//     // -------------------------------------------------------------------------
//     //  MINIMUM SPEED SCALING
//     //  Remap [deadband+1 … 127] → [MIN_SPEED … 127] so the output jumps to a
//     //  usable value the moment the stick clears the deadband.
//     // -------------------------------------------------------------------------
//     auto applyMinSpeed = [&](double val) -> double
//     {
//         if (val == 0.0) return 0.0;

//         const double sign      = (val > 0.0) ? 1.0 : -1.0;
//         const double magnitude = std::abs(val);

//         // Normalise to [0, 1] relative to the usable input range
//         const double normalised = (magnitude - DEADBAND) / (127.0 - DEADBAND);

//         // Rescale to [MIN_SPEED, 127]
//         const double scaled = MIN_SPEED + normalised * (127.0 - MIN_SPEED);
//         return sign * scaled;
//     };

//     const double scaledThrottle = applyMinSpeed(filteredThrottle);
//     const double scaledSteer    = applyMinSpeed(filteredSteer);

//     // -------------------------------------------------------------------------
//     //  TURNING CURVE
//     //  Apply a power curve to the steering axis only.
//     //  Output = sign(x) * (|x| / 127)^EXPONENT * 127
//     // -------------------------------------------------------------------------
//     double curvedSteer = 0.0;
//     if (scaledSteer != 0.0)
//     {
//         const double sign      = (scaledSteer > 0.0) ? 1.0 : -1.0;
//         const double normalised = std::abs(scaledSteer) / 127.0;          // [0, 1]
//         const double curved    = std::pow(normalised, TURN_CURVE_EXPONENT); // power curve
//         curvedSteer = sign * curved * 127.0;
//     }

//     // -------------------------------------------------------------------------
//     //  THROTTLE / STEER PRIORITY MIXING
//     //
//     //  The available "budget" for steering shrinks proportionally as the
//     //  throttle increases, ensuring full-speed straight driving is reachable.
//     //
//     //  steering headroom = 127 * (1 - THROTTLE_PRIORITY * |throttle| / 127)
//     //  The final mix:
//     //      left  = throttle + steerContribution
//     //      right = throttle - steerContribution
//     // -------------------------------------------------------------------------
//     const double throttleNorm = std::abs(scaledThrottle) / 127.0;           // [0, 1]
//     const double steerBudget  = 127.0 * (1.0 - THROTTLE_PRIORITY * throttleNorm);
//     const double steerContrib = curvedSteer * (steerBudget / 127.0);

//     double left  = scaledThrottle + steerContrib;
//     double right = scaledThrottle - steerContrib;

//     // -------------------------------------------------------------------------
//     //  OUTPUT CLAMPING  →  keep within [-127, 127]
//     // -------------------------------------------------------------------------
//     return {
//         static_cast<int>(std::clamp(left,  -127.0, 127.0)),
//         static_cast<int>(std::clamp(right, -127.0, 127.0))
//     };
// }
/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol()
{
    double drivetrain_slow = 0.5;
    left_mg.set_brake_mode_all(MOTOR_BRAKE_COAST);
    right_mg.set_brake_mode_all(MOTOR_BRAKE_COAST);
    long long count = 0;
    int intake_state = 0;
    int intake_pressed_dur = 0;
    bool intaking = false;
    while (true)
    {
        // Update the LCD at ~4 Hz, not every 10ms — hammering pros::lcd::print
        // (an LVGL wrapper) from opcontrol races the LVGL render daemon.
        if (count % 25 == 0)
            pros::lcd::print(1, "count: %lld", count);
        ++count;
        left_mg.move(master.get_analog(ANALOG_LEFT_Y));
        right_mg.move(master.get_analog(ANALOG_RIGHT_Y));

        if (master.get_digital_new_press(DIGITAL_R1)) // Intake
        {
            if (!intaking)
            {
                intake_state = 1;
            }
            else
            {
                intake_state = 0;
            }
            intaking = !intaking;
        }
        if (master.get_digital(DIGITAL_R1))
        {
            intake_pressed_dur += 10;
            if (intake_pressed_dur >= 150)
            {
                intake_state = 2;
            }
        }
        else
        {
            intake_pressed_dur = 0;
            if (intake_state == 2)
                intake_state = 0;
        }
        intake.set_state(intake_state);


        // Hold L1 to raise; on release lift.update() captures the current
        // angle and holds it, so the lift stays where the driver left it.
        if (master.get_digital(DIGITAL_L1))
        {
            lift.raise();
        }
        // Single-press macros: score (B) and return to bottom (L2).
        else if (master.get_digital_new_press(DIGITAL_B))
        {
            lift.score();
        }
        else if (master.get_digital_new_press(DIGITAL_L2))
        {
            lift.move_down();
        }


        // While the lift eases down, gently outtake so a piece clears the intake.
        if (lift.is_lowering())
        {
            intake.set_state(4);
        }

        intake.update();
        lift.update();

        pros::delay(10);
        // ------------- END MATCH DRIVER CONTROLS --------------------
    }
}