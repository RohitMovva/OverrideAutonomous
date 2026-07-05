#include "main.h"

// Robot config
bool skills = false;
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({-11, -12, -13});
pros::MotorGroup right_mg({18, 19, 20});
pros::MotorGroup intake_motors({4, 5});
pros::Motor lift_motor(7);

pros::Optical color_sort_os(21);
pros::Distance color_sort_ds(10);
pros::Rotation lift_rotation_sensor(6);
pros::Distance left_distance_sensor(3);
pros::Distance right_distance_sensor(5);
pros::Distance front_distance_sensor(16);

DistanceResetOdometry::DistanceSensor left_sensor(&left_distance_sensor, 5.9272, 0);
DistanceResetOdometry::DistanceSensor right_sensor(&right_distance_sensor, 6.3209, 0);
DistanceResetOdometry::DistanceSensor front_sensor(&front_distance_sensor, 2.8, -4.5);

EnhancedDigitalOut claw_clamp(3, false);
EnhancedDigitalOut claw_angler(2, false);
EnhancedDigitalOut flipper(7, false);
EnhancedDigitalOut intake_widener(5, false);
EnhancedDigitalOut bobby_wheel(6, false);
Intake intake(intake_motors, intake_widener, color_sort_os, color_sort_ds);

Trajectory trajectory;
Trajectory goal_to_matchloader;
Trajectory low_goal;

pros::Imu imu_sensor(15);


Logger *logger = Logger::getInstance();

pros::Rotation side_encoder(5);

std::string program_type = "autonomous";



// Routes

std::vector<std::vector<double>> route;
// std::string route_name = "";
std::string route_name = "splitsevenlow";
// std::vector< std::string > routes = std::vector< std::string >{"skills8.1", "skills8.2"};
std::vector< std::string > routes;
std::vector< Trajectory > trajectories;

RamseteController* ramsete_controller;
DrivetrainController* drive_controller;

Robot* robot;


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
    
    ramsete_controller = new RamseteController(2.4, 0.4, 4.5 * 12, 5.0, 0.0254000508);
    drive_controller = new DrivetrainController(2.5, 1.45, 0.275, 3.0, 0.00, 0.0);

    robot = new Robot(&left_mg, &right_mg, &imu_sensor, &master, drive_controller, ramsete_controller, std::move(localization), 
                      &little_will, &ear2, &intake);

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


void testDistanceReset()
{
    double sensorReadingMillimeters = front_distance_sensor.get();
    if (sensorReadingMillimeters >= 9999) {
        Logger::getInstance()->logWarning("Invalid right sensor reading: %f", sensorReadingMillimeters);
        return;
    }

    double sensorReadingInches = sensorReadingMillimeters / 25.4;
    pros::lcd::print(0, "sensorReadingInches: %.2f", sensorReadingInches);
    // Logger::getInstance()->log("Right sensor reading: %f in", sensorReadingInches);

    // Get current pose and heading
    double robotHeading = imu_sensor.get_heading() * (M_PI / 180.0); // Convert to radians
    pros::lcd::print(1, "Heading: %.2f", robotHeading * (180.0 / M_PI));

    double directionOffset = 0.0; // 90 bc left

    int headingDeg = (int)(robotHeading * (180.0 / M_PI) + directionOffset);

    // headingDeg = headingDeg % 360;

    bool resettingX = false;
    double wallSign = 1.0;

    if (315 <= headingDeg || headingDeg <= 45) // Right wall 
    {
        resettingX = true;
        wallSign = 1.0;
    }
    else if (45 <= headingDeg && headingDeg < 135) // Top wall
    {
        resettingX = false;
        wallSign = -1.0;
    }
    else if (135 <= headingDeg && headingDeg < 225) // Left wall
    {
        resettingX = true;
        wallSign = -1.0;
    }
    else { // Bottom wall
        resettingX = false;
        wallSign = 1.0;
    }

    if (resettingX && int(directionOffset)%180 == 90) {
        robotHeading += M_PI / 2;
    } else if (!resettingX && int(directionOffset)%180 == 0){
        robotHeading += M_PI / 2;
    }

    if (robotHeading > M_PI){
        robotHeading -= 2*M_PI;
    }
    pros::lcd::print(5, "Heading: %.2f", robotHeading);

    double sensorToWall = std::cos(robotHeading) * sensorReadingInches;
    pros::lcd::print(4, "Sesnor to wall: %.2f", sensorToWall);

    double sensorToRobot = front_sensor.offsetX * std::cos(robotHeading) + front_sensor.offsetY * std::sin(robotHeading);

    double robotToWall = sensorToWall + sensorToRobot;

    double actualPos = wallSign * (70.7 - robotToWall);

    pros::lcd::print(3, "Distance From Wall: %.2f", robotToWall);

    if (resettingX){
        pros::lcd::print(2, "Current X: %.2f", actualPos);
    }
    else {
        pros::lcd::print(2, "Current Y: %.2f", actualPos);
    }
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
    ear2.input_toggle(true);

    if (program_type == "autonomous")
    {
        // intake.set_state(1);
        // intake.update();
        // left_mg.move_voltage(3000);
        // right_mg.move_voltage(3000);
        // intake.update();
        // pros::delay(500);
        // left_mg.move_voltage(-3000);
        // right_mg.move_voltage(-3000);
        // pros::delay(200);
        // left_mg.move_voltage(0);
        // right_mg.move_voltage(0);
        // pros::delay(500);
        // for (int i = -1; i < 2; i +=2){
        //     left_mg.move_voltage(3000*i);
        //     right_mg.move_voltage(3000*i*-1);
        //     pros::delay(250);
        //     if (i==1)pros::delay(150);
        // }
        // left_mg.move_voltage(3000);
        // right_mg.move_voltage(3000);
        // pros::delay(500);
        // for (int i = -1; i < 2; i +=2){
        //     left_mg.move_voltage(3000*i);
        //     right_mg.move_voltage(3000*i*-1);
        //     pros::delay(250);
        //     if (i==1)pros::delay(100);
        // }
        // left_mg.move_voltage(3000);
        // right_mg.move_voltage(3000);
        // pros::delay(500);
        // left_mg.move_voltage(-5000);
        // right_mg.move_voltage(-5000);
        // pros::delay(1000);

        // left_mg.move_voltage(2000);
        // right_mg.move_voltage(2000);
        // pros::delay(1000);

        // left_mg.move_voltage(0);
        // right_mg.move_voltage(0);
        // left_mg.move_voltage(8000);
        // right_mg.move_voltage(8000);
        // pros::delay(500);
        // for (int i = -1; i < 2; i +=2){
        //     left_mg.move_voltage(5000*i);
        //     right_mg.move_voltage(5000*i*-1);
        //     pros::delay(250);
        // }
        logger->log("Starting autonomous");

        if (trajectories.size() == 0){
            robot->followTrajectory(trajectory);
        } else {
            robot->followTrajectories(trajectories);
        }
        // robot->tuneTrackWidth();
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
    return int(((std::pow((double(x) / 5.7), 2.0) -std::abs(std::pow((double(x) / 5.7), 3.0) / 6.0) +(std::pow((double(x) / 5.7), 4.0)/120.0) )*double(x)) / (3.0 * std::abs(double(x))));
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
std::pair<int, int> arcade_drive(int throttle, int steer)
{
    // -------------------------------------------------------------------------
    //  TUNING CONSTANTS  (edit these to suit your robot / controller)
    // -------------------------------------------------------------------------

    // Deadband: joystick inputs whose absolute value falls at or below this
    // threshold are treated as zero.  Eliminates controller drift.
    // Range: [0, 127]  –  typical value: 5–15
    constexpr int DEADBAND = 10;

    // Minimum speed: once a joystick clears the deadband, the output is
    // immediately scaled up to at least this value (avoids a "dead zone" where
    // the motor stalls because the command is too small to overcome friction).
    // Range: [0, 127]  –  typical value: 15–30
    constexpr int MIN_SPEED = 20;

    // Turning curve exponent: applies a power-curve to the *steering* axis so
    // small deflections produce finer control while full deflection still
    // reaches 100 %.  Higher values = gentler at low speeds.
    // Range: (1.0, ∞)  –  typical value: 1.5–3.0  (1.0 = linear / no curve)
    constexpr double TURN_CURVE_EXPONENT = 2.5;

    // Throttle/steer priority: blends throttle vs steering authority when both
    // axes are active simultaneously.  At 1.0 the robot drives straight at full
    // speed even with max steering input; at 0.0 steering gets no headroom at
    // all alongside full throttle.
    // Range: [0.0, 1.0]  –  typical value: 0.5–0.7
    constexpr double THROTTLE_PRIORITY = 0.1;

    // -------------------------------------------------------------------------
    //  INPUT CLAMPING
    // -------------------------------------------------------------------------
    const double rawThrottle = std::clamp(throttle, -127, 127);
    const double rawSteer    = std::clamp(steer,    -127, 127);

    // -------------------------------------------------------------------------
    //  DEADBAND
    //  Zero out inputs that fall within the dead zone.
    // -------------------------------------------------------------------------
    const double filteredThrottle = (std::abs(rawThrottle) <= DEADBAND) ? 0.0 : rawThrottle;
    const double filteredSteer    = (std::abs(rawSteer)    <= DEADBAND) ? 0.0 : rawSteer;

    // -------------------------------------------------------------------------
    //  MINIMUM SPEED SCALING
    //  Remap [deadband+1 … 127] → [MIN_SPEED … 127] so the output jumps to a
    //  usable value the moment the stick clears the deadband.
    // -------------------------------------------------------------------------
    auto applyMinSpeed = [&](double val) -> double
    {
        if (val == 0.0) return 0.0;

        const double sign      = (val > 0.0) ? 1.0 : -1.0;
        const double magnitude = std::abs(val);

        // Normalise to [0, 1] relative to the usable input range
        const double normalised = (magnitude - DEADBAND) / (127.0 - DEADBAND);

        // Rescale to [MIN_SPEED, 127]
        const double scaled = MIN_SPEED + normalised * (127.0 - MIN_SPEED);
        return sign * scaled;
    };

    const double scaledThrottle = applyMinSpeed(filteredThrottle);
    const double scaledSteer    = applyMinSpeed(filteredSteer);

    // -------------------------------------------------------------------------
    //  TURNING CURVE
    //  Apply a power curve to the steering axis only.
    //  Output = sign(x) * (|x| / 127)^EXPONENT * 127
    // -------------------------------------------------------------------------
    double curvedSteer = 0.0;
    if (scaledSteer != 0.0)
    {
        const double sign      = (scaledSteer > 0.0) ? 1.0 : -1.0;
        const double normalised = std::abs(scaledSteer) / 127.0;          // [0, 1]
        const double curved    = std::pow(normalised, TURN_CURVE_EXPONENT); // power curve
        curvedSteer = sign * curved * 127.0;
    }

    // -------------------------------------------------------------------------
    //  THROTTLE / STEER PRIORITY MIXING
    //
    //  The available "budget" for steering shrinks proportionally as the
    //  throttle increases, ensuring full-speed straight driving is reachable.
    //
    //  steering headroom = 127 * (1 - THROTTLE_PRIORITY * |throttle| / 127)
    //  The final mix:
    //      left  = throttle + steerContribution
    //      right = throttle - steerContribution
    // -------------------------------------------------------------------------
    const double throttleNorm = std::abs(scaledThrottle) / 127.0;           // [0, 1]
    const double steerBudget  = 127.0 * (1.0 - THROTTLE_PRIORITY * throttleNorm);
    const double steerContrib = curvedSteer * (steerBudget / 127.0);

    double left  = scaledThrottle + steerContrib;
    double right = scaledThrottle - steerContrib;

    // -------------------------------------------------------------------------
    //  OUTPUT CLAMPING  →  keep within [-127, 127]
    // -------------------------------------------------------------------------
    return {
        static_cast<int>(std::clamp(left,  -127.0, 127.0)),
        static_cast<int>(std::clamp(right, -127.0, 127.0))
    };
}
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
    // ear.input_toggle(true);
    ear2.input_toggle(true);
    double drivetrain_slow = 0.5;
    left_mg.set_brake_mode_all(MOTOR_BRAKE_COAST);
    right_mg.set_brake_mode_all(MOTOR_BRAKE_COAST);
    while (true)
    {
        // ------------- START SKILLS DRIVER CONTROLS --------------------
        if (skills) {
            int left = (master.get_analog(ANALOG_LEFT_Y));
            int right = (master.get_analog(ANALOG_RIGHT_Y));
            left_mg.move(left);
            right_mg.move(right);

            if (master.get_digital(DIGITAL_R1)) // Intake but hold
            {
                intake.set_state(1);
            }
            else if (master.get_digital(DIGITAL_R2)) // Outake to bottom goal
            {
                intake.set_state(2);
            }
            else if (master.get_digital(DIGITAL_L2)) // Outake into middle goal
            {
                intake.set_state(15);
                intake.state_decay(15, 16, 400); // 600
            }
            else if (master.get_digital(DIGITAL_L1)) // Outake to high goal
            {
                intake.set_state(4);
            }
            // else if (master.get_digital(DIGITAL_UP)){
            //     intake.set_state(7);
            // }
            else
            {
                intake.set_state(0); // Stop intake if no buttons are pressed
            }

            // if (master.get_digital(DIGITAL_A)){
            //     intake.stop_color_sorting();
            // }

            if (master.get_digital(DIGITAL_RIGHT)){
                intake.set_intake_speed(6000);
                drivetrain_slow = 0.5;
            }
            else {
                intake.set_intake_speed(120000);
                drivetrain_slow = 1.0;
            }

            if (master.get_digital(DIGITAL_A) && master.get_digital(DIGITAL_L1)){
                robot->followTrajectory(goal_to_matchloader);
            } else if (master.get_digital(DIGITAL_A) && master.get_digital(DIGITAL_R1)){
                robot->followTrajectory(low_goal);
            }

            intake.set_shift(master.get_digital(DIGITAL_Y));

            intake.update();

            little_will.input_toggle(master.get_digital(DIGITAL_DOWN));

            ear2.input_toggle(master.get_digital(DIGITAL_RIGHT));



            pros::delay(Config::DT);
            // ------------- END SKILLS DRIVER CONTROLS --------------------
        } else {
            // ------------- START MATCH DRIVER CONTROLS --------------------

            int dir = (master.get_analog(ANALOG_LEFT_Y));
            int turn = master.get_analog(ANALOG_RIGHT_X);
            std::pair<int, int> controls = arcade_drive(dir, turn);
            left_mg.move(controls.first);
            right_mg.move(controls.second);

            if (master.get_digital(DIGITAL_R1)) // Intake but hold
            {
                intake.set_state(1);
            }
            else if (master.get_digital(DIGITAL_R2)) // Outake to bottom goal
            {
                intake.set_state(2);
            }
            else if (master.get_digital(DIGITAL_DOWN)) // Outake into middle goal
            {
                intake.set_state(3);
            }
            else if (master.get_digital(DIGITAL_L1)) // Outake to high goal
            {
                intake.set_state(4);
            }
            else
            {
                intake.set_state(0); // Stop intake if no buttons are pressed
            }

            if (master.get_digital(DIGITAL_A)){
                intake.stop_color_sorting();
            }

            if (master.get_digital(DIGITAL_RIGHT)){
                intake.set_intake_speed(6000);
                drivetrain_slow = 0.5;
            }
            else {
                intake.set_intake_speed(120000);
                drivetrain_slow = 1.0;
            }

            intake.set_shift(master.get_digital(DIGITAL_Y));

            intake.update();

            little_will.input_toggle(master.get_digital(DIGITAL_RIGHT));
            midescore.set_state(master.get_digital(DIGITAL_B));

            intake_lift.set_state(master.get_digital(DIGITAL_B));
            ear2.set_state(!master.get_digital(DIGITAL_L2));

            pros::delay(Config::DT);
        }
        // ------------- END MATCH DRIVER CONTROLS --------------------
    }
}