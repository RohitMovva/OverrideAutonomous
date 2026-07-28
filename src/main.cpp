#include "main.h"
#include <malloc.h>  // mallinfo() — heap-usage diagnostics (temporary)

// Robot config
bool skills = false;
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({-18, 19});
pros::MotorGroup right_mg({14, -15});
pros::MotorGroup intake_motors({-20});
pros::MotorGroup winch_motors({-16, 17});                     // two 200 RPM lift winch motors
pros::Motor chainbar_motor(-10, pros::v5::MotorGears::red); // 11W bar pivot, geared 100->50; PLACEHOLDER port
pros::Rotation chainbar_rotation(11);                      // bar-axle angle sensor, 1:1; PLACEHOLDER port
pros::Motor chain_motor(-13, pros::v5::MotorGears::green);  // 5.5W 200 RPM chain, 1:1 end effector; PLACEHOLDER port
pros::Rotation chain_rotation(1);                          // EE-axle angle sensor, 1:1; PLACEHOLDER port

pros::Optical color_sort_os(21);
pros::Distance color_sort_ds(10);
pros::Distance left_distance_sensor(3);
pros::Distance right_distance_sensor(5);
pros::Distance front_distance_sensor(16);

DistanceResetOdometry::DistanceSensor left_sensor(&left_distance_sensor, 5.9272, 0);
DistanceResetOdometry::DistanceSensor right_sensor(&right_distance_sensor, 6.3209, 0);
DistanceResetOdometry::DistanceSensor front_sensor(&front_distance_sensor, 2.8, -4.5);

EnhancedDigitalOut claw(8, false);   // single open/1close claw solenoid; PLACEHOLDER ADI port
EnhancedDigitalOut flipper(3, false);
EnhancedDigitalOut bobby_wheel(7, false);

PIDController winch_pid(300, 0, 0, -12000, 12000, 50.0, 0.3);

// kD is the bar's only source of deceleration BEFORE it reaches a target. With
// kD=0 the return sweep could brake only by overshooting (P * how far past it
// already was), so it crossed the low-zone gate at 560 deg/s and took 36 deg to
// stop -- inside the clearance zone the gate exists to keep it out of, and hard
// enough to recoil the mechanism backwards (log 07262026, 09.58-09.93). At kD=10
// that same 560 deg/s contributes -5600 mV of braking while the bar is still
// short of the target. TUNE: raise if it still overruns, lower if the sweep gets
// sluggish or buzzes -- Lift feeds it the 50 ms windowed rate (9 the velocity
// ERROR on profiled moves), so there is real phase lag to respect.
PIDController chainbar_pid(200, 60, 10, -12000, 12000, 100.0, 0.3, 10.0);

PIDController chain_pid(120, 20, 6, -12000, 12000, 50.0, 0.3);
Lift lift(winch_motors, chainbar_motor, chainbar_rotation, chain_motor, chain_rotation,
          claw, winch_pid, chainbar_pid, chain_pid);
Intake intake(intake_motors, color_sort_os, color_sort_ds);

Trajectory trajectory;
Trajectory goal_to_matchloader;
Trajectory low_goal;

pros::Imu imu_sensor(12);

Logger *logger = Logger::getInstance();

pros::Rotation side_encoder(5);

std::string program_type = "autonomous";

// Routes

std::vector<std::vector<double>> route;
// std::string route_name = "";
std::string route_name = "sawp";
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
    // --- Diagnostic instrumentation (temporary) -----------------------------
    // Logs heap-in-use (mallinfo().uordblks) after each setup step so a post-run
    // SD log shows how far initialize() got and which step consumed memory (the
    // memory hypothesis). Higher usedHeap = more allocated.
    logger->log("=== INITIALIZE START (diagnostic build) === usedHeap=%d",
                mallinfo().uordblks);

    pros::lcd::initialize();
    logger->log("init: lcd::initialize done, usedHeap=%d", mallinfo().uordblks);
    logger->flush();  // persist each step before the next, so a crash pins the spot
    // pros::lcd::clear()


    left_mg.set_encoder_units_all(pros::E_MOTOR_ENCODER_COUNTS);
    right_mg.set_encoder_units_all(pros::E_MOTOR_ENCODER_COUNTS);

    left_mg.tare_position_all();
    right_mg.tare_position_all();

    imu_sensor.reset();
    logger->log("init: imu.reset issued, usedHeap=%d", mallinfo().uordblks);
    logger->flush();

    lift.initialize();
    logger->log("init: lift.initialize done, usedHeap=%d", mallinfo().uordblks);
    logger->flush();

    bobby_wheel.set_value(true);

    DistanceResetOdometry::DistanceSensors distance_sensors = {front_sensor, nullptr, left_sensor, right_sensor};

    auto localization = LocalizationManager::create(
        LocalizationType::DISTANCE_RESET_ODOMETRY,
        left_mg, right_mg, side_encoder, imu_sensor, distance_sensors,
        false, true
    );
    logger->log("init: localization created, usedHeap=%d", mallinfo().uordblks);
    logger->flush();

    ramsete_controller = new RamseteController(
        Config::RAMSETE_B, Config::RAMSETE_ZETA,
        Config::RAMSETE_SCALE_FACTOR);
    drive_controller = new DrivetrainController(
        Config::KS, Config::KV, Config::KA,
        Config::KP, Config::KI, Config::KD,
        Config::KS_ANGULAR, Config::KV_ANGULAR,
        Config::KA_DECEL);

    robot = new Robot(&left_mg, &right_mg, &imu_sensor, &master, drive_controller, ramsete_controller, std::move(localization), &intake);
    logger->log("init: robot created, usedHeap=%d", mallinfo().uordblks);
    logger->flush();

    pros::delay(5);
    trajectory.loadFromFile("/usd/routes/" + route_name + ".txt");
    logger->log("init: trajectory '%s' loaded size=%d, usedHeap=%d",
                route_name.c_str(), trajectory.size(), mallinfo().uordblks);
    logger->flush();
    // trajectory.loadFromVector();
    // if (trajectory.empty())
    // {
    //     logger->log("Error: Route file is empty or could not be opened.");
    //     return;
    // }

    logger->log("Robot initialized");
    logger->flush();
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

    if (program_type == "autonomous")
    {
        logger->log("Starting autonomous");

        if (trajectories.size() == 0){
            robot->followTrajectory(trajectory);
        } else {
            robot->followTrajectories(trajectories);
        }
    }
    left_mg.set_brake_mode_all(MOTOR_BRAKE_BRAKE);
    right_mg.set_brake_mode_all(MOTOR_BRAKE_BRAKE);
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
    bobby_wheel.set_value(false);
    double drivetrain_slow = 0.5;
    left_mg.set_brake_mode_all(MOTOR_BRAKE_COAST);
    right_mg.set_brake_mode_all(MOTOR_BRAKE_COAST);
    long long count = 0;
    int intake_state = 0;
    int intake_pressed_dur = 0;
    bool intaking = false;

    // --- Diagnostic instrumentation (temporary) -----------------------------
    // A post-freeze SD log (/usd/robot_log_*.txt) then shows: heap-in-use over
    // time (a leak/exhaustion makes usedHeap climb), the worst loop period/work
    // time per window (a blocking call or stall spikes these), and the last
    // heartbeat count before the log stops (pinpoints when it died). Logging is
    // queued to the background logger task, so it adds no blocking SD I/O here.
    uint32_t last_loop_ms = pros::millis();
    uint32_t worst_period_ms = 0;
    uint32_t worst_work_ms = 0;
    logger->log("=== OPCONTROL START (diagnostic build) === usedHeap=%d",
                mallinfo().uordblks);

    while (true)
    {
        uint32_t iter_start = pros::millis();
        uint32_t period = iter_start - last_loop_ms;
        last_loop_ms = iter_start;
        if (period > worst_period_ms)
            worst_period_ms = period;
        // Nominal period is ~10 ms; flag any single stalled iteration the instant
        // it happens so the log's last line lands near the freeze.
        if (period > 30)
            logger->log("LOOP STALL: period=%u ms at count=%lld usedHeap=%d",
                        period, count, mallinfo().uordblks);

        // Heartbeat (~4 Hz) with heap-in-use and the worst timing this window.
        if (count % 25 == 0)
        {
            logger->log("hb count=%lld usedHeap=%d worstPeriod=%u worstWork=%u",
                        count, mallinfo().uordblks,
                        worst_period_ms, worst_work_ms);
            worst_period_ms = 0;
            worst_work_ms = 0;
        }

        // Update the LCD at ~4 Hz, not every 10ms — hammering pros::lcd::print
        // (an LVGL wrapper) from opcontrol races the LVGL render daemon.
        if (count % 25 == 0)
            pros::lcd::print(1, "count: %lld", count);
        ++count;
        left_mg.move(master.get_analog(ANALOG_LEFT_Y));
        right_mg.move(master.get_analog(ANALOG_RIGHT_Y));

        // R1 is a combined tap/hold control. Disambiguate on RELEASE by how
        // long it was held so a hold-to-outtake never corrupts the toggle:
        //   - tap  (< 150 ms) -> toggle intake on/off
        //   - hold (>= 150 ms) -> outtake while held, then restore toggle state
        // `intaking` is the persistent toggle; it is only ever flipped by a tap.
        if (master.get_digital(DIGITAL_R1))
        {
            intake_pressed_dur += 10;
            if (intake_pressed_dur >= 150)
                intake_state = 2; // held long enough -> outtake
        }
        else
        {
            if (intake_pressed_dur > 0 && intake_pressed_dur < 150)
                intaking = !intaking; // it was a tap -> flip the toggle
            // On any release (tap or after a hold) settle back to the toggle state.
            if (intake_pressed_dur > 0)
                intake_state = intaking ? 1 : 0;
            intake_pressed_dur = 0;
        }
        intake.set_state(intake_state);


        // Lift: hold L1 to advance the coordinated lift/chainbar raise sequence
        // (starts from primed). It advances while held and freezes wherever it is
        // on release; the winch is otherwise fully owned by lift.update().
        if (master.get_digital(DIGITAL_L1))
        {
            lift.move_lift_up();
        }
        // Hold L2 to retreat the raise sequence back toward primed -- a plain manual
        // reverse (no release / auto-return, unlike DOWN/score). Raising (L1)
        // wins if both are held.
        if (master.get_digital(DIGITAL_L2))
        {
            lift.move_lift_down();
        }

        // Chainbar macros: B primes (grab + bar 45, winch flat), Y returns to
        // loading (claw open), DOWN scores (swings the bar out of primed, extends
        // the cascade to full height, releases, then returns to the pose it primed
        // from).
        //
        // B primes from EITHER resting pose -- loading or matchload -- so the driver
        // uses the same button whichever one they picked up from.
        if (master.get_digital_new_press(DIGITAL_B))
        {
            lift.prime();
        }
        // X is the matchload button. Sitting at a resting pose it swaps to the other
        // one (loading <-> matchload); from anywhere else -- primed, part way up the
        // sequence, mid-score -- it returns to matchload, the way Y returns to
        // loading. Every path runs the lift-clearance choreography, so it is a macro
        // like the rest, not an instant pose change.
        //
        // A score started from matchload also RETURNS to matchload on its own, so X
        // is only needed to change your mind mid-cycle.
        // UP runs the BACK cycle -- the other side of the mechanism, reached without
        // swinging the bar the long way round to the front primed pose. From a resting
        // pose it is the back prime (grab + move onto the BOTTOM back pose); pressed
        // again it toggles BOTTOM <-> TOP, each move running its own clearance
        // choreography. Once parked, L1/L2 jog the cascade freely, DOWN scores with the
        // same pressure detection, and RIGHT flips the end effector between the back
        // orientations. Y/X still bring it home.
        else if (master.get_digital_new_press(DIGITAL_UP))
        {
            lift.back_prime();
        }
        else if (master.get_digital_new_press(DIGITAL_R2))
        {
            lift.toggle_matchload();
        }
        else if (master.get_digital_new_press(DIGITAL_Y))
        {
            lift.return_to_loading();
        }
        else if (master.get_digital_new_press(DIGITAL_DOWN))
        {
            lift.score();
        }
        // RIGHT flips the end-effector orientation used by the raised poses
        // (270 <-> 90) so a piece can be presented from either side. Takes effect
        // at once -- before priming, while primed, or part-way up the sequence --
        // and sticks until pressed again.
        else if (master.get_digital_new_press(DIGITAL_RIGHT))
        {
            lift.toggle_scoring_orientation();
        }

        // A manually releases the piece wherever the lift happens to be -- primed,
        // part-way up the sequence, or mid-score -- without moving anything or
        // changing the mode. The next prime/score re-clamps.
        if (master.get_digital_new_press(DIGITAL_A)){
            lift.open_claw();
        }


        intake.update();
        lift.update();

        // Work time this iteration (excludes the 10 ms sleep). A blocking device
        // read or SD stall shows up here as a spike even if the period looks fine.
        uint32_t work_ms = pros::millis() - iter_start;
        if (work_ms > worst_work_ms)
            worst_work_ms = work_ms;

        pros::delay(10);
        // ------------- END MATCH DRIVER CONTROLS --------------------
    }
}