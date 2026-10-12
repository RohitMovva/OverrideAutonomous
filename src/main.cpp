#include "main.h"
#include <malloc.h>  // mallinfo() — heap-usage diagnostics (temporary)
#include <array>
#include <cmath>

// LLEMU debug lines. Any task may format them; only LVGL's own task draws them.
// PROS builds LVGL with LV_OS_NONE, so lv_lock() is a no-op and llemu's lcd_print
// calls LVGL with no lock at all. Printing straight from opcontrol() raced the
// display task's redraw of the route selector page as it was hidden, and froze
// driver control on its first tick (boots 0323, 0325-0328, 0330, 0331).
static constexpr int LCD_DEBUG_LINES = 8;
static pros::Mutex lcd_debug_mutex;
static std::array<std::string, LCD_DEBUG_LINES> lcd_debug_text;
static std::array<bool, LCD_DEBUG_LINES> lcd_debug_dirty{};

/// Queue a formatted LLEMU line; drawn on LVGL's task by lcdDebugTick()
__attribute__((format(printf, 2, 3)))
static void lcdDebugPrint(int line, const char *format, ...)
{
    if (line < 0 || line >= LCD_DEBUG_LINES)
        return;
    char buffer[64];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    lcd_debug_mutex.take();
    if (lcd_debug_text[line] != buffer)
    {
        lcd_debug_text[line] = buffer;
        lcd_debug_dirty[line] = true;
    }
    lcd_debug_mutex.give();
}

/// lv_timer callback: runs on LVGL's task, the only place llemu is safe to call
static void lcdDebugTick(lv_timer_t *)
{
    std::array<std::string, LCD_DEBUG_LINES> text;
    std::array<bool, LCD_DEBUG_LINES> dirty{};
    // Try-lock only, so a writer can never stall the display
    if (!lcd_debug_mutex.take(5))
        return;
    for (int i = 0; i < LCD_DEBUG_LINES; ++i)
    {
        if (lcd_debug_dirty[i])
            text[i] = lcd_debug_text[i];
        dirty[i] = lcd_debug_dirty[i];
        lcd_debug_dirty[i] = false;
    }
    lcd_debug_mutex.give();
    for (int i = 0; i < LCD_DEBUG_LINES; ++i)
    {
        if (dirty[i])
            pros::lcd::set_text(i, text[i]);
    }
}

/// Tare the drive encoders and wait for them to report zero/timeout
static bool tareDriveEncodersBlocking(pros::MotorGroup &left, pros::MotorGroup &right,
                                      int timeout_ms = 250)
{
    left.tare_position_all();
    right.tare_position_all();

    const auto settled = [](const std::vector<double> &positions)
    {
        for (double pos : positions)
        {
            if (std::isfinite(pos) && std::abs(pos) > 5.0)
            {
                return false;
            }
        }
        return true;
    };

    for (int waited = 0; waited < timeout_ms; waited += 10)
    {
        pros::delay(10);
        if (settled(left.get_position_all()) && settled(right.get_position_all()))
        {
            return true;
        }
    }
    return false;
}

// Robot config
bool skills = false;
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({-14, -16});
pros::MotorGroup right_mg({12, 15});

pros::MotorGroup pto_motors({-18, -19, 17});                     ///< shared lift / intake power (all 3)
pros::Rotation lift_winch_rotation(13);                          ///< on the winch axle; lift height reference (PLACEHOLDER port)
pros::Motor lift_claw_motor(7, pros::v5::MotorGears::green);    ///< claw open/close (Lift::CLAW_DIRECTION)

EnhancedDigitalOut sunroof_solenoid(5, false);
EnhancedDigitalOut pto_solenoid(3, false);
EnhancedDigitalOut intake_lift_solenoid(2, false);
EnhancedDigitalOut sticks(1, false);
pros::Distance intake_distance_sensor(16);     ///< watches for a piece under the raised intake
pros::Distance lift_tower_distance_sensor(3); ///< on the end effector, sees the tower while scoring
pros::Distance lift_loader_distance_sensor(4); ///< sees a piece in the match loader while the lift waits in MATCHLOAD
pros::Optical roller_optical(2);                ///< reads the field roller colour
pros::Rotation roller_rotation(9);              ///< on the roller axle; roll-boundary reference

// Wall-reset distance sensors (DistanceResetOdometry). TUNE ports and offsets here.
// Offsets are in inches from the tracking centre to the sensor face, measured relative
// to each sensor's beam (see DistanceResetOdometry::DistanceSensor):
//   ALONG  = along the beam, positive toward the wall (face outside the centre)
//   ACROSS = across the beam, positive to the beam's RIGHT when looking along it
// Gates and wall position live in config.hpp (DISTANCE_RESET_*).
// 2026-10-09: the tracking centre moved 1.75 in back with the single-traction rebuild
// (config.hpp, LATERAL_WHEEL_OFFSET), so every sensor sits 1.75 in further forward of it:
// front ALONG 3 -> 4.75, left ACROSS 1 -> 2.75 (its beam's right is forward), right ACROSS
// -2 -> -3.75 (its beam's right is backward). Keep make_suite.py RESET_SENSORS in step.
constexpr int FRONT_DISTANCE_PORT = 11;
constexpr double FRONT_DISTANCE_ALONG = 4.75;
constexpr double FRONT_DISTANCE_ACROSS = 5.75; ///< 5.75 in to the robot's right

constexpr int LEFT_DISTANCE_PORT = 8;
constexpr double LEFT_DISTANCE_ALONG = 5.5;
constexpr double LEFT_DISTANCE_ACROSS = 2.75;

constexpr int RIGHT_DISTANCE_PORT = 1;
constexpr double RIGHT_DISTANCE_ALONG = 6.0;
constexpr double RIGHT_DISTANCE_ACROSS = -3.75;

pros::Distance front_distance_sensor(FRONT_DISTANCE_PORT);
pros::Distance left_distance_sensor(LEFT_DISTANCE_PORT);
pros::Distance right_distance_sensor(RIGHT_DISTANCE_PORT);

DistanceResetOdometry::DistanceSensor front_sensor(&front_distance_sensor, FRONT_DISTANCE_ALONG, FRONT_DISTANCE_ACROSS);
DistanceResetOdometry::DistanceSensor left_sensor(&left_distance_sensor, LEFT_DISTANCE_ALONG, LEFT_DISTANCE_ACROSS);
DistanceResetOdometry::DistanceSensor right_sensor(&right_distance_sensor, RIGHT_DISTANCE_ALONG, RIGHT_DISTANCE_ACROSS);

Pto pto(pto_motors, pto_solenoid, Pto::Mode::LIFT);
Lift lift(lift_winch_rotation, lift_claw_motor, lift_tower_distance_sensor,
          lift_loader_distance_sensor, pto);
Intake intake(pto, intake_lift_solenoid, intake_distance_sensor);
RollerMech roller_mech(pto, roller_optical, roller_rotation);

Trajectory trajectory;
Trajectory goal_to_matchloader;
Trajectory low_goal;

pros::Imu imu_sensor(20);

Logger *logger = Logger::getInstance();

// Tracking wheels (Odometry). PLACEHOLDER ports and directions: a negative port reverses
// the sensor. Pushing the robot forward must raise vertical_encoder and pushing it to
// its left must raise lateral_encoder; test_tracking_wheel_directions flags a reversed
// one from any log. Diameters and offsets are in config.hpp.
pros::Rotation vertical_encoder(6); ///< 2 in forward/back wheel, on the tracking centre line
pros::Rotation lateral_encoder(-5);  ///< 2.75 in sideways wheel, ~3 in in front of the tracking centre

std::string program_type = "autonomous";

// Routes

std::vector<std::vector<double>> route;
// Default route when the card has no saved choice yet. The brain-screen selector
// (RouteSelector) lists /usd/routes and remembers the last pick in
// /usd/route_select.txt, so this is only a first-boot fallback.
std::string route_name = "turn_test";
RouteSelector route_selector(trajectory);
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
    lv_timer_create(lcdDebugTick, 100, nullptr);
    logger->log("init: lcd::initialize done, usedHeap=%d", mallinfo().uordblks);
    logger->flush();  // persist each step before the next, so a crash pins the spot
    // pros::lcd::clear()


    left_mg.set_encoder_units_all(pros::E_MOTOR_ENCODER_COUNTS);
    right_mg.set_encoder_units_all(pros::E_MOTOR_ENCODER_COUNTS);

    if (!tareDriveEncodersBlocking(left_mg, right_mg))
    {
        // Not fatal -- odometry still works off deltas, and its implausible-delta guard
        // will absorb the tare if it lands later. But every absolute position is now
        // offset by an unknown amount, so say so rather than starting a match quietly.
        logger->logError("init: drive encoders did NOT tare within timeout; "
                         "odometry baseline offset is unknown");
    }
    logger->flush();

    // Rotation sensors default to a 20 ms data rate, so at the 10 ms odometry tick the
    // tracking wheels alternated between a stale zero and a two-tick jump (boot 989).
    vertical_encoder.set_data_rate(5);
    lateral_encoder.set_data_rate(5);

    imu_sensor.reset();
    logger->log("init: imu.reset issued, usedHeap=%d", mallinfo().uordblks);
    logger->flush();

    lift.initialize();
    roller_mech.set_target_color(RollerMech::Color::BLUE); // TODO: pick from alliance selector

    logger->log("init: lift.initialize done, usedHeap=%d", mallinfo().uordblks);
    logger->flush();

    DistanceResetOdometry::DistanceSensors distance_sensors = {front_sensor, nullptr, left_sensor, right_sensor};

    // create() returns nullptr, rather than failing to compile, when the arguments match
    // no constructor. Pin the arguments below to DistanceResetOdometry's signature.
    static_assert(std::is_constructible_v<DistanceResetOdometry,
                                          pros::MotorGroup &, pros::MotorGroup &,
                                          pros::Rotation &, pros::Rotation &, pros::Imu &,
                                          DistanceResetOdometry::DistanceSensors &, bool, bool>,
                  "LocalizationManager::create arguments no longer match DistanceResetOdometry");
    auto localization = LocalizationManager::create(
        LocalizationType::DISTANCE_RESET_ODOMETRY,
        left_mg, right_mg, vertical_encoder, lateral_encoder, imu_sensor, distance_sensors,
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

    robot = new Robot(&left_mg, &right_mg, &imu_sensor, &master, drive_controller, ramsete_controller, std::move(localization), &intake, &lift, &roller_mech);
    logger->log("init: robot created, usedHeap=%d", mallinfo().uordblks);
    logger->flush();

    pros::delay(5);
    route_selector.initialize(route_name);
    logger->log("init: route selector up, selected '%s', usedHeap=%d",
                route_selector.selected().c_str(), mallinfo().uordblks);
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
void disabled() {
    lift.stop();
    intake.set_state(0);
    intake.lower_lift();
    // Back to the route picker between runs (opcontrol hides it).
    route_selector.show();
}

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
    route_selector.show();
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

    // The roller only runs where the route enables it (action slot `roller`), and may
    // shift the PTO itself since no driver is running the intake to do it.
    roller_mech.set_enabled(false);
    roller_mech.set_shifts_pto(true);

    if (program_type == "autonomous")
    {
        logger->log("Starting autonomous");

        if (trajectories.size() == 0){
            // Runs whatever the brain-screen selector has loaded; a selection made
            // during the run is picked up after it.
            if (route_selector.prepareForRun())
            {
                robot->followTrajectory(trajectory);
            }
            route_selector.finishRun();
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
    double drivetrain_slow = 0.5;
    // Uncover the LLEMU debug lines the loop below prints to.
    route_selector.hide();
    // Driver control: always armed, but never shifts the PTO (autonomous changes both)
    roller_mech.set_enabled(true);
    roller_mech.set_shifts_pto(false);
    left_mg.set_brake_mode_all(MOTOR_BRAKE_COAST);
    right_mg.set_brake_mode_all(MOTOR_BRAKE_COAST);
    long long count = 0;
    int intake_state = 0;
    int intake_pressed_dur = 0;
    bool intaking = false;

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
        // Check to see if anything is blocking
        if (period > 30)
            logger->log("LOOP STALL: period=%u ms at count=%lld usedHeap=%d",
                        period, count, mallinfo().uordblks);

        if (count % 25 == 0)
        {
            logger->log("hb count=%lld usedHeap=%d worstPeriod=%u worstWork=%u",
                        count, mallinfo().uordblks,
                        worst_period_ms, worst_work_ms);
            worst_period_ms = 0;
            worst_work_ms = 0;
        }

        if (robot)
            robot->logDriveHealth("driver");

        // Update the LCD at ~4 Hz
        if (count % 25 == 0)
        {
            lcdDebugPrint(1, "count: %lld", count);
            lcdDebugPrint(2, "Lift %s / %s", Lift::state_name(lift.state()), Lift::mode_name(lift.mode()));
            lcdDebugPrint(3, "winch %.1f  claw %s %.0f deg %d mV", lift.get_position(),
                          Lift::clamp_name(lift.clamp_target()), lift.get_claw_position(),
                          lift.last_claw_mv());
            lcdDebugPrint(4, "tower %d  loader %d mm  home=%d max=%d", lift.get_tower_distance_mm(),
                          lift.get_loader_distance_mm(), lift.is_home() ? 1 : 0, lift.is_at_max_height() ? 1 : 0);
            lcdDebugPrint(5, "PTO %s  intake st=%d lift=%d", pto.mode() == Pto::Mode::LIFT ? "LIFT" : "INTAKE",
                          intake.get_state(), intake.is_lifted() ? 1 : 0);
            // Commanded = what the code sent; measured = what the motors report back
            lcdDebugPrint(6, "cmd mV: winch %d  pto %d", lift.last_winch_mv(), pto.last_command_mv());
            {
                const auto pto_v = pto_motors.get_voltage_all();
                lcdDebugPrint(7, "meas mV pto: %d %d %d",
                              pto_v.size() > 0 ? pto_v[0] : 0, pto_v.size() > 1 ? pto_v[1] : 0,
                              pto_v.size() > 2 ? pto_v[2] : 0);
            }
        }
        ++count;
        left_mg.move(master.get_analog(ANALOG_LEFT_Y));
        right_mg.move(master.get_analog(ANALOG_RIGHT_Y));

        // ------------- INTAKE: R1 tap toggles intake, R1 hold outtakes -------------
        if (master.get_digital(DIGITAL_R1))
        {
            intake_pressed_dur += 10;
            if (intake_pressed_dur >= 300)
                intake_state = 2; // held long enough -> outtake
        }
        else
        {
            if (intake_pressed_dur > 0 && intake_pressed_dur < 300)
                intaking = !intaking; // it was a tap -> flip the toggle
            // On any release (tap or after a hold) settle back to the toggle state.
            if (intake_pressed_dur > 0)
                intake_state = intaking ? 1 : 0;
            intake_pressed_dur = 0;
        }

        // Intake raise. Will drop itself once the distance sensor sees a stack come through
        if (master.get_digital_new_press(DIGITAL_R2))
            intake.raise_lift();

        // ------------- LIFT: L1 hold = up, L2 hold = down, Y = score macro -------------
        if (master.get_digital(DIGITAL_L1))
            lift.move_up();
        if (master.get_digital(DIGITAL_L2))
            lift.hold_down();
        if (master.get_digital_new_press(DIGITAL_Y))
            lift.score();
        if (master.get_digital_new_press(DIGITAL_B))
            lift.matchload();

        if (master.get_digital_new_press(DIGITAL_DOWN))
            sticks.toggle();
            
        if (master.get_digital_new_press(DIGITAL_RIGHT)){
            sunroof_solenoid.toggle();
        }

        // Motors shared by PTO so winch takes priority if moving. Commanding the lift
        // cancels the intake outright rather than just pausing it: the driver asked for
        // the lift, so releasing L1 must leave the intake stopped instead of handing the
        // PTO back and spinning it up again underneath them. R1 is likewise dead while
        // the lift holds the PTO -- the toggle is cleared again on the next tick.
        if (lift.wants_pto())
        {
            intaking = false;
            intake_state = 0;
        }
        intake.set_state(intake_state);
        // wants_pto(), not is_busy(): move_up()/hold_down() above only set a flag, so on
        // the tick the button is first read is_busy() is still false and the intake would
        // get one more full-voltage tick on the shared motors before the lift shifts.
        intake.set_inhibited(lift.wants_pto());
        // Roller mech is lowest priority
        roller_mech.set_inhibited(lift.wants_pto() || intake.get_state() != 0);

        intake.update();
        roller_mech.update();
        lift.update();

        // Work time this iteration (excludes the 10 ms sleep)
        uint32_t work_ms = pros::millis() - iter_start;
        if (work_ms > worst_work_ms)
            worst_work_ms = work_ms;

        pros::delay(10);
    }
}
