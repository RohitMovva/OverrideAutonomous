#ifndef LIFT_HPP
#define LIFT_HPP

#include "pros/motors.hpp"
#include "hardware/pto.hpp"
#include "pros/distance.hpp"
#include "pros/rotation.hpp"
#include <cstdint>

/**
 * @brief Lift + motorised claw.
 *
 *  COMPONENTS
 *   - PTO motors:   the three shared motors that raise the lift when the PTO is in
 *                   LIFT mode (see @ref Pto). The lift claims that mode whenever it
 *                   moves and owns no motor of its own -- when the PTO is in INTAKE
 *                   mode nothing here can move (or hold) the winch.
 *   - Winch rotation sensor: the lift's position reference. Zeroed at the start
 *                   (fully lowered) position in @ref initialize(); every height in
 *                   this class is in winch degrees from there.
 *   - Claw motor:   opens and closes the claw jaws by spinning one way or the other
 *                   (CLAW_DIRECTION sets which; positive = open in this class). There
 *                   are no position targets:
 *                     OPEN   -- runs open at CLAW_OPEN_MV for CLAW_OPEN_MS, then brakes
 *                               and holds where it stopped.
 *                     CLOSED -- runs closed at CLAW_CLOSE_MV for CLAW_CLOSE_MS, then
 *                               keeps squeezing at CLAW_SQUEEZE_MV for as long as it is
 *                               closed, so the piece cannot slip out.
 *                   A move counts as done once its time has passed. The claw starts
 *                   closed.
 *
 *  STARTUP
 *   The lift boots in MATCHLOAD, physically in the matchload pose. The claw starts
 *   closed and holds closed through the first update(), then opens -- unless a
 *   @ref grab has come in, which keeps it closed. The hold is because the auton loop
 *   updates the lift before it reads the route's first action point, and a grab there
 *   should not see the claw start opening for one tick first. The loader sensor is only
 *   armed while the claw is open at MATCHLOAD and done moving.
 *
 *  DRIVER MODEL (barebones)
 *   - Hold @ref move_up() every tick -> lift runs at full voltage. On release the
 *     voltage slews to zero over SLEW_STOP_MS so the stop is not a jolt, then the
 *     lift holds where it is. Upward drive is cut at MAX_HEIGHT_DEG regardless.
 *     From MATCHLOAD the first press makes the mode PRIMED on the spot (claw closes).
 *   - Hold @ref hold_down() every tick -> lift runs down (capped at MAX_LOWER_MV) and
 *     stops on release or at MATCHLOAD_HEIGHT_DEG, where the mode goes back to
 *     MATCHLOAD.
 *   - @ref return_home() -> automatic run back to the matchload pose, used by the
 *     macros and auton.
 *
 *  END-EFFECTOR MODEL
 *   - MATCHLOAD: the rest pose, waiting on the loader at MATCHLOAD_HEIGHT_DEG. Claw
 *                OPEN (CLOSED while a grab holds it). The mode the lift boots in, and
 *                the one every return comes back to.
 *   - PRIMED:    holding a piece. Claw CLOSED.
 *   - SCORING:   the score macro. Claw OPEN. Held until the lift is back at MATCHLOAD.
 *
 *  SCORE MACRO (@ref score, one button press while PRIMED)
 *   1. SCORE_DESCEND: still PRIMED, the lift lowers at SCORE_DESCEND_MV until the tower
 *      distance sensor reads under TOWER_DISTANCE_MM.
 *      @ref score_at instead lowers to a given winch height and ignores the tower sensor,
 *      for a goal the sensor cannot be trusted on (the slanted centre goal reads 127-150 mm
 *      and 9999 with the piece still ~490 deg up: boots 0676, 0679).
 *   2. SCORE_UNLOAD: mode -> SCORING. The winch holds and the claw opens, until the
 *      claw reports OPEN and done.
 *   3. SCORE_RELEASE: holds until the tower sensor no longer reads under the threshold
 *      (the robot drove off) and the loader sensor reads over SCORE_CLEAR_LOADER_MM,
 *      then returns to MATCHLOAD. In autonomous it also returns after
 *      SCORE_RELEASE_TIMEOUT_MS, so a sensor that never clears cannot strand the lift.
 *   Any @ref move_up / @ref hold_down / @ref return_home / @ref stop during the macro cancels it.
 *
 *  MATCHLOAD MACRO (@ref matchload, one button press)
 *   1. MATCHLOAD_MOVE: mode -> MATCHLOAD (claw opens) and the cascade runs to
 *      MATCHLOAD_HEIGHT_DEG, then IDLE. Resting in IDLE in MATCHLOAD keeps the loader
 *      sensor armed, so the cycle re-arms itself after every score.
 *   2. MATCHLOAD_CLAMP: the loader sensor reads under LOADER_DISTANCE_MM -> mode PRIMED
 *      (claw closes). The winch holds for CLAW_CLOSE_MS while the claw closes, so the
 *      lift never pulls the piece off the loader before it is gripped.
 *   3. MATCHLOAD_RAISE: the cascade runs up to MATCHLOAD_PRIME_HEIGHT_DEG, then IDLE.
 *      @ref score works from here; the lift comes back to MATCHLOAD afterwards.
 *   Cancelled by the same commands as the score macro.
 *
 *  GRAB (@ref grab, autonomous action points)
 *   Keeps the claw closed while the lift sits in MATCHLOAD, so a route can hold its
 *   preload from the first node without raising. It also disarms the loader sensor,
 *   since the jaws are not open. Cleared by priming, either driver button,
 *   @ref return_home and @ref stop.
 *
 *  HEIGHT MOVES (@ref raise_to, autonomous action points)
 *   Runs the cascade to a commanded winch height and holds it there. From MATCHLOAD the
 *   mode becomes PRIMED first, exactly as the up button does. Cancelled by the same
 *   commands as the macros, plus either driver button.
 *
 * Call @ref update() every Config::DT ms; nothing moves outside it.
 */
class Lift
{
public:
    enum class State
    {
        IDLE,            ///< holding position, motors braked
        HEIGHT_MOVE,     ///< auton: running to a commanded winch height, then holding there
        RAISING,         ///< driver holding up, full voltage
        SLEWING,         ///< up released, ramping voltage to zero
        LOWERING,        ///< automatic return to the matchload pose (macros / auton)
        LOWERING_HELD,   ///< driver holding down; stops at the matchload pose or on release
        SCORE_DESCEND,   ///< score macro: lowering slowly onto the tower
        SCORE_UNLOAD,    ///< score macro: holding while the claw opens
        SCORE_RELEASE,   ///< score macro: claw open, holding, waiting for the tower to clear
        MATCHLOAD_MOVE,  ///< matchload: running to MATCHLOAD_HEIGHT_DEG
        MATCHLOAD_CLAMP, ///< matchload: piece seen, holding while the claw closes on it
        MATCHLOAD_RAISE  ///< matchload: piece gripped, up to MATCHLOAD_PRIME_HEIGHT_DEG with it
    };

    enum class Mode
    {
        PRIMED,
        SCORING,  ///< claw open on the tower
        MATCHLOAD ///< the rest pose: at MATCHLOAD_HEIGHT_DEG, claw open, waiting on the loader
    };

    /// Claw targets; see "Claw motor" in the class docs for what each one drives
    enum class ClampPos
    {
        CLOSED,
        OPEN
    };

    static constexpr int MAX_VOLTAGE = 12000;
    /// Sign applied to every winch voltage so that positive in this class = up. Flip if
    /// the winch runs the wrong way; leave the motor ports alone since the PTO motors
    /// are shared with the intake.
    static constexpr int WINCH_DIRECTION = -1;
    /// Fastest the winch is allowed to run DOWN, mV (positive number). Applied to every
    /// lowering move, macros included; going up is still allowed MAX_VOLTAGE. TUNE
    static constexpr int MAX_LOWER_MV = 12000;
    /// Sign applied to every claw voltage so that positive in this class = open. Flip
    /// (to -1) if the claw closes when it should open. TUNE
    static constexpr int CLAW_DIRECTION = 1;
    /// Voltage the claw opens at, mV (positive). TUNE
    static constexpr int CLAW_OPEN_MV = 12000;
    /// How long the claw spins open before it brakes and holds. TUNE
    static constexpr uint32_t CLAW_OPEN_MS = 500;
    /// Voltage the claw closes at, mV (positive). TUNE
    static constexpr int CLAW_CLOSE_MV = 12000;
    /// How long the claw closes at CLAW_CLOSE_MV before it drops to CLAW_SQUEEZE_MV and
    /// counts as closed. The matchload macro holds the winch this long. TUNE
    static constexpr uint32_t CLAW_CLOSE_MS = 300;
    /// Voltage the claw keeps closing at while it is closed, mV (positive): enough to
    /// hold a piece, low enough not to cook the motor against it all match. TUNE
    static constexpr int CLAW_SQUEEZE_MV = 3000;
    /// Tower distance reading (mm) under which the lift is "on the tower".
    static constexpr int TOWER_DISTANCE_MM = 150;
    /// Loader distance reading (mm) under which a piece is in the match loader.
    static constexpr int LOADER_DISTANCE_MM = 100;
    /// Score release: the loader sensor must read the robot is far enough
    //// from the goal so that the lift will lower and be clear
    static constexpr int SCORE_CLEAR_LOADER_MM = 150;
    /// Autonomous only: longest SCORE_RELEASE waits for the tower to clear before the
    /// lift returns anyway. Driver control keeps waiting for the sensors. TUNE
    static constexpr uint32_t SCORE_RELEASE_TIMEOUT_MS = 1000;
    /// Descent voltage while the score macro hunts for the tower (negative = down).
    static constexpr int SCORE_DESCEND_MV = -10000;
    /// Height the matchload pose waits at: the rest pose every return comes back to,
    /// and the floor the down button stops at.
    static constexpr double MATCHLOAD_HEIGHT_DEG = 0.0;
    /// Height the matchload macro raises to once the loader sees a piece.
    static constexpr double MATCHLOAD_PRIME_HEIGHT_DEG = 100.0;
    /// How long the post-raise slew takes to bring the voltage to zero
    static constexpr uint32_t SLEW_STOP_MS = 200;
    /// Winch position (deg) at or below which the lift counts as home
    static constexpr double HOME_TOLERANCE_DEG = 5.0;
    /// Winch position (deg) the lift will not be driven above.
    static constexpr double MAX_HEIGHT_DEG = 2000.0;
    /// How close a move to a height has to get to its target before it holds. TUNE
    static constexpr double HEIGHT_TOLERANCE_DEG = 15.0;
    /// Inside this much of the target a move drops to HEIGHT_APPROACH_MV, so it does
    /// not sail past the tolerance band at full voltage. TUNE
    static constexpr double HEIGHT_APPROACH_DEG = 150.0;
    /// Voltage for the last HEIGHT_APPROACH_DEG of a move to a height (positive). TUNE
    static constexpr int HEIGHT_APPROACH_MV = 6000;
    /// Longest a @ref raise_to move (or the matchload raise) runs before it holds wherever
    /// it got to. A winch that cannot reach its target must not keep the PTO -- and so
    /// the intake -- for the rest of the run.
    static constexpr uint32_t HEIGHT_TIMEOUT_MS = 4000;

    Lift(pros::Rotation &winch_rotation, pros::Motor &claw_motor,
         pros::Distance &tower_distance_sensor, pros::Distance &loader_distance_sensor,
         Pto &pto);

    /// Zero the winch rotation sensor at the start position (lift fully lowered, in the
    /// matchload pose), start the claw closing and set brake modes. Call once.
    void initialize();

    /// Call every tick while the up button is held
    void move_up();
    /// Call every tick while the down button is held
    void hold_down();
    /// Automatically return the lift to the matchload pose (macros / auton)
    void return_home();
    /// Run the cascade to a winch height (deg above home) and hold there (auton action
    /// points). Primes first if the lift is sat in MATCHLOAD; see class docs.
    void raise_to(double winch_deg);
    /// Cancel any motion and hold
    void stop();

    /// Start the score macro (see class docs). Ignored unless PRIMED
    void score();
    /// The score macro without the tower sensor: lower to @p height_deg (winch deg above
    /// home) and release there. For a goal whose face the sensor misreads, so the macro
    /// can neither fire early nor wait on a reading that never comes. Already at or
    /// below the height, it releases where it is. Route: lift_state = score plus a
    /// lift_height on the same point (Robot::processAction).
    void score_at(double height_deg);
    /// Start the matchload macro (see class docs). Ignored if already running it
    void matchload();
    /// Keep the claw closed in the matchload pose (auton action points). Only honoured
    /// in MATCHLOAD; held until the lift primes, a driver button, @ref return_home or
    /// @ref stop.
    void grab();
    /// True while @ref grab is holding the claw closed in the matchload pose
    bool is_grabbing() const { return grab_held_; }
    bool is_scoring() const
    {
        return state_ == State::SCORE_DESCEND || state_ == State::SCORE_UNLOAD ||
               state_ == State::SCORE_RELEASE;
    }
    bool is_matchloading() const
    {
        return state_ == State::MATCHLOAD_MOVE || state_ == State::MATCHLOAD_CLAMP ||
               state_ == State::MATCHLOAD_RAISE;
    }
    /// True once the claw has opened on the tower.
    /// SCORE_RELEASE then holds until the tower sensor clears, which only happens when
    /// the robot drives away, so this -- not is_settled() -- is when an auton wait on a
    /// score is over.
    bool score_released() const { return state_ == State::SCORE_RELEASE; }
    /// True while any macro (score or matchload) is running
    bool in_macro() const { return is_scoring() || is_matchloading(); }

    void update();

    State state() const { return state_; }
    Mode mode() const { return mode_; }
    static const char *state_name(State s);
    static const char *mode_name(Mode m);
    bool is_busy() const { return state_ != State::IDLE; }
    /// True whenever the lift needs the shared PTO motors: either it is already moving,
    /// or a move_up()/hold_down() has come in this tick that update() has not consumed
    /// yet. Arbitration must inhibit the intake on THIS, not is_busy() -- is_busy() only
    /// turns true once update() has run, which is a tick after the button was read, and
    /// in that tick the intake would still drive the shared motors at full voltage right
    /// before the gearbox shifts under them.
    bool wants_pto() const { return up_requested_ || down_requested_ || is_busy(); }
    bool is_settled() const { return state_ == State::IDLE; }
    bool is_home() const;
    /// True when the lift is at or above MAX_HEIGHT_DEG
    bool is_at_max_height() const;
    /// True while a @ref raise_to move is running
    bool is_moving_to_height() const { return state_ == State::HEIGHT_MOVE; }
    /// Height the running (or last) move to a height was given, deg above home
    double height_target() const { return height_target_deg_; }
    /// Winch rotation sensor, deg from the zeroed start position (positive = up)
    double get_position() const;
    /// Claw motor encoder, deg from where it was tared at boot (closed). Telemetry only;
    /// nothing is controlled on it.
    double get_claw_position() const;
    /// Claw target currently commanded
    ClampPos clamp_target() const { return clamp_target_; }
    static const char *clamp_name(ClampPos p);
    /// True once the current claw move's time (CLAW_OPEN_MS / CLAW_CLOSE_MS) has passed
    bool clamp_done() const;
    /// Tower distance sensor, mm
    int get_tower_distance_mm() const;
    /// Last voltage this class sent to the PTO motors (mV, after WINCH_DIRECTION)
    int last_winch_mv() const { return last_winch_mv_; }
    /// Last voltage sent to the claw (mV, positive = open, before CLAW_DIRECTION)
    int last_claw_mv() const { return last_claw_mv_; }
    /// True when the tower sensor reads a valid value under TOWER_DISTANCE_MM
    bool tower_in_range() const;
    /// Loader distance sensor, mm
    int get_loader_distance_mm() const;
    /// True when the loader sensor reads a valid value under LOADER_DISTANCE_MM
    bool loader_in_range() const;

private:
    void drive(int millivolts);
    /// Brake the PTO motors (only if they are still in LIFT mode) and hold position
    void hold();
    /// Drive toward @p target_deg and return true (motors stopped) once there. A target
    /// at home runs down at full voltage until is_home(); any other target slows to
    /// HEIGHT_APPROACH_MV inside HEIGHT_APPROACH_DEG and stops within
    /// HEIGHT_TOLERANCE_DEG. Stores the target in height_target_deg_ for LIFTDBG.
    bool run_to(double target_deg);
    /// Loader in range while resting in MATCHLOAD -> take the piece (-> MATCHLOAD_CLAMP).
    /// Called from IDLE, so the sensor stays armed between cycles without the lift
    /// holding the PTO.
    void try_take_from_loader();
    /// Command a claw target, restarting the claw's move clock only when it changes
    void set_clamp(ClampPos pos);
    /// Claw target the current mode holds
    ClampPos mode_clamp() const;
    /// Set the mode's claw target and drive the claw motor for this tick
    void apply_end_effector();
    /// One flat key=value LIFTDBG line per tick, for tests/log_analysis/lift_parser.py
    void log_debug_state() const;

    pros::Rotation &winch_rotation_;
    pros::Motor &claw_motor_;
    pros::Distance &tower_distance_sensor_;
    pros::Distance &loader_distance_sensor_;
    Pto &pto_;

    State state_ = State::IDLE;
    bool up_requested_ = false;   ///< set by move_up(), consumed by update()
    bool down_requested_ = false; ///< set by hold_down(), consumed by update()
    int slew_mv_ = 0;           ///< current voltage during SLEWING
    Mode mode_ = Mode::MATCHLOAD; ///< boots in MATCHLOAD; see "STARTUP" in the class docs
    double height_target_deg_ = 0;       ///< target of the current move to a height
    double score_at_deg_ = -1.0; ///< score_at() release height; < 0 = normal score (tower sensor)
    uint32_t move_start_ms_ = 0;         ///< pros::millis() when HEIGHT_MOVE / MATCHLOAD_RAISE began
    uint32_t release_start_ms_ = 0;      ///< pros::millis() when SCORE_RELEASE began
    /// @ref grab is holding the claw closed in MATCHLOAD. apply_end_effector() clears it
    /// as soon as the lift primes, so it never outlives the rest pose it was set in.
    bool grab_held_ = false;
    /// Keeps the claw at its boot position through the first update(); see "STARTUP"
    bool boot_hold_ = true;
    /// Starts CLOSED because initialize() closes the claw; the second update() opens
    /// it unless a grab has come in
    ClampPos clamp_target_ = ClampPos::CLOSED;
    uint32_t clamp_switch_ms_ = 0;      ///< pros::millis() when the claw target last changed
    int last_winch_mv_ = 0;     ///< for telemetry, see last_winch_mv()
    /// What drive() was last asked for, in this class's convention (positive = up)
    /// rather than the motor's, and captured before the PTO check can zero it. Lets the
    /// LIFTDBG line show a command that never reached the motors; see log_debug_state()
    int last_winch_cmd_mv_ = 0;
    int last_claw_mv_ = 0;       ///< for telemetry, see last_claw_mv()
    bool at_cap_logged_ = false; ///< so the height-cap message prints once, not every tick
};

#endif // LIFT_HPP
