#ifndef LIFT_HPP
#define LIFT_HPP

#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "controllers/pid_controller.hpp"
#include "hardware/enhanced_digital_out.hpp"
#include <cstdint>

/**
 * @brief Continuous lift + chainbar
 *
 *  COMPONENTS
 *   - Winch:    1 11W and 1 5.5W motors that control the continuous lift
 *   - Chainbar: one 11W motor geared 100->50 RPM that pivots the bar at the top
 *               of the lift. Also has a rot sensor on the axle for readings
 *   - Chain:    one 5.5W 200 RPM motor, 1:1 to the end effector, with a rot
 *               sensor on the axle for readings.
 *   - Claw:     one open/close solenoid. Closes on @ref prime() to grab a piece,
 *               stays closed through the raise, and opens again when @ref score()
 *               detects the piece meeting the tower (or on @ref return_to_loading())
 *               A lift state carries its own claw position too, see @ref LiftState
 */
class Lift
{
public:
    /**
     * @brief Dictates side robot scores on in auton lift states
     */
    enum class Side
    {
        FRONT,
        BACK
    };

    /**
     * @brief Lift state: three angles (winch, chainbar, chain) plus the claw solenoid state
     *
     * Each row in @ref FRONT_LIFT_STATES or @ref BACK_LIFT_STATES becomes one
     * of these
     *
     * Used for all the auton states so we can easily add/extend more
     *
     * each is range-checked against its soft limits when building
     */
    struct LiftState
    {
        const char *key;  ///< name passed to @ref run_lift_state()
        double winch_pos; ///< continuous height, motor deg, LOADING-referenced
        double bar_deg;   ///< chainbar angle, output deg (LOADING = 0)
        double ee_deg;    ///< claw angle, EE deg (1:1)
        /// Claw solenoid the state commands: CLAW_CLOSED_STATE (clamped) or
        /// CLAW_OPEN_STATE (released). Written as soon as the state is commanded,
        /// BEFORE the move runs -- see @ref run_lift_state()
        bool claw_state;
    };

    /**
     * @brief One step of a lift CHAIN: a pose to go to, and how long to sit on it.
     *
     * A chain is a scripted sequence of lift states the mechanism plays out on its own,
     * so a single action point can run a whole pickup->score cycle instead of the route
     * having to place one action point per pose (and guess at the travel time between
     * them, which is what @p wait_settled removes the need for)
     *
     * Timing per step, which is the point of the whole thing:
     *   - @p wait_settled true: the step is not "done" until the mechanism is actually ON
     *     the pose (or @p settle_timeout_ms expires), and only THEN does @p hold_ms start
     *   - @p wait_settled false: @p hold_ms is measured from the moment the step is
     *     COMMANDED, so the next one fires on a fixed clock whether or not this arrived.
     *     What you want for a pose you only need to pass THROUGH
     */
    struct ChainStep
    {
        Side side;        ///< which lift state table @ref key is looked up in
        const char *key;  ///< a `LiftState::key`, or nullptr for a pure dwell (no move)
        /// How long to sit on this step before the next one is commanded. Measured from
        /// ARRIVAL when @ref wait_settled is set, otherwise from the command
        uint32_t hold_ms;
        /// Gate @ref hold_ms on the mechanism reaching the pose, rather than starting it
        /// the moment the step is commanded
        bool wait_settled;
        /// Backstop on that gate, so a jammed mechanism cannot hang the chain forever.
        /// Ignored when @ref wait_settled is false
        uint32_t settle_timeout_ms;
    };

    /**
     * @brief A named sequence of @ref ChainStep, selected by @ref id.
     *
     * Rows live in @ref LIFT_CHAINS and are validated at compile time
     */
    struct LiftChain
    {
        /// Action value that selects this chain. Must satisfy @ref is_chain_id(), so the
        /// chain band and the single-pose action values can never collide
        double id;
        const char *name;        ///< what the LIFTCHAIN log lines call it
        const ChainStep *steps;  ///< the steps, run in order
        int step_count;
    };

    /// Action values with |value| >= this select a chain rather than a normal lift pose
    static constexpr double LIFT_CHAIN_ID_MIN_ABS = 10.0;
    /// Prevents chain step from running too long
    static constexpr uint32_t LIFT_CHAIN_SETTLE_TIMEOUT_MS = 2500;

    /**
     * @brief Construct the lift/chainbar mechanism.
     *
     * @param winch_motors    Two 200 RPM winch motors (spin same axle)
     * @param chainbar_motor  11W bar-pivot motor (geared 100->50)
     * @param chainbar_rotation Rotation sensor on the bar axle (1:1, output deg)
     * @param chain_motor     5.5W chain motor (1:1 to the end effector)
     * @param chain_rotation  Rotation sensor on the end-effector axle (1:1, EE deg)
     * @param claw            Claw solenoid (open = released, closed = clamped)
     * @param winch_pid       Winch position-hold PID (error deg, output mV)
     * @param chainbar_pid    Chainbar position PID (error output-deg, output mV)
     * @param chain_pid       Chain position PID (error deg, output mV)
     */
    Lift(pros::MotorGroup &winch_motors,
         pros::Motor &chainbar_motor,
         pros::Rotation &chainbar_rotation,
         pros::Motor &chain_motor,
         pros::Rotation &chain_rotation,
         EnhancedDigitalOut &claw,
         PIDController winch_pid,
         PIDController chainbar_pid,
         PIDController chain_pid);

    /**
     * @brief Configure encoder units, brake modes, and tares winch IMEs.
     * Doesn't tare down the rotation sensors on the chainbar (chain and bar part)
     * since rotations preserve rotation even if program restarts/motor disconnects
     */
    void initialize();

    // Driver macros

    /**
     * @brief Move to the primed pose (bottom of front scoring)
     * This function is used to first trigger the sequence,
     * see @ref advance_prime_sequence to view the full process
     */
    void prime();

    /**
     * @brief Start back scoring, and cycle between the back states.
     *
     * Behavior
     *  - From a resting pose (e.g. STOW or MATCHLOAD) it enters the cycle initially
     *    if triggered from loading then the winch will lift to not clip onto intake
     *  - Pressed again while already active will cycle chainbar state between the three
     *    heights we are able to score at (low, mid, and high)
     *  - Ignored mid-return, where the bar isn't parked
     *
     * Once there: can run
     *  - @ref move_lift_up() / @ref move_lift_down() commands winch at max voltage
     *  - @ref score() runs release macro
     *  - @ref toggle_scoring_orientation() flips claw between orientations
     */
    void back_prime();

    /**
     * @brief Move to the MATCHLOAD pose (from STOW) or back to STOW (from MATCHLOAD).
     *
     * Matchloading is the back state that lets us pick up pin/cup stacks
     * Open claw, chain bar at ~-45, continuous lift down
     * Most often used for matchloading (since front to back cycles)
     */
    void toggle_matchload();

    /**
     * @brief Raise this iteration. Hold-to-run: one call is one Config::DT of travel.
     *
     * If front primed (now deprecated since switch to back scoring soley)
     *  - Will run through the front prime keyframes, essentially a synchronized
     *    sequence of raise/lowering the winch
     * If back primed:
     *  - Will move winch up/down at max voltage
     *
     * Either way, letting go stops the mechanism where it stands and holds it there
     */
    void move_lift_up();

    /**
     * @brief Lower this iteration. The exact reverse of @ref move_lift_up(), either cycle.
     */
    void move_lift_down();

    /**
     * @brief Score: release the piece where the mechanism is standing.
     *
     * Only operates when in a primed state
     * When claw is in normal orientation: releases claw (simple)
     * When claw is in alt orientation: Claw piston would hit the tower if
     * the claw only release and drive drove back. Instead we can lift the cascade
     * a bit then lift the chain bar to -180 (and brake it there) to auto avoid the
     * tower
     */
    void score();

    /**
     * @brief Return to the STOW (loading) pose from any state.
     *
     * Opens the claw (releases the cup-pin), ensures the lift is high enough
     * to clear intake poly then swings arm back to loading state
     * The swing eases back
     *
     */
    void return_to_loading();

    /**
     * @brief Go to the MATCHLOAD pose from any state, opening claw.
     *
     * If triggered from loading it will move the cascade to avoid intake
     * If triggered from other poses that don't clip into low zone, just swings
     * over chain bar and moves cascade to WINCH_MATCHLOAD_POS
     */
    void return_to_matchload();

    /**
     * @brief Swing the chainbar to @ref CHAINBAR_BAR_POSE_DEG and hold it there.
     *
     * This is used for triggering our roller mechanism. It is attached to our 
     * chainbar using latex tubing, this allows us to start in size. When our driver
     * wants to play rollers, this functionality is critical to deploy the mechanism
     */
    void bar_pose();

    /**
     * @brief Toggle the claw where it is, without moving anything else.
     *
     * Allows driver to override commands/macros to manually pick up or 
     * release game pieces
     * @ref run_lift_state() writes whatever its row asks for)
     */
    void open_claw();

    /**
     * @brief Flip the claw orientation used by the raised poses.
     *
     * Stores which one is picked (default or alt), not an angle, so the same button
     * works for front and back scoring and the choice carries into the next prime
     * Takes effect right away wherever a raised pose is commanded, so it can be flipped
     * before priming or part way through a cycle
     * Picking alt also arms the retreat after @ref score(), see SCORE_ALT_RETREAT_BAR_DEG
     */
    void toggle_scoring_orientation();

    /**
     * @brief Startup sequence: move from the power-on START pose to STOW.
     *
     * The bar can't swing straight from start to loading without hitting the intake
     * poly, so this runs five steps (one per @ref update()):
     *  - rotate the bar clear of the start pose
     *  - raise the lift to give the bar room to swing
     *  - swing the bar and claw down to just past loading
     *  - lower the lift back down with the bar held there
     *  - bring the bar up onto loading and settle in STOW
     *
     * Deprecated
     */
    void start_to_loading();


    /**
     * @brief True if nothing has commanded the lift yet (still in the power-on pose).
     *
     * Decides whether @ref start_to_loading() is run at the start of driver
     */
    bool needs_startup() const;

    // Autonomous interface (keyed macros)

    /**
     * @brief Run one of the driver macros above by name (for auton).
     *
     * Each key calls the macro that already exists so its gating
     * and logging are unchanged
     *
     *   "prime"             @ref prime() (FRONT) / @ref back_prime() (BACK)
     *   "matchload"         @ref toggle_matchload()
     *   "score"             @ref score()
     *   "return"            @ref return_to_loading()
     *   "return_matchload"  @ref return_to_matchload()
     *   "claw"              @ref open_claw()
     *   "flip"              @ref toggle_scoring_orientation()
     *   "bar_pose"          @ref bar_pose()
     *   "up" / "down"       @ref move_lift_up() / @ref move_lift_down(), ONE tick each
     *                       (they're hold-to-run, so one call advances by one Config::DT)
     *
     * @param side  only used by "prime", everything else logs it and ignores it
     * @param key   one of the names above, anything else gets refused
     * @return true if the key was recognised. Doesn't mean the macro did anything, one
     *         that's invalid from the current mode logs IGNORED and still returns true
     */
    bool run_macro(Side side, const char *key);

    /**
     * @brief Move to a named pose from the lift state table and hold it.
     *
     * @param side  which table to search, and which orientation pair a later flip uses
     * @param key   a `LiftState::key` from that table
     * @return true if the pose was found and the move armed
     */
    bool run_lift_state(Side side, const char *key);

    /// Look up a lift state by side + key, nullptr if there's no such row.
    /// Lets a caller check a key (or read a pose) without commanding it
    static const LiftState *find_lift_state(Side side, const char *key);

    /**
     * @brief Run a CHAIN: a scripted sequence of lift states with a dwell on each.
     *
     * Non-blocking, exactly like @ref run_lift_state(). @ref update() commands step 1
     * here and then walks the rest on its own, so the drivetrain keeps following the
     * trajectory the whole time a chain is playing out -- a chain that blocked would
     * stop the robot dead in the middle of the path
     *
     * The chain reports @ref is_busy() for its WHOLE run, dwells included, so a
     * Robot::waitForLift() waits out the entire sequence rather than returning the moment
     * the first pose lands
     *
     * ANY other lift command (@ref run_lift_state, @ref prime, @ref score,
     * @ref return_to_loading, ...) CANCELS a running chain -- the new command is the
     * driver's / route's latest intent, and letting a chain keep firing poses underneath
     * it is how two things end up fighting over the mechanism
     *
     * @param id  a `LiftChain::id` from @ref LIFT_CHAINS
     * @return true if a chain with that id exists and was started
     */
    bool run_lift_chain(double id);

    /// @copydoc run_lift_chain(double)
    /// @param name a `LiftChain::name` from @ref LIFT_CHAINS
    bool run_lift_chain(const char *name);

    /// Stop a running chain where it is. The pose it last commanded remains.
    void cancel_chain(const char *why);

    /// True while a chain is playing out
    bool chain_active() const;

    /// Name of the running chain and which step it is on (1-based), for telemetry.
    /// nullptr / 0 when no chain is running
    const char *current_chain() const;
    int current_chain_step() const;

    /// True if @p value selects a CHAIN rather than a single lift state. The band is
    /// |value| >= LIFT_CHAIN_ID_MIN_ABS, so a route's single-pose values (0.5, -2, ...)
    /// and its chain ids can never be confused for one another
    static constexpr bool is_chain_id(double value)
    {
        return value >= LIFT_CHAIN_ID_MIN_ABS || value <= -LIFT_CHAIN_ID_MIN_ABS;
    }

    /// Look up a chain without commanding it, nullptr if there's no such row
    static const LiftChain *find_chain(double id);
    static const LiftChain *find_chain(const char *name);

    /**
     * @brief True while a sequence is still playing out.
     *
     * All the multi step moves (startup, prime, return, matchload, back poses, lift
     * states, score retreat) are busy until they finish
     * The resting poses aren't, and neither is the raise sequence since that's walked
     * by the driver's buttons and has nowhere to arrive at
     */
    bool is_busy() const;

    /**
     * @brief True while a loading->primed move is running, the claw grab wait included.
     *
     * What the intake's assist feed is gated on (@ref Intake::set_assist_feed): the piece
     * is being pulled out of the intake through this whole window, so a gentle forward
     * feed keeps it pressed into the claw
     * A CROSSING out of a raised cycle doesn't count, the piece is nowhere near the intake
     */
    bool is_priming_from_rest() const;

    /**
     * @brief True while resting at the matchload pose (@ref toggle_matchload()).
     *
     * The STOW->matchload move on its way there does NOT count, only the pose itself
     * Lets the driver layer give a button a second meaning while matchloading, where the
     * scoring macros are refused anyway
     */
    bool is_matchloading() const;

    void toggle_kill_switch();

    /**
     * @brief True when nothing is moving and all three joints are on target.
     *
     * What @ref run_lift_state callers wait on
     * Uses WINCH_POS_TOL, CHAINBAR_POS_TOL and CHAIN_POS_TOL
     * The bar is exempt in the loading (STOW) pose while CHAINBAR_LOADING_HOLD_MV is in
     * force: it's held by a constant drive there, so its resting angle is wherever the
     * mechanism stops it and not a target to wait on
     */
    bool is_settled() const;

    /// Key of the lift state we're moving onto / holding, nullptr if we're not in one.
    /// Points into the constexpr table so it's safe to hold on to
    const char *current_lift_state() const;

    // Loop + telemetry -

    /**
     * @brief Resolve intent, run the sequence, drive all four actuators.
     *        Must be called every loop iteration.
     */
    void update();

    /// Winch height in motor degrees (average of the two winch encoders).
    double get_winch_position() const;
    /// Chainbar angle in output degrees (0 = loading pose).
    double get_bar_angle() const;
    /// End-effector angle in degrees (chain motor, 1:1).
    double get_ee_angle() const;
    /// Position along the front raise sequence, in [0, SEQ_KEYFRAMES - 1]. The back cycle
    /// walks no table, so it holds this at 0 rather than reporting someone else's progress
    double get_progress() const;

    // Winch (motor degrees)
    /// Fully-down winch height
    static constexpr double WINCH_DOWN_POS = -100.0;
    /// Offset from Winch start pos to loading pos (since we re-zero at loading)
    /// Helps make sure things work even when we don't rezero immedietly when loading
    static constexpr double WINCH_START_POS = -251.2;
    /// Floor of the commandable winch range to prevent string from unspooling
    static constexpr double WINCH_TARGET_FLOOR =
        (WINCH_START_POS < WINCH_DOWN_POS) ? WINCH_START_POS : WINCH_DOWN_POS;
    /// Cascade height in the STOW (loading) pose
    static constexpr double WINCH_LOADING_POS = 0.0;
    /// Upper travel limit / fully-extended height.
    static constexpr double WINCH_MAX_POS = 900.0;
    /// "Fully extended" keyframe height.
    static constexpr double WINCH_FULL_POS = WINCH_MAX_POS;
    /// Clearance height for the chain bar to clear the intake poly
    static constexpr double WINCH_CLEARANCE_POS = 600.0;
    /// Absolute pos Winch raises during the startup sequence
    static constexpr double WINCH_START_CLEARANCE_RAISE = WINCH_CLEARANCE_POS;
    /// Height held while the bar sweeps loading<->primed the long way during prime().
    static constexpr double WINCH_PRIME_CLEARANCE_POS = WINCH_CLEARANCE_POS;
    /// Height the winch has to reach for the tuck sequence for chainbar/claw to start.
    /// Shared by the front prime and the back cycle, which run the same tuck
    static constexpr double WINCH_PRIME_TUCK_POS = 0.0;
    /// Height tolerance for Winch to have "reached target"
    static constexpr double WINCH_POS_TOL = 20.0;

    // Defines whether the respool workflow is enabled
    static constexpr bool WINCH_STOW_HOMING_ENABLED = true;
    /// Gentle raise voltage (mV, positive = up) applied while taking up slack.
    static constexpr double WINCH_RESPOOL_MV = 3000.0;
    /// Below this |motor velocity| (RPM) during a respool counts as taut.
    static constexpr double WINCH_RESPOOL_STALL_RPM = 10.0;
    /// Grace time after a respool starts before the stall check is trusted.
    static constexpr uint32_t WINCH_RESPOOL_GRACE_MS = 150;
    /// Max the re-zero may shift the winch reference in one homing pass
    static constexpr double WINCH_REZERO_MAX_DEG = 100.0;
    /// Abort a respool that never finds resistance
    static constexpr uint32_t WINCH_RESPOOL_TIMEOUT_MS = 1600;
    /// Gentle DOWN drive (mV, negative = retract) that releases the string tension the
    /// respool ends stalled against, so the lift doesn't park with the spool under load
    static constexpr double WINCH_UNTENSION_MV = -3000.0;
    /// How long that release runs for. THE knob: long enough that the string goes slack,
    /// short enough that it doesn't start unspooling the drum
    static constexpr uint32_t WINCH_UNTENSION_MS = 200;
    /// Backstop on how far below the (taut) reference the release may pay out, and the
    /// clamp on the height the bottom hold then parks at
    static constexpr double WINCH_UNTENSION_MAX_DEG = 150.0;
    /// Gentle DOWN drive (mV, negative = retract) for the seek pre-roll
    static constexpr double WINCH_SEEK_MV = -3000.0;
    /// How long the seek pre-roll drives down for
    static constexpr uint32_t WINCH_SEEK_MS = 400;
    /// Max the seek may travel below where it started before it gives up
    static constexpr double WINCH_SEEK_MAX_TRAVEL = 250.0;

    // Chainbar (output degrees, read directly from the axle rotation sensor)
    /// Set true if the rotation sensor counts DOWN as the bar angle increases
    static constexpr bool CHAINBAR_ROTATION_REVERSED = false;
    /// How far the boot reading may sit from the START pose before initialize() warns
    static constexpr double START_POSE_SANITY_DEG = 30.0;
    /// Bar angle in the power-on START pose. Descriptive only  the rotation sensor
    /// keeps its reading across a power cycle, so nothing writes this into it
    static constexpr double CHAINBAR_START_DEG = -25.0;
    /// How far to rotate the bar clear of the start pose before raising the winch
    static constexpr double CHAINBAR_START_CLEARANCE_DEG = -30.0;
    /// Bar angle at the STOW (loading) pose  the 0 all bar angles are measured from
    static constexpr double CHAINBAR_LOADING_DEG = 2.5;
    /// Bar angle held while the winch lowers at the end of the startup/return sequences,
    /// so the bar is out of the way as the lift comes down
    static constexpr double CHAINBAR_LOWER_CLEAR_DEG = 2.0;
    /// Constant drive (mV) the bar is held with in the loading (STOW) pose
    static constexpr double CHAINBAR_LOADING_HOLD_MV = 2000.0;
    /// Angle tolerance for the bar to have "reached target"
    static constexpr double CHAINBAR_POS_TOL = 5.0;
    /// Far edge of the low-clearance zone: between here and loading the bar swings low
    /// through the frame and needs the winch raised to pass
    static constexpr double CHAINBAR_LOW_ZONE_DEG = -10.0;
    /// Bar angle a climb out of a resting pose tucks to, avoids hitting lift bracing
    /// and the intake poly from the bottom up
    static constexpr double CHAINBAR_PRIME_TUCK_DEG = -40.0;

    // Bar-only pose (@ref bar_pose())
    static constexpr double CHAINBAR_BAR_POSE_DEG = -210.0;

    // Chainbar gravity feed-forward
    /// Peak gravity feed-forward (mV), applied when the arm is horizontal
    static constexpr double CHAINBAR_GRAVITY_FF_MV = 1500.0;
    /// Peak gravity feed-forward (mV) on the scoring sequence's DOWN swing
    static constexpr double CHAINBAR_GRAVITY_FF_DOWN_MV = 500.0;
    /// Bar angle where the arm hangs straight down  the feed-forward crosses zero here
    static constexpr double CHAINBAR_GRAVITY_ZERO_DEG = 0.0;

    /**
     * @brief Compile-time check that a raise table can be walked off the bar angle.
     *
     * Every segment must move the bar, all in the same direction. Progress is READ BACK from
     * the angle now that nothing is clocked, so a segment whose two keyframes share a bar
     * angle has no progress to read (the walk would sit on the boundary forever), and one
     * that reverses would run progress backwards and drag the cascade with it.
     *
     * @return true if the table is usable. Called only from static_assert
     */
    static constexpr bool seq_bar_walkable(const double *bar_kf, int n_kf)
    {
        if (n_kf < 2)
        {
            return false;
        }
        const bool descending = bar_kf[1] < bar_kf[0];
        for (int i = 0; i < n_kf - 1; ++i)
        {
            if (descending ? !(bar_kf[i + 1] < bar_kf[i]) : !(bar_kf[i + 1] > bar_kf[i]))
            {
                return false;
            }
        }
        return true;
    }

    // Synchronized raise sequence
    // Commands chainbar to move using FF and will set winch target pos
    // based on the position of the chainbar. This helps keep the chainbar
    // movement smooth
    static constexpr int SEQ_KEYFRAMES = 2;
    /// Winch height at each keyframe
    static constexpr double SEQ_WINCH_DEG[SEQ_KEYFRAMES] = {
        WINCH_DOWN_POS, WINCH_FULL_POS};
    /// Bar angle at each keyframe (negative-going: primed is the long way round since we have
    /// bracing on the cascade preventing it moving under)
    static constexpr double SEQ_BAR_DEG[SEQ_KEYFRAMES] = {-305.0, -235.0};
    /// Constants for back/front prime swings
    static constexpr double SEQ_SWING_UP_MAX_DPS = 100.0; // Up swing speed
    static constexpr double SEQ_SWING_DOWN_MAX_DPS = 100.0; // Down swing speed
    static constexpr double SEQ_SWING_DECEL_DEG = 25.0; // Distance over which the swing decelerates to a stop

    // Chainbar approach to a pose (slew-limited, no overshoot)
    // Defaults for slew_bar_toward(); a move wanting its own speed passes its own pair.
    /// Default cruise rate of a slewed bar move (output deg/s)
    static constexpr double CHAINBAR_APPROACH_MAX_DPS = 250.0;
    /// Default distance from the goal over which that rate tapers to a stop
    static constexpr double CHAINBAR_APPROACH_DECEL_DEG = 45.0;
    /// Floor on the tapered rate (fraction of MAX_DPS) so the target still finishes
    static constexpr double CHAINBAR_APPROACH_MIN_FRAC = 0.15;
    /// Max the slewed target may lead the actual bar angle, so the PID can't build up a
    /// sling past the pose
    static constexpr double CHAINBAR_MAX_LEAD_DEG = 20.0;

    /// Cruise rate of the swing between lift states (output deg/s)
    static constexpr double CHAINBAR_STATE_SWING_MAX_DPS = 400.0;
    /// Distance from the pose over which that rate tapers to a stop. Scale it with the
    /// rate: too short and the bar is still at cruise when it arrives
    static constexpr double CHAINBAR_STATE_SWING_DECEL_DEG = 30.0;

    /// Cruise rate of the matchload hop's swing (output deg/s)
    static constexpr double CHAINBAR_MATCHLOAD_SWING_MAX_DPS = 400.0;

    /// Ticks the bar rate is averaged over  the sensor updates slower than the loop, so
    /// a per-tick delta aliases into a 0/spike sawtooth. 5 ticks = 50 ms
    static constexpr int CHAINBAR_RATE_WINDOW = 5;

    /// Chainbar static-friction feed-forward (breakaway)
    static constexpr double CHAINBAR_KS_MV = 3500.0;
    /// Breakaway amplitude (mV) on the scoring sequence's down swing
    /// This is smaller due to gravity helping the chainbar move already
    static constexpr double CHAINBAR_KS_DOWN_MV = 700.0;
    /// Error band over which the breakaway fades to zero as the bar nears its target. Keep
    /// it well inside CHAINBAR_POS_TOL's neighbourhood -- at 8.0 the term was still at half
    /// amplitude at the edge of "arrived", which is what it kept kicking the bar back out of
    static constexpr double CHAINBAR_KS_DEADBAND_DEG = 3.0;
    /// Bar speed above which the stall counter resets. A bar moving this fast is not stuck,
    /// whichever direction it happens to be going
    static constexpr double CHAINBAR_KS_STALL_DPS = 15.0;
    /// Position error the bar must be holding to count as STALLED rather than arrived --
    /// below this there is nothing worth breaking out toward
    static constexpr double CHAINBAR_KS_MIN_ERR_DEG = 4.0;
    /// Consecutive ticks of that stall before breakaway triggers
    static constexpr int CHAINBAR_KS_STALL_TICKS = 10;
    /// Profile speed the kinetic feed-forward reaches full scale at
    static constexpr double CHAINBAR_KS_MOVING_DPS = 80.0;

    /// Voltage it physically takes to start the bar moving from rest.
    static constexpr double CHAINBAR_BREAKOUT_MV = 4500.0;

    // Chainbar velocity feed-forward (smooth profiled moves)
    static constexpr double CHAINBAR_KV_MV_PER_DPS = 12.0;
    static constexpr double CHAINBAR_KV_MV_PER_DPS_DOWN = 3.0;
    /// Kinetic-friction feed-forward (mV) in the direction of travel while sweeping
    static constexpr double CHAINBAR_KINETIC_FF_MV = 1800.0;
    /// Profile speed above which update() uses the velocity path, not the breakaway one
    static constexpr double CHAINBAR_PROFILE_EPS_DPS = 1.0;

    // Chain / end effector (EE degrees, from the EE axle rotation sensor, 1:1)
    /// Set true if the rotation sensor counts DOWN as the EE angle increases
    static constexpr bool CHAIN_ROTATION_REVERSED = false;
    /// EE angle in the power-on START pose. Descriptive only, like CHAINBAR_START_DEG
    static constexpr double CHAIN_START_DEG = -100.0;
    /// EE orientation while loading / stowed
    static constexpr double CHAIN_LOADING_DEG = -24.0;
    /// EE orientation held through prime()'s climb, until the flip to the raised one
    static constexpr double CHAIN_PRIME_RAISE_DEG = 0.0;
    /// EE half of the tuck pose, turned to when the bar tucks (WINCH_PRIME_TUCK_POS),
    /// front prime and back cycle alike
    static constexpr double CHAIN_PRIME_TUCK_DEG = -24.0;
    /// Default EE orientation once raised (primed / sequence / scoring)
    static constexpr double CHAIN_SCORING_DEG = -260.0;
    /// Alternate raised orientation  the same claw axis flipped half a turn, so a piece
    /// can be presented from the other side without re-priming
    static constexpr double CHAIN_SCORING_ALT_DEG = -425.0;
    /// Bar angle at which the EE swaps between its loading and raised orientations during
    /// the prime / return sweeps, about half way round the long swing
    static constexpr double CHAIN_FLIP_BAR_DEG = -180.0;

    // Size-keep zones to not expand too much
    static constexpr int SIZE_KEEP_ZONE_COUNT = 2;
    static constexpr double CHAINBAR_SIZE_KEEP_ZONES[SIZE_KEEP_ZONE_COUNT][2] = {
        //  low     high
        // {15.0, 6.0}
        {-100.0, -80.0},   // back state 1 sits at -90, dead centre
        {-275.0, -250.0},  // bar straight up: the prime/return sweeps cross here
    };
    /// EE orientation check to make sure we are in size
    static constexpr double CHAIN_SIZE_KEEP_EE_DEG = -180.0;
    /// How far past a band's edge the bar has to travel before the override releases.
    static constexpr double CHAINBAR_SIZE_KEEP_HYST_DEG = 3.0;

    /**
     * @brief Compile-time check on the size-keep table.
     *
     * Each row must be ordered low-then-high (an inverted band is empty, so it would never
     * engage and the claw would quietly stay out of size) and sit inside the bar's own
     * travel. Rows must not overlap ONCE WIDENED by the hysteresis either: overlapping bands
     * would hand the bar back and forth between two gates on the same tick, and the widening
     * is what the release is measured against.
     *
     * @return true if the table is usable. Called only from static_assert
     */
    static constexpr bool size_keep_zones_valid()
    {
        if (SIZE_KEEP_ZONE_COUNT < 1 || CHAINBAR_SIZE_KEEP_HYST_DEG < 0.0)
        {
            return false;
        }
        if (CHAIN_SIZE_KEEP_EE_DEG < CHAIN_MIN_DEG || CHAIN_SIZE_KEEP_EE_DEG > CHAIN_MAX_DEG)
        {
            return false;
        }
        for (int i = 0; i < SIZE_KEEP_ZONE_COUNT; ++i)
        {
            const double lo = CHAINBAR_SIZE_KEEP_ZONES[i][0];
            const double hi = CHAINBAR_SIZE_KEEP_ZONES[i][1];
            if (!(lo < hi) || lo < CHAINBAR_MIN_DEG || hi > CHAINBAR_MAX_DEG)
            {
                return false;
            }
            for (int j = 0; j < i; ++j)
            {
                const double j_lo = CHAINBAR_SIZE_KEEP_ZONES[j][0] - CHAINBAR_SIZE_KEEP_HYST_DEG;
                const double j_hi = CHAINBAR_SIZE_KEEP_ZONES[j][1] + CHAINBAR_SIZE_KEEP_HYST_DEG;
                if (lo - CHAINBAR_SIZE_KEEP_HYST_DEG <= j_hi &&
                    j_lo <= hi + CHAINBAR_SIZE_KEEP_HYST_DEG)
                {
                    return false;
                }
            }
        }
        return true;
    }

    // Sweep coast-through
    // Past the top of the swing gravity and inertia will drive the chainbar,
    // so the sweep switches to brake mode COAST and lets the swing do the work
    static constexpr double CHAINBAR_COAST_START_DEG = -225.0;
    /// Bar angle where the coast ends: brake back to HOLD, and the sweep hands the drive
    /// to the ordinary feed-forward law to close the last few degrees
    static constexpr double CHAINBAR_COAST_CATCH_DEG = -10.0;
    /// Coast start for the PRIME sweep
    static constexpr double CHAINBAR_PRIME_COAST_START_DEG = -180.0;
    /// How far short of primed the prime sweep hands over to the feed-forward law.
    static constexpr double CHAINBAR_PRIME_CATCH_LEAD_DEG = 20.0;

    // Both sweeps need their coast start to come BEFORE their catch in the direction of
    // travel, or bar_coast_active() can never be true and the sweep silently runs the whole
    // way on the brake. Caught here rather than in a log, since these are all constexpr.
    static_assert(CHAINBAR_COAST_START_DEG < CHAINBAR_COAST_CATCH_DEG,
                  "Return sweep travels toward 0, so its coast start must be BELOW its "
                  "catch angle -- otherwise the coast window is empty.");
    static_assert(CHAINBAR_PRIME_COAST_START_DEG >
                      SEQ_BAR_DEG[0] + CHAINBAR_PRIME_CATCH_LEAD_DEG,
                  "Prime sweep travels toward SEQ_BAR_DEG[0], so its coast start must be "
                  "ABOVE its catch angle (SEQ_BAR_DEG[0] + CHAINBAR_PRIME_CATCH_LEAD_DEG) "
                  "-- otherwise the coast window is empty and the coast never runs.");

    // Drive caps (mV) for the long sweeps
    static constexpr double CHAINBAR_SWEEP_MAX_MV = 12000.0;
    /// Top of the prime swing
    static constexpr double CHAINBAR_PRIME_CLIMB_END_DEG = -180.0;
    /// Cap for the prime sweep's climb (sweep start -> CHAINBAR_PRIME_CLIMB_END_DEG)
    static constexpr double CHAINBAR_PRIME_CLIMB_MAX_MV = 8000.0;
    /// Cap for the prime sweep past the top (slower since gravity helps chainbar move)
    static constexpr double CHAINBAR_PRIME_DESCENT_MAX_MV = 7000.0;
    /// Cap for the startup sweep (BAR_LOADING) small since it's a tiny
    /// arc that can't overshoot because of the cascade crossbracing
    static constexpr double CHAINBAR_START_SWEEP_MAX_MV = 2000.0;
    /// Bar speed below which a command is never treated as braking, so the caps still
    /// apply the windowed rate wobbles a few deg/s either way at rest
    static constexpr double CHAINBAR_BRAKE_MIN_DPS = 20.0;

    /// Speed cap for the end effector, which otherwise rails on the big orientation flips.
    /// Helps make sure that stacks don't go wonky in the claw
    static constexpr double CHAIN_MAX_MV = 4000.0;
    /// Ticks the EE rate is averaged over, same rationale as CHAINBAR_RATE_WINDOW
    static constexpr int CHAIN_RATE_WINDOW = 5;
    /// Angle tolerance for the EE to have "reached target". Only used by is_settled(), and
    /// loose, since the EE is back-driven by the bar and gives up position under load
    static constexpr double CHAIN_POS_TOL = 8.0;

    // Matchloading pose
    static constexpr double WINCH_MATCHLOAD_POS = -200.0;
    /// Clearance height for the loading->matchload swing
    static constexpr double WINCH_MATCHLOAD_CLEARANCE_POS = 300.0;
    static constexpr double CHAINBAR_MATCHLOAD_DEG = -312.3;
    static constexpr double CHAIN_MATCHLOAD_DEG = -265.0;

    // Back cycle
    /// @note checked by a static_assert below the class
    static constexpr int BACK_STATE_COUNT = 3;
    /// Which row the winch-interpolated angle applies to: the LAST one, the top of the cycle
    static constexpr int BACK_TOP_STATE = BACK_STATE_COUNT - 1;

    // Winch is angled so the chainbar must be moved a few degrees
    // to account for that chain in X
    // Top state for back scoring when winch is all the way down
    static constexpr double CHAINBAR_BACK_TOP_LOW_DEG = -147.0;
    /// Top state for back scoring when winch is all the way up
    static constexpr double CHAINBAR_BACK_TOP_HIGH_DEG = -147.0;
    /// Winch height the low chainbar angle corresponds to
    static constexpr double WINCH_BACK_TOP_LERP_BOTTOM = 0.0;
    /// Winch height the high chainbar angle corresponds to
    static constexpr double WINCH_BACK_TOP_LERP_TOP = WINCH_MAX_POS;

    static constexpr double CHAINBAR_BACK_STATE_DEG[BACK_STATE_COUNT] = {
        -32.5, -110.0, CHAINBAR_BACK_TOP_LOW_DEG};
    /// Height the cascade is put at when the cycle is first entered
    /// Helps reduce cycle time since on average winch has to move much less
    static constexpr double WINCH_BACK_ENTRY_POS = 400.0;
    /// Drive (mV) the L1/L2 triggers runs the winch at inside the back cycle.
    static constexpr double WINCH_JOG_MV = 12000.0;
    /// Default EE orientation for the back cycle (the back family's CHAIN_SCORING_DEG)
    static constexpr double CHAIN_BACK_DEG = -85.0;
    /// Alternate back orientation, (will trigger backoff logic when scored from)
    static constexpr double CHAIN_BACK_ALT_DEG = -265.0;

    /**
     * @brief Compile-time check on the back-state table.
     *
     * Validates all chainbar angles are within the valid bounds for the chainbar
     * 
     *
     * The top state's HIGH angle is checked as if it were a fourth row, because it is one --
     * @ref back_state_bar_deg() can command anything between LOW and HIGH, and an
     * interpolation between two angles that both satisfy the invariants satisfies them at
     * every point in between. The winch band is checked too: a zero or inverted span would
     * make that interpolation a divide-by-zero or run it backwards.
     *
     * @return true if the table is usable. Called only from static_assert
     */
    static constexpr bool back_states_valid()
    {
        for (int i = 0; i <= BACK_STATE_COUNT; ++i)
        {
            // One past the end is the top state's HIGH angle -- the other end of the only
            // row that is a range rather than a number.
            const double bar = (i < BACK_STATE_COUNT) ? CHAINBAR_BACK_STATE_DEG[i]
                                                      : CHAINBAR_BACK_TOP_HIGH_DEG;
            if (bar < CHAINBAR_MIN_DEG || bar > CHAINBAR_MAX_DEG)
            {
                return false;
            }
            if (!(bar < CHAINBAR_LOW_ZONE_DEG))
            {
                return false;
            }
        }
        if (!(WINCH_BACK_TOP_LERP_TOP > WINCH_BACK_TOP_LERP_BOTTOM))
        {
            return false;
        }
        return BACK_STATE_COUNT > 0;
    }

    // Soft rotation limits (prevent wire/cable overwrap)
    // MUST BRACKET EVERY POSE THE MACROS COMMAND: outside its limit a pose is not
    // truncated, the clamp becomes the setpoint and the motor looks reversed instead.
    static constexpr double CHAINBAR_MIN_DEG = -360.0; ///< output degrees
    /// Upper bar limit  the loading pose, with the nogo object just past it. Positive
    /// commands are zeroed here; braking ones still pass so the PID can pull back
    static constexpr double CHAINBAR_MAX_DEG = 3.0;
    /// EE travel. The real bound is where the EE's wiring starts to overwrap
    static constexpr double CHAIN_MIN_DEG = -500.0; ///< EE degrees (1:1)
    static constexpr double CHAIN_MAX_DEG = 110.0;

    // Claw solenoid polarity
    static constexpr bool CLAW_CLOSED_STATE = true; ///< clamped
    static constexpr bool CLAW_OPEN_STATE = false;  ///< released
    /// How long @ref prime() waits after closing the claw before it starts moving, so the
    /// solenoid has actually clamped the piece before the lift pulls it off the loader.
    /// The robot stays in its resting pose (and its resting hold) for this window
    static constexpr uint32_t CLAW_GRAB_SETTLE_MS = 200;
    /// Claw state held in the power-on START pose.
    static constexpr bool CLAW_START_STATE = CLAW_CLOSED_STATE;

    // Post-score retreat (ALTERNATE raised orientation only)
    // The flipped EE is left sitting in the structure, so the score lifts and swings the
    // bar up to carry it clear, then parks. In the DEFAULT orientation it moves nothing.
    /// Bar angle the retreat swings to  also where the alt-orientation return hold
    /// releases, so a return from this pose can turn the EE and lower straight away
    static constexpr double SCORE_ALT_RETREAT_BAR_DEG = -180.0;
    /// Distance from the retreat angle at which the bar parks on 0 mV + brake HOLD. That
    /// angle is straight up, so driving it just fed a limit cycle. Parks on first crossing
    static constexpr double SCORE_RETREAT_PARK_DEG = 6.0;
    /// Distance at which a parked bar wakes and is driven back. The hysteresis against the
    /// park band is what stops the park becoming its own oscillation
    static constexpr double SCORE_RETREAT_WAKE_DEG = 20.0;
    /// How far the winch rises before the retreat swings, so the flipped EE comes up and
    /// out of the structure rather than dragging round through it
    static constexpr double WINCH_ALT_RETREAT_LIFT_DEG = 75.0;
    /// Drive cap (mV) for the retreat's swing, which is still a STEP target under a cap --
    /// it has one pose to reach and no table to walk. Was the raise sequence's climb cap,
    /// SEQ_SWING_UP_MAX_MV, back when that swing was capped too; split out so pacing the
    /// raise (SEQ_SWING_UP_MAX_DPS) does not quietly slow the retreat as well. Must stay
    /// above CHAINBAR_BREAKOUT_MV or the swing cannot start itself
    static constexpr double SCORE_RETREAT_SWING_MAX_MV = 12000.0;

    /// Case-sensitive equality for lift-state keys. Hand-rolled rather than std::strcmp
    /// so it works during constant evaluation, which is what lets @ref lift_state_table_valid
    /// run the duplicate-key check at compile time.
    static constexpr bool keys_equal(const char *a, const char *b)
    {
        while (*a != '\0' && *a == *b)
        {
            ++a;
            ++b;
        }
        return *a == *b;
    }

    /**
     * @brief Compile-time sanity check on a lift-state table.
     *
     * Every pose must sit inside the soft limits: a pose outside one is not truncated,
     * the clamp silently becomes the setpoint and the mechanism drives to the LIMIT and
     * holds there against the sequence (log 07262026-4). A hand-written pose at least
     * gets read next to its limit; a table row is one careless number away from that
     * failure, so it is checked here.
     *
     * Also rejects a duplicate key, which would silently shadow the later row
     * (@ref find_lift_state returns the first match).
     *
     * The claw is not checked: both solenoid states are legal in every pose, so there is
     * no bad value to catch.
     *
     * @return true if the table is usable. Called only from static_assert  a bad row
     *         is a build error, never a runtime surprise.
     */
    static constexpr bool lift_state_table_valid(const LiftState *table, int count)
    {
        for (int i = 0; i < count; ++i)
        {
            const LiftState &s = table[i];
            if (s.key == nullptr || *s.key == '\0')
            {
                return false;
            }
            // Against WINCH_TARGET_FLOOR, the bound update() clamps to  NOT
            // WINCH_DOWN_POS. A row below the taut bottom is legal (see the floor's
            // comment); a row the clamp would alter is not.
            if (s.winch_pos < WINCH_TARGET_FLOOR || s.winch_pos > WINCH_MAX_POS)
            {
                return false;
            }
            if (s.bar_deg < CHAINBAR_MIN_DEG || s.bar_deg > CHAINBAR_MAX_DEG)
            {
                return false;
            }
            if (s.ee_deg < CHAIN_MIN_DEG || s.ee_deg > CHAIN_MAX_DEG)
            {
                return false;
            }
            for (int j = 0; j < i; ++j)
            {
                if (keys_equal(s.key, table[j].key))
                {
                    return false;
                }
            }
        }
        return true;
    }

    /// Front lift states (these + back ones are used by @ref robot.cpp)
    static constexpr LiftState FRONT_LIFT_STATES[] = {
        //  key        winch     bar       ee       claw
        {"pickup_stack", -200.0, -320.0, -260.0, CLAW_OPEN_STATE},
        {"toggle", -200.0, -230.0, -260.0, CLAW_OPEN_STATE},
        {"pickup_stack_closed", -200.0, -320.0, -260.0, CLAW_CLOSED_STATE},
        {"neutral_one_pin", 400.0, -290.0, -440.0, CLAW_CLOSED_STATE},
        {"neutral_one_pin_down", 0.0, -290.0, -440.0, CLAW_CLOSED_STATE},
        {"eject_front_state", 300.0, -280.0, -282.0, CLAW_CLOSED_STATE},
    };
    /// Back lift states
    static constexpr LiftState BACK_LIFT_STATES[] = {
        //  key        winch     bar       ee       claw
        {"trigger_roller", 100, -31.0, -85.0, CLAW_CLOSED_STATE},
        {"pickup_stack", -100, -31.0, -85.0, CLAW_OPEN_STATE},
        {"move_out", -200, -220.0, -270.0, CLAW_CLOSED_STATE},
        {"load_stack", 350.0, -34.0, -85.0, CLAW_CLOSED_STATE},
        {"nest_stack", 200.0, -34.0, -85.0, CLAW_CLOSED_STATE},
        {"neutral_one_pin", 850.0, -28.0, -75.0, CLAW_CLOSED_STATE},
        {"alliance_empty", 200.0, -25.5, -270.0, CLAW_CLOSED_STATE},
        {"alliance_empty_alt", 200.0, -25.5, -75.0, CLAW_CLOSED_STATE},
        {"alliance_empty_alt_down", -200.0, -32.5, -75.0, CLAW_CLOSED_STATE},
        {"alliance_empty_alt_down_open", -200.0, -32.5, -75.0, CLAW_OPEN_STATE},
        {"alliance_empty_down", -200.0, -25.5, -270.0, CLAW_CLOSED_STATE},
        {"alliance_empty_down_released", -200.0, -25.5, -270.0, CLAW_OPEN_STATE},
        {"alliance_empty_down_released_out", -200.0, -35.5, -270.0, CLAW_OPEN_STATE},
        {"alliance_empty_up", 250.0, -25.5, -270.0, CLAW_OPEN_STATE},
        {"alliance_one_pin", 675.0, -32.5, -270.0, CLAW_CLOSED_STATE},
        {"alliance_one_pin_down", 500.0, -32.5, -270.0, CLAW_CLOSED_STATE},
        {"alliance_one_pin_release", 700.0, -32.5, -270.0, CLAW_OPEN_STATE},
        {"alliance_two_pin", 0.0, -105.0, -270.0, CLAW_CLOSED_STATE},
        {"alliance_two_pin_down", -200.0, -105.0, -270.0, CLAW_CLOSED_STATE},
        {"alliance_two_pin_open", -105.0, -105.0, -270.0, CLAW_OPEN_STATE},
        {"alliance_three_pin", -200.0, -144.0, -270.0, CLAW_CLOSED_STATE},
        {"alliance_three_pin_down", -200.0, -135.0, -270.0, CLAW_CLOSED_STATE},
        {"alliance_three_pin_released", -200.0, -144.0, -270.0, CLAW_OPEN_STATE},
        {"alliance_four_pin", 300.0, -148.0, -270.0, CLAW_CLOSED_STATE},
        {"alliance_four_pin_down", 300.0, -135.0, -270.0, CLAW_CLOSED_STATE},
        {"alliance_four_pin_released", 300.0, -148.0, -270.0, CLAW_OPEN_STATE},
        {"center_one_pin", 500.0, -33.0, -85.0, CLAW_CLOSED_STATE},
        {"center_one_pin_lowered", 100.0, -33.0, -85.0, CLAW_CLOSED_STATE},
        {"center_one_pin_lowered_open", 100.0, -33.0, -85.0, CLAW_OPEN_STATE},
        {"flip_clearance", 400.0, -45.0, -75.0, CLAW_CLOSED_STATE},
        {"flipped_clearance", 400.0, -45.0, -270.0, CLAW_CLOSED_STATE},

    };
    static constexpr int FRONT_LIFT_STATE_COUNT =
        static_cast<int>(sizeof(FRONT_LIFT_STATES) / sizeof(LiftState));
    static constexpr int BACK_LIFT_STATE_COUNT =
        static_cast<int>(sizeof(BACK_LIFT_STATES) / sizeof(LiftState));

    /// @ref find_lift_state() in a form the constant evaluator can run, so
    /// @ref lift_chain_table_valid can resolve every step's key at BUILD time -- a chain
    /// naming a pose that doesn't exist is a typo, and it should not wait until the middle
    /// of a match to turn into a step that silently does nothing
    static constexpr const LiftState *find_lift_state_ce(Side side, const char *key)
    {
        if (key == nullptr)
        {
            return nullptr;
        }
        const LiftState *table = (side == Side::BACK) ? BACK_LIFT_STATES : FRONT_LIFT_STATES;
        const int count = (side == Side::BACK) ? BACK_LIFT_STATE_COUNT : FRONT_LIFT_STATE_COUNT;
        for (int i = 0; i < count; ++i)
        {
            if (keys_equal(table[i].key, key))
            {
                return &table[i];
            }
        }
        return nullptr;
    }

    // Chains arescripted sequences of the poses above
    static constexpr ChainStep CHAIN_CREATE_STACK[] = {
        //  side          key                 hold_ms  wait_settled  settle_timeout_ms
        {Side::BACK, "nest_stack", 300, false, LIFT_CHAIN_SETTLE_TIMEOUT_MS},
        {Side::BACK, "pickup_stack", 700, false, LIFT_CHAIN_SETTLE_TIMEOUT_MS},
        {Side::FRONT, "neutral_one_pin", 0, false, 0},
    };

    static constexpr ChainStep CHAIN_EJECT_CUPK[] = {
        //  side          key                 hold_ms  wait_settled  settle_timeout_ms
        {Side::BACK, "alliance_empty_down", 350, false, 0},
        {Side::BACK, "alliance_empty_down_released", 250, false, 0},
        {Side::BACK, "alliance_empty_down_released_out", 500, false, 0},
        {Side::BACK, "alliance_empty_up", 500, false, 0},
        {Side::FRONT, "eject_front_state", 250, false, 0},
        {Side::FRONT, "pickup_stack", 0, false, 0},
    };

    static constexpr ChainStep SAFE_ALLIANCE_ONE_PIN[] = {
        //  side          key                 hold_ms  wait_settled  settle_timeout_ms
        {Side::FRONT, "neutral_one_pin_down", 350, false, 0},
        {Side::BACK, "pickup_stack", 0, false, 0},
    };

    static constexpr ChainStep SAFE_ALLIANCE_ONE_PIN_COOL[] = {
        //  side          key                 hold_ms  wait_settled  settle_timeout_ms
        {Side::BACK, "alliance_one_pin", 350, false, 0},
        {Side::BACK, "alliance_one_pin_down", 350, false, 0},
        {Side::BACK, "alliance_one_pin_release", 0, false, 0},
    };

    static constexpr LiftChain LIFT_CHAINS[] = {
        //  id     name                 steps                      step_count
        {10.0, "front_cycle", CHAIN_CREATE_STACK,
         static_cast<int>(sizeof(CHAIN_CREATE_STACK) / sizeof(ChainStep))},
         {11.0, "safe_stack", SAFE_ALLIANCE_ONE_PIN,
         static_cast<int>(sizeof(SAFE_ALLIANCE_ONE_PIN) / sizeof(ChainStep))},
         {12.0, "safe_alliance_stack", SAFE_ALLIANCE_ONE_PIN_COOL,
         static_cast<int>(sizeof(SAFE_ALLIANCE_ONE_PIN_COOL) / sizeof(ChainStep))},
        {-10.0, "back_cycle", CHAIN_EJECT_CUPK,
         static_cast<int>(sizeof(CHAIN_EJECT_CUPK) / sizeof(ChainStep))},
    };
    static constexpr int LIFT_CHAIN_COUNT =
        static_cast<int>(sizeof(LIFT_CHAINS) / sizeof(LiftChain));

    /**
     * @brief Compile-time sanity check on the chain table, the @ref lift_state_table_valid
     *        of chains.
     *
     * Catches the four ways a chain row goes wrong silently at runtime: a step naming a
     * pose that isn't in the table (a typo -> a step that does nothing), an id inside the
     * single-pose band (the route's 0.5 would start a chain, or the chain would never be
     * reachable), a duplicate id or name (@ref find_chain returns the first match, so the
     * second row is dead), and a step that neither moves nor waits (a no-op)
     */
    static constexpr bool lift_chain_table_valid(const LiftChain *table, int count)
    {
        for (int i = 0; i < count; ++i)
        {
            const LiftChain &c = table[i];
            if (c.name == nullptr || *c.name == '\0')
            {
                return false;
            }
            if (c.steps == nullptr || c.step_count <= 0)
            {
                return false;
            }
            if (!is_chain_id(c.id))
            {
                return false;
            }
            for (int j = 0; j < i; ++j)
            {
                if (table[j].id == c.id || keys_equal(table[j].name, c.name))
                {
                    return false;
                }
            }
            for (int s = 0; s < c.step_count; ++s)
            {
                const ChainStep &step = c.steps[s];
                // nullptr is legal -- that is a pure dwell -- but a NAMED pose has to exist.
                if (step.key != nullptr && find_lift_state_ce(step.side, step.key) == nullptr)
                {
                    return false;
                }
                // A dwell of zero that also moves nothing would just be skipped.
                if (step.key == nullptr && step.hold_ms == 0)
                {
                    return false;
                }
            }
        }
        return true;
    }

    // The static_asserts that enforce all of the above sit just past the closing brace of
    // this class, since a constexpr member function isn't usable in a constant expression
    // until the class is complete.

private:
    /// What the mechanism is doing right now. We leave STARTUP once and never come back
    /// to it during a match
    enum class Mode
    {
        STARTUP,       ///< Power-on START->STOW sequence
        STOW,          ///< Resting at the loading pose (lift down, claw open)
        MATCHLOAD,     ///< Resting at the matchload pose, which @ref prime() also works from
        TO_MATCHLOAD,  ///< Running the STOW->matchload move
        PRIMING,       ///< Running the loading->primed move
        SEQUENCE,      ///< In the raise sequence (position = raise_progress_)
        BACK,          ///< In the back cycle, holding or moving between the two back poses
                       ///< (@ref back_prime()). back_step_ says which
        LIFT_STATE,    ///< Moving onto, or holding, a pose from a lift state table
                       ///< (@ref run_lift_state()). The auton version of the back cycle
        SCORE_RETREAT, ///< Alt orientation only: @ref score() has released and the bar is
                       ///< swinging up to lift the flipped claw clear, then holding there
        BAR_POSE,      ///< Moving the BAR ALONE onto CHAINBAR_BAR_POSE_DEG, then holding it
                       ///< there (@ref bar_pose()). bar_pose_step_ says which
        RETURNING      ///< Running the ->loading move (reverse of PRIMING)
    };
    /// Steps of the START->STOW startup sequence (@ref start_to_loading()).
    enum class StartStep
    {
        IDLE,        ///< No start sequence active
        BAR_CLEAR,   ///< Rotate the bar clear of the start pose
        RAISE,       ///< Raise the lift for bar swing room
        BAR_LOADING, ///< Swing the bar + claw to just past loading
        LOWER,       ///< Lower the lift back down (bar held clear of loading)
        BAR_SETTLE   ///< Bring the bar up to loading, then finish in STOW
    };
    /// Steps of the loading->primed move (@ref prime()). Raise the lift, tucking the bar
    /// and claw part way up without pausing the climb, then sweep the long way to primed,
    /// lower, and settle onto the pose
    ///
    /// RAISE and TUCK share one winch target, so the boundary between them is a bar/claw
    /// event only and the lift climbs continuously across both
    ///
    /// SWEEP / LOWER / SETTLE are the SAME three steps the return runs (ReturnStep), in the
    /// other direction: one sweep on a step target that coasts over the top, then the lift,
    /// then the bar landed on the pose. See @ref bar_sweeping
    enum class PrimeStep
    {
        RAISE,  ///< Climb to the clearance height, bar held on the resting pose
        TUCK,   ///< Past WINCH_PRIME_TUCK_POS: bar + claw onto the tuck pose while
                ///< the lift carries on up
        SWEEP,  ///< Sweep the long way to primed with the lift held up
        LOWER,  ///< Bar parked on primed: lower the lift onto the primed height
        SETTLE  ///< Land the bar on primed, then hand off to SEQUENCE
    };
    /// Steps of the ->loading sequence (@ref return_to_loading()), the mirror of
    /// PrimeStep. Raise and sweep at the same time (the sweep only waits at the edge of
    /// the low clearance zone for the lift), lower, then bring the bar onto loading
    enum class ReturnStep
    {
        RAISE_SWEEP, ///< Raise the lift while sweeping back, capped at the zone edge
        LOWER,       ///< Lower the lift to the loading height (bar held clear)
        BAR_SETTLE   ///< Bring the bar up to loading, then finish in STOW
    };
    /// Steps of the ->matchload move (@ref toggle_matchload(), @ref return_to_matchload()).
    /// Same shape as the startup and prime moves: lift up before the bar crosses the low
    /// clearance zone, swing, then come back down on the new pose
    ///
    /// RAISE only actually raises when the swing CROSSES that zone, exactly as
    /// @ref StateStep::RAISE does. Out of STOW it always does -- that pose sits AT loading,
    /// inside it. From a raised pose (the back cycle, the sequence, a lift state) nothing
    /// crosses it, so RAISE passes straight through and the whole move is the swing
    enum class MatchloadStep
    {
        RAISE,     ///< Raise the lift for clearance, if the swing needs it (bar held where
                   ///< it started)
        BAR_SWEEP, ///< Swing the bar + claw onto the matchload pose, lift held up. Slewed on
                   ///< CHAINBAR_MATCHLOAD_SWING_MAX_DPS, this move's own rate
        LOWER      ///< Lower onto the matchload height, then MATCHLOAD
    };
    /// Steps of the move onto a back state. Same shape as the matchload hop, and the same
    /// path for both cases (entering the cycle, and cycling from one state to the next),
    /// since each step passes straight through when it has nothing to do
    ///
    /// A CYCLE press is the degenerate case: both ends of the swing are clear of the low
    /// zone and back_target_winch_ is the height the cascade is already at, so RAISE hands
    /// straight to SWING and LOWER passes through -- the whole move is the swing
    ///
    /// RAISE and TUCK share one winch target, exactly as the prime's pair do: the boundary
    /// between them is a bar/claw event and the lift climbs continuously across both
    enum class BackStep
    {
        RAISE, ///< Lift off the resting height, bar held where it is
        TUCK,  ///< Past WINCH_PRIME_TUCK_POS: bar + claw onto the tuck pose (the prime's,
               ///< capped at the destination) while the lift carries on up to clearance
        SWING, ///< Bar (and claw) onto the destination state, lift held up
        LOWER, ///< Lift onto back_target_winch_, bar held on the state
        HOLD   ///< Arrived: the winch jog is live on L1/L2, score armed. Where the back
               ///< cycle SITS -- its counterpart of Mode::SEQUENCE
    };
    /// Steps of the move onto a lift state. Same shape as @ref BackStep since it's the
    /// same problem, the bar may have to cross the low clearance zone to reach the pose
    ///
    /// Only difference is HOLD: the back cycle's is a live driver pose with the lift on
    /// L1/L2, this one is a plain hold since auton has no jog buttons
    ///
    /// The claw isn't a step: @ref run_lift_state() writes it before RAISE begins, so it's
    /// already in the state's position for the whole move
    ///
    /// RAISE only actually raises when the swing CROSSES the low clearance zone. Between
    /// two rows that both sit clear of it (bar below CHAINBAR_LOW_ZONE_DEG) there's nothing
    /// to climb over, so it passes straight through and the winch only ever moves toward
    /// the pose's own height
    enum class StateStep
    {
        RAISE, ///< Lift to the clearance height if the swing needs it, bar held where it is
        SWING, ///< Bar (and claw) onto the pose, lift held up. Slewed on
               ///< CHAINBAR_STATE_SWING_MAX_DPS / _DECEL_DEG, this move's own pair
        LOWER, ///< Lift onto the pose's height, bar held on the pose
        HOLD   ///< Arrived: hold all three. score() is armed here (and only here)
    };
    /// Steps of the bar-only move (@ref bar_pose()). The lift-state choreography with the
    /// two axes it does NOT own taken out of it: the cascade has no destination of its own
    /// (it goes back to bar_pose_winch_, where the move found it) and the EE simply holds
    /// entry_ee_deg_ the whole way through
    ///
    /// RAISE and LOWER only do anything out of a pose whose swing CROSSES the low-clearance
    /// zone, which is LOADING and nothing else -- see @ref move_skips_clearance. Everywhere
    /// else both pass straight through on their first tick and the move is the swing alone,
    /// with the cascade never commanded off the height it was already holding
    enum class BarPoseStep
    {
        RAISE, ///< Climb to the clearance height if the swing needs it, bar held where it is
        SWING, ///< Bar onto CHAINBAR_BAR_POSE_DEG, cascade held up
        LOWER, ///< Cascade back onto bar_pose_winch_, bar held on the pose
        HOLD   ///< Arrived: hold the bar there, cascade and EE where the move left them
    };
    /// Which PAIR of raised claw orientations is in force. The flip button picks between
    /// the two members of a pair (@ref ee_alt_), this picks the pair
    /// Set by whichever prime started the cycle, so a score, its retreat and the return
    /// home all stay in the family they were primed in
    enum class EeFamily
    {
        FRONT, ///< CHAIN_SCORING_DEG / CHAIN_SCORING_ALT_DEG
        BACK   ///< CHAIN_BACK_DEG / CHAIN_BACK_ALT_DEG
    };
    /// Winch control mode. HOLD is the normal position hold, SEEK drives down to ground
    /// the lift, RESPOOL then reels up until the spool stalls and re-zeros there, and
    /// UNTENSION pays a little back out so it doesn't park pulling on a taut string
    /// A homing descent runs HOLD -> SEEK -> RESPOOL -> UNTENSION -> HOLD. A respool that
    /// times out never found tension, so it skips straight back to HOLD
    enum class WinchMode
    {
        HOLD,
        SEEK,
        RESPOOL,
        UNTENSION
    };

    // Devices
    pros::MotorGroup &winch_motors_;
    pros::Motor &chainbar_motor_;
    pros::Rotation &chainbar_rotation_; ///< Bar-axle angle sensor (1:1, output deg)
    pros::Motor &chain_motor_;
    pros::Rotation &chain_rotation_; ///< EE-axle angle sensor (1:1, EE deg)
    EnhancedDigitalOut &claw_;

    // Controllers
    PIDController winch_pid_;
    PIDController chainbar_pid_;
    PIDController chain_pid_;

    // State
    Mode mode_;
    // Where each choreography is. Each is only meaningful in its own mode.
    StartStep start_step_;         ///< START->STOW step (IDLE = none)
    PrimeStep prime_step_;         ///< loading->primed step
    ReturnStep return_step_;       ///< ->resting-pose step
    MatchloadStep matchload_step_; ///< ->matchload step
    BackStep back_step_;           ///< back-cycle step (HOLD = arrived)
    /// Which back state is selected, an index into CHAINBAR_BACK_STATE_DEG. Advanced by
    /// @ref back_prime() and never read outside Mode::BACK
    int back_state_;
    /// Height the running back move puts the cascade at. WINCH_BACK_ENTRY_POS entering the
    /// cycle; the height the mechanism was ALREADY at when cycling between states, which is
    /// how the swing leaves the driver's jogged height alone
    double back_target_winch_;
    StateStep state_step_;         ///< lift-state step (HOLD = arrived)
    BarPoseStep bar_pose_step_;    ///< bar-only move step (HOLD = arrived)
    /// Height the running bar-only move puts the cascade back on: the height the mechanism
    /// was ALREADY at when it was commanded. Latched, so the clearance raise out of loading
    /// is a round trip and the move ends the cascade exactly where it found it
    double bar_pose_winch_;
    /// Pose being moved onto / held, nullptr if we've never been in a lift state.
    /// Points into the constexpr tables, so there's nothing to copy or own
    const LiftState *state_target_;
    Mode return_dest_;       ///< Where the running return is headed (STOW/MATCHLOAD)
    /// Bar angle a move started at. Every sequence holds the bar here while the lift
    /// climbs, so it can be entered from any pose without dragging the bar somewhere
    /// else first. SCORE_RETREAT also sits on it while it waits for clearance
    double entry_bar_deg_;
    /// EE angle a move holds while the lift climbs. The tuck (CHAIN_PRIME_RAISE_DEG) out
    /// of a low pose, the orientation we already had when crossing between raised cycles
    double entry_ee_deg_;
    bool startup_complete_;    ///< Arms the WINCH_REZERO_MAX_DEG cap on re-zeros
    bool chainbar_coasting_;   ///< In the sweep's coast window, brake-mode edge
    bool bar_coast_done_;      ///< This sweep coasted already, no second coast
    bool bar_kill;             ///< Bar kill switch to disable control signals
    bool alt_return_gate_;     ///< Alt orientation return hold on, brake-mode edge
    /// Which size-keep band the override is engaged on, -1 when it is off. Latched state,
    /// not a recomputation: it is what widens that band on the way out (see
    /// @ref size_keep_zone_at) and the edge the LIFTEVT line is logged on
    int size_keep_zone_;
    double return_hold_winch_; ///< Height the return began at, the floor for that hold
    double raise_progress_;    ///< Position along the raise sequence [0, N-1]
    /// The capped swing running now has been seen moving, so its cap can be enforced.
    /// Latched for the whole swing, see the drive cap block in update()
    bool bar_swing_broke_out_;
    /// Consecutive ticks the bar has held a real error without moving. Arms the breakaway
    /// feed-forward at CHAINBAR_KS_STALL_TICKS; reset the moment the bar moves, which is
    /// what keeps the term off a bar that is merely turning around
    int bar_stalled_ticks_;

    // Chain runner. Points into the constexpr LIFT_CHAINS table, so it is safe to hold.
    const LiftChain *chain_;   ///< running chain, nullptr when none is
    int chain_step_;           ///< index into chain_->steps of the step running now
    uint32_t chain_step_ms_;   ///< millis() the step's dwell is measured from
    /// This step's settle gate has been satisfied (or timed out), so chain_step_ms_ now
    /// marks the ARRIVAL rather than the command. Meaningless when wait_settled is false
    bool chain_step_settled_;
    /// The chain is inside its own run_lift_state() call right now. Stops the cancel hook
    /// on that entry point from tearing down the chain that is driving it
    bool chain_dispatching_;
    /// Movement 1 was commanded last tick. Edge trigger for freezing the bar where it is
    /// on release: while held the target is the far end of the swing, so we swap in the
    /// measured angle ONCE. Re-read every tick and the bar would just sag
    bool seq_swing_driving_;
    bool advancing_;             ///< move_lift_up() called this cycle (update consumes)
    bool retreating_;            ///< move_lift_down() called this cycle (update consumes)
    /// The back cycle's winch jog was driving last tick. Edge trigger for clearing the hold
    /// PID on release, which has spent the whole jog reset and would otherwise re-engage
    /// against a target that has been dragged along under it
    bool winch_jogging_;
    /// @ref prime() has closed the claw and is waiting out CLAW_GRAB_SETTLE_MS before the
    /// choreography starts. mode_ is still the resting pose through this, so the pose keeps
    /// its own hold -- @ref is_busy reports it so a macro's wait does not sail past it
    bool prime_grab_pending_;
    uint32_t prime_grab_ms_;     ///< millis() when that claw close was commanded
    /// The prime running now started from a RESTING pose (so it's pulling a piece out of
    /// the intake), not a crossing out of a raised cycle. See @ref is_priming_from_rest
    bool prime_from_rest_;
    /// The move running now may skip its clearance raise: both ends of its bar travel are
    /// already past the low-clearance zone, so the cascade has nothing to be held up FOR
    /// and goes straight for the destination height. Set by the prime, the back cycle and
    /// the RETURN alike -- primed <-> matchload skips it in both directions. Latched when
    /// the move is commanded (see @ref move_skips_clearance) so it cannot flip part way
    /// through the swing
    bool skip_clearance_raise_;
    double winch_manual_target_; ///< Resolved winch height target (motor deg)
    double bar_target_deg_;      ///< Resolved chainbar goal angle (output deg)
    double bar_target_vel_dps_;  ///< Bar profile velocity this tick (0 = not profiled)
    double chain_target_deg_;    ///< Chain hold angle (motor deg)
    EeFamily ee_family_;         ///< Which raised-orientation PAIR is in force
    /// Which member of that pair the flip button has picked. On its own it's also what
    /// arms the alt orientation return hold and the retreat after a score
    bool ee_alt_;
    double retreat_winch_target_; ///< Height the alt retreat lifts to; latched by score()
    bool score_retreat_parked_;   ///< Retreat is parked on 0 mV + brake HOLD (hysteretic)
    /// Offset added to the raw winch encoder average to get @ref get_winch_position().
    /// The reference has to live in software: one motor of the group is reversed, and
    /// set_zero_position_all() writes the value into each motor WITHOUT the reversal, so
    /// the average always comes back to ~0 whatever we ask for. initialize() zeroes the
    /// encoders instead and the respool re-zero shifts this
    double winch_ref_offset_;
    uint32_t last_update_ms_;   ///< millis() of previous update (for dt)
    WinchMode winch_mode_;      ///< Winch control mode (HOLD / SEEK / RESPOOL)
    bool winch_respooled_;      ///< Homed (sought + respooled) since last raise
    uint32_t respool_start_ms_; ///< millis() the current respool began
    uint32_t untension_start_ms_; ///< millis() the current tension release began
    /// Height the bottom hold parks at: where the untension left the lift with the string
    /// slack. The reference (WINCH_DOWN_POS) until a pass completes, so nothing changes
    /// until one has actually paid the string out
    double winch_slack_pos_;
    uint32_t seek_start_ms_;    ///< millis() the current home-seek began
    /// Winch reading the current seek began at. The over-travel backstop measures from
    /// here, since a grounded lift with slack out already sits below WINCH_DOWN_POS
    double seek_start_pos_;
    double bar_angle_hist_[CHAINBAR_RATE_WINDOW]; ///< recent bar angles, 0 = newest
    double chain_angle_hist_[CHAIN_RATE_WINDOW];  ///< recent EE angles, 0 = newest

    // Helpers
    double motor_group_position() const;
    /// Bar angular velocity in output deg/s, from the axle rotation sensor.
    double bar_velocity_dps() const;
    /// Interpolate the winch height + bar angle for a given sequence progress.
    static void seq_targets(double progress, const double *bar_kf, const double *winch_kf,
                            int n_kf, double &winch_out, double &bar_out);
    /// Where @p bar sits along ONE segment, as a progress value in [0, 1]. Measured off
    /// the bar rather than integrated, since the movement is PID driven not clocked
    static double seq_swing_progress(double bar, double bar_from, double bar_to);
    /// Where @p bar sits along the WHOLE table, in [0, n_kf - 1]: the segment it falls in
    /// plus the fraction across it. The only thing that advances the raise sequence now that
    /// no part of it is clocked. Requires a table @ref seq_bar_walkable accepts
    static double seq_progress_from_bar(double bar, const double *bar_kf, int n_kf);
    /// Ramp @p current toward @p goal one tick, cruising at @p max_dps and tapering over
    /// the last @p decel_deg so the bar arrives at ~0 speed. Both default to the
    /// CHAINBAR_APPROACH_* constants, a move wanting its own speed passes its own pair
    /// The target is held within CHAINBAR_MAX_LEAD_DEG of @p actual_bar so it can't
    /// sprint ahead of a lagging bar, and @p out_vel_dps comes back for the velocity FF
    /// The taper is measured from @p actual_bar, NOT from @p current: the lead clamp is
    /// written back into the caller's target, so a taper on the target would ramp down
    /// while the BAR is still a whole lead-clamp short of the pose and arriving at cruise
    static double slew_bar_toward(double current, double goal, double actual_bar,
                                  double &out_vel_dps,
                                  double max_dps = CHAINBAR_APPROACH_MAX_DPS,
                                  double decel_deg = CHAINBAR_APPROACH_DECEL_DEG);
    /// True once the bar is past the low-clearance zone, so it no longer needs the lift
    /// held up and the two motions can overlap
    static bool bar_clear_of_low_zone(double bar);
    /// Index into @ref CHAINBAR_SIZE_KEEP_ZONES of the band @p bar is inside, or -1 if it
    /// is in none. The band already engaged is widened by CHAINBAR_SIZE_KEEP_HYST_DEG, so
    /// this is not static: the hysteresis reads the gate it is currently latched on
    int size_keep_zone_at(double bar) const;
    /// Pick the multiple of 360 that puts @p raw_deg closest to @p expected_deg. The
    /// rotation sensors boot reporting their angle folded into [0, 360), so the turn
    /// count has to be resolved once against the pose we expect to boot in
    /// Works as long as we boot within +/-180 deg of that pose
    static double unwrap_angle_near(double raw_deg, double expected_deg);
    /// Zero out a command that would drive @p angle further past a soft limit
    /// (positive voltage assumed to increase the angle).
    static double apply_soft_limit(double output_mv, double angle,
                                   double min_deg, double max_deg);
    /// Begin a return to @p dest (STOW or MATCHLOAD): opens the claw and arms the
    /// sweep aimed at that pose
    void begin_return(Mode dest);
    /// True if a move onto @p goal_bar may leave @p m -- the power-on START pose -- with
    /// no startup choreography and no clearance raise. The startup sequence exists for the
    /// intake poly, which sits between START and LOADING; a goal BEHIND the start angle and
    /// past the low-clearance zone (matchload, back BOTTOM) travels away from it over the
    /// arc the startup's own BAR_CLEAR step already turns through with the lift down
    static bool leaves_start_backward(Mode m, double goal_bar);
    /// Take the mechanism off the power-on mode: cancel any running startup choreography
    /// and arm the re-zero cap, which the choreography that would have vouched for the
    /// reference is now never going to do
    void supersede_startup();
    /// True if @p m is one of the RAISED cycles, i.e. a mode @ref prime() and
    /// @ref back_prime() may cross straight out of instead of grabbing a piece.
    /// TO_MATCHLOAD counts as well, so the hop can be abandoned mid-swing for another
    /// pose -- it releases only on arrival, so a crossing out of it keeps what is held
    static bool crossable_from(Mode m);
    /// True if a move onto @p goal_bar commanded RIGHT NOW may skip its clearance raise.
    /// The raise exists for ONE thing: getting the bar over the low-clearance zone with the
    /// frame in the way. The bar travels monotonically from where it is to the goal, so both
    /// ends sitting past CHAINBAR_LOW_ZONE_DEG means the whole arc is clear -- and the
    /// several hundred degrees of cascade travel up and straight back down buy nothing.
    /// Geometry ONLY, the same test @ref run_lift_state already makes: a crossing between
    /// the raised cycles and a prime out of MATCHLOAD both qualify, and only a pose that
    /// actually starts or ends in the zone (STOW) still pays for the climb
    /// Call BEFORE the move commits, while the bar still reads the pose being left
    bool move_skips_clearance(double goal_bar) const;
    /// Latch the bar + EE angles the RAISE step of a move holds while the lift climbs.
    /// The EE tucks out of a low pose and simply holds when crossing between two raised
    /// cycles, where turning to the tuck would spin it round and straight back
    void latch_entry_pose();
    /// Bar angle where the running sweep hands the drive back to the feed-forward law.
    /// A fixed lead-in short of whatever that sweep aims at, so the matchload return
    /// hands over the same distance from its pose that the loading return does
    double sweep_catch_angle() const;
    /// Which way round the swing the running sweep travels: +1 toward a LESS negative bar
    /// angle (the returns and the startup), -1 for the prime sweep. Every angle window in
    /// the sweep law is compared through this, so one set of tests serves both directions
    double sweep_dir() const;
    /// Bar angle where the running sweep starts coasting, per direction
    double sweep_coast_start_angle() const;
    /// True while the running sweep has not yet reached its catch angle, either direction
    bool bar_before_catch(double bar) const;
    /// Advance the START->STOW sequence one tick.
    void advance_start_sequence(double bar, double winch);
    /// Arm the hop onto the matchload pose from wherever the bar and lift are standing.
    /// Split out of @ref toggle_matchload() so the direct move out of START enters the
    /// SAME choreography rather than a second copy of it, and so @ref return_to_matchload()
    /// can reach the pose the one way that actually goes there. @p from is where to enter
    /// it: RAISE for the full hop, LOWER to just command the pose (the direct move).
    /// Sets @ref skip_clearance_raise_, so entering at RAISE from a pose that is already
    /// past the low zone costs nothing
    void begin_matchload_hop(MatchloadStep from);
    /// Start the loading->primed choreography. Split out of @ref prime() so the claw-grab
    /// wait can call it a few ticks later without duplicating the entry state
    void begin_prime();
    /// Advance the loading->primed sequence one tick.
    void advance_prime_sequence(double bar, double winch);
    /// Advance the ->loading sequence one tick.
    void advance_return_sequence(double bar, double winch);
    /// Advance the STOW->matchload sequence one tick.
    void advance_matchload_sequence(double bar, double winch);
    /// Walk the raise sequence one tick over @p bar_kf / @p winch_kf: the bar swings toward
    /// the end of the table at the caps' speed and the cascade rides the progress that swing
    /// reports, so the walk is driven by MOVEMENT and not by any clock. The FRONT cycle only
    /// -- the back's L1/L2 jog the winch directly and walk nothing.
    /// @p bar_swing_cap_mv is the swing's drive cap for this tick, 0 when neither button is
    /// held; @p bar_swing_down is set when the swing is the gravity-fed DESCENT, which
    /// update() feeds its own feed-forward set
    void advance_raise_sequence(double bar, const double *bar_kf, const double *winch_kf,
                                int n_kf, double &bar_swing_cap_mv, bool &bar_swing_down);
    /// Bar angle the back state currently selected asks for, at winch height @p winch.
    /// A constant for every state but the top one, where it is the LOW..HIGH interpolation
    /// (see CHAINBAR_BACK_TOP_LOW_DEG) -- which is why the height is an argument at all.
    /// Everything in the back cycle goes through here, so the swing onto a state, the lower
    /// and the hold all ask for the same angle and there is no step between them
    double back_state_bar_deg(double winch) const;
    /// @copydoc back_state_bar_deg(double)
    /// Reads the winch itself. For callers outside @ref update(), which has it already
    double back_state_bar_deg() const;
    /// Arm the move onto the state @ref back_state_ now names, from wherever the mechanism
    /// is standing, with the cascade headed for @p target_winch. The one path into the back
    /// choreography: entering the cycle and cycling between states differ only in that
    /// height and in whether the claw is touched (which the caller does)
    void begin_back_move(double target_winch);
    /// Advance the back cycle one tick: the move onto the selected state, then the HOLD that
    /// sits on it with the winch under the driver's thumb.
    /// @p winch_jog_mv comes back non-zero when a jog button is held in HOLD, and is what
    /// update() drives the winch with instead of the position hold
    void advance_back_sequence(double bar, double winch, double &winch_jog_mv);
    /// Advance the move onto state_target_ one tick. Same shape as the back cycle,
    /// then a plain hold on the pose
    void advance_lift_state_sequence(double bar, double winch);
    /// Advance the bar-only move one tick: the clearance round trip if the swing needs one,
    /// the swing onto CHAINBAR_BAR_POSE_DEG, then a hold that keeps commanding the cascade
    /// and EE the values the move started with -- so nothing but the bar ever changes
    void advance_bar_pose_sequence(double bar, double winch);
    /// Advance the running chain one tick: hold the current step until its gate and dwell
    /// are satisfied, then command the next one. Called at the TOP of @ref update(), so a
    /// step commanded on this tick gets its targets resolved by the same tick's mode switch
    void advance_chain(uint32_t now);
    /// Command the step chain_step_ points at and start its clock. A step with a nullptr
    /// key commands nothing -- it is a dwell on whatever the previous step left
    void dispatch_chain_step(uint32_t now);
    /// Common half of the two @ref run_lift_chain overloads
    bool start_chain(const LiftChain *chain);
    /// @ref cancel_chain unless the chain is the one doing the commanding right now --
    /// dispatch_chain_step() goes through the public @ref run_lift_state(), which cancels
    void cancel_chain_if_external(const char *why);
    /// @ref is_busy() WITHOUT the chain's contribution. The chain's own settle gate has to
    /// use this: gating on is_busy() would deadlock, since a running chain is itself busy
    bool mode_busy() const;
    /// The position half of @ref is_settled(): all three joints on target, no "is anything
    /// still running" test. Split out for the same reason as @ref mode_busy()
    bool pose_arrived() const;
    /// Bar angle / winch height of the back pose currently selected.
    /// The raised claw orientation in force: the ee_alt_ member of the ee_family_ pair.
    /// Read every tick, so a flip turns the claw right where it is
    double raised_ee_deg() const;
    /// The two long sweeps back toward loading that share one control law, the return's
    /// RAISE_SWEEP and the startup's BAR_LOADING. Both just have to get the bar round
    /// rather than land it, and both hand off at the catch angle
    bool bar_sweeping() const;
    /// Whether the chainbar should be coasting this tick: mid sweep, past the coast
    /// start but not yet caught, with the lift already up. In the window the swing does
    /// most of the work (brake COAST) but the PID still drives its sweep command
    bool bar_coast_active(double bar, double winch) const;
    /// Whether the alt-orientation return hold applies this tick: returning with the claw
    /// flipped and the bar not yet swung back round. While it holds the chain gets no
    /// command at all and the lift may not drop below where the return started
    bool alt_return_hold_active(double bar) const;
    /// Whether the final BAR_SETTLE move onto loading (shared by the startup and return
    /// sequences) is done
    bool settle_bar_arrived(double bar, double rest_bar) const;

    static const char *mode_name(Mode m);
    static const char *start_step_name(StartStep s);
    static const char *prime_step_name(PrimeStep s);
    static const char *return_step_name(ReturnStep s);
    static const char *matchload_step_name(MatchloadStep s);
    static const char *back_step_name(BackStep s);
    static const char *state_step_name(StateStep s);
    static const char *bar_pose_step_name(BarPoseStep s);
    static const char *winch_mode_name(WinchMode m);
    /// Emit one CSV-style debug line covering every command/target/actual/output.
    void log_debug_state(double winch_target, double winch, double winch_mv,
                         double winch_vel_rpm,
                         double bar, double bar_mv, double bar_vel_dps,
                         double bar_gravity_ff, double bar_static_ff,
                         double ee, double chain_mv, double chain_vel_dps) const;
};

// Checked out here rather than inside the class, since a constexpr member function isn't
// usable in a constant expression until the class is complete. A bad row should be a
// build error, not a mechanism that quietly drives into a soft limit and holds there.
static_assert(Lift::lift_state_table_valid(Lift::FRONT_LIFT_STATES, Lift::FRONT_LIFT_STATE_COUNT),
              "FRONT_LIFT_STATES: a row has an empty or duplicate key, or a pose outside "
              "[WINCH_TARGET_FLOOR, WINCH_MAX_POS] / [CHAINBAR_MIN_DEG, CHAINBAR_MAX_DEG] / "
              "[CHAIN_MIN_DEG, CHAIN_MAX_DEG]");
static_assert(Lift::lift_state_table_valid(Lift::BACK_LIFT_STATES, Lift::BACK_LIFT_STATE_COUNT),
              "BACK_LIFT_STATES: a row has an empty or duplicate key, or a pose outside "
              "[WINCH_TARGET_FLOOR, WINCH_MAX_POS] / [CHAINBAR_MIN_DEG, CHAINBAR_MAX_DEG] / "
              "[CHAIN_MIN_DEG, CHAIN_MAX_DEG]");
static_assert(Lift::seq_bar_walkable(Lift::SEQ_BAR_DEG, Lift::SEQ_KEYFRAMES),
              "SEQ_BAR_DEG must move the bar monotonically across EVERY segment: the raise "
              "walk is parameterized on the bar angle, so a flat segment has no progress to "
              "read and a reversing one would run the cascade backwards");
static_assert(Lift::back_states_valid(),
              "CHAINBAR_BACK_STATE_DEG: every back state must sit inside [CHAINBAR_MIN_DEG, "
              "CHAINBAR_MAX_DEG] and past CHAINBAR_LOW_ZONE_DEG -- a state inside the low "
              "zone would need the clearance raise the cycle skips between states");
static_assert(Lift::CHAINBAR_BAR_POSE_DEG >= Lift::CHAINBAR_MIN_DEG &&
                  Lift::CHAINBAR_BAR_POSE_DEG <= Lift::CHAINBAR_MAX_DEG &&
                  Lift::CHAINBAR_BAR_POSE_DEG < Lift::CHAINBAR_LOW_ZONE_DEG,
              "CHAINBAR_BAR_POSE_DEG must sit inside [CHAINBAR_MIN_DEG, CHAINBAR_MAX_DEG] "
              "and past CHAINBAR_LOW_ZONE_DEG: bar_pose() leaves the bar there with the "
              "cascade back at the height it started from, so an angle inside the low zone "
              "would park the bar in the frame with the lift down");
static_assert(Lift::size_keep_zones_valid(),
              "CHAINBAR_SIZE_KEEP_ZONES: every row must be {low, high} with low BELOW high "
              "(an inverted band is empty, so the override would never engage), sit inside "
              "[CHAINBAR_MIN_DEG, CHAINBAR_MAX_DEG], and not overlap another row once "
              "widened by CHAINBAR_SIZE_KEEP_HYST_DEG. CHAIN_SIZE_KEEP_EE_DEG must also sit "
              "inside [CHAIN_MIN_DEG, CHAIN_MAX_DEG] -- an angle outside them is clamped "
              "away and the claw stays out of size");
static_assert(Lift::lift_chain_table_valid(Lift::LIFT_CHAINS, Lift::LIFT_CHAIN_COUNT),
              "LIFT_CHAINS: a row has an empty/duplicate name, a duplicate id, an id "
              "inside the single-pose band (|id| < LIFT_CHAIN_ID_MIN_ABS), no steps, a "
              "step naming a pose that is not in FRONT_LIFT_STATES/BACK_LIFT_STATES for "
              "its side, or a step that neither moves nor waits");

#endif // LIFT_HPP
