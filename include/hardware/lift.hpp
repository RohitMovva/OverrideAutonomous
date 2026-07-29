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
 *
 */
class Lift
{
public:
    /**
     * @brief Dictates side robot scores on in macro in auto
     *
     * Used to decide between the back scoring and front scoring table
     * for autonomous routine specialized scoring states
     */
    enum class Side
    {
        FRONT,
        BACK
    };

    /**
     * @brief Scoring pose: Three angles for each part of lift
     *
     * Each row in @ref FRONT_SCORE_STATES or @ref BACK_SCORE_STATES becomes one
     * of these
     *
     * Used for all the auton states so we can easily add/extend more
     *
     * each is range-checked against its soft limits when building
     */
    struct ScoreState
    {
        const char *key;  ///< name passed to @ref run_score_state()
        double winch_pos; ///< continuous height, motor deg, LOADING-referenced
        double bar_deg;   ///< chainbar angle, output deg (LOADING = 0)
        double ee_deg;    ///< claw angle, EE deg (1:1)
    };

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
     * Valid to call either from matchloading or from loading
     * This function is used to first trigger the sequence,
     * see @ref advance_prime_sequence to view the full process
     */
    void prime();

    /**
     * @brief Start back scoring, and toggle it between top and bottom scoring.
     *
     * Behavior
     *  - From a resting pose (STOW or MATCHLOAD) moves to back prime pose
     *    (really just prime but in the back), this is the bottom prime pose
     *  - Toggles between top and bottom scoring if already in back scoring state (not continuous)
     *  - Ignored if called from any other state
     *
     * When primed: can run
     *  - @ref move_lift_up() / @ref move_lift_down() just commands continuous lift now
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
     * @brief Advance the raise sequence.
     *
     * When in front prime: Run through key frames (1 second to hit each goal pose
     * for chain bar and cascade, synchronizing their movement)
     *
     * When in back prime: Simply moves continuous lift up
     */
    void move_lift_up();

    /**
     * @brief Retreat the coordinated raise sequence this iteration.
     *
     * Identical to @ref move_lift_up() but in reverse.
     * When in front prime: will follow lift up's key frames in reverse.
     * When in back prime: Simply moves continuous lift down
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
     * @brief Return to the MATCHLOAD pose from any state.
     *
     * Same as @ref return_to_loading() but the sweep stops early at the matchload
     * angle, so it's a shorter swing back
     * After a score we auto-return here if the cycle was primed from matchload,
     * this is the manual override
     */
    void return_to_matchload();

    /**
     * @brief Toggle the claw where it is, without moving anything else.
     *
     * Valid in any state since it only touches the solenoid
     * Nothing in @ref update() re-commands the claw, so it stays how you leave it
     * until a macro sets it again (prime closes it, score and the returns open it)
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
     * @ref initialize() runs this
     */
    void start_to_loading();

    // Autonomous interface (keyed macros)

    /**
     * @brief Run one of the driver macros above by name (for auton).
     *
     * Just a dispatcher, every key calls the macro that already exists so its gating
     * and logging are unchanged
     *
     *   "prime"             @ref prime() (FRONT) / @ref back_prime() (BACK)
     *   "matchload"         @ref toggle_matchload()
     *   "score"             @ref score()
     *   "return"            @ref return_to_loading()
     *   "return_matchload"  @ref return_to_matchload()
     *   "claw"              @ref open_claw()
     *   "flip"              @ref toggle_scoring_orientation()
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
     * @brief Move to a named pose from the score state table and hold it.
     *
     * Non-blocking, this just arms the move and @ref update() walks it. Poll
     * @ref is_settled() or call Robot::waitForLift to wait for it to arrive
     *
     * Runs the same raise -> swing -> lower as the other moves, so the bar never
     * crosses the low clearance zone with the lift down, whatever the pose is
     * Doesn't touch the claw: prime grabs, this places, score releases, return comes home
     * Valid from anywhere except STARTUP
     *
     * @param side  which table to search, and which orientation pair a later flip uses
     * @param key   a `ScoreState::key` from that table
     * @return true if the pose was found and the move armed
     */
    bool run_score_state(Side side, const char *key);

    /// Look up a score state by side + key, nullptr if there's no such row.
    /// Lets a caller check a key (or read a pose) without commanding it
    static const ScoreState *find_score_state(Side side, const char *key);

    /**
     * @brief True while a sequence is still playing out.
     *
     * All the multi step moves (startup, prime, return, matchload, back poses, score
     * states, score retreat) are busy until they finish
     * The resting poses aren't, and neither is the raise sequence since that's walked
     * by the driver's buttons and has nowhere to arrive at
     */
    bool is_busy() const;

    /**
     * @brief True when nothing is moving and all three joints are on target.
     *
     * What @ref run_score_state callers wait on
     * Uses WINCH_POS_TOL, CHAINBAR_POS_TOL and CHAIN_POS_TOL
     */
    bool is_settled() const;

    /// Key of the score state we're moving onto / holding, nullptr if we're not in one.
    /// Points into the constexpr table so it's safe to hold on to
    const char *current_score_state() const;

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
    /// Position along the raise sequence, in [0, SEQ_KEYFRAMES-1].
    double get_progress() const;
    // ========================================================================
    // Tunable geometry / thresholds  VERIFY ON THE ROBOT.
    // ========================================================================

    // Winch (motor degrees)
    /// Fully-down winch height
    static constexpr double WINCH_DOWN_POS = 0.0;
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
    /// Height winch has to reach when moving from loading->primed that starts tuck
    /// sequence for chainbar/claw
    static constexpr double WINCH_PRIME_TUCK_POS = 0.0;
    /// Height tolerance for Winch to have "reached target"
    static constexpr double WINCH_POS_TOL = 40.0;

    // Defines whether the respool workflow is enabled
    static constexpr bool WINCH_STOW_HOMING_ENABLED = true;
    /// Gentle raise voltage (mV, positive = up) applied while taking up slack.
    static constexpr double WINCH_RESPOOL_MV = 3000.0;
    /// Below this |motor velocity| (RPM) during a respool counts as taut.
    static constexpr double WINCH_RESPOOL_STALL_RPM = 15.0;
    /// Grace time after a respool starts before the stall check is trusted.
    static constexpr uint32_t WINCH_RESPOOL_GRACE_MS = 150;
    /// Max the re-zero may shift the winch reference in one homing pass
    static constexpr double WINCH_REZERO_MAX_DEG = 100.0;
    /// Abort a respool that never finds resistance
    static constexpr uint32_t WINCH_RESPOOL_TIMEOUT_MS = 1600;
    /// Gentle DOWN drive (mV, negative = retract) for the seek pre-roll
    static constexpr double WINCH_SEEK_MV = -3000.0;
    /// How long the seek pre-roll drives down for
    static constexpr uint32_t WINCH_SEEK_MS = 150;
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
    static constexpr double CHAINBAR_LOADING_DEG = 1.0;
    /// Bar angle held while the winch lowers at the end of the startup/return sequences,
    /// so the bar is out of the way as the lift comes down
    static constexpr double CHAINBAR_LOWER_CLEAR_DEG = -1.0;
    /// Angle tolerance for the bar to have "reached target"
    static constexpr double CHAINBAR_POS_TOL = 4.0;
    /// Far edge of the low-clearance zone: between here and loading the bar swings low
    /// through the frame and needs the winch raised to pass
    static constexpr double CHAINBAR_LOW_ZONE_DEG = -90.0;
    /// Bar angle the prime's climb tucks to, so the bar is part way round before the long
    /// sweep to primed takes over
    static constexpr double CHAINBAR_PRIME_TUCK_DEG = -40.0;

    // Chainbar gravity feed-forward
    /// Peak gravity feed-forward (mV), applied when the arm is horizontal
    static constexpr double CHAINBAR_GRAVITY_FF_MV = 1500.0;
    /// Bar angle where the arm hangs straight down  the feed-forward crosses zero here
    static constexpr double CHAINBAR_GRAVITY_ZERO_DEG = 0.0;

    // Coordinated raise sequence (progress-parameterized)
    // KF0->KF1 swings the bar out of primed; KF1->KF2 runs the lift to full extension.
    /// Number of keyframes; progress runs [0, N-1]
    static constexpr int SEQ_KEYFRAMES = 3;
    /// Winch height at each keyframe
    static constexpr double SEQ_WINCH_DEG[SEQ_KEYFRAMES] = {
        WINCH_DOWN_POS, WINCH_DOWN_POS, WINCH_FULL_POS};
    /// Bar angle at each keyframe (negative-going: primed is the long way round since we have
    /// bracing on the cascade preventing it moving under)
    static constexpr double SEQ_BAR_DEG[SEQ_KEYFRAMES] = {-305.0, -230.0, -235.0};
    /// Progress units per second while the raise button is held (movement 2 only)
    static constexpr double SEQ_RATE_PER_S = 1.0;
    /// Drive cap (mV) for the bar-only swing out of primed (KF0->KF1), which climbs
    /// against gravity. The alt-orientation score retreat uses it too
    static constexpr double SEQ_SWING_UP_MAX_MV = 9000.0;
    /// Drive cap (mV) coming back down (KF1->KF0). Far softer, since this is the speed a
    /// held piece meets whatever is under it
    static constexpr double SEQ_SWING_DOWN_MAX_MV = 2000.0;

    // Chainbar approach to a pose (slew-limited, no overshoot)
    // Defaults for slew_bar_toward(); a move wanting its own speed passes its own pair.
    /// Default cruise rate of a slewed bar move (output deg/s)
    static constexpr double CHAINBAR_APPROACH_MAX_DPS = 150.0;
    /// Default distance from the goal over which that rate tapers to a stop
    static constexpr double CHAINBAR_APPROACH_DECEL_DEG = 45.0;
    /// Floor on the tapered rate (fraction of MAX_DPS) so the target still finishes
    static constexpr double CHAINBAR_APPROACH_MIN_FRAC = 0.15;
    /// Max the slewed target may lead the actual bar angle, so the PID can't build up a
    /// sling past the pose
    static constexpr double CHAINBAR_MAX_LEAD_DEG = 20.0;
    /// Ticks the bar rate is averaged over  the sensor updates slower than the loop, so
    /// a per-tick delta aliases into a 0/spike sawtooth. 5 ticks = 50 ms
    static constexpr int CHAINBAR_RATE_WINDOW = 5;

    // Chainbar static-friction feed-forward (breakaway)
    // The bar has heavy stiction, so P alone stalls short of a pose or sticks then lurches.
    /// Peak breakaway voltage (mV), applied at/above the deadband
    static constexpr double CHAINBAR_KS_MV = 4500.0;
    /// Error band over which the breakaway fades to zero as the bar nears its target
    static constexpr double CHAINBAR_KS_DEADBAND_DEG = 8.0;
    /// Bar speed at which the breakaway has fully faded  it is only needed to START a
    /// stalled bar, and applying it while moving drove a stick-slip oscillation
    static constexpr double CHAINBAR_KS_MOVING_DPS = 80.0;

    // Chainbar velocity feed-forward (smooth profiled moves)
    // Makes the bar TRACK a slew's velocity profile rather than letting its speed emerge
    // from P vs. friction. Only applied while a profile velocity is active.
    /// Velocity feed-forward gain: motor mV per (output deg/s) of profile speed
    static constexpr double CHAINBAR_KV_MV_PER_DPS = 12.0;
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
    static constexpr double CHAIN_LOADING_DEG = -30.0;
    /// EE orientation held through prime()'s climb, until the flip to the raised one
    static constexpr double CHAIN_PRIME_RAISE_DEG = 0.0;
    /// EE half of the tuck pose, turned to when the bar tucks (WINCH_PRIME_TUCK_POS)
    static constexpr double CHAIN_PRIME_TUCK_DEG = -24.0;
    /// Default EE orientation once raised (primed / sequence / scoring)
    static constexpr double CHAIN_SCORING_DEG = -260.0;
    /// Alternate raised orientation  the same claw axis flipped half a turn, so a piece
    /// can be presented from the other side without re-priming
    static constexpr double CHAIN_SCORING_ALT_DEG = -425.0;
    /// Bar angle at which the EE swaps between its loading and raised orientations during
    /// the prime / return sweeps, about half way round the long swing
    static constexpr double CHAIN_FLIP_BAR_DEG = -180.0;

    // Return sweep coast-through
    // Past the top of the swing gravity and inertia carry the bar home, so the sweep
    // switches to brake mode COAST and lets the swing do the work (the PID still drives).
    /// Bar angle where the return sweep switches to the coasting brake mode
    static constexpr double CHAINBAR_COAST_START_DEG = -225.0;
    /// Bar angle where the coast ends: brake back to HOLD, and the sweep hands the drive
    /// to the ordinary feed-forward law to close the last few degrees
    static constexpr double CHAINBAR_COAST_CATCH_DEG = -10.0;

    // Drive caps (mV) for the three long sweeps, which chase step targets hundreds of
    // degrees away. A braking command is never capped.
    /// Cap for the return sweep (RAISE_SWEEP). Must stay above the bar's breakaway
    /// (CHAINBAR_KS_MV) or the sweep cannot start itself
    static constexpr double CHAINBAR_SWEEP_MAX_MV = 8000.0;
    /// Cap for the startup sweep (BAR_LOADING)  much lower, since it is short, downhill
    /// and has to stop next to the nogo object. Only applied once the bar is moving
    static constexpr double CHAINBAR_START_SWEEP_MAX_MV = 2000.0;
    /// Backstop cap for the prime sweep (SWEEP_LOWER), which commands its own speed below
    static constexpr double CHAINBAR_PRIME_SWEEP_MAX_MV = 6000.0;
    /// Commanded cruise speed of the prime sweep past the top of the swing  the knob for
    /// how hard the bar comes down onto primed
    static constexpr double CHAINBAR_PRIME_SWEEP_DPS = 150.0;
    /// Commanded cruise speed of that sweep's climb. Deliberately above what the bar can
    /// do, so it saturates and climbs on essentially the whole rail
    static constexpr double CHAINBAR_PRIME_CLIMB_DPS = 450.0;
    /// Decel zone for that sweep  wider than the default, since it cruises faster
    static constexpr double CHAINBAR_PRIME_SWEEP_DECEL_DEG = 90.0;
    /// Top of the swing, where gravity stops opposing the prime sweep and starts feeding
    /// it. Both the drive cap and the climb/sweep rate hand over here
    static constexpr double CHAINBAR_PRIME_CAP_START_DEG = -180.0;
    /// Bar speed below which a command is never treated as braking, so the caps still
    /// apply  the windowed rate wobbles a few deg/s either way at rest
    static constexpr double CHAINBAR_BRAKE_MIN_DPS = 20.0;

    /// Speed cap for the end effector, which otherwise rails on the big orientation flips.
    /// Only binds while |error| > CHAIN_MAX_MV / kP, so holds keep full authority
    static constexpr double CHAIN_MAX_MV = 4000.0;
    /// Ticks the EE rate is averaged over, same rationale as CHAINBAR_RATE_WINDOW
    static constexpr int CHAIN_RATE_WINDOW = 5;
    /// Angle tolerance for the EE to have "reached target". Only used by is_settled(), and
    /// loose, since the EE is back-driven by the bar and gives up position under load
    static constexpr double CHAIN_POS_TOL = 8.0;

    // Matchloading pose
    // A second resting pose alongside STOW, for taking a piece from the match loader
    // instead of the floor. prime() works from here exactly as it does from STOW.
    /// Winch height at the matchload pose
    static constexpr double WINCH_MATCHLOAD_POS = 0.0;
    /// Clearance height for the loading->matchload swing. Lower than WINCH_CLEARANCE_POS,
    /// since this hop is a short arc at the loading end rather than a full sweep
    static constexpr double WINCH_MATCHLOAD_CLEARANCE_POS = 300.0;
    /// Bar angle at the matchload pose. NOTE it rests inside the low-clearance zone, with
    /// the lift down  only safe if the obstruction is confined to part of that travel
    static constexpr double CHAINBAR_MATCHLOAD_DEG = -37.0;
    /// EE orientation at the matchload pose
    static constexpr double CHAIN_MATCHLOAD_DEG = -75.0;

    // Back cycle (working the other side of the mechanism)
    // Two poses the driver toggles between, over the BACK of the robot instead of the long
    // sweep to the front primed pose. Moves between them use the matchload choreography.
    /// Bar angle at the back BOTTOM pose. Like the matchload pose it rests inside the
    /// low-clearance zone; the travel to and from it is still gated on the lift being up
    static constexpr double CHAINBAR_BACK_BOTTOM_DEG = -30.0;
    /// Winch height at the back BOTTOM pose
    static constexpr double WINCH_BACK_BOTTOM_POS = 100.0;
    /// Bar angle at the back TOP pose  clear of the low-clearance zone, which is what
    /// lets the lift sit at full extension there
    static constexpr double CHAINBAR_BACK_TOP_DEG = -155.0;
    /// Winch height at the back TOP pose
    static constexpr double WINCH_BACK_TOP_POS = WINCH_FULL_POS;
    /// Default EE orientation for the back cycle (the back family's CHAIN_SCORING_DEG)
    static constexpr double CHAIN_BACK_DEG = -85.0;
    /// Alternate back orientation, and it arms the same post-score retreat the front one does
    static constexpr double CHAIN_BACK_ALT_DEG = -265.0;
    /// Winch jog rate (motor deg/s) for L1 / L2 while parked in a back pose
    static constexpr double WINCH_BACK_JOG_DPS = 600.0;

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
    /// Claw state held in the power-on START pose.
    static constexpr bool CLAW_START_STATE = CLAW_OPEN_STATE;

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

    // ========================================================================
    // SCORE STATES  the named poses @ref run_score_state() can be sent to.
    //
    // ADD A ROW, THAT IS ALL: a key, a winch height, a bar angle and an EE angle, in the
    // same frames the constants above use. The move onto any of them is the same
    // raise -> swing -> lower choreography, so a new row needs no new code.
    // ========================================================================

    /// Case-sensitive equality for score-state keys. Hand-rolled rather than std::strcmp
    /// so it works during constant evaluation, which is what lets @ref score_table_valid
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
     * @brief Compile-time sanity check on a score-state table.
     *
     * Every pose must sit inside the soft limits: a pose outside one is not truncated,
     * the clamp silently becomes the setpoint and the mechanism drives to the LIMIT and
     * holds there against the sequence (log 07262026-4). A hand-written pose at least
     * gets read next to its limit; a table row is one careless number away from that
     * failure, so it is checked here.
     *
     * Also rejects a duplicate key, which would silently shadow the later row
     * (@ref find_score_state returns the first match).
     *
     * @return true if the table is usable. Called only from static_assert  a bad row
     *         is a build error, never a runtime surprise.
     */
    static constexpr bool score_table_valid(const ScoreState *table, int count)
    {
        for (int i = 0; i < count; ++i)
        {
            const ScoreState &s = table[i];
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

    /// Poses on the FRONT of the robot  the side the raise sequence works.
    static constexpr ScoreState FRONT_SCORE_STATES[] = {
        //  key        winch     bar       ee
        {"neutral_one_pin", 0.0, -280.0, -260.0},
    };
    /// Poses on the BACK of the robot.
    static constexpr ScoreState BACK_SCORE_STATES[] = {
        //  key        winch     bar       ee
        {"neutral_one_pin", 850.0, -28.0, -85.0},
        {"alliance_empty", 70.0, -28.0, -85.0},

    };
    static constexpr int FRONT_SCORE_STATE_COUNT =
        static_cast<int>(sizeof(FRONT_SCORE_STATES) / sizeof(ScoreState));
    static constexpr int BACK_SCORE_STATE_COUNT =
        static_cast<int>(sizeof(BACK_SCORE_STATES) / sizeof(ScoreState));

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
        SCORE_STATE,   ///< Moving onto, or holding, a pose from a score state table
                       ///< (@ref run_score_state()). The auton version of the back cycle
        SCORE_RETREAT, ///< Alt orientation only: @ref score() has released and the bar is
                       ///< swinging up to lift the flipped claw clear, then holding there
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
    /// and claw part way up without pausing the climb, then sweep the long way to primed
    /// and lower again as soon as the bar is clear of the low clearance zone
    ///
    /// RAISE and TUCK share one winch target, so the boundary between them is a bar/claw
    /// event only and the lift climbs continuously across both
    enum class PrimeStep
    {
        RAISE,      ///< Climb to the clearance height, bar held on the resting pose
        TUCK,       ///< Past WINCH_PRIME_TUCK_POS: bar + claw onto the tuck pose while
                    ///< the lift carries on up
        SWEEP_LOWER ///< Sweep to primed, lower once the bar is out of the zone
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
    /// Steps of the STOW->matchload move (@ref toggle_matchload()). Same shape as the
    /// startup and prime moves: lift up before the bar crosses the low clearance zone,
    /// swing, then come back down on the new pose
    enum class MatchloadStep
    {
        RAISE,     ///< Raise the lift for clearance (bar held where it started)
        BAR_SWEEP, ///< Swing the bar + claw onto the matchload pose, lift held up
        LOWER      ///< Lower onto the matchload height, then MATCHLOAD
    };
    /// The two poses of the back cycle, toggled with one button (@ref back_prime()).
    /// Both hold the claw closed on the back orientation, so the only difference is how
    /// far round the bar is and how high the lift sits
    enum class BackPose
    {
        BOTTOM, ///< bar CHAINBAR_BACK_BOTTOM_DEG, lift WINCH_BACK_BOTTOM_POS
        TOP     ///< bar CHAINBAR_BACK_TOP_DEG, lift WINCH_BACK_TOP_POS (full)
    };
    /// Steps of the move onto a back pose. Same shape as the matchload hop, and the same
    /// path for all three cases (entering the cycle, BOTTOM -> TOP, TOP -> BOTTOM), since
    /// each step passes straight through when it has nothing to do
    enum class BackStep
    {
        RAISE, ///< Lift up to at least WINCH_CLEARANCE_POS, bar held where it is
        SWING, ///< Bar (and claw) onto the destination pose, lift held up
        LOWER, ///< Lift onto the destination height, bar held on the pose
        HOLD   ///< Arrived: hold the pose, L1/L2 jog live, score armed
    };
    /// Steps of the move onto a score state. Same shape as @ref BackStep since it's the
    /// same problem, the bar may have to cross the low clearance zone to reach the pose
    ///
    /// Only difference is HOLD: the back cycle's is a live driver pose with the lift on
    /// L1/L2, this one is a plain hold since auton has no jog buttons
    enum class ScoreStep
    {
        RAISE, ///< Lift up to the clearance height, bar held where it is
        SWING, ///< Bar (and claw) onto the pose, lift held up
        LOWER, ///< Lift onto the pose's height, bar held on the pose
        HOLD   ///< Arrived: hold all three. score() is armed here (and only here)
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
    /// the lift, RESPOOL then reels up until the spool stalls and re-zeros there
    /// A homing descent runs HOLD -> SEEK -> RESPOOL -> HOLD
    enum class WinchMode
    {
        HOLD,
        SEEK,
        RESPOOL
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
    ScoreStep score_step_;         ///< score-state step (HOLD = arrived)
    /// Pose being moved onto / held, nullptr if we've never been in a score state.
    /// Points into the constexpr tables, so there's nothing to copy or own
    const ScoreState *score_target_;
    BackPose back_pose_;     ///< Back pose the cycle is on / heading for
    double back_hold_bar_;   ///< Bar angle held in BackStep::HOLD
    double back_hold_winch_; ///< Height held in BackStep::HOLD, moved by the jog
    Mode return_dest_;       ///< Where the running return is headed (STOW/MATCHLOAD)
    /// Bar angle a move started at. Every sequence holds the bar here while the lift
    /// climbs, so it can be entered from any pose without dragging the bar somewhere
    /// else first. SCORE_RETREAT also sits on it while it waits for clearance
    double entry_bar_deg_;
    bool startup_complete_;    ///< Arms the WINCH_REZERO_MAX_DEG cap on re-zeros
    bool chainbar_coasting_;   ///< In the sweep's coast window, brake-mode edge
    bool bar_coast_done_;      ///< This sweep coasted already, no second coast
    bool alt_return_gate_;     ///< Alt orientation return hold on, brake-mode edge
    double return_hold_winch_; ///< Height the return began at, the floor for that hold
    double raise_progress_;    ///< Position along the raise sequence [0, N-1]
    /// The capped swing running now has been seen moving, so its cap can be enforced.
    /// Latched for the whole swing, see the drive cap block in update()
    bool bar_swing_broke_out_;
    /// Movement 1 was commanded last tick. Edge trigger for freezing the bar where it is
    /// on release: while held the target is the far end of the swing, so we swap in the
    /// measured angle ONCE. Re-read every tick and the bar would just sag
    bool seq_swing_driving_;
    bool advancing_;             ///< move_lift_up() called this cycle (update consumes)
    bool retreating_;            ///< move_lift_down() called this cycle (update consumes)
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
    static void seq_targets(double progress, double &winch_out, double &bar_out);
    /// Where @p bar sits along movement 1, as a progress value in [0, 1]. Measured off
    /// the bar rather than integrated, since that movement is PID driven not clocked
    static double seq_swing_progress(double bar);
    /// Ramp @p current toward @p goal one tick, cruising at @p max_dps and tapering over
    /// the last @p decel_deg so the bar arrives at ~0 speed. Both default to the
    /// CHAINBAR_APPROACH_* constants, a move wanting its own speed passes its own pair
    /// The target is held within CHAINBAR_MAX_LEAD_DEG of @p actual_bar so it can't
    /// sprint ahead of a lagging bar, and @p out_vel_dps comes back for the velocity FF
    static double slew_bar_toward(double current, double goal, double actual_bar,
                                  double &out_vel_dps,
                                  double max_dps = CHAINBAR_APPROACH_MAX_DPS,
                                  double decel_deg = CHAINBAR_APPROACH_DECEL_DEG);
    /// True once the bar is past the low-clearance zone, so it no longer needs the lift
    /// held up and the two motions can overlap
    static bool bar_clear_of_low_zone(double bar);
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
    /// Bar angle where the running sweep hands the drive back to the feed-forward law.
    /// A fixed lead-in short of whatever that sweep aims at, so the matchload return
    /// hands over the same distance from its pose that the loading return does
    double sweep_catch_angle() const;
    /// Advance the START->STOW sequence one tick.
    void advance_start_sequence(double bar, double winch);
    /// Advance the loading->primed sequence one tick.
    void advance_prime_sequence(double bar, double winch);
    /// Advance the ->loading sequence one tick.
    void advance_return_sequence(double bar, double winch);
    /// Advance the STOW->matchload sequence one tick.
    void advance_matchload_sequence(double bar, double winch);
    /// Advance the back cycle one tick: the move onto back_pose_, then the HOLD that
    /// keeps it there and applies the L1/L2 lift jog (@p dt is what the jog integrates over)
    void advance_back_sequence(double bar, double winch, double dt);
    /// Advance the move onto score_target_ one tick. Same shape as the back cycle,
    /// then a plain hold on the pose
    void advance_score_state_sequence(double bar, double winch);
    /// Bar angle / winch height of the back pose currently selected.
    double back_bar_deg() const;
    double back_winch_pos() const;
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
    static const char *score_step_name(ScoreStep s);
    static const char *back_pose_name(BackPose p);
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
static_assert(Lift::score_table_valid(Lift::FRONT_SCORE_STATES, Lift::FRONT_SCORE_STATE_COUNT),
              "FRONT_SCORE_STATES: a row has an empty or duplicate key, or a pose outside "
              "[WINCH_TARGET_FLOOR, WINCH_MAX_POS] / [CHAINBAR_MIN_DEG, CHAINBAR_MAX_DEG] / "
              "[CHAIN_MIN_DEG, CHAIN_MAX_DEG]");
static_assert(Lift::score_table_valid(Lift::BACK_SCORE_STATES, Lift::BACK_SCORE_STATE_COUNT),
              "BACK_SCORE_STATES: a row has an empty or duplicate key, or a pose outside "
              "[WINCH_TARGET_FLOOR, WINCH_MAX_POS] / [CHAINBAR_MIN_DEG, CHAINBAR_MAX_DEG] / "
              "[CHAIN_MIN_DEG, CHAIN_MAX_DEG]");

#endif // LIFT_HPP
