#ifndef LIFT_HPP
#define LIFT_HPP

#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "controllers/pid_controller.hpp"
#include "hardware/enhanced_digital_out.hpp"
#include <cstdint>

/**
 * @brief Continuous lift + chainbar chainbar
 *
 *  COMPONENTS
 *   - Winch:    1 11W and 1 5.5W motors that control the continuous lift
 *   - Chainbar: one 11W motor geared 100->50 RPM that pivots the bar at the top
 *               of the lift. Also has a rot sensor on the axle for readings
 *   - Chain:    one 5.5W 200 RPM motor, 1:1 to the end effector, with a rot
 *               sensor on the EE axle for readings. Holds a fixed
 *               orientation (it is no longer sequenced by the macros).
 *   - Claw:     one open/close solenoid. Closes on @ref prime() to grab a piece,
 *               stays closed through the raise, and opens again when @ref score()
 *               detects the piece meeting the tower (or on @ref return_to_loading()).
 *
 * Coordinated raise/score sequence
 * --------------------------------
 * The heart of the mechanism is a single scalar, `raise_progress_`, in
 * [0, SEQ_KEYFRAMES-1] (= [0, 2]). Both the winch height and the chainbar angle
 * are piecewise-linear interpolations of three keyframes, so the whole motion is
 * fluid and can be paused / resumed anywhere. It is two movements:
 *
 *   p=0  primed : winch flat,           bar -305  (grab pose; claw closed)
 *   p=1         : winch flat,           bar -230  <- movement 1: bar swings out
 *   p=2         : winch fully extended, bar -235  <- movement 2: cascade extends
 *
 * @ref move_lift_up() advances the progress toward 3 while its button is held;
 * the instant it is released the progress (and therefore both actuators) freeze
 * wherever they are and PID-hold. Because the winch is scripted explicitly by the
 * keyframes (up while the bar sweeps through the low band, down at the rest poses),
 * the coordination itself provides the frame clearance -- there is no separate band
 * interlock any more.
 *
 * @ref score() does NOT run on this scalar. It is its own two-movement macro out of
 * the primed pose -- swing the bar out, then extend the cascade -- see the SCORE_*
 * constants.
 *
 * There are two resting poses at the bottom, STOW (loading) and MATCHLOAD, and the
 * raise sequence can be started from either -- see @ref toggle_matchload().
 *
 * There is also a second, simpler cycle working the BACK of the robot -- two poses
 * toggled by one button rather than a keyframed sequence -- which shares the score
 * macro, the orientation flip and the returns. See @ref back_prime().
 *
 * Driver-facing macros:
 *   - @ref prime()            STOW or MATCHLOAD -> primed (grab + KF0 pose)
 *   - @ref back_prime()       STOW or MATCHLOAD -> back cycle; then BOTTOM <-> TOP
 *   - @ref toggle_matchload() STOW <-> MATCHLOAD (via the clearance choreography)
 *   - @ref move_lift_up()     hold to advance the raise sequence (from primed)
 *   - @ref move_lift_down()   hold to retreat the raise sequence (manual reverse)
 *   - @ref score()            reverse the raise sequence w/ pressure detect; on
 *                             contact release + settle, then hold for the driver
 *   - @ref return_to_loading() any state -> STOW (release + loading pose)
 *   - @ref return_to_matchload() any state -> MATCHLOAD (release + matchload pose)
 *   - @ref open_claw()        manual claw toggle, any state (no motion, no mode change)
 *
 * Autonomous interface
 * --------------------
 * The macros above are written for a driver holding a button. Autonomous drives them
 * through two string-keyed entry points instead, both taking the same two arguments --
 * a @ref Side and a key:
 *
 *   - @ref run_macro(side, key)        one of the driver macros above, by name
 *                                      ("prime", "score", "return", ...). @p side picks
 *                                      the front/back variant where one exists.
 *   - @ref run_score_state(side, key)  drive to a named pose from that side's
 *                                      score-state table (@ref FRONT_SCORE_STATES /
 *                                      @ref BACK_SCORE_STATES).
 *
 * Both are NON-BLOCKING: they arm the move and return, and @ref update() walks it as
 * the caller keeps ticking. @ref is_busy() / @ref is_settled() are how the caller waits
 * (Robot::waitForLift wraps that loop).
 *
 * TO ADD A SCORE STATE: add one row to the table for its side -- key, winch height, bar
 * angle, end-effector angle -- and rebuild. Nothing else changes; a row whose pose falls
 * outside the soft limits is a COMPILE error (@ref score_table_valid), not a mechanism
 * that silently drives into a limit and holds there.
 *
 * @ref update() must be called every loop iteration (every Config::DT ms) for
 * the PID holds, sequencing and stall detection to work. Call @ref initialize()
 * once, at startup, with the mechanism physically in the START pose (it seeds all
 * three encoders to the START_* angles there and arms the START->STOW startup
 * choreography).
 *
 * NOTE: every geometry/threshold constant below is a starting point -- the signs
 * and magnitudes MUST be verified on the robot. In particular the sign
 * conventions assume: positive winch voltage RAISES/EXTENDS, positive chainbar
 * voltage increases the measured bar angle, and CLAW_CLOSED_STATE clamps the claw.
 */
class Lift {
public:
    /**
     * @brief Which side of the robot a keyed call is aimed at.
     *
     * The mechanism already works both sides -- the front raise sequence out of the
     * primed pose, and the back cycle's two poses -- so this is the same distinction
     * the driver makes with two different prime buttons, named. It selects the
     * front/back variant of a macro (@ref run_macro), which score-state table is
     * searched (@ref run_score_state), and which pair of raised end-effector
     * orientations the flip button resolves against afterwards (@ref EeFamily).
     */
    enum class Side { FRONT, BACK };

    /**
     * @brief One named scoring pose: the three joint positions, and nothing else.
     *
     * Adding a score state is one row in @ref FRONT_SCORE_STATES or
     * @ref BACK_SCORE_STATES -- no new enum member, no new branch, no new constants.
     * The move onto it is the same raise -> swing -> lower clearance choreography the
     * back cycle and the matchload hop use, so the low-clearance interlock is satisfied
     * for any pose in the table by construction.
     *
     * Every field is in the same frame as the hand-written pose constants below, and
     * each is range-checked against its soft limits at COMPILE time -- see
     * @ref score_table_valid for why that check is worth having.
     */
    struct ScoreState {
        const char* key;    ///< name passed to @ref run_score_state()
        double winch_pos;   ///< cascade height, motor deg, LOADING-referenced
        double bar_deg;     ///< chainbar angle, output deg (LOADING = 0)
        double ee_deg;      ///< end-effector angle, EE deg (1:1)
    };

    /**
     * @brief Construct the lift/chainbar mechanism.
     *
     * @param winch_motors    Two 200 RPM winch motors (mechanically coupled)
     * @param chainbar_motor  11W bar-pivot motor (geared 100->50)
     * @param chainbar_rotation Rotation sensor on the bar axle (1:1, output deg)
     * @param chain_motor     5.5W chain motor (1:1 to the end effector)
     * @param chain_rotation  Rotation sensor on the end-effector axle (1:1, EE deg)
     * @param claw            Claw solenoid (open = released, closed = clamped)
     * @param winch_pid       Winch position-hold PID (error deg, output mV)
     * @param chainbar_pid    Chainbar position PID (error output-deg, output mV)
     * @param chain_pid       Chain position PID (error deg, output mV)
     */
    Lift(pros::MotorGroup& winch_motors,
         pros::Motor& chainbar_motor,
         pros::Rotation& chainbar_rotation,
         pros::Motor& chain_motor,
         pros::Rotation& chain_rotation,
         EnhancedDigitalOut& claw,
         PIDController winch_pid,
         PIDController chainbar_pid,
         PIDController chain_pid);

    /**
     * @brief Configure encoder units, brake modes, and tare all three encoders.
     *
     * Call once at startup with the mechanism physically in the START pose. It zeroes
     * the WINCH encoders and seeds the software reference (winch_ref_offset_) to
     * WINCH_START_POS, so winch positions are measured from the taut bottom from the
     * very first tick -- the winch is on motor IMEs, which come up at 0 every boot, so
     * the robot MUST be in the START pose when this runs for the winch reference to be
     * right.
     *
     * That seed is now the AUTHORITY on the winch frame, not a placeholder: autonomous
     * may never visit the loading pose, so it cannot wait for the seek/respool pass
     * that used to establish the reference on the first descent into STOW. That pass
     * still runs when the mechanism does reach STOW, but only as a bounded correction
     * (WINCH_REZERO_MAX_DEG) on top of this seed.
     *
     * The bar and end-effector rotation sensors are NOT seeded: they hold their
     * reading across a power cycle, so the code takes them as given and only
     * configures their orientation. Their zero must therefore agree with the pose
     * convention (LOADING = 0) as a one-time commissioning step -- see
     * CHAINBAR_START_DEG / CHAIN_START_DEG, which now only describe what those
     * sensors should read in the START pose. The actual readings are logged as
     * "LIFTINIT sensors (unseeded)" on every boot so a drifted or re-magnetised
     * sensor is visible in the log rather than silently overwritten.
     *
     * This also arms the START->STOW startup choreography (@ref start_to_loading()),
     * which then plays out as update() is called each loop.
     */
    void initialize();

    // ---- Driver macros ------------------------------------------------------

    /**
     * @brief Move to the primed pose (raise-sequence keyframe 0).
     *
     * Closes the claw to grab the piece, then runs a short clearance choreography
     * to reach the primed pose: raise the lift for clearance, then sweep the bar
     * the long way round to the primed angle, dropping the lift back to the primed
     * (flat) height as soon as the bar has cleared the low-clearance zone
     * (CHAINBAR_LOW_ZONE_DEG) rather than waiting for the sweep to land. The end
     * effector tucks to CHAIN_PRIME_RAISE_DEG as the lift climbs and holds it until
     * the bar is far enough round to flip to the raised orientation. Both
     * motions must finish before the raise sequence takes over.
     *
     * The climb is not dead time: once the cascade passes WINCH_PRIME_TUCK_POS the bar
     * and the end effector move onto the TUCK pose (CHAINBAR_PRIME_TUCK_DEG /
     * CHAIN_PRIME_TUCK_DEG) WHILE the lift keeps rising to the clearance height -- the
     * winch target is the same either side of that point, so nothing waits for the tuck
     * and nothing waits for the lift. The long sweep then takes over at the clearance
     * height exactly as before, just from an angle already part way round.
     *
     * Valid from EITHER resting pose -- STOW (loading) or MATCHLOAD. The clearance
     * raise holds the bar wherever that pose left it rather than at a fixed angle,
     * so the same driver button primes from both and the bar never detours back to
     * the other pose first. The raise sequence always begins here. After this, hold
     * @ref move_lift_up() to raise.
     */
    void prime();

    /**
     * @brief Enter the BACK cycle, or toggle it between its two poses.
     *
     * One button, three jobs -- which one it does depends on where the mechanism is:
     *
     *  - From a resting pose (STOW or MATCHLOAD) it is the BACK PRIME: it closes the
     *    claw on the piece exactly as @ref prime() does, selects the back family of
     *    end-effector orientations, and moves onto the BOTTOM back pose.
     *  - Already in the back cycle, it TOGGLES the pose, BOTTOM <-> TOP.
     *  - Anywhere else it is ignored (and logged).
     *
     * Unlike the front, this is not a keyframed sequence walked by a progress scalar.
     * The back cycle is two poses and a clearance move between them:
     *
     *   BOTTOM : bar CHAINBAR_BACK_BOTTOM_DEG, cascade WINCH_BACK_BOTTOM_POS
     *   TOP    : bar CHAINBAR_BACK_TOP_DEG,    cascade WINCH_BACK_TOP_POS (full height)
     *
     * Every move onto a pose -- entering the cycle included -- runs raise -> swing ->
     * lower, so the bar never crosses the low-clearance zone with the cascade down.
     * Once parked, three things are live:
     *
     *  - @ref move_lift_up() / @ref move_lift_down() jog the CASCADE freely off the
     *    pose height (they drive the front raise sequence in the front cycle),
     *  - @ref score() runs the same release macro it does in the front cycle, and the
     *    same alternate-orientation retreat after it,
     *  - @ref toggle_scoring_orientation() flips the end effector between
     *    CHAIN_BACK_DEG and CHAIN_BACK_ALT_DEG -- the same button, the same meaning,
     *    and the alternate arms the same post-score -180 retreat the front one does.
     *
     * ENTERING AND LEAVING, both ways, both resting poses:
     *
     *  - IN from STOW or from MATCHLOAD -- the move onto BOTTOM opens by holding the
     *    bar at whatever angle that pose left it (entry_bar_deg_) while the cascade
     *    climbs, so it does not first drag the bar back to the other resting pose. The
     *    matchload angle is only a few degrees off the BOTTOM one, so from there the
     *    swing is nearly a no-op and only the cascade really moves.
     *  - OUT to either, from either pose, mid-move or after a score:
     *    @ref return_to_loading() and @ref return_to_matchload() work from anywhere in
     *    the cycle and run the ordinary return choreography. From TOP the cascade is
     *    already above the clearance height, so the bar sweeps home immediately; from
     *    BOTTOM the bar is inside the low-clearance zone and the return's own gate
     *    holds it on the pose until the cascade has come up, then sweeps. Neither case
     *    needs anything special here -- the return reads the mechanism, not the cycle.
     *
     * So the back cycle sits alongside the front one: both are entered from either
     * resting pose and both come home to either.
     */
    void back_prime();

    /**
     * @brief Move to the MATCHLOAD pose (from STOW) or back to STOW (from MATCHLOAD).
     *
     * One button, both directions. Matchload is a second resting pose for taking a
     * piece from the match loader: cascade at WINCH_MATCHLOAD_POS, bar at
     * CHAINBAR_MATCHLOAD_DEG, end effector at CHAIN_MATCHLOAD_DEG. @ref prime() works
     * from it exactly as it does from STOW.
     *
     * Both directions go through the lift-clearance choreography rather than swinging
     * the bar at the resting height: STOW->matchload raises the cascade, swings the
     * bar onto the pose, then lowers; matchload->STOW is @ref return_to_loading(),
     * which is the same move in reverse and is already tuned for it.
     *
     * From anywhere ELSE -- primed, part way up the sequence, mid-score, mid-return
     * -- it means "come back, to matchload", i.e. @ref return_to_matchload(). So the
     * one button is: at a resting pose, swap poses; otherwise, come home to the
     * matchload one. Only STARTUP ignores it, since that choreography owns the
     * mechanism until it has a known pose.
     */
    void toggle_matchload();

    /**
     * @brief Advance the coordinated raise sequence this iteration.
     *
     * Call every loop while the driver holds the raise button. The progress
     * scalar ramps toward the top of the sequence while held; when you stop
     * calling it the winch + bar freeze wherever they are and PID-hold.
     *
     * In the BACK cycle the same call means something simpler: it jogs the CASCADE up
     * off the back pose's height at WINCH_BACK_JOG_DPS, with the bar left on its pose.
     * (There is no sequence to advance there -- see @ref back_prime().)
     *
     * No-op anywhere else.
     */
    void move_lift_up();

    /**
     * @brief Retreat the coordinated raise sequence this iteration.
     *
     * The mirror image of @ref move_lift_up(): call every loop while the driver
     * holds the lower button and the progress scalar ramps back toward the primed
     * pose (progress 0) while held, freezing on release. Unlike @ref score() this
     * is a plain manual reverse -- no release and no auto-return to loading. In the
     * BACK cycle it jogs the cascade DOWN, the mirror of the raise jog. No-op
     * anywhere else. If both the raise and lower buttons are held, raising wins.
     */
    void move_lift_down();

    /**
     * @brief Score: release the piece where the mechanism is standing.
     *
     * It does NOT move anything onto the goal and does not try to detect the piece
     * meeting it. The driver has already put the mechanism where the piece should go
     * -- with the raise sequence, or the back cycle's poses plus the cascade jog -- so
     * the button's whole job is to let go. The claw opens, and that is the end of it
     * in the default orientation: no descent, no sweep, no overtravel, no contact
     * detection, and no mode change. The mechanism holds its pose, the raise/lower
     * buttons stay live, and the driver decides when to come home
     * (@ref return_to_loading() / @ref return_to_matchload()).
     *
     * ONE EXCEPTION, and only in the ALTERNATE raised orientation
     * (CHAIN_SCORING_ALT_DEG / CHAIN_BACK_ALT_DEG): there the end effector is flipped
     * half a turn, so the piece has just been dropped out of an EE that is itself
     * still sitting in the structure it was dropped into. Rather than leave it there
     * the macro swings the BAR (only) up to SCORE_ALT_RETREAT_BAR_DEG to lift it
     * clear, then parks. The cascade first rises WINCH_ALT_RETREAT_LIFT_DEG (clamped
     * to its travel) and the bar waits until it has, so the EE comes up and out rather
     * than dragging round through whatever it was dropped into. Nothing else moves:
     * the chain is not commanded and the cascade may only RISE, never fall, while the
     * EE is flipped and the bar is on the far side of the travel (the same interlock
     * the alternate-orientation return applies). That park
     * does not take itself home either, and the retreat pose is the angle the return's
     * hold releases at, so a Y/X from there comes down cleanly; the raise/lower buttons
     * are inert until then.
     *
     * Valid in the raise sequence and in the BACK cycle once PARKED on a pose
     * (@ref back_prime()) -- mid-move in the back cycle it is refused and logged,
     * since the alternate-orientation retreat would swing the bar while the cascade is
     * still in transit. No-op in any other state.
     */
    void score();

    /**
     * @brief Return to the STOW (loading) pose from any state.
     *
     * Opens the claw (releases the piece), then runs the same clearance
     * choreography as @ref prime() in reverse rather than swinging the bar
     * straight to loading: raise the lift into the safe zone while the bar already
     * sweeps the long way back -- driven by a plain step target with no slew and no
     * feed-forward, since this stretch only has to reach the top of the swing (the
     * sweep still pauses at the edge of the low-clearance zone,
     * CHAINBAR_LOW_ZONE_DEG, if the lift is not up yet). Once
     * the bar is past CHAINBAR_COAST_START_DEG and the lift is clear, it is switched
     * to brake mode COAST for the stretch where gravity and momentum do most of the
     * work -- but the PID keeps driving it (it used to be cut dead here), so a swing
     * that runs out of momentum over the top is pushed through instead of hanging on
     * an unpowered stretch. At CHAINBAR_COAST_CATCH_DEG brake HOLD comes back and
     * the drive hands over to the ordinary feed-forward law for the finish. The
     * sweep lands on CHAINBAR_LOWER_CLEAR_DEG -- just past loading -- so the bar is
     * clear while the lift lowers to the loading height; the bar then comes up onto
     * CHAINBAR_LOADING_DEG and the mechanism settles in STOW (which is what arms the
     * winch homing/respool pass). Also usable as a manual abort.
     *
     * If the end effector is in the ALTERNATE raised orientation
     * (CHAIN_SCORING_ALT_DEG, i.e. the flipped one), the first half of the sweep
     * runs under an extra hold: the chain motor is given no command at all (and is
     * switched to COAST, since a HOLD brake at 0 mV is still a command) and the
     * cascade is not allowed below the height the return started at, until the bar
     * has swung back to CHAIN_FLIP_BAR_DEG. All of it releases together at that
     * angle -- brake mode included -- and the rest of the return is unchanged.
     */
    void return_to_loading();

    /**
     * @brief Return to the MATCHLOAD pose from any state.
     *
     * The same choreography as @ref return_to_loading(), aimed at the other resting
     * pose: the sweep stops at CHAINBAR_MATCHLOAD_DEG instead of carrying on past
     * loading, the cascade comes down on that angle, and the end effector finishes
     * at CHAIN_MATCHLOAD_DEG. Because matchload sits part way round the return
     * travel, this is a SHORTER sweep than the loading return, not a longer one.
     *
     * Usable from anywhere the loading return is -- primed, part way up the
     * sequence, or mid-score -- so a matchload cycle can be closed without detouring
     * through loading. After a score the auto-return picks this on its own if the
     * cycle was primed from matchload; this is the manual override.
     */
    void return_to_matchload();

    /**
     * @brief Manually toggle the claw where it is: release the piece, or re-clamp.
     *
     * A pure claw command -- it does not move the winch, bar or chain and does not
     * change the mode, so it can be used at any point in the primed / raise /
     * score states to drop a piece the macros would otherwise hold on to (a
     * misgrab, a piece that has to come off before scoring, a jam). Nothing in
     * @ref update() re-commands the claw, so it stays open until a macro sets it
     * again: @ref prime() re-clamps, and @ref score() and the startup / return
     * choreographies open it. Valid in every mode; in STOW and during the return the
     * release direction is a no-op because the claw is already open there.
     */
    void open_claw();

    /**
     * @brief Flip the end-effector (claw) orientation used by the raised poses.
     *
     * The button stores a SELECTION -- default or alternate -- not an angle. Which
     * angles that selection means depends on the cycle in force: the front poses use
     * CHAIN_SCORING_DEG / CHAIN_SCORING_ALT_DEG, the back cycle uses CHAIN_BACK_DEG /
     * CHAIN_BACK_ALT_DEG. So the same button flips the end effector in both, and the
     * selection carries across a return and into the next prime of either kind.
     *
     * It takes effect immediately everywhere the raised orientation is commanded -- the
     * primed pose, the raise / score sequence driven from it, and both back poses -- so
     * it can be flipped before priming, while primed, or part-way through a cycle. The
     * STOW / loading and matchload poses always hold their own orientation regardless.
     *
     * Selecting the ALTERNATE also arms the post-score retreat in either cycle: see
     * SCORE_ALT_RETREAT_BAR_DEG.
     */
    void toggle_scoring_orientation();

    /**
     * @brief Startup choreography: move from the power-on START pose to STOW.
     *
     * The bar can't swing straight from START to the loading angle without
     * fouling the structure, so this runs a five-step sequence (one step per
     * @ref update()):
     *   1. rotate the bar CHAINBAR_START_CLEARANCE_DEG clear of the start pose,
     *   2. raise the cascade to WINCH_START_CLEARANCE_RAISE for swing room,
     *   3. swing the bar (and end effector) down to CHAINBAR_LOWER_CLEAR_DEG --
     *      just past loading, so the bar is clear as the cascade comes down. This
     *      is the same sweep the return runs, under the same control law
     *      (@ref bar_sweeping): step target, coasting brake, no feed-forward until
     *      the last few degrees,
     *   4. lower the cascade back down with the bar held there,
     *   5. bring the bar up onto CHAINBAR_LOADING_DEG and settle into the STOW
     *      pose (only then does the winch homing/respool pass run).
     *
     * Normally you don't call this yourself: @ref initialize() arms it
     * automatically. Non-blocking: it plays out as @ref update() is called every
     * loop. No-op unless currently in the STARTUP mode (power-on START pose).
     */
    void start_to_loading();

    // ---- Autonomous interface (keyed macros) --------------------------------

    /**
     * @brief Run one of the driver macros above by name.
     *
     * A thin dispatcher, not new behaviour: every key forwards to the macro that
     * already exists, with its own gating and logging unchanged.
     *
     *   "prime"             @ref prime() (FRONT) / @ref back_prime() (BACK)
     *   "matchload"         @ref toggle_matchload()
     *   "score"             @ref score()
     *   "return"            @ref return_to_loading()
     *   "return_matchload"  @ref return_to_matchload()
     *   "claw"              @ref open_claw()
     *   "flip"              @ref toggle_scoring_orientation()
     *   "up" / "down"       @ref move_lift_up() / @ref move_lift_down(), ONE tick each
     *                       (those two are hold-to-run, so a single call advances the
     *                       sequence by one Config::DT and no further)
     *
     * @param side  picks the front/back variant where one exists -- today only
     *              "prime". The rest are side-agnostic (the cycle in force already
     *              determines what they mean), and @p side is logged and ignored.
     * @param key   one of the names above; anything else is logged and refused.
     * @return true if the key was recognised and the macro was called. NOT a
     *         statement that the macro did anything -- a macro invalid from the
     *         current mode logs "IGNORED" and still returns true here.
     */
    bool run_macro(Side side, const char* key);

    /**
     * @brief Drive to a named pose from @p side's score-state table and hold it.
     *
     * NON-BLOCKING: this arms the move and returns; @ref update() walks it. Poll
     * @ref is_settled() (or call Robot::waitForLift) to wait for arrival.
     *
     * The move is the raise -> swing -> lower clearance choreography: the cascade
     * comes up to WINCH_CLEARANCE_POS (or the pose height, whichever is higher) with
     * the bar held wherever the previous pose left it, the bar and end effector then
     * swing onto the pose with the cascade held up, and only then does the cascade come
     * down onto the pose's height. So the bar never crosses the low-clearance zone
     * (CHAINBAR_LOW_ZONE_DEG) with the cascade down, whatever the pose is.
     *
     * It does NOT touch the claw. A scoring cycle is: "prime" to grab, this to place
     * the mechanism, "score" to release, "return" to come home.
     *
     * Valid from anywhere except STARTUP -- from either resting pose, part way up the
     * raise sequence, from the back cycle, or from another score state. STARTUP is
     * refused because that choreography owns the mechanism until it has a known pose.
     *
     * @param side  which table to search, and which raised-orientation family
     *              (@ref EeFamily) the flip button resolves against afterwards
     * @param key   a `ScoreState::key` from that table
     * @return true if the pose was found and the move armed; false for an unknown key
     *         or a call from STARTUP (both logged).
     */
    bool run_score_state(Side side, const char* key);

    /// Look up a score state by side + key, or nullptr if there is no such row.
    /// Exposed so a caller can validate a key (or read a pose) without commanding it.
    static const ScoreState* find_score_state(Side side, const char* key);

    /**
     * @brief True while a choreography is still playing out.
     *
     * The multi-step moves -- startup, prime, return, the matchload hop, the move onto
     * a back pose or a score state, and the alternate-orientation score retreat -- are
     * busy until they reach their final step. The resting poses are not, and neither is
     * the raise SEQUENCE: that one is walked by the driver's buttons and has no
     * destination of its own to arrive at.
     */
    bool is_busy() const;

    /**
     * @brief True when nothing is in flight AND all three joints are on their targets.
     *
     * The arrival test @ref run_score_state callers wait on. Tolerances are the ones
     * the sequences already use: WINCH_POS_TOL, CHAINBAR_POS_TOL, CHAIN_POS_TOL.
     */
    bool is_settled() const;

    /// Key of the score state currently being moved onto / held, or nullptr if the
    /// mechanism is not in a score state. Points into the constexpr table (static
    /// lifetime), so it is safe to hold on to.
    const char* current_score_state() const;

    // ---- Loop + telemetry ---------------------------------------------------

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
    //  Tunable geometry / thresholds -- VERIFY ON THE ROBOT.
    // ========================================================================

    // -- Winch (motor degrees) --
    /// Fully-down / flat height (the tare reference).
    static constexpr double WINCH_DOWN_POS = 0.0;
    /// Winch reading in the power-on START pose, in the TAUT-BOTTOM-referenced frame
    /// (negative, because the cascade powers on with string paid out below the taut
    /// bottom). THIS IS THE SINGLE KNOB for that offset: initialize() seeds
    /// winch_ref_offset_ with it, so every other WINCH_* height, the SEQ_WINCH_DEG
    /// keyframes and the clearance heights stay as they are -- they are all absolute in
    /// this frame.
    ///
    /// WHY IT IS NOW NON-ZERO. The reference used to be established only by the STOW
    /// respool: boot at 0, run the startup choreography, ground the lift, reel in until
    /// the string goes taut, call THAT 0. Autonomous does not necessarily visit the
    /// loading pose at all, so it cannot rely on that pass ever running -- and until it
    /// does, every winch height is off by however much slack the boot state happens to
    /// carry. Seeding the offset here makes the frame correct from the first tick and
    /// leaves the respool as a refinement rather than a prerequisite.
    ///
    /// MEASUREMENT (tests/logs/07282026.txt, the boot at 00:00:03-04):
    ///   - initialize() seeded the reference to 0 in the START pose
    ///     ("LIFTINIT refs: ... winch=0.0"),
    ///   - the startup choreography ran and settled in STOW, and the homing pass then
    ///     reeled in until the string went taut and the spool stalled at reading
    ///     +251.2 ("LIFTEVT respool TAUT ... @ winch=251.2 vel=10.1 cur=1198").
    /// That taut bottom IS WINCH_DOWN_POS by definition, so the START pose sits 251.2
    /// motor degrees below it: WINCH_START_POS = 0 - 251.2.
    ///
    /// CAVEAT -- this number is only as repeatable as the boot state. The 251.2 is the
    /// slack the spool carries at power-on, and the first-homing-after-boot readings
    /// across the 20-odd boots in tests/logs/ range from about 129 to 447 deg. This is
    /// the value from the run named above, as measured; re-measure it the same way
    /// (boot, let the startup choreography finish, read the "respool TAUT @ winch=" in
    /// the log) whenever the string, spool or start pose changes, and expect the
    /// bounded respool correction to absorb the run-to-run remainder.
    ///
    /// NOTE a negative value is below WINCH_DOWN_POS. update() therefore floors the
    /// winch target clamp at min(WINCH_DOWN_POS, WINCH_START_POS) rather than at
    /// WINCH_DOWN_POS -- otherwise the first tick would command the cascade up by the
    /// whole offset, and at winch kP=300 that is a railed 12000 mV yank on a slack
    /// string rather than the gentle WINCH_RESPOOL_MV take-up it deserves.
    static constexpr double WINCH_START_POS = -251.2;
    /// Floor of the commandable winch range -- the LOWER of the taut bottom and the
    /// seeded START reading, not WINCH_DOWN_POS.
    ///
    /// Below the taut bottom a hard stop holds the cascade and the spool simply pays out
    /// string, so readings down here are real (that is what WINCH_START_POS is) and
    /// commanding into them is legal: it means "sit on the stop with this much slack
    /// out", which is how the mechanism powers on. update() clamps every winch target to
    /// [this, WINCH_MAX_POS], and @ref score_table_valid checks table rows against the
    /// same bound -- they MUST agree, or the validator rejects poses the clamp would
    /// have honoured (or, worse, passes ones it would silently alter).
    ///
    /// NOTE a pose below WINCH_DOWN_POS is not a lower HEIGHT. The cascade is already
    /// grounded there; two rows at 0 and -200 park at the same place and differ only in
    /// how much slack the string carries.
    ///
    /// Written with a conditional rather than std::min so this header does not need
    /// <algorithm>.
    static constexpr double WINCH_TARGET_FLOOR =
        (WINCH_START_POS < WINCH_DOWN_POS) ? WINCH_START_POS : WINCH_DOWN_POS;
    /// Winch height the cascade settles at in the STOW (loading) pose. Set to
    /// the bottom: with the bar in loading a physical hard stop holds the lift
    /// up, so driving the winch fully down just pays out string, which the
    /// respool pass then tautens against the stop. Because this is within
    /// WINCH_POS_TOL of WINCH_DOWN_POS, STOW satisfies the respool trigger
    /// (see Lift::update, "commanded_down").
    static constexpr double WINCH_LOADING_POS = 0.0;
    /// Upper travel limit / fully-extended height for the sequence. PLACEHOLDER.
    static constexpr double WINCH_MAX_POS = 900.0;
    /// "Fully extended" keyframe height (top of the lift). Defaults to the travel
    /// limit; lower it if the sequence should top out short of the hard stop.
    static constexpr double WINCH_FULL_POS = WINCH_MAX_POS;
    /// Clearance height when lift moves near bottom of it's arc.
    ///
    /// RE-CHECK THIS (and WINCH_MAX_POS) NOW THAT WINCH_START_POS IS SEEDED. Every
    /// height here is absolute in the taut-bottom frame, and until the seed existed
    /// that frame was wrong by however much slack the boot carried -- in
    /// tests/logs/07282026.txt the startup RAISE reached a reading of 592 while the taut
    /// bottom was still 251 above the boot zero, so the cascade physically rose only
    /// ~341 above the bottom, not the 600 this constant asks for. With the frame
    /// corrected these numbers finally mean what they say, which means the cascade will
    /// travel FURTHER than it used to on every clearance raise. Confirm on the robot
    /// that 600 (and 900 at WINCH_MAX_POS) still clear the hard stop before running the
    /// mechanism at speed.
    static constexpr double WINCH_CLEARANCE_POS = 600.0;
    /// Height the cascade raises TO during startup for bar swing room -- an absolute
    /// height in the LOADING frame like every other clearance target, not a delta off
    /// the start pose (the room the bar needs does not move with where the mechanism
    /// happened to power on). PLACEHOLDER.
    static constexpr double WINCH_START_CLEARANCE_RAISE = WINCH_CLEARANCE_POS;
    /// Lift height raised for clearance while the bar sweeps loading<->primed the
    /// long way during prime(). Defaults to the startup clearance height; raise it
    /// if the long bar sweep needs more room. PLACEHOLDER.
    static constexpr double WINCH_PRIME_CLEARANCE_POS = WINCH_CLEARANCE_POS;
    /// Cascade height part way up the prime's clearance raise at which the bar and the
    /// end effector set off for the tuck pose (CHAINBAR_PRIME_TUCK_DEG /
    /// CHAIN_PRIME_TUCK_DEG). A TRIGGER height, not a pose to arrive at: the winch
    /// target is WINCH_PRIME_CLEARANCE_POS on both sides of it, so the lift climbs
    /// straight through and the tuck plays out UNDERNEATH a still-rising cascade rather
    /// than after it. Low enough that the tuck has the whole rest of the climb to
    /// finish in, high enough that the bar is not swinging while the mechanism is still
    /// sitting on the resting pose. PLACEHOLDER -- TUNE on robot.
    static constexpr double WINCH_PRIME_TUCK_POS = 0.0;
    /// Height tolerance for "winch reached target" / "winch is down".
    static constexpr double WINCH_POS_TOL = 40.0;

    // -- Winch respool (take up string slack at the bottom) --
    // After the lift bottoms out in STOW the string can go slack. Once settled at
    // the bottom, spin gently in the raise direction until the motor stalls
    // (string taut), then re-zero here and resume the position hold. Runs once per
    // descent; re-arms whenever the lift is next raised. Only ever runs in STOW so
    // it can never re-zero the encoder mid-sequence.
    //
    // IT IS A CORRECTION, NOT THE REFERENCE. It used to be the only thing that ever
    // established the winch frame, which made every winch height meaningless until the
    // mechanism had been to STOW once. Autonomous drives the lift through score states
    // and may never go there at all, so the frame is now seeded outright at boot from
    // WINCH_START_POS and this pass only trims it, by at most WINCH_REZERO_MAX_DEG per
    // descent. Nothing outside STOW depends on it any more.
    /// Master switch for the whole STOW homing pass (seek + respool + re-zero).
    ///
    /// Set FALSE to run purely on the boot seed (WINCH_START_POS) with nothing ever
    /// trimming it. That is the honest test of the seed: with homing on, a boot that
    /// happens to reach STOW quietly corrects the frame by up to WINCH_REZERO_MAX_DEG,
    /// so a score state commanded afterwards is testing the trimmed frame rather than
    /// the seeded one. With it off, whatever the seed says is what the mechanism runs
    /// on for the whole session -- which is exactly the situation autonomous is in when
    /// it never visits the loading pose.
    ///
    /// Leave TRUE for matches: the trim is free accuracy whenever the lift does happen
    /// to come home.
    static constexpr bool WINCH_STOW_HOMING_ENABLED = true;
    /// Gentle raise voltage (mV, positive = up) applied while taking up slack.
    static constexpr double WINCH_RESPOOL_MV = 3000.0;
    /// Below this |motor velocity| (RPM) during a respool counts as taut.
    static constexpr double WINCH_RESPOOL_STALL_RPM = 15.0;
    /// Grace time after a respool starts before the stall check is trusted.
    static constexpr uint32_t WINCH_RESPOOL_GRACE_MS = 150;
    /// Safeguard on the re-zero the respool performs when it finds the taut bottom:
    /// the reference may be shifted by at most this much (motor deg) in one pass.
    /// Armed once the startup choreography has finished (@ref startup_complete_);
    /// before that a re-zero of any size is allowed, since the pre-startup reference
    /// is only the seeded START-pose guess. A taut stall that claims a bigger
    /// correction than this is not a plausible amount of slack -- it is a false
    /// stall (snag, stiff spool, sensor glitch), and honouring it would teleport the
    /// winch reference and hand the PID a huge phantom error. Instead only this much
    /// of the correction is applied and the event is logged.
    static constexpr double WINCH_REZERO_MAX_DEG = 100.0;
    /// Safety cap: abort a respool that never finds resistance after this long.
    /// Generous, because with no return string the down-drive can pay out a lot
    /// of slack that the reel-in has to take up before the string goes taut --
    /// too short and a big slack reel times out before it can re-zero.
    static constexpr uint32_t WINCH_RESPOOL_TIMEOUT_MS = 1600;

    // -- Winch home-to-bottom (physical re-zero, IME-drift immunity) --
    // Homing runs once per descent in STOW as SEEK -> RESPOOL -> re-zero, so the
    // winch zero references a physical point (the taut bottom) rather than the
    // drift-prone IME.
    //
    // SEEK is only a short DOWN pre-roll. With no return string this lift just
    // unspools slack once it grounds -- it gives no clean bottom signature while
    // lowering (confirmed on the robot: velocity settles to a steady payout and
    // never stalls). So SEEK does NOT detect the bottom; it drives down briefly to
    // guarantee the lift is grounded with a little slack, then the RESPOOL reels
    // back up and stalls at the taut bottom -- the one clean signal -- and re-zeros
    // there. The per-tick LIFTSEEK log still prints pos/vel/current so the descent
    // stays visible.
    /// Gentle DOWN drive (mV, negative = retract) applied during the pre-roll.
    static constexpr double WINCH_SEEK_MV = -3000.0;
    /// Pre-roll duration (ms): drive down this long to ground the lift, then hand
    /// off to the respool. Keep it short -- the PID has already parked the lift at
    /// ~0 (near grounded), so this only needs to pay out a little slack.
    static constexpr uint32_t WINCH_SEEK_MS = 150;
    /// Safety cap: end the pre-roll early if the winch over-travels this far below
    /// WHERE THE SEEK STARTED (motor deg), so a fault can't pay out a birds-nest.
    /// Relative, not absolute: with the frame seeded from WINCH_START_POS a grounded
    /// lift with its slack out reads a long way below WINCH_DOWN_POS already.
    static constexpr double WINCH_SEEK_MAX_TRAVEL = 250.0;

    // -- Chainbar (output degrees, read directly from the axle rotation sensor) --
    /// Set true if the rotation sensor counts DOWN when positive chainbar voltage
    /// moves the bar, so the sensed angle still agrees with "positive voltage
    /// increases the angle"
    static constexpr bool CHAINBAR_ROTATION_REVERSED = false;
    /// How far a boot reading may sit from the START pose before initialize() logs a
    /// LIFTWARN. Not an error path -- the turn-count resolution below is anchored on
    /// the START pose, so if the mechanism was actually parked somewhere else the
    /// resolved angle is wrong by whole turns and the startup choreography will drive
    /// to the wrong place. Loose enough not to fire on ordinary hand-placement slop,
    /// tight enough that a genuinely wrong pose (start vs. loading vs. primed are all
    /// far apart) shows up in the log instead of in the cable.
    static constexpr double START_POSE_SANITY_DEG = 30.0;
    /// Bar angle in the power-on START pose. Descriptive only: the axle rotation
    /// sensor keeps its reading across a power cycle, so nothing writes this into it
    /// -- it is what the sensor SHOULD read there (and what the startup
    /// choreography's first clearance move is measured from). If the boot log's
    /// "LIFTINIT sensors (unseeded)" line disagrees with this, the sensor zero has
    /// moved and wants re-commissioning; the code will not paper over it.
    static constexpr double CHAINBAR_START_DEG = -25.0;
    /// How far to rotate the bar clear of the start pose before raising the
    /// cascade, during the START->STOW startup choreography..
    static constexpr double CHAINBAR_START_CLEARANCE_DEG = -30.0;
    /// Bar angle in the STOW (loading) pose. LOADING = 0 is the reference every pose
    /// angle here is measured from, so the sensor's zero has to agree with it.
    static constexpr double CHAINBAR_LOADING_DEG = 1.0;
    /// Bar angle held WHILE THE WINCH LOWERS at the end of the startup and return
    /// choreographies: a few degrees past loading in the negative (sweep) direction,
    /// so the bar is out of the way as the cascade comes down. Both sequences then
    /// bring the bar back to CHAINBAR_LOADING_DEG before entering STOW, i.e. before
    /// the winch homing/respool pass runs (that pass is gated to STOW).
    static constexpr double CHAINBAR_LOWER_CLEAR_DEG = -1.0;
    /// "Reached target" tolerance for the bar (used by the startup steps).
    static constexpr double CHAINBAR_POS_TOL = 4.0;
    /// Far edge of the LOW-CLEARANCE ZONE: the stretch of bar travel between the
    /// loading pose (0) and this angle where the bar swings low through the frame
    /// and the lift must be raised for it to pass. Past it (bar < this) the bar is
    /// clear and the winch is free to move regardless of where the sweep is -- which
    /// is what lets prime() lower while it is still sweeping to primed, and
    /// return_to_loading() raise while it is already sweeping back toward loading.
    static constexpr double CHAINBAR_LOW_ZONE_DEG = -90.0;
    /// Bar angle the prime's clearance raise tucks to on its way up, once the cascade
    /// passes WINCH_PRIME_TUCK_POS -- so the bar is already part way round by the time
    /// the long sweep to primed takes over at the clearance height, instead of setting
    /// off from the resting angle then.
    ///
    /// NOTE this sits INSIDE the low-clearance zone above, i.e. the bar enters the top
    /// of that zone with the cascade only WINCH_PRIME_TUCK_POS up, not at the full
    /// clearance height the long sweeps wait for. It is the same assumption
    /// CHAINBAR_MATCHLOAD_DEG (-37) already makes -- that the obstruction the zone
    /// guards is confined to part of that travel -- and this one is made with the
    /// cascade HIGHER than the matchload pose makes it, not lower. Verify on the robot
    /// before trusting it, and keep it on the loading side of CHAINBAR_LOW_ZONE_DEG:
    /// past that the tuck would be a full sweep, which is what SWEEP_LOWER is for.
    static constexpr double CHAINBAR_PRIME_TUCK_DEG = -40.0;

    // -- Chainbar gravity feed-forward (cures the standing undershoot) --
    // The bar is a pivoting arm, so holding it away from vertical needs a standing
    // torque to balance gravity. Pure-P can only make that by holding a standing
    // error, so the bar parks short of its target. This feed-forward supplies the
    // holding effort directly so the PID no longer fights gravity with a lingering
    // error.
    /// Peak feed-forward voltage (mV), applied when the arm is horizontal. PLACEHOLDER.
    static constexpr double CHAINBAR_GRAVITY_FF_MV = 1500.0;
    /// Bar angle (output deg) at which the arm hangs straight down -- where the
    /// feed-forward crosses zero and reverses sign.
    static constexpr double CHAINBAR_GRAVITY_ZERO_DEG = 0.0;

    // -- Coordinated raise sequence (progress-parameterized) --
    // The raise out of primed is TWO movements, one per keyframe segment:
    //   KF0 -> KF1  swing the bar from primed round to -230, cascade parked down
    //   KF1 -> KF2  run the cascade to full extension while the bar eases -230 -> -235
    // Holding the raise button walks progress 0 -> 2 through them and the lower
    // button walks them back, so the way down is the way up by construction.
    /// Number of keyframes in the raise sequence; progress runs [0, N-1].
    static constexpr int SEQ_KEYFRAMES = 3;
    /// Winch height (motor deg) at each keyframe: the cascade stays parked at the
    /// primed height for the whole bar swing, then goes to full extension.
    static constexpr double SEQ_WINCH_DEG[SEQ_KEYFRAMES] = {
        WINCH_DOWN_POS, WINCH_DOWN_POS, WINCH_FULL_POS};
    /// Chainbar angle (output deg) at each keyframe. Negative-going: the bar reaches
    /// the primed pose by rotating the LONG way round from the loading pose at 0,
    /// clearing the obstacle on the short path, so these all sit in the far-side
    /// range that the widened +/-360 limits allow.
    static constexpr double SEQ_BAR_DEG[SEQ_KEYFRAMES] = {-305.0, -230.0, -235.0};
    /// Progress units advanced per second while the raise button is held (and
    /// reversed per second while scoring). Only movement 2 (KF1<->KF2) runs on this
    /// clock -- it couples the cascade and the bar, so the two have to be walked
    /// together. Movement 1 is bar-only and is NOT timed: see SEQ_SWING_UP_MAX_MV /
    /// SEQ_SWING_DOWN_MAX_MV.
    /// PLACEHOLDER -- tune for feel.
    static constexpr double SEQ_RATE_PER_S = 1.0;
    /// Drive caps (mV) for movement 1, the bar-only swing between KF0 and KF1 --
    /// SPLIT BY DIRECTION, because the two directions are doing different jobs.
    ///
    /// That movement is not coupled to the cascade, so it is not interpolated on the
    /// clock: the target is simply the far end of the swing and the position PID
    /// takes the bar there at its own speed. Which means it is a ~75 deg step target
    /// -- P rails instantly -- and these caps are the only thing setting how fast the
    /// bar actually travels. Same idea as CHAINBAR_SWEEP_MAX_MV for the long sweeps.
    /// Braking is never capped (see the drive-cap block in update()), and a cap only
    /// binds while kP * error exceeds it.
    ///
    /// UP (KF0 -> KF1) climbs out of the primed pose against gravity and wants the
    /// authority; there is nothing past -230 to hit, so it is a pure feel knob. The
    /// alternate-orientation score retreat runs on this one too -- it is the same
    /// travel carried 50 deg further, climbing the same way. TUNE: raise for a
    /// snappier swing, lower if it arrives at KF1 hot.
    static constexpr double SEQ_SWING_UP_MAX_MV = 9000.0;
    /// DOWN (KF1 -> KF0) is that same travel with gravity now pulling the bar ALONG
    /// it, and it is the one the driver brings a held piece back down on -- so its
    /// arrival speed is the speed the piece meets whatever is under it, which is why
    /// it is set far below the up cap instead of sharing it.
    ///
    /// NOTE this sits BELOW the bar's breakaway command (CHAINBAR_KS_MV = 4500), so it
    /// cannot start the bar out of a hold on its own. update() handles that the same
    /// way it does for the other low caps: the swing gets full authority until it is
    /// genuinely moving, and the cap then LATCHES for the rest of that swing (see the
    /// drive-cap block). The latch matters as much as the cap -- keyed on "is the bar
    /// moving right now" instead, the limit would lift again the instant the piece met
    /// the tower and the bar stopped, which is the one moment it exists for.
    /// TUNE: lower for a softer descent, raise if it is too slow to be usable or the
    /// bar bogs down part way.
    static constexpr double SEQ_SWING_DOWN_MAX_MV = 2000.0;

    // -- Chainbar approach to the loading pose (slew-limited, no overshoot) --
    // The loading pose (0 deg) sits right next to a nogo object, so the bar must
    // NOT overshoot it. Stepping the target to 0 makes the bar arrive at ~450
    // deg/s and clip the object; instead the slewed steps ramp the target toward 0,
    // cruising at MAX_DPS and tapering the rate over the last DECEL_DEG so the bar
    // decelerates and arrives at ~0 speed.
    // WHO USES THIS LAW: every move that has to LAND on a pose -- the final
    // BAR_SETTLE step of the startup and return choreographies, the matchload hop,
    // the back-cycle swing, and the prime's tuck and sweep. The rate and the decel
    // zone are per-call (see @ref slew_bar_toward), so a long sweep can cruise at its
    // own speed and still decelerate into its pose; the constants here are the
    // defaults, used by everything that does not ask for its own.
    // WHO DOES NOT: the long sweeps toward loading (@ref bar_sweeping: the return's
    // RAISE_SWEEP and the startup's BAR_LOADING). They step their target and run
    // feed-forward-free, because they only have to carry the bar round to the loading
    // side, not arrive anywhere accurately.
    /// DEFAULT cruise rate of a slewed bar move (output deg/s). This is the
    /// commanded speed -- with the velocity feed-forward the bar actually tracks it
    /// (before, it crawled at ~100 dps regardless). Raise for a faster move, lower if
    /// the bar cannot keep up / overshoots. TUNE on robot.
    static constexpr double CHAINBAR_APPROACH_MAX_DPS = 150.0;
    /// DEFAULT distance from the goal (deg) over which a slewed move's rate tapers
    /// to a stop. This is the deceleration zone -- widen it for a gentler slow-down
    /// into the pose (from the higher cruise speed), narrow it for a later, harder
    /// stop. Scale it with the cruise rate: a move commanded twice as fast needs
    /// roughly twice the room to shed the speed.
    static constexpr double CHAINBAR_APPROACH_DECEL_DEG = 45.0;
    /// Floor on the tapered rate (fraction of MAX_DPS) so the target still
    /// finishes instead of asymptoting.
    static constexpr double CHAINBAR_APPROACH_MIN_FRAC = 0.15;
    /// Max the slewed target may lead the actual bar angle (deg). Caps the
    /// standing position error during the sweep so the PID output / integral
    /// can't build the sling that overshot the loading pose; also front-loads a
    /// firm breakaway command from the first tick. Big enough to break stiction
    /// (~kP*lead mV), small enough that the residual can't sling. TUNE on robot:
    /// raise if the bar stalls mid-sweep, lower if it still overshoots.
    static constexpr double CHAINBAR_MAX_LEAD_DEG = 20.0;
    /// Number of 100 Hz ticks the bar rate is averaged over (moving-window
    /// slope). The rotation sensor updates slower than the loop, so a per-tick
    /// delta aliases to a 0/spike sawtooth; a fixed multi-tick window gives a
    /// smooth rate with low lag and, unlike holding a stale rate, decays cleanly
    /// to zero once the bar stops. 5 ticks = 50 ms. Used in update().
    static constexpr int CHAINBAR_RATE_WINDOW = 5;

    // -- Chainbar static-friction feed-forward (breakaway; cures the stall) --
    // The bar has heavy stiction: a position PID with only a few degrees of
    // near-target error can't break it loose, so it stalls short of the clearance
    // pose (needing a hand push) and elsewhere sticks then lurches (stick-slip
    // jerk). This supplies a breakaway voltage in the DIRECTION of the position
    // error, so the bar starts moving without the integral having to wind up (and
    // then sling). It is faded linearly to zero within CHAINBAR_KS_DEADBAND_DEG
    // of the target so it neither chatters at rest nor drives the bar through the
    // loading pose into the nogo zone; the fade also eases the bar into the goal.
    /// Peak static-friction feed-forward (mV), applied at/above the deadband.
    /// Sized just under the observed breakaway command. PLACEHOLDER -- TUNE: raise
    /// if the bar still stalls short, lower if it creeps past / overshoots.
    static constexpr double CHAINBAR_KS_MV = 4500.0;
    /// Error band (deg) over which the static FF fades to zero as the bar nears
    /// its target (also sets how sharply it decelerates into the goal).
    static constexpr double CHAINBAR_KS_DEADBAND_DEG = 8.0;
    /// Bar speed (deg/s) at which the static FF has fully faded out. Static
    /// friction is a BREAKAWAY term -- only needed to start a stalled bar moving.
    /// Applying it while the bar is already moving made it reverse the instant the
    /// bar overran the slewed target, driving a stick-slip oscillation on the
    /// return-to-loading sweep (log 07242026-3). So the FF scales from full at
    /// rest to zero once |bar rate| reaches this, leaving P + gravity FF to carry
    /// the motion; if the bar re-sticks (rate falls), it re-engages for a nudge.
    static constexpr double CHAINBAR_KS_MOVING_DPS = 80.0;

    // -- Chainbar velocity feed-forward (smooth profiled sweeps) --
    // On the slewed approach to loading, the static breakaway FF alone left the bar
    // crawling at ~100 dps with a stick-slipping speed (log 07242026-4): its speed
    // was just an emergent result of P vs. friction, never commanded. These make the
    // bar TRACK the slew's velocity profile instead: kV feeds the voltage for the
    // commanded speed, and a kinetic-friction term keeps low speeds moving. Both
    // scale with the profile velocity (which tapers to ~0 over the decel zone), so
    // the bar cruises smoothly and decelerates into the pose with nothing left over
    // to push it past. Only applied when a profile velocity is active (the sweeps).
    /// Velocity feed-forward gain: motor mV per (output deg/s) of profile speed.
    /// ~= (cruise voltage above friction) / cruise speed. PLACEHOLDER -- TUNE: raise
    /// if the bar lags the profile, lower if it runs ahead / overshoots.
    static constexpr double CHAINBAR_KV_MV_PER_DPS = 12.0;
    /// Kinetic-friction feed-forward (mV) applied in the direction of travel while
    /// sweeping (faded with speed over CHAINBAR_KS_MOVING_DPS). Below the static
    /// breakaway (CHAINBAR_KS_MV) since sliding friction < stiction. TUNE.
    static constexpr double CHAINBAR_KINETIC_FF_MV = 1800.0;
    /// Profile-velocity magnitude (deg/s) above which update() uses the velocity
    /// feed-forward path instead of the position-error breakaway path.
    static constexpr double CHAINBAR_PROFILE_EPS_DPS = 1.0;

    // -- Chain / end effector (EE degrees, read from the EE axle rotation sensor) --
    // The chain is 1:1 to the end effector and the sensor sits on that axle, so
    // every CHAIN_* angle below is a true EE angle. It used to be read from the
    // chain motor's IME, which is the same 1:1 number in principle but goes through
    // the chain: any skip, slack or backlash there shows up as a permanent offset,
    // and it cannot be re-homed the way the winch can.
    /// Set true if the rotation sensor counts DOWN when positive chain voltage
    /// rotates the end effector, so the sensed angle still agrees with "positive
    /// voltage increases the angle" (which apply_soft_limit assumes).
    static constexpr bool CHAIN_ROTATION_REVERSED = false;
    /// EE angle in the power-on START pose. Descriptive only, same as
    /// CHAINBAR_START_DEG -- the EE sensor is not seeded either.
    static constexpr double CHAIN_START_DEG = -100.0;
    /// Orientation the end effector holds while loading / stowed.
    static constexpr double CHAIN_LOADING_DEG = -30.0;
    /// Orientation the end effector is tucked to during prime()'s climb out of the
    /// loading pose -- while the winch raises for clearance and through the first
    /// part of the bar sweep, up to the point the EE flips to the raised
    /// orientation. Held instead of CHAIN_LOADING_DEG for the whole pre-flip
    /// stretch, so the EE makes one move out of loading rather than tucking for the
    /// raise and swinging back to 29 for the sweep.
    static constexpr double CHAIN_PRIME_RAISE_DEG = 0.0;
    /// Orientation the end effector turns to at the same point in the climb the bar
    /// tucks at (WINCH_PRIME_TUCK_POS), the EE half of the tuck pose. Held for the rest
    /// of the raise; the sweep that follows puts the EE back on CHAIN_PRIME_RAISE_DEG
    /// until the bar is round far enough to flip (CHAIN_FLIP_BAR_DEG), so this angle
    /// only has to be safe for the climb itself. PLACEHOLDER -- TUNE on robot.
    static constexpr double CHAIN_PRIME_TUCK_DEG = -24.0;
    /// Orientation the end effector holds once raised (primed / sequence / scoring).
    /// This is the DEFAULT; @ref toggle_scoring_orientation() swaps it for the
    /// alternate below at run time.
    static constexpr double CHAIN_SCORING_DEG = -260.0;
    /// Alternate raised orientation: the same claw axis flipped half a turn, so a
    /// piece can be presented from the other side without re-priming.
    static constexpr double CHAIN_SCORING_ALT_DEG = -425.0;
    /// Bar angle at which the end effector swaps between its loading orientation and
    /// its raised one during the prime / return sweeps -- roughly the half-way point
    /// of the long swing, where the EE has room to turn. Also the release point of
    /// the alternate-orientation return hold (see ALT_RETURN_* below).
    static constexpr double CHAIN_FLIP_BAR_DEG = -180.0;

    // -- Return sweep coast-through --
    // Past the top of the swing the bar does not need to be driven home: gravity
    // pulls it toward the loading pose (which is where it hangs, see
    // CHAINBAR_GRAVITY_ZERO_DEG) and its own inertia carries it the rest of the way.
    // So for the middle of the travel the return sweep switches to brake mode COAST
    // (a HOLD brake at 0 mV would fight the swing) and lets the swing do the work.
    // The PID is NOT cut out in there, though: it used to be (0 mV, held in reset),
    // which left the bar with no drive at all if its momentum ran out on the way
    // over the top and no control of it until the catch angle. It now keeps driving
    // its ordinary sweep command through the window, so the coast is a brake-mode
    // and gravity-assist window rather than an unpowered one.
    /// Bar angle at which the return sweep switches to the coasting brake mode.
    /// (Numerically the same as CHAIN_FLIP_BAR_DEG today, but a separate knob -- one
    /// is about where the EE may turn, this is about where the swing takes over.)
    static constexpr double CHAINBAR_COAST_START_DEG = -225.0;
    /// Bar angle at which the coast window ends, about 10 deg short of the loading
    /// pose at 0. Two things happen here: brake mode goes back to HOLD, and the
    /// sweep hands the drive from its FF-free big-error law to the ordinary gravity
    /// + breakaway feed-forward, which is what actually closes the last few degrees
    /// (P alone loses to stiction there -- log 07262026-2). Earlier (more negative)
    /// = a longer, feed-forward-assisted finish; later = more of the travel left to
    /// the coast, but too late and the bar parks itself short of the pose.
    static constexpr double CHAINBAR_COAST_CATCH_DEG = -10.0;

    /// Speed cap for the return sweep: the largest command (mV) the chainbar motor
    /// may be given while RAISE_SWEEP is running (coast window included). Same idea
    /// as CHAIN_MAX_MV -- the sweep target is a step ~215 deg away, so P rails the
    /// output for the whole first stretch and the bar reached 560 deg/s downhill
    /// (log 07262026, 09.58), which is more than it can shed before the
    /// CHAINBAR_LOW_ZONE_DEG gate or the loading pose. Capping the command caps the
    /// cruise speed roughly in proportion, and arrival ENERGY with its square. Must
    /// stay above the bar's breakaway (CHAINBAR_KS_MV = 4500) or the sweep cannot
    /// start itself. Applied outside the PID, like CHAIN_MAX_MV, so the near-target
    /// behaviour and the anti-windup are untouched -- and only to a command that
    /// DRIVES the bar; a braking command keeps the full +/-12000 (see update()).
    /// Lower for a gentler sweep, raise for a faster one. TUNE on robot: the first
    /// 125 deg out of the primed pose is uphill, so if the bar bogs down or stalls
    /// on that climb this is the constant that starved it.
    static constexpr double CHAINBAR_SWEEP_MAX_MV = 8000.0;
    /// The same cap for the STARTUP sweep (BAR_LOADING), which is a different move
    /// wearing the same control law: 54 deg instead of 215, downhill from the first
    /// degree, and it has to stop next to the nogo object at the end. Under the
    /// return's cap it arrived at the pose doing 300-400 deg/s, overshot past the
    /// clear angle, and the brake bounced it back into the descending cascade
    /// (log 07262026-3). Applied ONLY once the bar is moving (above
    /// CHAINBAR_BRAKE_MIN_DPS) -- breaking this bar out of rest takes ~7000 mV and
    /// the sweep law has no breakaway feed-forward, so a low cap at rest would leave
    /// it parked in the START pose. TUNE: lower if it still comes in hot, raise if
    /// the sweep bogs down mid-travel. Note the cap only binds while
    /// kP * error exceeds it (below ~20 deg of error at 4000 mV the PID is under it
    /// anyway), so it governs the run-up, not the arrival -- if the bar still
    /// overshoots with this well down, the missing piece is deceleration, not drive.
    static constexpr double CHAINBAR_START_SWEEP_MAX_MV = 2000.0;
    /// The same cap for the PRIME sweep (SWEEP_LOWER), loading -> primed -- now a
    /// BACKSTOP rather than the speed knob. That sweep is slewed and commands its own
    /// speed (CHAINBAR_PRIME_SWEEP_DPS), so it no longer rails at 12000 mV for the
    /// whole travel the way the old step target did (516 deg/s on the downhill half,
    /// ringing for ~150 ms on arrival -- log 07262026-2, 07.9-08.2). What is left for
    /// this to do is bound the command if the profile ever asks for more than the
    /// mechanism should be given on the gravity-fed half. Applied only once the bar is
    /// moving, for the same breakaway reason as the startup cap, and never to a
    /// braking command. Leave it well above what the profile normally draws; to change
    /// the sweep's SPEED, change the DPS constant below instead.
    static constexpr double CHAINBAR_PRIME_SWEEP_MAX_MV = 6000.0;
    /// Commanded cruise speed of the PRIME sweep PAST THE TOP of the swing (output
    /// deg/s), i.e. from CHAINBAR_PRIME_CAP_START_DEG down to the primed pose -- the
    /// stretch gravity is feeding, and the one that has to arrive on a pose. THIS is
    /// the knob for how hard the bar comes down onto primed: the move is slewed
    /// (@ref slew_bar_toward with this rate and CHAINBAR_PRIME_SWEEP_DECEL_DEG), so
    /// the velocity feed-forward commands the speed directly instead of it emerging
    /// from whatever the drive cap above happened to allow, and the taper decelerates
    /// the bar into the pose rather than leaving it to arrive at cruise and be stopped
    /// by the shrinking position error alone.
    ///
    /// The CLIMB up to the top runs on its own, much faster rate -- see
    /// CHAINBAR_PRIME_CLIMB_DPS. PLACEHOLDER -- TUNE on robot: this is a driver-facing
    /// macro, so it is a feel trade. The old step-target sweep reached 516 deg/s on
    /// this half.
    static constexpr double CHAINBAR_PRIME_SWEEP_DPS = 150.0;
    /// Commanded cruise speed of the PRIME sweep's CLIMB (output deg/s): the stretch
    /// from the loading side up to CHAINBAR_PRIME_CAP_START_DEG, where the bar is
    /// working against gravity and there is nothing to arrive at.
    ///
    /// Deliberately set ABOVE what the bar can actually do (the old step-target sweep
    /// cruised 250-300 deg/s on the full rail here). A velocity command the mechanism
    /// cannot meet saturates: the profile runs ahead, the target sits at its full
    /// CHAINBAR_MAX_LEAD_DEG lead, and between the velocity feed-forward and that
    /// lead's P term the bar gets essentially the whole rail -- so it climbs as hard
    /// as it can, which is what this stretch wants and what it did before the sweep
    /// was profiled. There is no drive cap up here either (see
    /// CHAINBAR_PRIME_CAP_START_DEG), so nothing else is holding it back.
    ///
    /// Raising this further does nothing once the bar is already saturating; LOWER it
    /// (toward CHAINBAR_PRIME_SWEEP_DPS) if the climb wants to be gentler, at which
    /// point it becomes a real speed command again.
    static constexpr double CHAINBAR_PRIME_CLIMB_DPS = 450.0;
    /// Deceleration zone for that sweep (deg from the primed pose). Wider than the
    /// default because the sweep cruises faster than a short hop does and so has more
    /// speed to shed; this is the constant to widen if the bar still arrives hot at
    /// primed, and to narrow if the last stretch of the prime feels slow.
    static constexpr double CHAINBAR_PRIME_SWEEP_DECEL_DEG = 90.0;
    /// Bar angle that splits the prime sweep in two -- the top of the swing. It is
    /// where the drive cap above starts applying AND where the sweep hands over from
    /// CHAINBAR_PRIME_CLIMB_DPS to CHAINBAR_PRIME_SWEEP_DPS, which are the same
    /// distinction: above it the bar is climbing and wants everything, below it
    /// gravity is feeding the move and it has a pose to land on. Originally just the
    /// cap threshold -- the top of the
    /// swing, where gravity stops opposing the sweep and starts feeding it. Above
    /// this the bar is CLIMBING and wants every millivolt (it cruised 250-300 deg/s
    /// on the full rail; halving that would roughly double the climb time on a
    /// driver-facing macro), below it the motor is piling energy into a descent that
    /// gravity is already accelerating -- between -180 and -290 the motor did ~12300
    /// mV*rad of work against gravity's ~2000 (log 07262026-2). Costs little to gate
    /// here: the remaining 125 deg is long enough for the bar to settle at whatever
    /// speed the cap sustains either way. (Numerically the same as CHAIN_FLIP_BAR_DEG
    /// today, but a separate knob -- that one is about where the EE may turn, this is
    /// about where gravity changes sides.)
    static constexpr double CHAINBAR_PRIME_CAP_START_DEG = -180.0;
    /// Bar speed (deg/s) below which a command is never treated as braking, so the
    /// cap above always applies. The exemption keys on the command opposing the
    /// bar's travel, and the windowed rate wobbles a couple of deg/s either way
    /// when the bar is standing still -- without this floor a -2.2 deg/s flicker
    /// scored a full-drive command as "braking" and let 12000 mV through with 214
    /// deg of error still to go (log 07262026-2, 09.24). Well above that noise,
    /// well below any real swing speed.
    static constexpr double CHAINBAR_BRAKE_MIN_DPS = 20.0;

    /// Speed cap for the end effector: the largest command (mV) the chain motor may
    /// be given. The chain PID saturates on the big orientation changes -- a 240 deg
    /// flip times kP=120 rails the +/-12000 output on the first tick -- so the EE
    /// whipped round at near its free speed (200 RPM = 1200 deg/s). Capping the
    /// command caps the cruise speed roughly in proportion (4000 mV ~ a third of
    /// free speed). It only binds while |error| > CHAIN_MAX_MV / kP (~33 deg at
    /// kP=120), so holds, settling and small corrections keep the PID's full
    /// authority and only the long rotations are slowed. Lower for a gentler flip,
    /// raise for a faster one. TUNE on robot.
    static constexpr double CHAIN_MAX_MV = 4000.0;

    /// Ticks the chain rate is averaged over (windowed slope, same rationale as
    /// CHAINBAR_RATE_WINDOW): get_actual_velocity() flips +/-150 deg/s tick to tick
    /// with the EE physically at rest, which kD turns into ~850 mV of buzz
    /// (log 07242026-5, holding 270 deg). 5 ticks = 50 ms.
    static constexpr int CHAIN_RATE_WINDOW = 5;

    /// "Reached target" tolerance for the end effector -- the EE's counterpart to
    /// WINCH_POS_TOL and CHAINBAR_POS_TOL. Nothing in the sequences gates on the EE
    /// arriving (they gate on the bar and the cascade, and the EE turns underneath
    /// them), so this exists for @ref is_settled(): a caller waiting on a score state
    /// wants to know the CLAW is oriented, not just that the arm stopped. Loose enough
    /// that the chain's standing error under load does not hang the wait -- the EE is
    /// back-driven by the bar and its 5.5 W motor gives up position under load -- tight
    /// enough that a genuinely un-turned end effector does not read as arrived.
    /// TUNE on robot.
    static constexpr double CHAIN_POS_TOL = 8.0;

    // -- Matchloading pose --
    // A second resting pose alongside STOW, for taking a piece from the match
    // loader instead of the floor: same cascade height, bar swung part way round
    // and the end effector turned to meet the loader. @ref prime() works from here
    // exactly as it does from STOW, so the driver's prime button is unchanged.
    /// Cascade height at the matchload pose. Same as the loading height today, but
    /// kept separate so the two poses can move independently.
    static constexpr double WINCH_MATCHLOAD_POS = 0.0;
    /// Clearance height the cascade raises to for the loading->matchload swing --
    /// deliberately LOWER than WINCH_CLEARANCE_POS. That height is sized for the long
    /// sweeps, which carry the bar all the way round past the primed side; this hop
    /// only walks it from ~3 to CHAINBAR_MATCHLOAD_DEG, a short arc at the loading
    /// end of the travel, so it does not need the full lift and is quicker and
    /// steadier without it. TUNE: this is the height the bar clears the obstruction
    /// at, so raise it if the swing catches anything, lower it for a snappier hop.
    /// Must stay above WINCH_POS_TOL or the step's arrival check trips immediately.
    static constexpr double WINCH_MATCHLOAD_CLEARANCE_POS = 300.0;
    /// Bar angle at the matchload pose. NOTE this sits inside the nominal
    /// low-clearance zone (CHAINBAR_LOW_ZONE_DEG = -90 .. 0), which is the band the
    /// sweeps hold the cascade up for. Resting there with the cascade DOWN is
    /// therefore only safe if the obstruction that zone guards is confined to part
    /// of that travel -- verify before trusting the descent in the LOWER step.
    static constexpr double CHAINBAR_MATCHLOAD_DEG = -37.0;
    /// End-effector orientation at the matchload pose (same negative-going frame as
    /// the other CHAIN_* poses).
    static constexpr double CHAIN_MATCHLOAD_DEG = -75.0;

    // -- Back cycle (working the other side of the mechanism) --
    // A second scoring cycle that works over the BACK of the robot instead of swinging
    // the bar the long way round to the front primed pose. It is NOT a keyframed
    // sequence like the front one: it is two poses the driver toggles between with a
    // single button -- BOTTOM and TOP -- plus a free cascade jog on L1/L2 and the same
    // release-and-retreat score. Every move between the poses runs the same
    // raise -> swing -> lower clearance choreography the matchload hop uses, because
    // the bar has to cross the low-clearance zone (CHAINBAR_LOW_ZONE_DEG) to get
    // between them and cannot do that with the cascade down.
    /// Bar angle at the back BOTTOM pose. Like CHAINBAR_MATCHLOAD_DEG this sits INSIDE
    /// the nominal low-clearance zone, so RESTING here with the cascade low carries the
    /// same assumption -- that the obstruction the zone guards is confined to part of
    /// that travel. The TRAVEL to and from this angle is still gated on the cascade
    /// being up; only sitting here is not.
    static constexpr double CHAINBAR_BACK_BOTTOM_DEG = -30.0;
    /// Cascade height at the back BOTTOM pose.
    static constexpr double WINCH_BACK_BOTTOM_POS = 100.0;
    /// Bar angle at the back TOP pose -- swung clear of the low-clearance zone, which
    /// is what lets the cascade sit at full extension there.
    static constexpr double CHAINBAR_BACK_TOP_DEG = -155.0;
    /// Cascade height at the back TOP pose: all the way up.
    static constexpr double WINCH_BACK_TOP_POS = WINCH_FULL_POS;
    /// End-effector orientation held throughout the back cycle -- the back family's
    /// equivalent of CHAIN_SCORING_DEG, with CHAIN_BACK_ALT_DEG as its flipped partner.
    /// @ref toggle_scoring_orientation() picks between the two of whichever family is
    /// in force, so the driver's flip button means the same thing in both cycles.
    static constexpr double CHAIN_BACK_DEG = -85.0;
    /// Flipped back orientation (half a turn from CHAIN_BACK_DEG): the back family's
    /// CHAIN_SCORING_ALT_DEG. Selecting it arms the same post-score behaviour the front
    /// alternate does -- the -180 retreat, SCORE_ALT_RETREAT_BAR_DEG.
    static constexpr double CHAIN_BACK_ALT_DEG = -265.0;
    /// Cascade jog rate (motor deg/s) for L1 / L2 while parked in a back pose. The
    /// front sequence's movement 2 covers the full travel in about a second
    /// (SEQ_RATE_PER_S), so this is deliberately in the same ballpark. TUNE for feel.
    static constexpr double WINCH_BACK_JOG_DPS = 600.0;

    // -- Soft rotation limits (prevent wire/cable overwrap) --
    // Targets are clamped inside these bounds and the motor is never driven further
    // past a limit (positive voltage assumed to increase the measured angle).
    //
    // THESE MUST BRACKET EVERY POSE THE MACROS COMMAND. A pose outside its own soft
    // limit does not just get truncated -- the clamp silently becomes the setpoint,
    // so the mechanism drives to the LIMIT and then holds there against the sequence,
    // which reads as the motor running the wrong way. That is what CHAIN_MIN_DEG =
    // -110 did to the re-signed EE poses (log 07262026-4: scoring -260 clamped to
    // -110, and the EE reversed at -110 mid prime sweep and shoved back at the
    // +4000 cap). Re-sign or re-measure the poses and these have to move with them.
    static constexpr double CHAINBAR_MIN_DEG = -360.0;  ///< output degrees
    /// Upper bar limit = the loading pose / start of the nogo zone. The bar
    /// operates in [primed, loading] = [-305, 0], and there is an object just
    /// past 0, so the motor is never driven above it (apply_soft_limit zeros any
    /// positive command at/above this angle). Braking (negative) commands are
    /// still allowed here, so the PID can pull an overshoot back.
    static constexpr double CHAINBAR_MAX_DEG = 3.0;
    /// EE travel, in the same (negative-going) frame as the CHAIN_* poses: those run
    /// from CHAIN_PRIME_RAISE_DEG = 0 down to CHAIN_SCORING_DEG = -260, so the limits
    /// bracket that with margin -- 50 deg past the scoring pose, 110 deg the other
    /// side of the raise pose, mirroring the margins the old positive-going frame
    /// had. PLACEHOLDER magnitudes: the real bound is where the EE's wiring starts to
    /// overwrap, so confirm both ends on the robot.
    static constexpr double CHAIN_MIN_DEG = -500.0;     ///< EE degrees (1:1)
    static constexpr double CHAIN_MAX_DEG = 110.0;

    // -- Claw solenoid polarity --
    static constexpr bool CLAW_CLOSED_STATE = true;   ///< clamped
    static constexpr bool CLAW_OPEN_STATE = false;    ///< released
    /// Claw state held in the power-on START pose. PLACEHOLDER.
    static constexpr bool CLAW_START_STATE = CLAW_OPEN_STATE;

    /// Bar angle the score macro swings to once it has released -- but ONLY when the
    /// end effector is in the ALTERNATE raised orientation (CHAIN_SCORING_ALT_DEG). In
    /// that orientation the EE is flipped half a turn, so the piece has just been
    /// dropped out of an end effector still sitting in the structure it was dropped
    /// into; swinging the bar back up to here lifts it clear without waiting for the
    /// driver. In the DEFAULT orientation the score moves nothing at all -- the claw
    /// opens and the mechanism holds its pose.
    ///
    /// Numerically CHAIN_FLIP_BAR_DEG today, and deliberately so: that is the angle
    /// the alternate-orientation return hold releases at (see @ref
    /// alt_return_hold_active), so a return commanded from this pose starts with the
    /// EE already free to turn and the cascade already free to come down. Kept a
    /// separate knob because it is a POSE the macro drives to, not a threshold it
    /// tests against. TUNE: further negative parks the EE lower / less far round.
    static constexpr double SCORE_ALT_RETREAT_BAR_DEG = -180.0;

    // -- Score retreat park (stop balancing the bar at the top of the swing) --
    // SCORE_ALT_RETREAT_BAR_DEG is straight up: the one angle in the bar's travel
    // where gravity does not pick a side. Holding it with the ordinary drive gave a
    // sustained +/-20 deg limit cycle, and the DRIVE was the thing feeding it -- at 20
    // deg of error the bar gets kP*20 = 4000 mV of P plus the full CHAINBAR_KS_MV =
    // 4500 mV breakaway feed-forward, and that breakaway is scaled by how STOPPED the
    // bar is (ks_stuck_scale), so it arrives at full strength exactly at the
    // turnarounds where the bar is slowest. Every crossing therefore got re-kicked;
    // kD=10 could not dissipate that much per swing, and at the top there is no
    // gravity restoring term to help it settle.
    //
    // So once the bar first reaches the retreat angle the macro stops driving it: the
    // chainbar is commanded 0 mV with brake HOLD and the motor's own hold does the
    // work. That removes the energy source outright, and the brake absorbs the
    // arrival momentum instead of the PID reflecting it. Near vertical the gravity
    // load is at its smallest (sin ~ 0), which is the one thing that makes an
    // unpowered park practical here.
    /// Distance from the retreat angle (deg) at which the bar PARKS: drive cut to 0,
    /// brake HOLD takes over. Deliberately not tight -- this is a clearance pose, not
    /// a precision one, and parking on the FIRST crossing is the whole point (wait for
    /// a tighter window and the bar has already been kicked through it again). TUNE:
    /// widen if it still gets a kick in before parking, narrow for a tidier pose.
    static constexpr double SCORE_RETREAT_PARK_DEG = 6.0;
    /// Distance from the retreat angle (deg) at which a PARKED bar wakes up and is
    /// driven back. The hysteresis against SCORE_RETREAT_PARK_DEG is what keeps the
    /// park from becoming its own oscillation: only a real escape (the brake losing
    /// the bar, a knock) re-engages the drive, and it re-parks on the way through.
    /// Keep it comfortably wider than the park band. TUNE: lower if the bar is allowed
    /// to sag too far before anything reacts.
    static constexpr double SCORE_RETREAT_WAKE_DEG = 20.0;
    /// How far the CASCADE rises (motor deg) before the retreat swings the bar, off
    /// whatever height the piece was released at. The swing lifts the flipped end
    /// effector out of the structure it was dropped into by rotating the bar; giving
    /// it some height first means the EE comes up and out rather than dragging round
    /// through whatever it is sitting in. Raising is always allowed here even though
    /// LOWERING is not -- the interlock on a flipped EE is one-way (see the
    /// SCORE_RETREAT case in update() and @ref alt_return_hold_active()).
    ///
    /// Latched at release into retreat_winch_target_, CLAMPED to the cascade's travel:
    /// released at or near full extension there is no room for it and the swing simply
    /// starts straight away. The bar waits (on entry_bar_deg_) until the cascade has
    /// arrived, within WINCH_POS_TOL. TUNE on robot.
    static constexpr double WINCH_ALT_RETREAT_LIFT_DEG = 75.0;

    // ========================================================================
    //  SCORE STATES -- the named poses @ref run_score_state() can be sent to.
    //
    //  ADD A ROW, THAT IS ALL. Each row is one pose: a key, a cascade height, a bar
    //  angle and an end-effector angle, in exactly the frames the constants above use
    //  (winch = motor deg referenced to LOADING, bar = output deg with LOADING = 0,
    //  EE = degrees on the EE axle). The move onto any of them is the same
    //  raise -> swing -> lower clearance choreography, so a new row needs no new code
    //  and cannot break the low-clearance interlock.
    //
    //  The values below are seeded from the poses the hand-written macros already use
    //  -- the top of the front raise sequence, and the two back poses -- so the first
    //  build has working entries. They are STARTING POINTS: measure the real scoring
    //  poses on the robot and replace them.
    // ========================================================================

    /// Case-sensitive equality for score-state keys. A hand-rolled compare rather than
    /// std::strcmp so it also works during constant evaluation, which is what lets
    /// @ref score_table_valid run the duplicate-key check at compile time.
    static constexpr bool keys_equal(const char* a, const char* b) {
        while (*a != '\0' && *a == *b) { ++a; ++b; }
        return *a == *b;
    }

    /**
     * @brief Compile-time sanity check on a score-state table.
     *
     * Every pose must sit inside the soft limits, because a pose outside its own limit
     * is NOT merely truncated: update() clamps the target, the clamp silently becomes
     * the setpoint, and the mechanism drives to the LIMIT and holds there against the
     * sequence -- which reads as the motor running the wrong way. That is exactly what
     * CHAIN_MIN_DEG = -110 did to the re-signed EE poses (log 07262026-4). A
     * hand-written pose at least gets read in context next to its limit; a table row is
     * one careless number away from the same failure, so it is checked here instead.
     *
     * Also rejects a duplicate key, which would otherwise silently shadow the later row
     * (@ref find_score_state returns the first match).
     *
     * @return true if the table is usable. Called only from static_assert -- a bad row
     *         is a build error, never a runtime surprise.
     */
    static constexpr bool score_table_valid(const ScoreState* table, int count) {
        for (int i = 0; i < count; ++i) {
            const ScoreState& s = table[i];
            if (s.key == nullptr || *s.key == '\0') { return false; }
            // Against WINCH_TARGET_FLOOR, the same bound update() clamps to -- NOT
            // WINCH_DOWN_POS. A row below the taut bottom is legal (see the floor's
            // comment); a row the clamp would alter is not.
            if (s.winch_pos < WINCH_TARGET_FLOOR || s.winch_pos > WINCH_MAX_POS) { return false; }
            if (s.bar_deg   < CHAINBAR_MIN_DEG || s.bar_deg   > CHAINBAR_MAX_DEG) { return false; }
            if (s.ee_deg    < CHAIN_MIN_DEG    || s.ee_deg    > CHAIN_MAX_DEG)    { return false; }
            for (int j = 0; j < i; ++j) {
                if (keys_equal(s.key, table[j].key)) { return false; }
            }
        }
        return true;
    }

    /// Poses on the FRONT of the robot -- the side the raise sequence works.
    /// Seeded from the top of that sequence (SEQ_WINCH_DEG[2] / SEQ_BAR_DEG[2]) with
    /// the default raised orientation, plus a lower variant on the same bar angle.
    static constexpr ScoreState FRONT_SCORE_STATES[] = {
        //   key        winch     bar       ee
        {"neutral_one_pin",         0.0,  -280.0,  -260.0},
    };
    /// Poses on the BACK of the robot -- seeded from the back cycle's two poses.
    static constexpr ScoreState BACK_SCORE_STATES[] = {
        //   key        winch     bar       ee
        {"neutral_one_pin",         850.0,  -28.0,   -85.0},
        {"alliance_empty",        70.0,  -28.0,   -85.0},

    };
    static constexpr int FRONT_SCORE_STATE_COUNT =
        static_cast<int>(sizeof(FRONT_SCORE_STATES) / sizeof(ScoreState));
    static constexpr int BACK_SCORE_STATE_COUNT =
        static_cast<int>(sizeof(BACK_SCORE_STATES) / sizeof(ScoreState));

    // The static_asserts that enforce all of the above are just past the closing brace
    // of this class -- a constexpr member function is not usable in a constant
    // expression until the class is complete, so they cannot sit here.

private:
    /// What the mechanism is doing right now.
    /// STARTUP is the power-on START->STOW choreography; the mechanism leaves it
    /// once and never returns during a match.
    enum class Mode {
        STARTUP,   ///< Power-on START->STOW startup choreography
        STOW,      ///< Resting at the loading pose (winch flat-ish, claw open)
        MATCHLOAD, ///< Resting at the matchload pose -- a second loading pose that
                   ///< @ref prime() may be started from, exactly like STOW
        TO_MATCHLOAD, ///< Running the STOW->matchload clearance choreography
        PRIMING,   ///< Running the loading->primed clearance choreography
        SEQUENCE,  ///< In the coordinated raise sequence (position = raise_progress_)
        BACK,      ///< In the back cycle: holding, or moving between, the BOTTOM and
                   ///< TOP back poses (@ref back_prime()). back_step_ says which
        SCORE_STATE, ///< Moving onto, or holding, a named pose from a score-state
                     ///< table (@ref run_score_state()). score_step_ says which; the
                     ///< pose itself is score_target_. The autonomous counterpart of
                     ///< the back cycle -- same choreography, arbitrary pose
        SCORE_RETREAT, ///< ALTERNATE orientation only: @ref score() has released the
                       ///< piece and the bar is swinging back to
                       ///< SCORE_ALT_RETREAT_BAR_DEG to lift the flipped end effector
                       ///< clear of the structure, then holding there for the driver
        RETURNING  ///< Running the ->loading clearance choreography (reverse of PRIMING)
    };
    /// Steps of the START->STOW startup choreography (@ref start_to_loading()).
    enum class StartStep {
        IDLE,        ///< No start sequence active
        BAR_CLEAR,   ///< Rotate the bar CHAINBAR_START_CLEARANCE_DEG off start
        RAISE,       ///< Raise the cascade for bar swing room
        BAR_LOADING, ///< Swing the bar + end effector to CHAINBAR_LOWER_CLEAR_DEG
        LOWER,       ///< Lower the cascade back down (bar held clear of loading)
        BAR_SETTLE   ///< Bring the bar up to loading, then finish in STOW
    };
    /// Steps of the loading->primed clearance choreography (@ref prime()): raise
    /// the lift -- tucking the bar and EE part way up, without pausing the climb --
    /// then sweep the bar the long way to primed, lowering the lift again as soon as
    /// the bar is clear of the low-clearance zone, so the descent overlaps the back
    /// half of the sweep instead of following it.
    ///
    /// RAISE and TUCK share one winch target (WINCH_PRIME_CLEARANCE_POS), so the step
    /// boundary between them is a bar/EE event only: the cascade climbs continuously
    /// from the resting pose to the clearance height across both.
    enum class PrimeStep {
        RAISE,       ///< Climb to the clearance height, bar held on the resting pose
        TUCK,        ///< Past WINCH_PRIME_TUCK_POS: bar + EE onto the tuck pose while
                     ///< the cascade carries on up to the clearance height
        SWEEP_LOWER  ///< Sweep to primed; lower once bar < CHAINBAR_LOW_ZONE_DEG
    };
    /// Steps of the ->loading clearance choreography (@ref return_to_loading()):
    /// the mirror of PrimeStep -- raise the lift and sweep the bar back at the same
    /// time (the sweep only has to wait at the edge of the low-clearance zone for
    /// the lift to arrive), then lower to the loading height with the bar held just
    /// clear of loading, and finally bring the bar up onto the loading angle.
    enum class ReturnStep {
        RAISE_SWEEP,  ///< Raise the lift while sweeping back, capped at the zone edge
        LOWER,        ///< Lower the lift to the loading height (bar held clear)
        BAR_SETTLE    ///< Bring the bar up to loading, then finish in STOW
    };
    /// Steps of the STOW->matchload choreography (@ref matchload()). Same shape as
    /// the startup and prime clearance moves: get the cascade up before the bar
    /// crosses the low-clearance zone, swing, then come back down on the new pose.
    enum class MatchloadStep {
        RAISE,      ///< Raise the cascade for clearance (bar held where it started)
        BAR_SWEEP,  ///< Swing the bar + EE onto the matchload pose, cascade held up
        LOWER       ///< Lower the cascade onto the matchload height, then MATCHLOAD
    };
    /// The two poses of the back cycle. One button toggles between them
    /// (@ref back_prime()); both are held with the claw closed and the end effector on
    /// the back orientation, so the only difference is how far round the bar is and how
    /// high the cascade sits.
    enum class BackPose {
        BOTTOM,  ///< bar CHAINBAR_BACK_BOTTOM_DEG, cascade WINCH_BACK_BOTTOM_POS
        TOP      ///< bar CHAINBAR_BACK_TOP_DEG, cascade WINCH_BACK_TOP_POS (full)
    };
    /// Steps of the move onto a back pose -- the same raise / swing / lower clearance
    /// shape as the matchload hop, and the SAME path for all three cases that use it:
    /// entering the cycle from a resting pose, BOTTOM -> TOP, and TOP -> BOTTOM. Each
    /// step's arrival test passes straight through when it has nothing to do (going up,
    /// the cascade is already climbing to a height above the clearance one, so LOWER is
    /// a no-op; coming down, RAISE is), so one path covers every direction.
    enum class BackStep {
        RAISE,  ///< Cascade up to at least WINCH_CLEARANCE_POS, bar held where it is
        SWING,  ///< Bar (and EE) onto the destination pose, cascade held up
        LOWER,  ///< Cascade onto the destination height, bar held on the pose
        HOLD    ///< Arrived: hold the pose, L1/L2 cascade jog live, score armed
    };
    /// Steps of the move onto a score state -- deliberately the same raise / swing /
    /// lower shape as @ref BackStep, because it is the same clearance problem: the bar
    /// may have to cross the low-clearance zone (CHAINBAR_LOW_ZONE_DEG) to reach the
    /// pose and cannot do that with the cascade down. Each step's arrival test passes
    /// straight through when it has nothing to do, so one path covers a pose above the
    /// clearance height, a pose below it, and being entered from any angle.
    ///
    /// The one difference from the back cycle is HOLD: there it is a live driver pose
    /// with the cascade on L1/L2: here it is a plain hold. A score state is commanded
    /// by autonomous, which has no jog buttons, so advancing_ / retreating_ are ignored.
    enum class ScoreStep {
        RAISE,  ///< Cascade up to the clearance height, bar held where it is
        SWING,  ///< Bar (and EE) onto the pose, cascade held up
        LOWER,  ///< Cascade onto the pose's height, bar held on the pose
        HOLD    ///< Arrived: hold all three. score() is armed here (and only here)
    };
    /// Which PAIR of raised end-effector orientations is in force. The flip button
    /// picks between the two members of a pair (@ref ee_alt_); this picks the pair.
    /// Set by whichever prime started the cycle and left alone after that, so a score,
    /// its retreat and the return home all stay in the family they were primed in.
    enum class EeFamily {
        FRONT,  ///< CHAIN_SCORING_DEG / CHAIN_SCORING_ALT_DEG
        BACK    ///< CHAIN_BACK_DEG / CHAIN_BACK_ALT_DEG
    };
    /// Winch control mode. Normal position hold; SEEK actively drives down to
    /// find the physical bottom stop; RESPOOL then takes up slack in the raise
    /// direction until the spool stalls, and re-zeros there. A homing descent
    /// runs HOLD -> SEEK -> RESPOOL -> HOLD.
    enum class WinchMode { HOLD, SEEK, RESPOOL };

    // Devices
    pros::MotorGroup& winch_motors_;
    pros::Motor& chainbar_motor_;
    pros::Rotation& chainbar_rotation_;   ///< Bar-axle angle sensor (1:1, output deg)
    pros::Motor& chain_motor_;
    pros::Rotation& chain_rotation_;      ///< EE-axle angle sensor (1:1, EE deg)
    EnhancedDigitalOut& claw_;

    // Controllers
    PIDController winch_pid_;
    PIDController chainbar_pid_;
    PIDController chain_pid_;

    // State
    Mode mode_;
    StartStep start_step_;         ///< START->STOW sequence step (IDLE = none)
    PrimeStep prime_step_;         ///< loading->primed clearance step (valid while PRIMING)
    ReturnStep return_step_;       ///< ->loading clearance step (valid while RETURNING)
    MatchloadStep matchload_step_; ///< ->matchload clearance step (valid while TO_MATCHLOAD)
    BackStep back_step_;           ///< Where the back cycle is in its move onto a pose
                                   ///< (HOLD = arrived); valid while BACK
    ScoreStep score_step_;         ///< Where the score-state move is (HOLD = arrived);
                                   ///< valid while SCORE_STATE
    const ScoreState* score_target_; ///< Pose being moved onto / held, or nullptr if the
                                   ///< mechanism has never been in a score state. Points
                                   ///< into the constexpr tables, which have static
                                   ///< lifetime -- nothing to copy and nothing to own
    BackPose back_pose_;           ///< Back pose the cycle is on / heading for
    double back_hold_bar_;         ///< Bar angle held in BackStep::HOLD -- the pose's
                                   ///< own angle. Kept in state rather than read off
                                   ///< the pose each tick, to match back_hold_winch_
                                   ///< (which the driver's jog moves off the pose)
    double back_hold_winch_;       ///< Cascade height held in BackStep::HOLD -- the
                                   ///< pose's height, then whatever L1/L2 jog it to
    Mode return_dest_;             ///< Resting pose the running return is heading for
                                   ///< (STOW or MATCHLOAD). The return choreography is
                                   ///< one path with two sets of end poses.
    double entry_bar_deg_;         ///< Bar angle a move started at. PRIMING,
                                   ///< TO_MATCHLOAD and the back cycle hold the bar
                                   ///< here while the cascade climbs, so they can be
                                   ///< entered from either resting pose without first
                                   ///< dragging the bar back to the other one's angle;
                                   ///< SCORE_RETREAT sits on it while it waits for
                                   ///< cascade clearance.
    bool startup_complete_;        ///< START->STOW choreography has finished; arms the
                                   ///< WINCH_REZERO_MAX_DEG cap on respool re-zeros
    bool chainbar_coasting_;       ///< Chainbar is in the coast window in the middle
                                   ///< of the return sweep (brake COAST, still PID
                                   ///< driven); edge trigger for the brake-mode switch
    bool bar_coast_done_;          ///< This sweep has already coasted and been caught,
                                   ///< so a bounce back past the catch angle cannot
                                   ///< drop it into a second coast
    bool alt_return_gate_;         ///< Alternate-orientation return hold is in effect
                                   ///< (chain unpowered + coasting, cascade not lowering);
                                   ///< also the edge trigger for the brake-mode switch
    double return_hold_winch_;     ///< Winch height (motor deg) the return began at; the
                                   ///< floor the cascade is not allowed below while the
                                   ///< alternate-orientation hold is active
    double raise_progress_;        ///< Position along the raise sequence [0, N-1]
    bool bar_swing_broke_out_;     ///< The capped bar swing running right now has been
                                   ///< seen moving, so its (possibly sub-breakaway)
                                   ///< drive cap may be enforced. Latched for the whole
                                   ///< swing and cleared whenever no swing cap is in
                                   ///< play -- see the drive-cap block in update()
    bool seq_swing_driving_;       ///< Movement 1 was being commanded last tick. Edge
                                   ///< trigger for freezing the bar where it is when
                                   ///< the driver lets go mid-swing (the target is the
                                   ///< far end of the swing while held, so it has to
                                   ///< be replaced with the measured angle on release,
                                   ///< once -- not re-read every tick, or the bar
                                   ///< would sag with no restoring error).
    bool advancing_;               ///< move_lift_up() called this cycle (consumed by update)
    bool retreating_;              ///< move_lift_down() called this cycle (consumed by update)
    double winch_manual_target_;   ///< Resolved winch height target (motor deg)
    double bar_target_deg_;        ///< Resolved chainbar goal angle (output deg)
    double bar_target_vel_dps_;    ///< Profile velocity for the bar this tick (0 = not profiled)
    double chain_target_deg_;      ///< Chain hold angle (motor deg)
    EeFamily ee_family_;           ///< Which raised-orientation PAIR is in force (set by
                                   ///< the prime that started the cycle)
    bool ee_alt_;                  ///< Which member of that pair is selected -- the flip
                                   ///< button's state. Together these resolve to
                                   ///< @ref raised_ee_deg(), and ee_alt_ on its own is
                                   ///< what arms the alternate-orientation return hold
                                   ///< and the post-score -180 retreat, in EITHER family
    double retreat_winch_target_;  ///< Cascade height the alternate-orientation retreat
                                   ///< lifts to before it swings the bar: the height at
                                   ///< release plus WINCH_ALT_RETREAT_LIFT_DEG, clamped
                                   ///< to the travel. Latched by @ref score()
    bool score_retreat_parked_;    ///< The alternate-orientation retreat has reached the
                                   ///< retreat angle and the bar is parked there on 0 mV
                                   ///< + brake HOLD rather than being balanced by the
                                   ///< PID. Latched, with SCORE_RETREAT_WAKE_DEG of
                                   ///< hysteresis; re-armed on entry to SCORE_RETREAT
    /// Offset added to the raw winch encoder average to get @ref get_winch_position().
    ///
    /// The winch reference lives HERE, in software, rather than in the motors. It has
    /// to: the winch group is `{-16, 17}`, one motor reversed, and
    /// MotorGroup::set_zero_position_all() writes the requested value into each motor
    /// WITHOUT the reversal -- so the reversed motor then reads -p while its partner
    /// reads +p, and motor_group_position() averages them back to ~0 for ANY p. That is
    /// not a guess: the four capped re-zeros in the logs asked for 63, 127, 135 and 151
    /// (tests/logs/07262026-2, -4, 07262026, 07282026) and every one of them landed on a
    /// reading of ~0 on the very next tick. The re-zero cap has therefore never actually
    /// worked, and seeding a non-zero WINCH_START_POS through that call would have done
    /// nothing at all.
    ///
    /// So initialize() zeroes the encoders (p = 0, the one value the bug cannot corrupt)
    /// and puts the reference in this member instead; the respool re-zero shifts it
    /// rather than rewriting the motors. Both paths are then reversal-proof.
    double winch_ref_offset_;
    uint32_t last_update_ms_;      ///< millis() of previous update (for dt)
    WinchMode winch_mode_;         ///< Winch control mode (HOLD / SEEK / RESPOOL)
    bool winch_respooled_;         ///< Homed (sought + respooled) since last raise
    uint32_t respool_start_ms_;    ///< millis() when the current respool began
    uint32_t seek_start_ms_;       ///< millis() when the current home-seek began
    double seek_start_pos_;        ///< Winch reading the current home-seek began at.
                                   ///< The over-travel backstop is measured from here,
                                   ///< since a grounded lift with slack out already
                                   ///< sits well below WINCH_DOWN_POS
    double bar_angle_hist_[CHAINBAR_RATE_WINDOW];  ///< recent bar angles (idx 0 newest) for the windowed rate
    double chain_angle_hist_[CHAIN_RATE_WINDOW];   ///< recent EE angles (idx 0 newest) for the windowed rate

    // Helpers
    double motor_group_position() const;
    /// Bar angular velocity in output deg/s, from the axle rotation sensor.
    double bar_velocity_dps() const;
    /// Interpolate the winch height + bar angle for a given sequence progress.
    static void seq_targets(double progress, double& winch_out, double& bar_out);
    /// Where @p bar sits along movement 1, as a progress value in [0, 1]. That
    /// movement is PID-driven rather than clocked, so its progress is MEASURED off
    /// the bar instead of integrated -- which keeps raise_progress_ meaningful (the
    /// logs, and the hand-over into movement 2) without pretending to know how long
    /// the swing takes.
    static double seq_swing_progress(double bar);
    /// Ramp @p current toward @p goal one tick, cruising at @p max_dps and tapering
    /// over the last @p decel_deg so the bar arrives with ~0 speed (no overshoot into
    /// the nogo object beside the loading pose, and no arriving at cruise anywhere
    /// else). Both default to the CHAINBAR_APPROACH_* constants; a long move that
    /// wants its own speed passes its own pair -- the prime sweep does
    /// (CHAINBAR_PRIME_SWEEP_DPS / _DECEL_DEG). The returned target is also held
    /// within CHAINBAR_MAX_LEAD_DEG of @p actual_bar so it never sprints ahead of a
    /// lagging bar. @p out_vel_dps receives the signed profile velocity (deg/s) this
    /// tick, for the velocity feed-forward.
    static double slew_bar_toward(double current, double goal, double actual_bar,
                                  double& out_vel_dps,
                                  double max_dps = CHAINBAR_APPROACH_MAX_DPS,
                                  double decel_deg = CHAINBAR_APPROACH_DECEL_DEG);
    /// True once the bar has swept past the low-clearance zone, i.e. it no longer
    /// needs the lift held up and the two motions may overlap.
    static bool bar_clear_of_low_zone(double bar);
    /// Pick the multiple of 360 that puts @p raw_deg closest to @p expected_deg.
    /// The rotation sensors come up reporting their absolute angle folded into
    /// [0, 360) -- the bar in its START pose reads ~335, not -25 -- and
    /// get_position() then accumulates continuously from whatever that start value
    /// was. So the boot reading needs its turn count resolved, once, against the pose
    /// the mechanism is documented to be in; after that the sensor tracks itself.
    /// Unambiguous as long as the mechanism boots within +/-180 deg of that pose.
    static double unwrap_angle_near(double raw_deg, double expected_deg);
    /// Zero out a command that would drive @p angle further past a soft limit
    /// (positive voltage assumed to increase the angle).
    static double apply_soft_limit(double output_mv, double angle,
                                   double min_deg, double max_deg);
    /// Begin a return to the STOW pose (opens the claw, drops the winch).
    /// Begin a return to @p dest (STOW or MATCHLOAD): opens the claw, drops the
    /// winch reference, and arms the RAISE_SWEEP choreography aimed at that pose.
    void begin_return(Mode dest);
    /// Bar angle at which the running sweep hands the drive from the FF-free sweep
    /// law back to the ordinary feed-forward one. Held a fixed lead-in short of
    /// whatever that sweep is aiming at, so a return to matchload hands over the
    /// same distance from ITS pose that a return to loading does from loading.
    double sweep_catch_angle() const;
    /// Advance the START->STOW sequence one tick.
    void advance_start_sequence(double bar, double winch);
    /// Advance the loading->primed clearance choreography one tick.
    void advance_prime_sequence(double bar, double winch);
    /// Advance the ->loading clearance choreography one tick.
    void advance_return_sequence(double bar, double winch);
    /// Advance the STOW->matchload clearance choreography one tick.
    void advance_matchload_sequence(double bar, double winch);
    /// Advance the back cycle one tick: the raise/swing/lower move onto back_pose_,
    /// then the HOLD that keeps it there and applies the L1/L2 cascade jog (@p dt is
    /// the control step the jog integrates over).
    void advance_back_sequence(double bar, double winch, double dt);
    /// Advance the move onto score_target_ one tick: the same raise / swing / lower
    /// clearance shape the back cycle uses, then a plain hold on the pose.
    void advance_score_state_sequence(double bar, double winch);
    /// Bar angle / cascade height of the back pose currently selected.
    double back_bar_deg() const;
    double back_winch_pos() const;
    /// The raised end-effector orientation in force: the ee_alt_ member of the
    /// ee_family_ pair. Every mode that commands a raised pose reads this each tick, so
    /// flipping the orientation turns the EE immediately wherever the mechanism is.
    double raised_ee_deg() const;
    /// The two long bar sweeps toward the loading side that share one control law:
    /// the return's RAISE_SWEEP and the startup's BAR_LOADING. Both cross the same
    /// travel with the cascade held up, both only have to get the bar round rather
    /// than land it accurately, and both hand the finish to the feed-forward branch
    /// at CHAINBAR_COAST_CATCH_DEG. See the sweep branch of update().
    bool bar_sweeping() const;
    /// Whether the chainbar should be coasting this tick: mid sweep (@ref
    /// bar_sweeping), past CHAINBAR_COAST_START_DEG but not yet at
    /// CHAINBAR_COAST_CATCH_DEG, with the lift already at its clearance height and no
    /// catch having happened yet. In the window the swing does most of the work
    /// (brake COAST, no feed-forward) but the PID still drives its sweep command.
    bool bar_coast_active(double bar, double winch) const;
    /// Whether the alternate-orientation return hold applies this tick: a return
    /// (typically the one after a score) running with the EE in the ALTERNATE raised
    /// orientation, with the bar not yet swung back to CHAIN_FLIP_BAR_DEG. While it
    /// holds, the chain motor gets NO command at all and the cascade may not descend
    /// below the height the return started at.
    bool alt_return_hold_active(double bar) const;
    /// Whether the final BAR_SETTLE move (CHAINBAR_LOWER_CLEAR_DEG ->
    /// CHAINBAR_LOADING_DEG, shared by the startup and return sequences) is done.
    bool settle_bar_arrived(double bar, double rest_bar) const;

    static const char* mode_name(Mode m);
    static const char* start_step_name(StartStep s);
    static const char* prime_step_name(PrimeStep s);
    static const char* return_step_name(ReturnStep s);
    static const char* matchload_step_name(MatchloadStep s);
    static const char* back_step_name(BackStep s);
    static const char* score_step_name(ScoreStep s);
    static const char* back_pose_name(BackPose p);
    static const char* winch_mode_name(WinchMode m);
    /// Emit one CSV-style debug line covering every command/target/actual/output.
    void log_debug_state(double winch_target, double winch, double winch_mv,
                         double winch_vel_rpm,
                         double bar, double bar_mv, double bar_vel_dps,
                         double bar_gravity_ff, double bar_static_ff,
                         double ee, double chain_mv, double chain_vel_dps) const;
};

// Score-state tables are checked here, at namespace scope, rather than inside the class:
// Lift::score_table_valid is a constexpr MEMBER function, and a member function defined
// in-class is not complete -- so not usable in a constant expression -- until the class
// itself is. Keep these next to the class, and keep the failure messages pointing back at
// the table: a bad row must be a build error rather than a mechanism that quietly drives
// into a soft limit and holds there (see the comment on score_table_valid).
static_assert(Lift::score_table_valid(Lift::FRONT_SCORE_STATES, Lift::FRONT_SCORE_STATE_COUNT),
              "FRONT_SCORE_STATES: a row has an empty or duplicate key, or a pose outside "
              "[WINCH_TARGET_FLOOR, WINCH_MAX_POS] / [CHAINBAR_MIN_DEG, CHAINBAR_MAX_DEG] / "
              "[CHAIN_MIN_DEG, CHAIN_MAX_DEG]");
static_assert(Lift::score_table_valid(Lift::BACK_SCORE_STATES, Lift::BACK_SCORE_STATE_COUNT),
              "BACK_SCORE_STATES: a row has an empty or duplicate key, or a pose outside "
              "[WINCH_TARGET_FLOOR, WINCH_MAX_POS] / [CHAINBAR_MIN_DEG, CHAINBAR_MAX_DEG] / "
              "[CHAIN_MIN_DEG, CHAIN_MAX_DEG]");

#endif // LIFT_HPP
