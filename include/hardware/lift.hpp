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
 * [0, SEQ_KEYFRAMES-1] (= [0, 3]). Both the winch height and the chainbar angle
 * are piecewise-linear interpolations of four keyframes, so the whole motion is
 * fluid and can be paused / resumed anywhere:
 *
 *   p=0  primed : winch flat,          bar +45   (grab pose; claw closed)
 *   p=1         : winch fully extended, bar +15
 *   p=2         : winch flat (bottom),  bar -45
 *   p=3         : winch fully extended, bar -15
 *
 * @ref move_lift_up() advances the progress toward 3 while its button is held;
 * the instant it is released the progress (and therefore both actuators) freeze
 * wherever they are and PID-hold. @ref score() reverses the progress back toward
 * 0 (the primed pose) and then SCORE_OVERTRAVEL_DEG past it, while watching for
 * the piece meeting the tower (pressure = chainbar motor stall); on contact it
 * releases the claw, holds still for SCORE_SETTLE_MS to let the piece settle, and
 * then returns to the STOW (loading) pose. Because the winch is scripted explicitly by the keyframes
 * (up while the bar sweeps through the low band, down at the rest poses), the
 * coordination itself provides the frame clearance -- there is no separate band
 * interlock any more.
 *
 * Driver-facing macros:
 *   - @ref prime()            STOW -> primed (grab + KF0 pose)
 *   - @ref move_lift_up()     hold to advance the raise sequence (from primed)
 *   - @ref move_lift_down()   hold to retreat the raise sequence (manual reverse)
 *   - @ref score()            reverse the sequence w/ pressure detect, then STOW
 *   - @ref return_to_loading() any state -> STOW (release + loading pose)
 *   - @ref open_claw()        manual release, any state (no motion, no mode change)
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
     * Call once at startup with the mechanism physically in the START pose. It seeds
     * the WINCH encoders to their START-pose value (not zero), so winch positions
     * stay measured from the LOADING reference -- the winch is on motor IMEs, which
     * come up at 0 every boot, so the robot MUST be in the START pose when this runs
     * for the winch reference to be right (and it is re-homed anyway by the
     * seek/respool pass once the mechanism first settles in STOW).
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
     * motions must finish before the raise sequence takes over. Only meaningful
     * from the STOW pose; the
     * raise sequence always begins here. After this, hold @ref move_lift_up() to
     * raise.
     */
    void prime();

    /**
     * @brief Advance the coordinated raise sequence this iteration.
     *
     * Call every loop while the driver holds the raise button. The progress
     * scalar ramps toward the top of the sequence while held; when you stop
     * calling it the winch + bar freeze wherever they are and PID-hold. No-op
     * unless the mechanism is in the raise sequence (primed or partway up).
     */
    void move_lift_up();

    /**
     * @brief Retreat the coordinated raise sequence this iteration.
     *
     * The mirror image of @ref move_lift_up(): call every loop while the driver
     * holds the lower button and the progress scalar ramps back toward the primed
     * pose (progress 0) while held, freezing on release. Unlike @ref score() this
     * is a plain manual reverse -- no pressure detection and no auto-return to
     * loading. No-op unless the mechanism is in the raise sequence. If both the
     * raise and lower buttons are held, raising wins.
     */
    void move_lift_down();

    /**
     * @brief Score: reverse the raise sequence with pressure detection.
     *
     * Runs the raise sequence backwards from wherever it currently is, back down
     * toward the primed pose, while watching the chainbar for the piece meeting
     * the tower (motor stall). The sweep does not stop at primed: it carries on
     * SCORE_OVERTRAVEL_DEG further (bar only -- the winch parks at the primed
     * height), so a tower sitting below the primed pose is still reachable. On
     * contact anywhere in that travel it releases the claw, holds the contact pose
     * still for SCORE_SETTLE_MS so the piece can settle on the tower, and only then
     * returns to the STOW (loading) pose. If it runs out the whole overtravel
     * without contact it holds at the primed pose (no release, no settle). No-op
     * unless currently in the raise sequence.
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
     * @brief Manually release the piece: open the claw where it is.
     *
     * A pure claw command -- it does not move the winch, bar or chain and does not
     * change the mode, so it can be used at any point in the primed / raise /
     * score states to drop a piece the macros would otherwise hold on to (a
     * misgrab, a piece that has to come off before scoring, a jam). Nothing in
     * @ref update() re-commands the claw, so it stays open until a macro sets it
     * again: @ref prime() and @ref score() re-clamp, and the startup / return
     * choreographies open it at their end. Valid in every mode; in STOW and during
     * the return it is a no-op because the claw is already open there.
     */
    void open_claw();

    /**
     * @brief Flip the end-effector (claw) orientation used by the raised poses.
     *
     * Toggles between CHAIN_SCORING_DEG (270, the default) and
     * CHAIN_SCORING_ALT_DEG (90). It takes effect immediately everywhere the
     * raised orientation is commanded -- the primed pose and the raise / score
     * sequence driven from it -- so it can be flipped before priming, while
     * primed, or part-way up the sequence. The choice persists until toggled
     * again; the STOW / loading pose always holds CHAIN_LOADING_DEG regardless.
     */
    void toggle_scoring_orientation();

    /**
     * @brief Startup choreography: move from the power-on START pose to STOW.
     *
     * The bar can't swing straight from START to the loading angle without
     * fouling the structure, so this runs a five-step sequence (one step per
     * @ref update()):
     *   1. rotate the bar CHAINBAR_START_CLEARANCE_DEG clear of the start pose,
     *   2. raise the cascade WINCH_START_CLEARANCE_RAISE for swing room,
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
    /// Whether the score macro is currently reversing onto the tower.
    bool is_scoring() const;

    // ========================================================================
    //  Tunable geometry / thresholds -- VERIFY ON THE ROBOT.
    // ========================================================================

    // -- Winch (motor degrees) --
    /// Fully-down / flat height (the tare reference).
    static constexpr double WINCH_DOWN_POS = 0.0;
    /// Winch height held in the power-on START pose. PLACEHOLDER.
    static constexpr double WINCH_START_POS = 0.0;
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
    /// Clearance height when lift moves near bottom of it's arc
    static constexpr double WINCH_CLEARANCE_POS = 600.0;
    /// How far to raise the cascade during startup for bar swing room. PLACEHOLDER.
    static constexpr double WINCH_START_CLEARANCE_RAISE = WINCH_CLEARANCE_POS;
    /// Lift height raised for clearance while the bar sweeps loading<->primed the
    /// long way during prime(). Defaults to the startup clearance height; raise it
    /// if the long bar sweep needs more room. PLACEHOLDER.
    static constexpr double WINCH_PRIME_CLEARANCE_POS = WINCH_CLEARANCE_POS;
    /// Height tolerance for "winch reached target" / "winch is down".
    static constexpr double WINCH_POS_TOL = 40.0;

    // -- Winch respool (take up string slack at the bottom) --
    // After the lift bottoms out in STOW the string can go slack. Once settled at
    // the bottom, spin gently in the raise direction until the motor stalls
    // (string taut), then re-zero here and resume the position hold. Runs once per
    // descent; re-arms whenever the lift is next raised. Only ever runs in STOW so
    // it can never re-zero the encoder mid-sequence.
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
    /// the nominal bottom (motor deg), so a fault can't pay out a birds-nest.
    static constexpr double WINCH_SEEK_MAX_TRAVEL = 250.0;

    // -- Chainbar (output degrees, read directly from the axle rotation sensor) --
    /// Set true if the rotation sensor counts DOWN when positive chainbar voltage
    /// moves the bar, so the sensed angle still agrees with "positive voltage
    /// increases the angle"
    static constexpr bool CHAINBAR_ROTATION_REVERSED = false;
    /// Bar angle in the power-on START pose.
    static constexpr double CHAINBAR_START_DEG = -25.0;
    /// How far to rotate the bar clear of the start pose before raising the
    /// cascade, during the START->STOW startup choreography..
    static constexpr double CHAINBAR_START_CLEARANCE_DEG = -30.0;
    /// Bar angle in the STOW (loading) pose (the tare reference).
    static constexpr double CHAINBAR_LOADING_DEG = 3.0;
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

    // -- Coordinated raise/score sequence (progress-parameterized) --
    /// Number of keyframes in the raise sequence; progress runs [0, N-1].
    static constexpr int SEQ_KEYFRAMES = 4;
    /// Winch height (motor deg) at each keyframe: flat, full, flat, full.
    static constexpr double SEQ_WINCH_DEG[SEQ_KEYFRAMES] = {
        WINCH_DOWN_POS, WINCH_FULL_POS, WINCH_DOWN_POS, WINCH_FULL_POS};
    /// Chainbar angle (output deg) at each keyframe. These are the far-side
    /// (negative) equivalents of the poses {80, 95, 155, 130} -- each is that
    /// pose minus 360, so they are the SAME physical orientations but the bar
    /// reaches the primed pose by rotating the LONG way round (negative, "the
    /// other way") from the loading pose at 0, clearing the obstacle on the short
    /// path. Only the loading->primed approach direction changes; the raise/score
    /// sweep between keyframes is unchanged. Requires the widened +/-360 limits.
    static constexpr double SEQ_BAR_DEG[SEQ_KEYFRAMES] = {-305.0, -290.0, -230.0, -235.0};
    /// Progress units advanced per second while the raise button is held (and
    /// reversed per second while scoring). 1.0 = one keyframe segment per second,
    /// so the full three-segment sweep takes ~3 s. PLACEHOLDER -- tune for feel.
    static constexpr double SEQ_RATE_PER_S = 1.0;

    // -- Chainbar approach to the loading pose (slew-limited, no overshoot) --
    // The loading pose (0 deg) sits right next to a nogo object, so the bar must
    // NOT overshoot it. Stepping the target to 0 makes the bar arrive at ~450
    // deg/s and clip the object; instead the slewed steps ramp the target toward 0,
    // cruising at MAX_DPS and tapering the rate over the last DECEL_DEG so the bar
    // decelerates and arrives at ~0 speed.
    // WHO USES THIS: only the final BAR_SETTLE step, shared by the startup and
    // return choreographies -- the one move that has to land ON the pose. The long
    // sweeps toward loading (@ref bar_sweeping: the return's RAISE_SWEEP and the
    // startup's BAR_LOADING) do NOT -- they step their target and run
    // feed-forward-free, because they only have to carry the bar round to the
    // loading side, not arrive anywhere accurately.
    /// Cruise rate of the bar target toward loading (output deg/s). This is the
    /// commanded sweep speed -- with the velocity feed-forward the bar actually
    /// tracks it (before, it crawled at ~100 dps regardless). Raise for a faster
    /// sweep, lower if the bar can't keep up / overshoots. TUNE on robot.
    static constexpr double CHAINBAR_APPROACH_MAX_DPS = 150.0;
    /// Distance from loading (deg) over which the approach rate tapers to a stop.
    /// This is the deceleration zone -- widen it for a gentler slow-down into the
    /// pose (from the higher cruise speed), narrow it for a later, harder stop.
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
    static constexpr double CHAIN_START_DEG = 100.0;
    /// Orientation the end effector holds while loading / stowed.
    static constexpr double CHAIN_LOADING_DEG = 29.0;
    /// Orientation the end effector is tucked to during prime()'s climb out of the
    /// loading pose -- while the winch raises for clearance and through the first
    /// part of the bar sweep, up to the point the EE flips to the raised
    /// orientation. Held instead of CHAIN_LOADING_DEG for the whole pre-flip
    /// stretch, so the EE makes one move out of loading rather than tucking for the
    /// raise and swinging back to 29 for the sweep.
    static constexpr double CHAIN_PRIME_RAISE_DEG = 0.0;
    /// Orientation the end effector holds once raised (primed / sequence / scoring).
    /// This is the DEFAULT; @ref toggle_scoring_orientation() swaps it for the
    /// alternate below at run time.
    static constexpr double CHAIN_SCORING_DEG = 270.0;
    /// Alternate raised orientation: the same claw axis flipped half a turn, so a
    /// piece can be presented from the other side without re-priming.
    static constexpr double CHAIN_SCORING_ALT_DEG = 450.0;
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
    /// The same cap for the PRIME sweep (SWEEP_LOWER), loading -> primed. That sweep
    /// keeps the feed-forward law (unlike the other two: at the primed pose gravity
    /// pulls the bar further negative, so the gravity term is load-bearing there and
    /// without it the bar settles several degrees past the target and the step never
    /// completes) -- but it was running railed at 12000 mV for the whole 308 deg,
    /// reaching 516 deg/s on the downhill half and ringing for ~150 ms on arrival
    /// (log 07262026-2, 07.9-08.2). This is the knob for that run-up. Also applied
    /// only once the bar is moving, for the same breakaway reason as the startup cap.
    /// TUNE: lower for a gentler prime, raise if it feels sluggish -- this is a
    /// driver-facing macro, so it is a feel trade, not a safety one.
    static constexpr double CHAINBAR_PRIME_SWEEP_MAX_MV = 6000.0;
    /// Bar angle past which the prime cap above starts applying -- the top of the
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

    // -- Soft rotation limits (prevent wire/cable overwrap) --
    // Both the bar and the chain rotator are fixed to roughly +/-200 deg of their
    // start (loading) pose. Targets are clamped inside these bounds and the motor
    // is never driven further past a limit (positive voltage assumed to increase
    // the measured angle). Kept a hair inside 200.
    static constexpr double CHAINBAR_MIN_DEG = -360.0;  ///< output degrees
    /// Upper bar limit = the loading pose / start of the nogo zone. The bar
    /// operates in [primed, loading] = [-305, 0], and there is an object just
    /// past 0, so the motor is never driven above it (apply_soft_limit zeros any
    /// positive command at/above this angle). Braking (negative) commands are
    /// still allowed here, so the PID can pull an overshoot back.
    static constexpr double CHAINBAR_MAX_DEG = 3.0;
    static constexpr double CHAIN_MIN_DEG = -110.0;     ///< motor degrees (1:1 EE)
    static constexpr double CHAIN_MAX_DEG = 500.0;

    // -- Claw solenoid polarity --
    static constexpr bool CLAW_CLOSED_STATE = true;   ///< clamped
    static constexpr bool CLAW_OPEN_STATE = false;    ///< released
    /// Claw state held in the power-on START pose. PLACEHOLDER.
    static constexpr bool CLAW_START_STATE = CLAW_OPEN_STATE;

    // -- Score resistance (pressure) detection on the chainbar --
    // The trigger is the bar's VELOCITY COLLAPSE: on contact it drops from the sweep
    // speed to ~0 within a tick or two, which the windowed rate sees in about one
    // window (~50 ms). The position error is only a confirmation that the PID is
    // really pushing. It used to be the other way round -- the error had to build
    // past 8 deg, which at the 25 deg/s sweep rate meant ~320 ms of the bar leaning
    // on the tower (with the chainbar integral, kI=60, winding up into it) before
    // the macro reacted.
    /// Below this |bar velocity| (output deg/s, the windowed slope from update(), not
    /// the aliased raw sensor velocity) counts as not moving. Must stay under the
    /// slowest sweep segment (15 deg/s at SEQ_RATE_PER_S=1); the lower it is the more
    /// completely the bar has to be stopped, which is safer against false trips but
    /// costs detection time on a soft/compliant contact.
    static constexpr double SCORE_STALL_DPS = 7.0;
    /// Bar position error (output deg) that must be standing for a stall to count as
    /// real resistance rather than a profile that simply is not commanding motion.
    /// A floor, not a delay: keep it just above the tracking error of a healthy
    /// sweep. Raise it if a normal score trips the detector early.
    static constexpr double SCORE_STALL_ERR_DEG = 2.5;
    /// Consecutive ticks both conditions must hold before contact is declared, so a
    /// single stick-slip dip in the rate cannot fire it. 3 ticks = 30 ms.
    static constexpr int SCORE_STALL_TICKS = 3;
    /// Grace time after a score starts before stall detection is trusted -- the bar
    /// is accelerating out of a PID hold there, which looks exactly like a stall
    /// (stopped, error building). Long enough to cover the breakaway, short enough
    /// that a score started from the primed pose still leaves most of its 400 ms of
    /// overtravel available for detection.
    static constexpr uint32_t SCORE_STALL_GRACE_MS = 250;
    /// Settle pause after a detected score: the claw opens the instant contact is
    /// detected, then the whole mechanism holds the contact pose for this long
    /// before the return sequence starts, so the piece can drop clear and settle on
    /// the tower instead of being dragged off by the bar sweeping away underneath
    /// it. Non-blocking -- update() keeps running (and the drivetrain keeps driving
    /// in autonomous) throughout.
    static constexpr uint32_t SCORE_SETTLE_MS = 500;

    // -- Score overtravel (push past the primed pose) --
    // The score sweep does not stop at the primed pose: a tower can sit a little
    // below it, so once progress reaches 0 the bar keeps travelling in the SAME
    // direction (further negative -- KF0 is the far-negative end of the sweep, see
    // SEQ_BAR_DEG) while the winch parks at the primed height. Pressure detection
    // runs throughout, so contact anywhere in here ends the score normally.
    /// How far past the primed bar angle the score macro may keep pushing (deg).
    /// Must stay comfortably above SCORE_STALL_ERR_DEG, or the bar can't build
    /// enough standing error inside the overtravel for a block to read as contact.
    static constexpr double SCORE_OVERTRAVEL_DEG = 10.0;
    /// Bar angle at the far end of the overtravel (kept inside CHAINBAR_MIN_DEG).
    static constexpr double SCORE_OVERTRAVEL_BAR_DEG = SEQ_BAR_DEG[0] - SCORE_OVERTRAVEL_DEG;
    /// Rate (output deg/s) the target travels through the overtravel. Matches the
    /// bar speed of the KF0<->KF1 segment (constexpr |diff| * SEQ_RATE_PER_S), so
    /// the push past primed is a seamless continuation of the score sweep and
    /// re-tunes itself if the keyframes or the sequence rate change.
    static constexpr double SCORE_OVERTRAVEL_DPS =
        (SEQ_BAR_DEG[0] < SEQ_BAR_DEG[1] ? SEQ_BAR_DEG[1] - SEQ_BAR_DEG[0]
                                         : SEQ_BAR_DEG[0] - SEQ_BAR_DEG[1]) * SEQ_RATE_PER_S;

private:
    /// What the mechanism is doing right now.
    /// STARTUP is the power-on START->STOW choreography; the mechanism leaves it
    /// once and never returns during a match.
    enum class Mode {
        STARTUP,   ///< Power-on START->STOW startup choreography
        STOW,      ///< Resting at the loading pose (winch flat-ish, claw open)
        PRIMING,   ///< Running the loading->primed clearance choreography
        SEQUENCE,  ///< In the coordinated raise sequence (position = raise_progress_)
        SCORING,   ///< Reversing the sequence onto the tower until resistance
        SCORE_SETTLE, ///< Contact made, claw released: holding still for
                      ///< SCORE_SETTLE_MS to let the piece settle, then returns
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
    /// the lift, then sweep the bar the long way to primed -- lowering the lift
    /// again as soon as the bar is clear of the low-clearance zone, so the descent
    /// overlaps the back half of the sweep instead of following it.
    enum class PrimeStep {
        RAISE,       ///< Raise the lift to the prime clearance height (bar held at loading)
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
    bool advancing_;               ///< move_lift_up() called this cycle (consumed by update)
    bool retreating_;              ///< move_lift_down() called this cycle (consumed by update)
    double winch_manual_target_;   ///< Resolved winch height target (motor deg)
    double bar_target_deg_;        ///< Resolved chainbar goal angle (output deg)
    double bar_target_vel_dps_;    ///< Profile velocity for the bar this tick (0 = not profiled)
    double chain_target_deg_;      ///< Chain hold angle (motor deg)
    double chain_scoring_deg_;     ///< Raised-pose EE orientation currently selected
                                   ///< (CHAIN_SCORING_DEG or CHAIN_SCORING_ALT_DEG)
    uint32_t score_start_ms_;      ///< millis() when the current score began
    double score_overtravel_deg_;  ///< How far the score sweep has pushed the bar past
                                   ///< the primed angle, in [0, SCORE_OVERTRAVEL_DEG]
    uint32_t score_settle_start_ms_;  ///< millis() the post-score settle pause began
    int score_stall_ticks_;        ///< Consecutive ticks the stall conditions have held
                                   ///< (debounce for SCORE_STALL_TICKS)
    uint32_t last_update_ms_;      ///< millis() of previous update (for dt)
    WinchMode winch_mode_;         ///< Winch control mode (HOLD / SEEK / RESPOOL)
    bool winch_respooled_;         ///< Homed (sought + respooled) since last raise
    uint32_t respool_start_ms_;    ///< millis() when the current respool began
    uint32_t seek_start_ms_;       ///< millis() when the current home-seek began
    double bar_angle_hist_[CHAINBAR_RATE_WINDOW];  ///< recent bar angles (idx 0 newest) for the windowed rate
    double chain_angle_hist_[CHAIN_RATE_WINDOW];   ///< recent EE angles (idx 0 newest) for the windowed rate

    // Helpers
    double motor_group_position() const;
    /// Bar angular velocity in output deg/s, from the axle rotation sensor.
    double bar_velocity_dps() const;
    /// Interpolate the winch height + bar angle for a given sequence progress.
    static void seq_targets(double progress, double& winch_out, double& bar_out);
    /// Ramp @p current toward @p goal one tick, capped at CHAINBAR_APPROACH_MAX_DPS
    /// and tapering over CHAINBAR_APPROACH_DECEL_DEG so the bar arrives with ~0
    /// speed (no overshoot into the nogo object beside the loading pose). The
    /// returned target is also held within CHAINBAR_MAX_LEAD_DEG of @p actual_bar
    /// so it never sprints ahead of a lagging bar. @p out_vel_dps receives the
    /// signed profile velocity (deg/s) this tick, for the velocity feed-forward.
    static double slew_bar_toward(double current, double goal, double actual_bar,
                                  double& out_vel_dps);
    /// True once the bar has swept past the low-clearance zone, i.e. it no longer
    /// needs the lift held up and the two motions may overlap.
    static bool bar_clear_of_low_zone(double bar);
    /// True once the score reverse has run past the grace time and is blocked.
    /// Contact test for the score macro: true once the bar has been stopped
    /// (@p bar_vel_dps, the windowed rate) with a standing position error for
    /// SCORE_STALL_TICKS ticks in a row, past the start-up grace. Not const -- it
    /// owns the debounce counter.
    bool score_stalled(uint32_t now, double bar_vel_dps);
    /// Zero out a command that would drive @p angle further past a soft limit
    /// (positive voltage assumed to increase the angle).
    static double apply_soft_limit(double output_mv, double angle,
                                   double min_deg, double max_deg);
    /// Begin a return to the STOW pose (opens the claw, drops the winch).
    void begin_return_to_loading();
    /// Advance the START->STOW sequence one tick.
    void advance_start_sequence(double bar, double winch);
    /// Advance the loading->primed clearance choreography one tick.
    void advance_prime_sequence(double bar, double winch);
    /// Advance the ->loading clearance choreography one tick.
    void advance_return_sequence(double bar, double winch);
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
    bool settle_bar_arrived(double bar) const;

    static const char* mode_name(Mode m);
    static const char* start_step_name(StartStep s);
    static const char* prime_step_name(PrimeStep s);
    static const char* return_step_name(ReturnStep s);
    static const char* winch_mode_name(WinchMode m);
    /// Emit one CSV-style debug line covering every command/target/actual/output.
    void log_debug_state(double winch_target, double winch, double winch_mv,
                         double winch_vel_rpm,
                         double bar, double bar_mv, double bar_vel_dps,
                         double bar_gravity_ff, double bar_static_ff,
                         double ee, double chain_mv, double chain_vel_dps) const;
};

#endif // LIFT_HPP
