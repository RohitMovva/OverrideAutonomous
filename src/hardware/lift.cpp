#include "hardware/lift.hpp"
#include "config.hpp"
#include "pros/llemu.hpp"
#include "utilities/logger.hpp"
#include <algorithm>
#include <cmath>

// Motor RPM -> deg/s.
static constexpr double RPM_TO_DPS = 6.0;

Lift::Lift(pros::MotorGroup& winch_motors,
           pros::Motor& chainbar_motor,
           pros::Rotation& chainbar_rotation,
           pros::Motor& chain_motor,
           pros::Rotation& chain_rotation,
           EnhancedDigitalOut& claw,
           PIDController winch_pid,
           PIDController chainbar_pid,
           PIDController chain_pid)
    : winch_motors_(winch_motors),
      chainbar_motor_(chainbar_motor),
      chainbar_rotation_(chainbar_rotation),
      chain_motor_(chain_motor),
      chain_rotation_(chain_rotation),
      claw_(claw),
      winch_pid_(winch_pid),
      chainbar_pid_(chainbar_pid),
      chain_pid_(chain_pid),
      mode_(Mode::STARTUP),
      start_step_(StartStep::IDLE),
      prime_step_(PrimeStep::RAISE),
      return_step_(ReturnStep::RAISE_SWEEP),
      startup_complete_(false),
      chainbar_coasting_(false),
      bar_coast_done_(false),
      alt_return_gate_(false),
      return_hold_winch_(WINCH_START_POS),
      raise_progress_(0.0),
      advancing_(false),
      retreating_(false),
      winch_manual_target_(WINCH_START_POS),
      bar_target_deg_(CHAINBAR_START_DEG),
      bar_target_vel_dps_(0.0),
      chain_target_deg_(CHAIN_START_DEG),
      chain_scoring_deg_(CHAIN_SCORING_DEG),
      score_start_ms_(0),
      score_overtravel_deg_(0.0),
      score_settle_start_ms_(0),
      score_stall_ticks_(0),
      last_update_ms_(0),
      winch_mode_(WinchMode::HOLD),
      winch_respooled_(false),
      respool_start_ms_(0),
      seek_start_ms_(0),
      bar_angle_hist_{},
      chain_angle_hist_{} {}

void Lift::initialize() {
    // --- Diagnostic probes (temporary) --------------------------------------
    // The async logger drops its un-flushed tail on a hard hang, so a plain log
    // line can't tell us which smart-port call froze. probe() logs then sleeps
    // ~30 ms so the background task flushes THIS line to SD before the next
    // motor op runs. The last "LIFTINIT:" line in a frozen log therefore names
    // the call that hung (the culprit is the operation AFTER the last probe).
    // The lift motors (winch 16/17, chainbar 10, chain 13) share ports with
    // distance sensors and are physically off the robot mid-rebuild, so these
    // ops are the leading hang suspect. Remove this block once diagnosed.
    Logger* dbg = Logger::getInstance();
    auto probe = [&](const char* msg) {
        dbg->log("LIFTINIT: %s", msg);
        pros::delay(30);
    };
    probe("enter; flushing backlog");
    pros::delay(80);  // let any earlier init backlog drain before we probe

    // The WINCH still has to be seeded here: it is read from the motor IMEs, which
    // come up at 0 every boot and drift besides (hence the seek/respool homing pass
    // in update()). Seed it to its START-pose value rather than taring to 0, since
    // all poses are measured from the LOADING reference.
    //
    // The BAR and the EE are not seeded at all any more. Both are rotation sensors on
    // their own axle, and those hold their reading across a power cycle, so writing
    // set_position() here would throw away a good absolute reference and replace it
    // with an assumption about where the mechanism was parked. They only get their
    // orientation (set_reversed) configured below. This does mean the sensor zero has
    // to AGREE with the pose convention (LOADING = 0) -- that is now a one-time
    // mechanical/commissioning step, not something the code re-establishes each boot.
    winch_motors_.set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);
    probe("winch set_encoder_units_all ok");
    winch_motors_.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    probe("winch set_brake_mode_all ok");
    winch_motors_.set_zero_position_all(WINCH_START_POS);
    probe("winch set_zero_position_all ok");

    chainbar_motor_.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    probe("chainbar set_brake_mode ok");
    // The bar angle is sensed by a rotation sensor on the axle (1:1), not the motor
    // encoder. Orientation only -- its position is left exactly as the sensor
    // reports it (see above).
    chainbar_rotation_.set_reversed(CHAINBAR_ROTATION_REVERSED);
    probe("chainbar rotation set_reversed ok");

    chain_motor_.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    probe("chain set_brake_mode ok");
    // Like the bar, the EE angle is sensed by a rotation sensor on its own axle
    // (1:1), not the chain motor's encoder -- so nothing here touches the motor's
    // encoder units or zero, and its position is left as the sensor reports it.
    chain_rotation_.set_reversed(CHAIN_ROTATION_REVERSED);
    probe("chain rotation set_reversed ok");

    // Start in the dedicated power-on START pose (STARTUP mode). Positions are the
    // START_* constants (placeholders -- set them to the real measured start pose).
    mode_ = Mode::STARTUP;
    start_step_ = StartStep::IDLE;
    prime_step_ = PrimeStep::RAISE;
    return_step_ = ReturnStep::RAISE_SWEEP;
    startup_complete_ = false;  // re-zero cap arms when the choreography finishes
    chainbar_coasting_ = false;
    bar_coast_done_ = false;
    alt_return_gate_ = false;
    return_hold_winch_ = WINCH_START_POS;
    winch_mode_ = WinchMode::HOLD;
    winch_respooled_ = false;
    raise_progress_ = 0.0;
    score_overtravel_deg_ = 0.0;
    advancing_ = false;
    retreating_ = false;
    winch_manual_target_ = WINCH_START_POS;
    bar_target_deg_ = CHAINBAR_START_DEG;
    chain_target_deg_ = CHAIN_START_DEG;
    chain_scoring_deg_ = CHAIN_SCORING_DEG;
    claw_.set_state(CLAW_START_STATE);
    probe("claw set_state ok");
    winch_pid_.reset();
    chainbar_pid_.reset();
    chain_pid_.reset();
    double bar0 = get_bar_angle();  // seed the windowed-rate history to the start pose
    for (int i = 0; i < CHAINBAR_RATE_WINDOW; ++i) {
        bar_angle_hist_[i] = bar0;
    }
    double ee0 = get_ee_angle();    // same for the chain (else tick 1 reads a huge rate)
    for (int i = 0; i < CHAIN_RATE_WINDOW; ++i) {
        chain_angle_hist_[i] = ee0;
    }
    probe("pids reset; state seeded");

    // Kick off the START->STOW startup choreography automatically. It runs as
    // update() is called each loop and lands the mechanism in the STOW pose.
    start_to_loading();
    probe("start_to_loading returned; initialize DONE");
}

// ---- Readouts --------------------------------------------------------------

double Lift::motor_group_position() const {
    std::vector<double> positions = winch_motors_.get_position_all();
    if (positions.empty()) {
        return 0.0;
    }
    double sum = 0.0;
    for (double p : positions) {
        sum += p;
    }
    return sum / static_cast<double>(positions.size());
}

double Lift::get_winch_position() const {
    return motor_group_position();
}

double Lift::get_bar_angle() const {
    // Rotation sensor on the axle, 1:1. get_position() is in centidegrees.
    return chainbar_rotation_.get_position() / 100.0;
}

double Lift::bar_velocity_dps() const {
    // Rotation sensor velocity is centidegrees/sec; the bar is 1:1 to the axle.
    return chainbar_rotation_.get_velocity() / 100.0;
}

double Lift::get_ee_angle() const {
    // Rotation sensor on the EE axle, 1:1. get_position() is in centidegrees, and it
    // accumulates past +/-360 (which CHAIN_SCORING_ALT_DEG = 450 relies on).
    return chain_rotation_.get_position() / 100.0;
}

double Lift::get_progress() const {
    return raise_progress_;
}

bool Lift::is_scoring() const {
    // True for the whole score macro, including the post-contact settle pause --
    // the piece is not clear of the mechanism until the return actually starts.
    return mode_ == Mode::SCORING || mode_ == Mode::SCORE_SETTLE;
}

// ---- Sequence interpolation ------------------------------------------------

void Lift::seq_targets(double progress, double& winch_out, double& bar_out) {
    // Piecewise-linear interpolation of the keyframes. progress in [0, N-1]; the
    // integer part selects the segment, the fraction blends within it.
    double p = std::clamp(progress, 0.0, static_cast<double>(SEQ_KEYFRAMES - 1));
    int i = static_cast<int>(std::floor(p));
    if (i > SEQ_KEYFRAMES - 2) {
        i = SEQ_KEYFRAMES - 2;  // clamp so i+1 stays in range at the top end
    }
    double f = p - static_cast<double>(i);
    winch_out = SEQ_WINCH_DEG[i] + (SEQ_WINCH_DEG[i + 1] - SEQ_WINCH_DEG[i]) * f;
    bar_out = SEQ_BAR_DEG[i] + (SEQ_BAR_DEG[i + 1] - SEQ_BAR_DEG[i]) * f;
}

double Lift::slew_bar_toward(double current, double goal, double actual_bar,
                             double& out_vel_dps) {
    double dt = Config::DT / 1000.0;
    double remaining = goal - current;
    double dist = std::abs(remaining);
    // Taper the rate over the decel zone so the bar decelerates into the goal;
    // full rate outside it. The floor keeps the tail from asymptoting.
    double frac = 1.0;
    if (CHAINBAR_APPROACH_DECEL_DEG > 0.0) {
        frac = std::clamp(dist / CHAINBAR_APPROACH_DECEL_DEG,
                          CHAINBAR_APPROACH_MIN_FRAC, 1.0);
    }
    double rate = CHAINBAR_APPROACH_MAX_DPS * frac;  // profile speed magnitude (deg/s)
    double step = rate * dt;
    // Desired velocity is the INTENDED profile rate (before the lead clamp below),
    // signed toward the goal, so the velocity feed-forward in update() commands the
    // full cruise/decel profile even on a tick where the lead clamp holds the target
    // back. Zero on the final snap tick (we're arriving).
    out_vel_dps = (dist <= step) ? 0.0 : std::copysign(rate, remaining);
    double next = (dist <= step) ? goal  // close enough this tick: snap to the goal
                                 : current + std::copysign(step, remaining);
    // Never let the target run more than CHAINBAR_MAX_LEAD_DEG ahead of where the
    // bar actually is. With the velocity feed-forward the bar tracks the profile so
    // this rarely binds; it stays as a backstop against a huge standing error
    // charging the PID / integral into an overshoot if the bar falls behind.
    // It also does real work either side of the return coast: the target is dragged
    // along with the free-swinging bar, so when the PID re-engages at the catch
    // angle it starts from a small error (and, if the bar has outrun the profile, a
    // braking one) rather than a step.
    return std::clamp(next, actual_bar - CHAINBAR_MAX_LEAD_DEG,
                      actual_bar + CHAINBAR_MAX_LEAD_DEG);
}

// ---- Driver / macro entry points -------------------------------------------

void Lift::prime() {
    // The raise sequence always starts from the stow pose.
    if (mode_ != Mode::STOW) {
        Logger::getInstance()->log("LIFTCMD prime IGNORED (mode=%s, not STOW)",
                                   mode_name(mode_));
        return;
    }
    Logger::getInstance()->log(
        "LIFTCMD prime -> clearance raise, then sweep to KF0 (bar %.1f) w/ lower from %.1f",
        SEQ_BAR_DEG[0], CHAINBAR_LOW_ZONE_DEG);
    claw_.set_state(CLAW_CLOSED_STATE);  // grab the piece before raising
    raise_progress_ = 0.0;
    // Run the clearance choreography (raise -> sweep -> lower) rather than moving
    // straight to the primed pose, so the lift is up while the bar sweeps the long
    // way from loading and clears the obstacle on the short path.
    mode_ = Mode::PRIMING;
    prime_step_ = PrimeStep::RAISE;
    chainbar_pid_.reset();
    winch_pid_.reset();
}

void Lift::move_lift_up() {
    // Consumed in update(); only actually advances while in the raise sequence.
    advancing_ = true;
}

void Lift::move_lift_down() {
    // Consumed in update(); only actually retreats while in the raise sequence.
    // A plain manual reverse of the sequence -- no pressure detect / auto-return.
    retreating_ = true;
}

void Lift::score() {
    // Only meaningful once in the raise sequence: reverse onto the tower.
    if (mode_ != Mode::SEQUENCE) {
        Logger::getInstance()->log("LIFTCMD score IGNORED (mode=%s, not in sequence)",
                                   mode_name(mode_));
        return;
    }
    Logger::getInstance()->log(
        "LIFTCMD score -> reverse from p=%.2f w/ pressure detect (then %.1f deg overtravel)",
        raise_progress_, SCORE_OVERTRAVEL_DEG);
    claw_.set_state(CLAW_CLOSED_STATE);  // stay clamped until we hit the tower
    mode_ = Mode::SCORING;
    score_start_ms_ = pros::millis();
    score_overtravel_deg_ = 0.0;  // re-arm the push past primed for this score
    score_stall_ticks_ = 0;       // fresh debounce for this score's contact detect
    chainbar_pid_.reset();
}

void Lift::open_claw() {
    // Manual release, deliberately ungated: it only touches the solenoid, so there
    // is no pose it is unsafe from, and the primed / raise / score states -- where
    // the macros otherwise keep the claw clamped -- are exactly where it is needed.
    Logger::getInstance()->log("LIFTCMD open_claw (mode=%s p=%.2f bar=%.1f claw was %d)",
                               mode_name(mode_), raise_progress_, get_bar_angle(),
                               claw_.get_state() ? 1 : 0);
    claw_.set_state(CLAW_OPEN_STATE);
}

void Lift::toggle_scoring_orientation() {
    // Flip which raised orientation the end effector holds. Only the selection is
    // stored here: every mode that commands the raised pose reads
    // chain_scoring_deg_ each tick, so an in-flight sequence turns immediately and
    // the choice carries into the next prime.
    chain_scoring_deg_ = (chain_scoring_deg_ == CHAIN_SCORING_DEG) ? CHAIN_SCORING_ALT_DEG
                                                                  : CHAIN_SCORING_DEG;
    Logger::getInstance()->log("LIFTCMD flip scoring orientation -> %.0f (mode=%s ee=%.1f)",
                               chain_scoring_deg_, mode_name(mode_), get_ee_angle());
}

void Lift::begin_return_to_loading() {
    Logger::getInstance()->log(
        "LIFTCMD return_to_loading -> raise + sweep to %.1f (gated at %.1f), lower, "
        "then settle bar to %.1f",
        CHAINBAR_LOWER_CLEAR_DEG, CHAINBAR_LOW_ZONE_DEG, CHAINBAR_LOADING_DEG);
    start_step_ = StartStep::IDLE;  // supersede any running startup sequence
    raise_progress_ = 0.0;
    claw_.set_state(CLAW_OPEN_STATE);  // release the piece before moving
    // Also includes clearance to make sure chain bar doesn't clip intake poly
    mode_ = Mode::RETURNING;
    return_step_ = ReturnStep::RAISE_SWEEP;
    bar_target_deg_ = get_bar_angle();
    // Height the cascade may not descend below while the alternate-orientation hold
    // is active (see alt_return_hold_active): wherever the lift happens to be when
    // the return starts -- which after a score is somewhere up the raise sequence.
    return_hold_winch_ = get_winch_position();
    bar_coast_done_ = false;  // re-arm the coast-through for this return
    chainbar_pid_.reset();
    winch_pid_.reset();
}

void Lift::return_to_loading() {
    begin_return_to_loading();
}

void Lift::start_to_loading() {
    // Used to avoid the intake poly
    if (mode_ != Mode::STARTUP) {
        Logger::getInstance()->log("LIFTCMD start_to_loading IGNORED (mode=%s, not STARTUP)",
                                   mode_name(mode_));
        return;
    }
    Logger::getInstance()->log("LIFTCMD start_to_loading -> begin startup choreography");
    start_step_ = StartStep::BAR_CLEAR;
    bar_coast_done_ = false;  // re-arm the coast-through for this sweep
    chainbar_pid_.reset();
}

void Lift::advance_start_sequence(double bar, double winch) {
    StartStep step_before = start_step_;
    switch (start_step_) {
        case StartStep::BAR_CLEAR: {
            // Back the bar off the start pose so it clears the intake poly.
            bar_target_deg_ = CHAINBAR_START_DEG + CHAINBAR_START_CLEARANCE_DEG;
            if (std::abs(bar - bar_target_deg_) < CHAINBAR_POS_TOL) {
                start_step_ = StartStep::RAISE;
            }
            break;
        }
        case StartStep::RAISE: {
            // Raise the cascade to give the bar room to swing down to loading.
            winch_manual_target_ = WINCH_START_POS + WINCH_START_CLEARANCE_RAISE;
            if (winch >= winch_manual_target_ - WINCH_POS_TOL) {
                start_step_ = StartStep::BAR_LOADING;
                chainbar_pid_.reset();
            }
            break;
        }
        case StartStep::BAR_LOADING: {
            // Swing the bar (and end effector) to just PAST loading
            // (CHAINBAR_LOWER_CLEAR_DEG) so it is clear of the cascade for the
            // descent in the next step; the cascade holds up at the raised height
            // set in the previous step.
            //
            // This is the return's RAISE_SWEEP over a shorter arc, so it is driven
            // the same way (@ref bar_sweeping): a STEP target, coasting brake, and
            // the FF-free capped law in update(), handing over to the feed-forward
            // branch at CHAINBAR_COAST_CATCH_DEG to close the last few degrees onto
            // the pose. It used to slew the target with the velocity feed-forward
            // instead, which fought its own gravity term and stick-slipped through
            // the whole move (log 07262026-2, 02.04-02.69: stuck at 7031 mV, lurch
            // to 312 deg/s, brake, reverse, stall again).
            bar_target_deg_ = CHAINBAR_LOWER_CLEAR_DEG;
            chain_target_deg_ = CHAIN_PRIME_RAISE_DEG;
            if (std::abs(bar - CHAINBAR_LOWER_CLEAR_DEG) < CHAINBAR_POS_TOL) {
                start_step_ = StartStep::LOWER;
            }
            break;
        }
        case StartStep::LOWER: {
            // Bring the cascade to the loading height, bar still held clear.
            winch_manual_target_ = WINCH_LOADING_POS;
            bar_target_deg_ = CHAINBAR_LOWER_CLEAR_DEG;
            chain_target_deg_ = CHAIN_PRIME_RAISE_DEG;
            if (std::abs(winch - WINCH_LOADING_POS) < WINCH_POS_TOL) {
                start_step_ = StartStep::BAR_SETTLE;
                chainbar_pid_.reset();
            }
            break;
        }
        case StartStep::BAR_SETTLE: {
            // Cascade is down: bring the bar the last few degrees up onto the
            // loading angle, then settle in STOW -- which is what arms the winch
            // homing/respool pass, so the respool always runs with the bar already
            // parked at loading.
            winch_manual_target_ = WINCH_LOADING_POS;
            bar_target_deg_ = slew_bar_toward(bar_target_deg_, CHAINBAR_LOADING_DEG, bar,
                                              bar_target_vel_dps_);
            chain_target_deg_ = CHAIN_LOADING_DEG;
            if (settle_bar_arrived(bar)) {
                start_step_ = StartStep::IDLE;
                mode_ = Mode::STOW;
                claw_.set_state(CLAW_OPEN_STATE);  // stow pose accepts a piece
                // From here on the winch reference is a real, homed one, so cap how
                // far any single respool re-zero may move it (WINCH_REZERO_MAX_DEG).
                startup_complete_ = true;
            }
            break;
        }
        case StartStep::IDLE:
            break;
    }
    if (start_step_ != step_before) {
        Logger::getInstance()->log("LIFTEVT startup step %s -> %s (bar=%.1f winch=%.1f)",
                                   start_step_name(step_before), start_step_name(start_step_),
                                   bar, winch);
    }
}

void Lift::advance_prime_sequence(double bar, double winch) {
    const double primed_bar = SEQ_BAR_DEG[0];      // primed keyframe bar angle
    const double primed_winch = SEQ_WINCH_DEG[0];  // primed keyframe winch height (flat)
    PrimeStep step_before = prime_step_;
    switch (prime_step_) {
        case PrimeStep::RAISE: {
            // Raise the lift for clearance; hold the bar at loading until the lift
            // is up so the long sweep never starts while the bar is still low. The
            // end effector tucks to CHAIN_PRIME_RAISE_DEG for the climb rather than
            // staying at the loading orientation.
            winch_manual_target_ = WINCH_PRIME_CLEARANCE_POS;
            bar_target_deg_ = CHAINBAR_LOADING_DEG;
            chain_target_deg_ = CHAIN_PRIME_RAISE_DEG;
            if (winch >= WINCH_PRIME_CLEARANCE_POS - WINCH_POS_TOL) {
                prime_step_ = PrimeStep::SWEEP_LOWER;
                chainbar_pid_.reset();
            }
            break;
        }
        case PrimeStep::SWEEP_LOWER: {
            // Sweep the bar the long way to the primed angle, and start the descent
            // to the primed (flat) height the moment the bar is out of the
            // low-clearance zone -- the lift only has to stay up while the bar is
            // swinging through it, so the rest of the sweep and the whole descent
            // overlap instead of running back to back. Hand off to the raise
            // sequence once BOTH have landed.
            winch_manual_target_ = bar_clear_of_low_zone(bar) ? primed_winch
                                                             : WINCH_PRIME_CLEARANCE_POS;
            bar_target_deg_ = primed_bar;
            // Stay tucked at the raise orientation until the bar is far enough round
            // to flip to the raised one -- going back to CHAIN_LOADING_DEG here would
            // swing the EE back to 29 for a moment and then straight on to scoring.
            chain_target_deg_ = CHAIN_PRIME_RAISE_DEG;
            if (bar < CHAIN_FLIP_BAR_DEG){
                chain_target_deg_ = chain_scoring_deg_;
            }
            if (std::abs(bar - primed_bar) < CHAINBAR_POS_TOL &&
                std::abs(winch - primed_winch) < WINCH_POS_TOL) {
                mode_ = Mode::SEQUENCE;
                raise_progress_ = 0.0;
            }
            break;
        }
    }
    if (prime_step_ != step_before || mode_ == Mode::SEQUENCE) {
        Logger::getInstance()->log("LIFTEVT prime step %s -> %s (bar=%.1f winch=%.1f)",
                                   prime_step_name(step_before),
                                   mode_ == Mode::SEQUENCE ? "PRIMED" : prime_step_name(prime_step_),
                                   bar, winch);
    }
}

void Lift::advance_return_sequence(double bar, double winch) {
    ReturnStep step_before = return_step_;
    switch (return_step_) {
        case ReturnStep::RAISE_SWEEP: {
            // Raise the lift into the safe zone and sweep the bar the long way back
            // toward loading at the same time: only the last stretch of the sweep
            // (the low-clearance zone) needs the lift up, so until the lift arrives
            // the sweep goal is capped at the edge of that zone and the bar waits
            // there. If the bar is already inside the zone the cap holds the target
            // where it is rather than dragging it back out.
            // The goal is a STEP target for the whole sweep -- no slew, and the
            // drive runs with no feed-forward either (see update()). The sweep only
            // has to carry the bar round to the loading side; there is no pose to
            // arrive at accurately here (the final BAR_SETTLE step does that, and it
            // is the one that slews), so a ramped profile would buy nothing.
            // The sweep lands just PAST loading (CHAINBAR_LOWER_CLEAR_DEG) so the
            // bar is clear of the cascade for the descent in the next step.
            winch_manual_target_ = WINCH_CLEARANCE_POS;
            if (alt_return_hold_active(bar)) {
                // Alternate orientation, bar not back round to CHAIN_FLIP_BAR_DEG
                // yet: the cascade may still RISE (to the clearance height, as
                // usual) but must not come down on the flipped end effector. After
                // a score the lift is typically above the clearance height, so
                // without this floor the plain WINCH_CLEARANCE_POS target would
                // start lowering it immediately.
                winch_manual_target_ = std::max(WINCH_CLEARANCE_POS, return_hold_winch_);
            }
            bool lift_clear = winch >= WINCH_CLEARANCE_POS - WINCH_POS_TOL;
            double goal = lift_clear ? CHAINBAR_LOWER_CLEAR_DEG
                                     : std::max(bar_target_deg_, CHAINBAR_LOW_ZONE_DEG);
            bar_target_deg_ = goal;
            chain_target_deg_ = chain_scoring_deg_;
            if (bar > CHAIN_FLIP_BAR_DEG){
                chain_target_deg_ = CHAIN_LOADING_DEG;
            }
            if (std::abs(bar - CHAINBAR_LOWER_CLEAR_DEG) < CHAINBAR_POS_TOL) {
                return_step_ = ReturnStep::LOWER;
            }
            break;
        }
        case ReturnStep::LOWER: {
            // Bar is parked just past loading; lower the lift to the loading height
            // with the bar held there.
            winch_manual_target_ = WINCH_LOADING_POS;
            bar_target_deg_ = CHAINBAR_LOWER_CLEAR_DEG;
            chain_target_deg_ = CHAIN_PRIME_RAISE_DEG;
            if (std::abs(winch - WINCH_LOADING_POS) < WINCH_POS_TOL) {
                return_step_ = ReturnStep::BAR_SETTLE;
                chainbar_pid_.reset();
            }
            break;
        }
        case ReturnStep::BAR_SETTLE: {
            // Lift is down: bring the bar the last few degrees up onto the loading
            // angle, then settle in STOW -- which is what arms the winch
            // homing/respool pass, so the respool always runs with the bar already
            // parked at loading.
            winch_manual_target_ = WINCH_LOADING_POS;
            bar_target_deg_ = slew_bar_toward(bar_target_deg_, CHAINBAR_LOADING_DEG, bar,
                                              bar_target_vel_dps_);
            chain_target_deg_ = CHAIN_LOADING_DEG;
            if (settle_bar_arrived(bar)) {
                mode_ = Mode::STOW;
            }
            break;
        }
    }
    if (return_step_ != step_before || mode_ == Mode::STOW) {
        Logger::getInstance()->log("LIFTEVT return step %s -> %s (bar=%.1f winch=%.1f)",
                                   return_step_name(step_before),
                                   mode_ == Mode::STOW ? "STOW" : return_step_name(return_step_),
                                   bar, winch);
    }
}

// ---- Helpers ---------------------------------------------------------------

double Lift::apply_soft_limit(double output_mv, double angle,
                              double min_deg, double max_deg) {
    if (angle >= max_deg && output_mv > 0.0) return 0.0;  // can't rotate further up
    if (angle <= min_deg && output_mv < 0.0) return 0.0;  // can't rotate further down
    return output_mv;
}

bool Lift::bar_clear_of_low_zone(double bar) {
    // The bar sweeps NEGATIVE from loading (0) to primed (-305). The low-clearance
    // zone is the near-loading end of that travel, [CHAINBAR_LOW_ZONE_DEG, 0]: only
    // in there does the bar need the lift held up. Past it in the negative
    // direction the bar is clear, so the winch may move independently of the sweep.
    return bar < CHAINBAR_LOW_ZONE_DEG;
}

bool Lift::bar_sweeping() const {
    // The return's RAISE_SWEEP and the startup's BAR_LOADING are the same move at
    // different lengths: cascade held up at the clearance height, bar driven the
    // long way toward the loading side, nothing to arrive at accurately until the
    // last few degrees. So they share one control law -- step target, no
    // feed-forward, capped, coasting through the middle (see update()).
    return (mode_ == Mode::RETURNING && return_step_ == ReturnStep::RAISE_SWEEP) ||
           (mode_ == Mode::STARTUP && start_step_ == StartStep::BAR_LOADING);
}

bool Lift::bar_coast_active(double bar, double winch) const {
    // The coast window of a bar sweep (@ref bar_sweeping -- the return's RAISE_SWEEP
    // or the startup's BAR_LOADING): past the top of the swing
    // (CHAINBAR_COAST_START_DEG) and not yet at the catch angle
    // (CHAINBAR_COAST_CATCH_DEG). The startup sweep starts inside that window
    // already, so for it the whole move is coast plus the feed-forward finish.
    // In here gravity plus the bar's own momentum do most
    // of the work of carrying it home and the motor is put in COAST brake so a zero
    // command does not fight the swing -- but the PID is NOT cut out any more; it
    // keeps driving the sweep command right through (see update()).
    //
    // Two guards beyond the angle window:
    //  - the lift must already be at its clearance height. A bar in the coast window
    //    is not stopped dead at the edge of the low-clearance zone the way the rest
    //    of the sweep can be (that is what the goal cap in RAISE_SWEEP does -- and
    //    with a COAST brake the momentum carries it past regardless), so opening the
    //    window while the cascade is still coming up would defeat the clearance
    //    interlock. Until the lift arrives the bar stays braked HOLD against the
    //    capped step target and the window opens later.
    //  - bar_coast_done_: once caught, a bounce back past the catch angle must
    //    not drop the bar into a second coast.
    return bar_sweeping() &&
           !bar_coast_done_ &&
           bar >= CHAINBAR_COAST_START_DEG &&
           bar < CHAINBAR_COAST_CATCH_DEG &&
           winch >= WINCH_CLEARANCE_POS - WINCH_POS_TOL;
}

bool Lift::alt_return_hold_active(double bar) const {
    // Only the ALTERNATE raised orientation needs this: coming back from a score in
    // that orientation the end effector is flipped half a turn, so until the bar has
    // swung round to CHAIN_FLIP_BAR_DEG it is neither safe to drive the chain (it
    // would rotate the EE through the structure) nor to bring the cascade down on
    // top of it. The default orientation returns as it always has.
    return mode_ == Mode::RETURNING &&
           chain_scoring_deg_ == CHAIN_SCORING_ALT_DEG &&
           bar < CHAIN_FLIP_BAR_DEG;
}

bool Lift::settle_bar_arrived(double bar) const {
    // The settle move is only (CHAINBAR_LOADING_DEG - CHAINBAR_LOWER_CLEAR_DEG) = 4
    // deg -- already inside CHAINBAR_POS_TOL -- so a position-only check would pass
    // on the very first tick and skip the settle entirely. Also require the SLEWED
    // TARGET to have arrived at loading, so the move is always commanded in full.
    // (Nothing can hold the target back here: the return path slews unclamped, and
    // the startup path's lead clamp only binds past CHAINBAR_MAX_LEAD_DEG of lag,
    // which a 4 deg move starting inside the tolerance cannot reach. So this always
    // completes.)
    bool target_arrived = std::abs(bar_target_deg_ - CHAINBAR_LOADING_DEG) < 1e-3;
    return target_arrived && std::abs(bar - CHAINBAR_LOADING_DEG) < CHAINBAR_POS_TOL;
}

bool Lift::score_stalled(uint32_t now, double bar_vel_dps) {
    // Resistance = the bar has STOPPED while the sweep is still commanding motion.
    //
    // The velocity collapse is the fast signal: the bar goes from its sweep speed to
    // ~0 within a tick or two of touching the tower, so the windowed rate sees it in
    // roughly one rate window (~50 ms). @p bar_vel_dps must therefore be that
    // windowed slope from update(), NOT bar_velocity_dps() -- the raw rotation-sensor
    // velocity aliases to a 0/spike sawtooth, which is too untrustworthy to trigger
    // on and is why this detector used to lean on the position error instead.
    //
    // The standing error is now only a CONFIRMATION that the PID is genuinely
    // pushing (rather than the profile having nothing to command), so
    // SCORE_STALL_ERR_DEG is a small floor instead of the thing that sets the
    // detection delay -- waiting for 8 deg of error to build at the 25 deg/s sweep
    // rate cost ~320 ms of pressing into the tower before the macro noticed.
    // SCORE_STALL_TICKS consecutive ticks are required so a single stick-slip dip
    // cannot fire it.
    if ((now - score_start_ms_) <= SCORE_STALL_GRACE_MS) {
        score_stall_ticks_ = 0;  // still spinning up: nothing counts yet
        return false;
    }
    bool not_moving = std::abs(bar_vel_dps) < SCORE_STALL_DPS;
    bool pushing = std::abs(bar_target_deg_ - get_bar_angle()) > SCORE_STALL_ERR_DEG;
    if (not_moving && pushing) {
        ++score_stall_ticks_;
    } else {
        score_stall_ticks_ = 0;
    }
    return score_stall_ticks_ >= SCORE_STALL_TICKS;
}

const char* Lift::mode_name(Mode m) {
    switch (m) {
        case Mode::STARTUP:  return "STARTUP";
        case Mode::STOW:     return "STOW";
        case Mode::PRIMING:  return "PRIMING";
        case Mode::SEQUENCE: return "SEQ";
        case Mode::SCORING:  return "SCORE";
        case Mode::SCORE_SETTLE: return "SETTLE";
        case Mode::RETURNING: return "RETURN";
    }
    return "?";
}

const char* Lift::prime_step_name(PrimeStep s) {
    switch (s) {
        case PrimeStep::RAISE:       return "RAISE";
        case PrimeStep::SWEEP_LOWER: return "SWEEP_LOWER";
    }
    return "?";
}

const char* Lift::return_step_name(ReturnStep s) {
    switch (s) {
        case ReturnStep::RAISE_SWEEP: return "RAISE_SWEEP";
        case ReturnStep::LOWER:       return "LOWER";
        case ReturnStep::BAR_SETTLE:  return "BAR_SETTLE";
    }
    return "?";
}

const char* Lift::start_step_name(StartStep s) {
    switch (s) {
        case StartStep::IDLE:        return "IDLE";
        case StartStep::BAR_CLEAR:   return "BAR_CLEAR";
        case StartStep::RAISE:       return "RAISE";
        case StartStep::BAR_LOADING: return "BAR_LOADING";
        case StartStep::LOWER:       return "LOWER";
        case StartStep::BAR_SETTLE:  return "BAR_SETTLE";
    }
    return "?";
}

const char* Lift::winch_mode_name(WinchMode m) {
    switch (m) {
        case WinchMode::HOLD:    return "HOLD";
        case WinchMode::SEEK:    return "SEEK";
        case WinchMode::RESPOOL: return "RESPOOL";
    }
    return "?";
}

// One flat, greppable debug line per (throttled) tick: mode, progress, every
// command, every desired vs. actual, every motor output. Prefixed "LIFTDBG" and
// formatted key=value so the Python log tools can parse it and so you can eyeball
// target-vs-actual while tuning the PIDs / feed-forward / stall thresholds.
// Angles are output deg (bar) / motor deg (winch, ee); mV are motor millivolts;
// velocities are motor RPM (winch) / output deg/s (bar) etc.
void Lift::log_debug_state(double winch_target, double winch, double winch_mv,
                           double winch_vel_rpm,
                           double bar, double bar_mv, double bar_vel_dps,
                           double bar_gravity_ff, double bar_static_ff,
                           double ee, double chain_mv, double chain_vel_dps) const {
    Logger::getInstance()->log(
        "LIFTDBG mode=%s seq=%s wmode=%s p=%.2f respooled=%d "
        "| WINCH tgt=%.1f act=%.1f err=%.1f mv=%.0f vel=%.1f "
        "| BAR tgt=%.1f act=%.1f err=%.1f mv=%.0f vel=%.1f ff=%.0f sff=%.0f "
        "| EE tgt=%.1f act=%.1f err=%.1f mv=%.0f vel=%.1f "
        "| claw=%d",
        mode_name(mode_), start_step_name(start_step_), winch_mode_name(winch_mode_),
        raise_progress_, winch_respooled_ ? 1 : 0,
        winch_target, winch, winch_target - winch, winch_mv, winch_vel_rpm,
        bar_target_deg_, bar, bar_target_deg_ - bar, bar_mv, bar_vel_dps,
        bar_gravity_ff, bar_static_ff,
        chain_target_deg_, ee, chain_target_deg_ - ee, chain_mv, chain_vel_dps,
        claw_.get_state() ? 1 : 0);
}

// ---- Main loop -------------------------------------------------------------

void Lift::update() {
    uint32_t now = pros::millis();
    // Fixed control step: the follower loop advances one iteration per tick, so
    // deriving dt from wall-clock would drift on a loop overrun. Config::DT is the
    // contract for the loop period.
    double dt = Config::DT / 1000.0;
    last_update_ms_ = now;

    double bar = get_bar_angle();
    double winch = get_winch_position();

    // Bar rate as a moving-window slope over CHAINBAR_RATE_WINDOW ticks. The
    // rotation sensor updates slower than the 100 Hz loop, so a per-tick delta
    // aliases into a 0/spike sawtooth (log 07242026: vel flips 0 <-> 500+ dps)
    // that gave the PID derivative nothing steady to damp against; a fixed window
    // averages that out with little lag and decays cleanly to 0 when the bar
    // stops. (get_velocity() is not used -- it badly under-reads fast bar motion.)
    // Computed here, before the mode switch, because the score stall detector needs
    // it too -- it is the signal that says "the bar has stopped", and the raw sensor
    // velocity it used to read is the aliased one this window exists to replace.
    for (int i = CHAINBAR_RATE_WINDOW - 1; i > 0; --i) {
        bar_angle_hist_[i] = bar_angle_hist_[i - 1];
    }
    bar_angle_hist_[0] = bar;
    double bar_vel_dps = (bar_angle_hist_[0] - bar_angle_hist_[CHAINBAR_RATE_WINDOW - 1])
                       / ((CHAINBAR_RATE_WINDOW - 1) * dt);

    // ---- Resolve this tick's winch / bar / chain targets by mode. ----
    // Default: no profiled bar motion this tick (holds / step targets). The slewed
    // sweeps overwrite this with their profile velocity for the velocity feed-forward.
    bar_target_vel_dps_ = 0.0;
    switch (mode_) {
        case Mode::STARTUP:
            // The startup choreography scripts the winch/bar/chain targets step
            // by step (see advance_start_sequence) and lands in STOW.
            advance_start_sequence(bar, winch);
            break;
        case Mode::STOW:
            winch_manual_target_ = WINCH_LOADING_POS;
            bar_target_deg_ = CHAINBAR_LOADING_DEG;
            chain_target_deg_ = CHAIN_LOADING_DEG;
            break;
        case Mode::PRIMING:
            // Clearance choreography: raise the lift, then sweep the bar the long
            // way to primed, lowering again as soon as the bar leaves the
            // low-clearance zone; hands off to SEQUENCE at progress 0.
            advance_prime_sequence(bar, winch);
            break;
        case Mode::RETURNING:
            // Reverse clearance choreography: raise the lift while the bar already
            // sweeps the long way back (it waits at the edge of the low-clearance
            // zone if the lift is still coming up), then lower; lands in STOW.
            advance_return_sequence(bar, winch);
            break;
        case Mode::SEQUENCE:
            // Advance the progress scalar while the raise button is held, or retreat
            // it while the lower button is held; freeze it (and therefore both
            // actuators) the instant neither is held. Raising wins if both are held.
            if (advancing_) {
                raise_progress_ = std::min(raise_progress_ + SEQ_RATE_PER_S * dt,
                                           static_cast<double>(SEQ_KEYFRAMES - 1));
            } else if (retreating_) {
                raise_progress_ = std::max(raise_progress_ - SEQ_RATE_PER_S * dt, 0.0);
            }
            seq_targets(raise_progress_, winch_manual_target_, bar_target_deg_);
            chain_target_deg_ = chain_scoring_deg_;
            break;
        case Mode::SCORING:
            // Reverse the sequence back toward the primed pose (progress 0),
            // watching for the piece meeting the tower (pressure = motor stall).
            if (raise_progress_ > 0.0) {
                raise_progress_ = std::max(raise_progress_ - SEQ_RATE_PER_S * dt, 0.0);
                seq_targets(raise_progress_, winch_manual_target_, bar_target_deg_);
            } else {
                // Past the primed pose: the progress scalar has bottomed out, so
                // carry the BAR on alone, further in the same (negative) direction,
                // up to SCORE_OVERTRAVEL_DEG beyond the primed angle. The winch
                // stays parked at the primed keyframe height. This is what lets the
                // macro reach a tower that sits below the primed pose; pressure
                // detection below is unchanged and still ends the score on contact.
                if (score_overtravel_deg_ <= 0.0) {
                    Logger::getInstance()->log(
                        "LIFTEVT score reached primed (bar=%.1f) -> overtravel to %.1f",
                        bar, SCORE_OVERTRAVEL_BAR_DEG);
                }
                score_overtravel_deg_ = std::min(score_overtravel_deg_ + SCORE_OVERTRAVEL_DPS * dt,
                                                 SCORE_OVERTRAVEL_DEG);
                seq_targets(0.0, winch_manual_target_, bar_target_deg_);
                bar_target_deg_ -= score_overtravel_deg_;
            }
            chain_target_deg_ = chain_scoring_deg_;
            if (score_stalled(now, bar_vel_dps)) {
                // Piece is against the tower: release it here and hold this exact
                // pose for SCORE_SETTLE_MS before returning, so the piece drops onto
                // the tower and settles instead of being dragged along by the bar
                // sweeping out from under it.
                Logger::getInstance()->log(
                    "LIFTEVT score PRESSURE @ p=%.2f over=%.1f bar=%.1f vel=%.1f err=%.1f "
                    "t=%u ms -> release + settle %u ms",
                    raise_progress_, score_overtravel_deg_, bar, bar_vel_dps,
                    bar_target_deg_ - bar, now - score_start_ms_, SCORE_SETTLE_MS);
                claw_.set_state(CLAW_OPEN_STATE);
                mode_ = Mode::SCORE_SETTLE;
                score_settle_start_ms_ = now;
            } else if (score_overtravel_deg_ >= SCORE_OVERTRAVEL_DEG &&
                       (now - score_start_ms_) > SCORE_STALL_GRACE_MS) {
                // Ran out the whole overtravel without contact: hold at primed.
                // Also wait out the stall grace before declaring "no contact":
                // scoring from the primed pose runs the whole overtravel in
                // SCORE_OVERTRAVEL_DEG / SCORE_OVERTRAVEL_DPS = 400 ms, which is
                // barely longer than the 350 ms grace, so exiting the instant the
                // target arrives would leave the detector almost no window to fire.
                Logger::getInstance()->log(
                    "LIFTEVT score overtravel exhausted w/o pressure (bar=%.1f) -> hold primed",
                    bar);
                mode_ = Mode::SEQUENCE;
            }
            break;
        case Mode::SCORE_SETTLE:
            // Claw is already open. Deliberately touch NOTHING: winch_manual_target_,
            // bar_target_deg_ and chain_target_deg_ keep the values the score left
            // them at, so every PID just holds the contact pose while the piece
            // settles. Timed, not blocking -- the rest of update() (and the caller's
            // loop) runs normally throughout.
            if (now - score_settle_start_ms_ >= SCORE_SETTLE_MS) {
                Logger::getInstance()->log(
                    "LIFTEVT score settle done after %u ms (bar=%.1f winch=%.1f) -> return",
                    now - score_settle_start_ms_, bar, winch);
                begin_return_to_loading();
            }
            break;
    }
    advancing_ = false;
    retreating_ = false;

    // Clamp the winch command to its physical travel.
    winch_manual_target_ = std::clamp(winch_manual_target_, WINCH_DOWN_POS, WINCH_MAX_POS);
    double winch_target = winch_manual_target_;

    // --- Winch: PID hold, plus a home-to-bottom pass at the bottom in STOW. ---
    // Once settled near the bottom in STOW, physically home the lift: SEEK drives
    // down until it bottoms out on the hard stop (so the reference is the stop,
    // not the drift-prone IME), then RESPOOL takes up the slack in the raise
    // direction until the spool stalls and re-zeros there. Gated to STOW so it can
    // never re-zero the encoder mid-sequence. Runs once per descent; re-arms on
    // the next raise. Full descent: HOLD -> SEEK -> RESPOOL -> HOLD.
    double winch_vel_rpm = winch_motors_.get_actual_velocity();
    double winch_vel_dps = winch_vel_rpm * RPM_TO_DPS;
    double winch_current = winch_motors_.get_current_draw();  // mA (diagnostic)

    bool commanded_down = (mode_ == Mode::STOW) &&
                          winch_target <= WINCH_DOWN_POS + WINCH_POS_TOL;
    if (!commanded_down) {
        // Going up / held up / not in stow: normal hold, re-arm for next descent.
        winch_mode_ = WinchMode::HOLD;
        winch_respooled_ = false;
    } else if (winch_mode_ == WinchMode::HOLD && !winch_respooled_ &&
               winch <= WINCH_DOWN_POS + WINCH_POS_TOL) {
        // Settled near the bottom and not yet homed: seek the physical stop first.
        winch_mode_ = WinchMode::SEEK;
        seek_start_ms_ = now;
        Logger::getInstance()->log("LIFTEVT seek START @ winch=%.1f", winch);
    }

    double winch_mv;
    if (winch_mode_ == WinchMode::SEEK) {
        // Short down pre-roll to ground the lift; the respool finds the true
        // bottom. Log winch pos/vel/current every tick so the descent stays
        // visible (e.g. to confirm the lift grounds and how much slack pays out).
        Logger::getInstance()->log("LIFTSEEK t=%u winch=%.1f vel=%.1f cur=%.0f mv=%.0f",
                                   now - seek_start_ms_, winch, winch_vel_rpm,
                                   winch_current, WINCH_SEEK_MV);
        // Fixed pre-roll: no bottom detection (this lift just unspools slack once
        // grounded, giving no clean down-signature). Drive down until the timer
        // elapses -- or the over-travel backstop trips -- then hand off to the
        // respool, which finds the true taut bottom on the way up and re-zeros.
        bool done = (now - seek_start_ms_) > WINCH_SEEK_MS;
        bool over_travel = winch <= WINCH_DOWN_POS - WINCH_SEEK_MAX_TRAVEL;
        if (done || over_travel) {
            Logger::getInstance()->log(
                "LIFTEVT seek done (%s) after %u ms @ winch=%.1f vel=%.1f -> respool",
                over_travel ? "over-travel" : "timed",
                now - seek_start_ms_, winch, winch_vel_rpm);
            winch_mode_ = WinchMode::RESPOOL;
            respool_start_ms_ = now;
            winch_mv = 0.0;
        } else {
            winch_mv = WINCH_SEEK_MV;  // gentle down pre-roll to ground the lift
        }
    } else if (winch_mode_ == WinchMode::RESPOOL) {
        // Per-tick trace so the current signature is visible: LOW while reeling
        // in slack, then a SPIKE when the string goes taut against the bottomed
        // lift (= stalls at the bottom, correct). Sustained HIGH the whole reel
        // would mean it is hauling the lift up instead (WINCH_RESPOOL_MV too big).
        Logger::getInstance()->log("LIFTRESPOOL t=%u winch=%.1f vel=%.1f cur=%.0f mv=%.0f",
                                   now - respool_start_ms_, winch, winch_vel_rpm,
                                   winch_current, WINCH_RESPOOL_MV);
        bool past_grace = (now - respool_start_ms_) > WINCH_RESPOOL_GRACE_MS;
        bool stalled = std::abs(winch_vel_rpm) < WINCH_RESPOOL_STALL_RPM;
        bool timed_out = (now - respool_start_ms_) > WINCH_RESPOOL_TIMEOUT_MS;
        if (past_grace && stalled) {
            // String taut: anchor the reference at this taut bottom and hold.
            // SAFEGUARD (armed once the startup choreography has finished): a re-zero
            // may move the winch reference by at most WINCH_REZERO_MAX_DEG. The stall
            // that triggers this is only evidence the spool stopped turning, so a
            // false stall (snag, stiff spool, sensor glitch) would otherwise rewrite
            // the reference by however far off it happened to be -- a single
            // set_zero_position_all() teleports the reading, and the STOW hold then
            // chases that phantom error into the hard stop. Capping the shift keeps
            // a bad homing pass to a bounded reference error that the next descent
            // can walk off, instead of a step change.
            double full_shift = WINCH_DOWN_POS - winch;  // correction the stall claims
            double applied_shift = std::clamp(full_shift, -WINCH_REZERO_MAX_DEG,
                                              WINCH_REZERO_MAX_DEG);
            bool capped = startup_complete_ && std::abs(full_shift) > WINCH_REZERO_MAX_DEG;
            double rezero_to = capped ? winch + applied_shift : WINCH_DOWN_POS;
            Logger::getInstance()->log(
                "LIFTEVT respool TAUT after %u ms @ winch=%.1f vel=%.1f cur=%.0f (re-zero to %.0f)",
                now - respool_start_ms_, winch, winch_vel_rpm, winch_current, rezero_to);
            if (capped) {
                Logger::getInstance()->log(
                    "LIFTWARN respool re-zero CAPPED: wanted %.1f deg shift (winch=%.1f -> %.0f), "
                    "applied %.1f (limit %.0f) -- suspect false stall",
                    full_shift, winch, WINCH_DOWN_POS, applied_shift, WINCH_REZERO_MAX_DEG);
            }
            winch_motors_.set_zero_position_all(rezero_to);
            winch_pid_.reset();
            winch_mode_ = WinchMode::HOLD;
            winch_respooled_ = true;
            winch_mv = 0.0;
        } else if (timed_out) {
            // Never found resistance -- give up so we can't drive up forever.
            Logger::getInstance()->log(
                "LIFTEVT respool TIMEOUT after %u ms @ winch=%.1f vel=%.1f cur=%.0f (no resistance found)",
                now - respool_start_ms_, winch, winch_vel_rpm, winch_current);
            winch_mode_ = WinchMode::HOLD;
            winch_respooled_ = true;  // don't immediately retry; wait for a raise
            winch_mv = 0.0;
        } else {
            winch_mv = WINCH_RESPOOL_MV;  // gentle raise to take up the slack
        }
    } else {
        winch_mv = winch_pid_.calculate(winch_target, winch, winch_vel_dps);
    }
    winch_motors_.move_voltage(static_cast<int>(winch_mv));

    // --- Chainbar: PID toward the resolved target, all the way through. ---
    // Mid return sweep the bar is let loose to swing home on gravity + momentum,
    // but the PID stays ENGAGED there to regulate that swing (see below).
    // Everywhere else it is the usual PID with the gravity + motion feed-forward.
    double bar_cmd = std::clamp(bar_target_deg_, CHAINBAR_MIN_DEG, CHAINBAR_MAX_DEG);
    double bar_gravity_ff = 0.0;
    double bar_static_ff = 0.0;
    double bar_mv;
    bool bar_coasting = bar_coast_active(bar, winch);
    if (bar_coasting != chainbar_coasting_) {
        // Brake mode still follows the window: the PID drives the bar throughout now
        // (see below), but if it ever does command 0 mV in here -- soft limit, a
        // settled error -- a HOLD brake would fight the swing, since the V5 brake is
        // itself a command. COAST leaves "no command" meaning no command; any real
        // command is unaffected by brake mode. Written on the edge only.
        chainbar_motor_.set_brake_mode(bar_coasting ? pros::E_MOTOR_BRAKE_COAST
                                                    : pros::E_MOTOR_BRAKE_HOLD);
        if (!bar_coasting) {
            // Catch: the same angle hands the drive from the FF-free sweep law to
            // the ordinary gravity + breakaway one (see bar_sweep below), so
            // clear the integral the long railed sweep leaves behind rather than
            // carrying it into the finish.
            chainbar_pid_.reset();
            bar_coast_done_ = true;  // one coast per sweep
        }
        Logger::getInstance()->log(
            "LIFTEVT bar coast %s, brake=%s (bar=%.1f vel=%.1f tgt=%.1f winch=%.1f)",
            bar_coasting ? "START" : "CATCH", bar_coasting ? "COAST" : "HOLD",
            bar, bar_vel_dps, bar_target_deg_, winch);
        chainbar_coasting_ = bar_coasting;
    }
    // The long part of a bar sweep, up to the catch angle. Past it the sweep is
    // within ~9 deg of its goal and hands back to the ordinary feed-forward branch
    // below: the FF-free control here is a big-error law (P against a step target up
    // to 215 deg away), and it has nothing left to give at the end of the travel. It
    // used to run to the end of RAISE_SWEEP, where ~1500 mV of P was all that stood
    // against ~2000 mV of stiction, so the bar parked itself 7 deg short and spent
    // 2.9 s creeping in on integral alone (log 07262026-2, 11.3-14.2). Handing over
    // at the catch gives that stretch the breakaway FF instead, which is what freed
    // the bar the instant LOWER took over in that same log.
    bool bar_sweep = bar_sweeping() && bar < CHAINBAR_COAST_CATCH_DEG;
    if (bar_coasting || bar_sweep) {
        // The long stretch of a bar sweep, coast included: plain position PID
        // against the step target, NO feed-forward -- no gravity term, no
        // kV/kinetic, no static breakaway. The step error rails the output, which is
        // all this stretch needs: carry the bar round to the loading side.
        //
        // Dropping the feed-forward is not just simplification: on this travel
        // gravity pulls the bar TOWARD the goal, so the gravity FF (a holding term)
        // points backwards the whole way, and the startup sweep needed 7031 mV to
        // break out against it before lurching to 312 deg/s and stalling again
        // (log 07262026-2, 02.04-02.69). Letting gravity pull and shaping with P is
        // what made the return sweep smooth.
        //
        // The coast stretch used to cut the motor dead here (0 mV, PID held in
        // reset) and let the bar free-swing home on gravity + momentum. It no longer
        // does: the PID drives it through, so a swing that runs out of momentum on
        // the way over the top is pushed through instead of being left hanging on an
        // unpowered stretch, and one that comes in fast is under control the whole
        // way rather than only from the catch angle. What is left of the coast is the
        // brake mode (COAST, so a zero command really is zero) -- see the edge above.
        //
        // The step target sits AHEAD of the bar for the whole sweep, so P pushes
        // with the swing the entire way and the only thing standing between the bar
        // and its free speed is kD (which brakes on the raw rate here -- the target
        // is a step, so there is no profile velocity to subtract). Hence the drive
        // cap after the branch: kD sets how early the bar slows down, the cap sets
        // how fast it gets going in the first place.
        bar_mv = chainbar_pid_.calculate(bar_cmd, bar, bar_vel_dps, 0.0);
    } else {
        // Gravity feed-forward for the pivoting bar: supply the standing holding
        // effort so the position PID doesn't need a lingering error to balance
        // gravity. Largest with the arm horizontal, zero at its vertical rest.
        static constexpr double DEG_TO_RAD = 3.14159265358979323846 / 180.0;
        bar_gravity_ff = CHAINBAR_GRAVITY_FF_MV *
            std::sin((bar - CHAINBAR_GRAVITY_ZERO_DEG) * DEG_TO_RAD);

        // Motion feed-forward: the term that actually moves the bar, so its speed is
        // COMMANDED rather than left to emerge from P vs. friction (which stick-slips
        // and crawls). Two regimes:
        double bar_err = bar_cmd - bar;
        if (std::abs(bar_target_vel_dps_) > CHAINBAR_PROFILE_EPS_DPS) {
            // Profiled sweep (the slewed approach to loading): follow the velocity
            // profile. kV * v_des tracks the commanded speed; a kinetic-friction term
            // in the direction of travel keeps low speeds from being eaten by
            // friction. Both scale with v_des, which the slew tapers to ~0 over the
            // decel zone -- so the bar cruises smoothly and then decelerates into the
            // pose, and there is no constant term left over at the target to push it
            // into the nogo zone.
            double kinetic = CHAINBAR_KINETIC_FF_MV *
                std::clamp(bar_target_vel_dps_ / CHAINBAR_KS_MOVING_DPS, -1.0, 1.0);
            bar_static_ff = kinetic + CHAINBAR_KV_MV_PER_DPS * bar_target_vel_dps_;
        } else {
            // Step target / hold (BAR_CLEAR, RAISE holds, sequence): position-error
            // BREAKAWAY, only when the bar is actually stuck. Points toward the target
            // and fades within CHAINBAR_KS_DEADBAND_DEG of it (no chatter at rest, no
            // shove through the loading pose); scaled from full at rest to zero once
            // the bar is moving (static friction is only needed to START motion --
            // applying it while moving made it reverse and oscillate, log 07242026-3).
            double ks_dir_scale = std::clamp(bar_err / CHAINBAR_KS_DEADBAND_DEG, -1.0, 1.0);
            double ks_stuck_scale =
                std::clamp(1.0 - std::abs(bar_vel_dps) / CHAINBAR_KS_MOVING_DPS, 0.0, 1.0);
            bar_static_ff = CHAINBAR_KS_MV * ks_dir_scale * ks_stuck_scale;
        }
        double bar_ff = bar_gravity_ff + bar_static_ff;

        // Feed the PID derivative the VELOCITY ERROR (actual - profile), not the raw
        // bar rate. The PID damps on -measuredRate assuming a stationary setpoint; but
        // during a profiled sweep the setpoint moves at bar_target_vel_dps_, so
        // raw-rate damping would brake the whole cruise (-kD*v) and fight the velocity
        // FF. Using (actual - desired) makes kD damp only DEVIATIONS from the profile:
        // zero while the bar tracks the cruise, active if it runs ahead or lags. For
        // holds/step targets bar_target_vel_dps_ is 0, so this is just the raw rate.
        double bar_vel_err = bar_vel_dps - bar_target_vel_dps_;

        // Pass the full FF (gravity + static) INTO the PID (not added afterward) so
        // the anti-windup sees the true post-FF command and the PID clamps the
        // FF-inclusive sum to its +/-12000 output range for us.
        bar_mv = chainbar_pid_.calculate(bar_cmd, bar, bar_vel_err, bar_ff);
    }

    // --- Drive cap for the long sweeps. -------------------------------------
    // All three of them (return RAISE_SWEEP, startup BAR_LOADING, prime
    // SWEEP_LOWER) chase a step target hundreds of degrees away, so P rails the
    // output for most of the travel and the bar's arrival speed is whatever the
    // motor and gravity happen to build. Capping the command caps the cruise speed
    // roughly in proportion, and the arrival ENERGY with its square. Each sweep has
    // its own constant because they are different moves: 215 deg with a long
    // friction runout, 54 deg straight downhill, and 308 deg with a powered climb
    // then a gravity-fed descent.
    //
    // Two carve-outs:
    //  - BRAKING is never capped. A command that opposes the bar's travel is the one
    //    thing these sweeps must not be short of (the original low-zone gate stop
    //    overran by 36 deg). The bar has to actually be MOVING for that to count as
    //    braking: a near-stationary bar has nothing to brake, and its windowed rate
    //    wobbles a couple of deg/s either way, which is enough to flip the sign test
    //    and wave a full-drive command straight past the cap.
    //  - the two LOW caps only apply once the bar is moving. Breaking this bar out
    //    of rest takes ~7000 mV (log 07262026-2: it sat still at 4560, then 6163,
    //    then went at 7031), so a low cap at rest can leave it parked in the pose
    //    for good -- the FF-free sweeps have no breakaway term at all to help.
    //    Static friction is far bigger than kinetic here, so full authority to break
    //    out and the low cap for the travel is the split that matches the mechanism;
    //    same reasoning as CHAINBAR_KS_MOVING_DPS fading the breakaway FF out.
    bool bar_moving = std::abs(bar_vel_dps) > CHAINBAR_BRAKE_MIN_DPS;
    double bar_drive_cap = 12000.0;  // no cap outside the sweeps
    if (bar_coasting || bar_sweep) {
        bar_drive_cap = (mode_ == Mode::STARTUP && bar_moving) ? CHAINBAR_START_SWEEP_MAX_MV
                                                              : CHAINBAR_SWEEP_MAX_MV;
    } else if (mode_ == Mode::PRIMING && prime_step_ == PrimeStep::SWEEP_LOWER &&
               bar < CHAINBAR_PRIME_CAP_START_DEG && bar_moving) {
        // The prime sweep is capped only PAST THE TOP of the swing: above
        // CHAINBAR_PRIME_CAP_START_DEG the bar is climbing against gravity and needs
        // the full rail to do it in reasonable time; past it gravity has changed
        // sides and the cap is what stops the motor pumping energy into a descent
        // that is already accelerating on its own.
        //
        // The prime sweep also keeps the feed-forward law (see its constant), so
        // unlike the other two its bar_mv already has gravity + breakaway folded in,
        // and the cap applies to that FF-inclusive command.
        bar_drive_cap = CHAINBAR_PRIME_SWEEP_MAX_MV;
    }
    bool bar_braking = bar_moving && bar_mv * bar_vel_dps < 0.0;
    if (!bar_braking) {
        bar_mv = std::clamp(bar_mv, -bar_drive_cap, bar_drive_cap);
    }

    // Soft rotation limit: never drive the bar past its cable-safe travel.
    bar_mv = apply_soft_limit(bar_mv, bar, CHAINBAR_MIN_DEG, CHAINBAR_MAX_DEG);
    chainbar_motor_.move_voltage(static_cast<int>(bar_mv));

    // --- Chain / end effector: PID-hold its fixed orientation. ---
    double ee = get_ee_angle();
    // Windowed-slope rate, same reason as the bar: the sensors' own velocity
    // readings swing +/-150 deg/s tick to tick with the EE physically at rest, and
    // kD turns that into ~850 mV of buzz (log 07242026-5, holding 270 deg -- that
    // was the motor IME, and chain_rotation_.get_velocity() aliases the same way the
    // bar's does, so the slope is still the right source). The position slope over
    // CHAIN_RATE_WINDOW ticks is quiet and still decays to 0 when it stops.
    for (int i = CHAIN_RATE_WINDOW - 1; i > 0; --i) {
        chain_angle_hist_[i] = chain_angle_hist_[i - 1];
    }
    chain_angle_hist_[0] = ee;
    double chain_vel_dps = (chain_angle_hist_[0] - chain_angle_hist_[CHAIN_RATE_WINDOW - 1])
                         / ((CHAIN_RATE_WINDOW - 1) * dt);

    double chain_cmd = std::clamp(chain_target_deg_, CHAIN_MIN_DEG, CHAIN_MAX_DEG);
    // Alternate-orientation return: kill the chain entirely until the bar is back
    // round to CHAIN_FLIP_BAR_DEG (see alt_return_hold_active). Not "hold where you
    // are" -- no command at all, so the EE is never driven while it is on the far
    // side of the swing. The PID is reset each tick so it re-engages from a clean
    // state (no stale integral / derivative from before the hold) the moment the bar
    // crosses the threshold.
    bool alt_hold = alt_return_hold_active(bar);
    if (alt_hold != alt_return_gate_) {
        // Brake mode has to follow the hold: at 0 mV a HOLD-braked motor still
        // fights any rotation (the V5 brake is itself a command), which is exactly
        // what this hold is trying to avoid. COAST leaves the EE genuinely free
        // while the bar is on the far side of the swing; HOLD comes back the moment
        // the chain is driven again. Only written on the edge -- a smart-port write
        // every tick would be wasted traffic.
        chain_motor_.set_brake_mode(alt_hold ? pros::E_MOTOR_BRAKE_COAST
                                             : pros::E_MOTOR_BRAKE_HOLD);
        Logger::getInstance()->log(
            "LIFTEVT alt-return chain hold %s, brake=%s (bar=%.1f ee=%.1f winch=%.1f floor=%.0f)",
            alt_hold ? "ON" : "OFF", alt_hold ? "COAST" : "HOLD",
            bar, ee, winch, return_hold_winch_);
        alt_return_gate_ = alt_hold;
    }
    double chain_mv;
    if (alt_hold) {
        chain_mv = 0.0;
        chain_pid_.reset();
    } else {
        chain_mv = chain_pid_.calculate(chain_cmd, ee, chain_vel_dps);
        // Speed cap (see CHAIN_MAX_MV): the PID rails on the big orientation flips,
        // so this is what sets how fast the EE rotates. Applied outside the PID
        // rather than as its output limit so the near-target behaviour is untouched;
        // the PID's own anti-windup still thinks it has +/-12000, but with kI=4 and
        // an integral limit of 50 the integral is worth at most ~200 mV, so it
        // cannot meaningfully wind up against this clamp during the slew.
        chain_mv = std::clamp(chain_mv, -CHAIN_MAX_MV, CHAIN_MAX_MV);
        // Soft rotation limit: never over-rotate the chain / end effector.
        chain_mv = apply_soft_limit(chain_mv, ee, CHAIN_MIN_DEG, CHAIN_MAX_DEG);
    }
    chain_motor_.move_voltage(static_cast<int>(chain_mv));

    // Full debug line to the SD log, throttled to ~20 Hz (every 5th 100 Hz tick)
    // so the async logger's SD writes keep up while still being dense enough to
    // tune the PIDs / gravity feed-forward / stall thresholds off-robot.
    static uint32_t dbg_log_counter = 0;
    if (++dbg_log_counter % 5 == 0) {
        log_debug_state(winch_target, winch, winch_mv, winch_vel_rpm,
                        bar, bar_mv, bar_vel_dps, bar_gravity_ff, bar_static_ff,
                        ee, chain_mv, chain_vel_dps);
    }

    // Compact brain-screen telemetry, throttled to ~5 Hz. pros::lcd::print wraps
    // LVGL; calling it on every 10 ms tick races the LVGL render daemon and
    // intermittently freezes the program. update() runs at 100 Hz in both
    // opcontrol and the autonomous follower, so this throttle protects both paths.
    static uint32_t lcd_print_counter = 0;
    if (++lcd_print_counter % 20 == 0) {
        const char* winch_status = (winch_mode_ == WinchMode::SEEK)    ? "SEEK"
                                 : (winch_mode_ == WinchMode::RESPOOL) ? "RESPOOL" : "";
        pros::lcd::print(2, "Winch t:%.0f a:%.0f %s", winch_target, winch, winch_status);
        pros::lcd::print(3, "Bar t:%.1f a:%.1f %s p:%.2f", bar_target_deg_, bar,
                         mode_name(mode_), raise_progress_);
        pros::lcd::print(4, "EE t:%.0f a:%.0f", chain_target_deg_, get_ee_angle());
    }
}
