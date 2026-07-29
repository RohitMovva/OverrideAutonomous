#include "hardware/lift.hpp"
#include "config.hpp"
#include "pros/llemu.hpp"
#include "utilities/logger.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>

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
      matchload_step_(MatchloadStep::RAISE),
      back_step_(BackStep::HOLD),
      score_step_(ScoreStep::HOLD),
      score_target_(nullptr),
      back_pose_(BackPose::BOTTOM),
      back_hold_bar_(CHAINBAR_BACK_BOTTOM_DEG),
      back_hold_winch_(WINCH_BACK_BOTTOM_POS),
      return_dest_(Mode::STOW),
      entry_bar_deg_(CHAINBAR_START_DEG),
      startup_complete_(false),
      chainbar_coasting_(false),
      bar_coast_done_(false),
      alt_return_gate_(false),
      return_hold_winch_(WINCH_START_POS),
      raise_progress_(0.0),
      bar_swing_broke_out_(false),
      seq_swing_driving_(false),
      advancing_(false),
      retreating_(false),
      winch_manual_target_(WINCH_START_POS),
      bar_target_deg_(CHAINBAR_START_DEG),
      bar_target_vel_dps_(0.0),
      chain_target_deg_(CHAIN_START_DEG),
      ee_family_(EeFamily::FRONT),
      ee_alt_(false),
      retreat_winch_target_(WINCH_START_POS),
      score_retreat_parked_(false),
      winch_ref_offset_(WINCH_START_POS),
      last_update_ms_(0),
      winch_mode_(WinchMode::HOLD),
      winch_respooled_(false),
      respool_start_ms_(0),
      seek_start_ms_(0),
      seek_start_pos_(WINCH_START_POS),
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
    // The BAR and the EE are no longer re-zeroed. Both are rotation sensors on their
    // own axle, and those hold their reading across a power cycle, so overwriting them
    // with a START_* constant would throw away a good absolute reference and replace
    // it with an assumption about where the mechanism was parked.
    //
    // They do still need their TURN COUNT resolved once, which is a different thing.
    // The sensor comes up reporting its absolute angle folded into [0, 360), so the
    // bar in its START pose reads ~335 rather than -25 -- same physical place, wrong
    // branch, and a 360 deg error would put every pose, soft limit and PID error in
    // the file on the wrong side of the mechanism. get_position() accumulates
    // continuously from its start value (nothing wraps once running), so choosing the
    // right branch ONCE here fixes every later reading. unwrap_angle_near() does that
    // by picking the multiple of 360 nearest the pose the mechanism is documented to
    // boot in -- the sensor's fine angle is kept exactly as measured; only the turn
    // count is decided. Booting in some other pose puts it on the wrong branch, hence
    // the sanity warning below.
    winch_motors_.set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);
    probe("winch set_encoder_units_all ok");
    winch_motors_.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    probe("winch set_brake_mode_all ok");
    // Zero the encoders and hold the reference in software instead. 0 is the one value
    // set_zero_position_all() gets right on this reversed group (see
    // winch_ref_offset_), so the seeded START offset goes into the offset member.
    winch_motors_.set_zero_position_all(0.0);
    winch_ref_offset_ = WINCH_START_POS;
    probe("winch encoders zeroed, reference seeded");

    chainbar_motor_.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    probe("chainbar set_brake_mode ok");
    // The bar angle is sensed by a rotation sensor on the axle (1:1), not the motor
    // encoder. Orient it, then resolve its turn count (see below).
    chainbar_rotation_.set_reversed(CHAINBAR_ROTATION_REVERSED);
    probe("chainbar rotation set_reversed ok");
    double bar_raw = chainbar_rotation_.get_position() / 100.0;
    double bar_seed = unwrap_angle_near(bar_raw, CHAINBAR_START_DEG);
    chainbar_rotation_.set_position(static_cast<std::int32_t>(bar_seed * 100.0));
    probe("chainbar rotation turn count resolved");

    chain_motor_.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    probe("chain set_brake_mode ok");
    // Like the bar, the EE angle is sensed by a rotation sensor on its own axle
    // (1:1), not the chain motor's encoder -- so nothing here touches the motor's
    // encoder units or zero. Same turn-count resolution as the bar.
    chain_rotation_.set_reversed(CHAIN_ROTATION_REVERSED);
    probe("chain rotation set_reversed ok");
    double ee_raw = chain_rotation_.get_position() / 100.0;
    double ee_seed = unwrap_angle_near(ee_raw, CHAIN_START_DEG);
    chain_rotation_.set_position(static_cast<std::int32_t>(ee_seed * 100.0));
    probe("chain rotation turn count resolved");

    // Start in the dedicated power-on START pose (STARTUP mode). CHAINBAR_START_DEG /
    // CHAIN_START_DEG now only DESCRIBE that pose (they are what the sensors should
    // read there, and what the choreography's first clearance move is measured from);
    // nothing writes them into a sensor any more.
    mode_ = Mode::STARTUP;
    start_step_ = StartStep::IDLE;
    prime_step_ = PrimeStep::RAISE;
    return_step_ = ReturnStep::RAISE_SWEEP;
    matchload_step_ = MatchloadStep::RAISE;
    return_dest_ = Mode::STOW;   // the startup choreography lands in STOW
    startup_complete_ = false;  // re-zero cap arms when the choreography finishes
    chainbar_coasting_ = false;
    bar_coast_done_ = false;
    alt_return_gate_ = false;
    return_hold_winch_ = WINCH_START_POS;
    winch_mode_ = WinchMode::HOLD;
    winch_respooled_ = false;
    raise_progress_ = 0.0;
    bar_swing_broke_out_ = false;
    seq_swing_driving_ = false;
    advancing_ = false;
    retreating_ = false;
    winch_manual_target_ = WINCH_START_POS;
    // Open the bar / EE targets on their MEASURED angles, not the START_* constants.
    // Now that the rotation sensors are not seeded, the reading is whatever the
    // mechanism was actually left at, which is only approximately the START pose --
    // and any difference would otherwise be a standing error on tick 1, i.e. a lurch
    // the moment update() first runs. The startup choreography overwrites both within
    // a tick or two anyway; this just makes the handover quiet.
    double bar0 = get_bar_angle();
    double ee0 = get_ee_angle();
    bar_target_deg_ = bar0;
    chain_target_deg_ = ee0;
    retreat_winch_target_ = WINCH_START_POS;
    score_retreat_parked_ = false;
    // Back cycle idle, and the raised orientation back to the front family's default.
    back_step_ = BackStep::HOLD;
    back_pose_ = BackPose::BOTTOM;
    back_hold_bar_ = CHAINBAR_BACK_BOTTOM_DEG;
    back_hold_winch_ = WINCH_BACK_BOTTOM_POS;
    ee_family_ = EeFamily::FRONT;
    ee_alt_ = false;
    claw_.set_state(CLAW_START_STATE);
    probe("claw set_state ok");
    winch_pid_.reset();
    chainbar_pid_.reset();
    chain_pid_.reset();
    for (int i = 0; i < CHAINBAR_RATE_WINDOW; ++i) {  // seed the windowed-rate history
        bar_angle_hist_[i] = bar0;
    }
    for (int i = 0; i < CHAIN_RATE_WINDOW; ++i) {     // else tick 1 reads a huge rate
        chain_angle_hist_[i] = ee0;
    }
    // Boot reference trace: raw sensor angle, the branch it was resolved onto, and
    // the START pose it should match. This is the line to read when a pose looks 360
    // deg out or the mechanism sets off the wrong way on power-up.
    Logger::getInstance()->log(
        "LIFTINIT refs: bar raw=%.1f -> %.1f (START %.1f) | ee raw=%.1f -> %.1f "
        "(START %.1f) | winch raw=%.1f + offset=%.1f -> %.1f (START %.1f)",
        bar_raw, bar0, CHAINBAR_START_DEG, ee_raw, ee0, CHAIN_START_DEG,
        motor_group_position(), winch_ref_offset_, get_winch_position(),
        WINCH_START_POS);
    if (std::abs(bar0 - CHAINBAR_START_DEG) > START_POSE_SANITY_DEG ||
        std::abs(ee0 - CHAIN_START_DEG) > START_POSE_SANITY_DEG) {
        // The turn count was resolved against the START pose, so if the mechanism was
        // not actually there the resolved angle can be whole turns out -- and the
        // startup choreography is about to drive to absolute angles on that reference.
        Logger::getInstance()->log(
            "LIFTWARN boot pose is NOT the START pose (bar %.1f vs %.1f, ee %.1f vs "
            "%.1f, limit %.0f) -- turn count may be resolved onto the wrong branch; "
            "power on with the mechanism in START",
            bar0, CHAINBAR_START_DEG, ee0, CHAIN_START_DEG, START_POSE_SANITY_DEG);
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
    // Raw encoder average plus the software reference offset -- see winch_ref_offset_
    // for why the reference cannot live in the motors on a reversed group.
    return motor_group_position() + winch_ref_offset_;
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

double Lift::seq_swing_progress(double bar) {
    double span = SEQ_BAR_DEG[1] - SEQ_BAR_DEG[0];
    if (std::abs(span) < 1e-9) return 0.0;
    return std::clamp((bar - SEQ_BAR_DEG[0]) / span, 0.0, 1.0);
}

double Lift::slew_bar_toward(double current, double goal, double actual_bar,
                             double& out_vel_dps, double max_dps, double decel_deg) {
    double dt = Config::DT / 1000.0;
    double remaining = goal - current;
    double dist = std::abs(remaining);
    // Taper the rate over the decel zone so the bar decelerates into the goal;
    // full rate outside it. The floor keeps the tail from asymptoting. Rate and zone
    // are per-call (defaulted to the CHAINBAR_APPROACH_* constants) so a long sweep
    // can cruise at its own speed and still land on its pose -- the prime sweep runs
    // this law at CHAINBAR_PRIME_SWEEP_DPS over CHAINBAR_PRIME_SWEEP_DECEL_DEG.
    double frac = 1.0;
    if (decel_deg > 0.0) {
        frac = std::clamp(dist / decel_deg, CHAINBAR_APPROACH_MIN_FRAC, 1.0);
    }
    double rate = max_dps * frac;  // profile speed magnitude (deg/s)
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
    // The raise sequence starts from either resting pose -- loading or matchload.
    // Both sit at the bottom with the claw open and a piece in reach, and the
    // clearance raise below holds the bar wherever it is, so the only difference
    // between them is the angle the long sweep sets off from.
    if (mode_ != Mode::STOW && mode_ != Mode::MATCHLOAD) {
        Logger::getInstance()->log("LIFTCMD prime IGNORED (mode=%s, not STOW/MATCHLOAD)",
                                   mode_name(mode_));
        return;
    }
    Logger::getInstance()->log(
        "LIFTCMD prime from %s -> clearance raise (bar held at %.1f, tuck to bar %.1f / "
        "EE %.1f above winch %.0f), then sweep to KF0 (bar %.1f) w/ lower from %.1f",
        mode_name(mode_), get_bar_angle(), CHAINBAR_PRIME_TUCK_DEG, CHAIN_PRIME_TUCK_DEG,
        WINCH_PRIME_TUCK_POS, SEQ_BAR_DEG[0], CHAINBAR_LOW_ZONE_DEG);
    claw_.set_state(CLAW_CLOSED_STATE);  // grab the piece before raising
    ee_family_ = EeFamily::FRONT;  // front pair of raised orientations; ee_alt_ untouched
    raise_progress_ = 0.0;
    // Hold the bar at the pose we are leaving while the cascade climbs, rather than
    // at a hardcoded loading angle: from matchload that would swing the bar back to
    // loading first, across the low-clearance zone, with the cascade still on its way
    // up. The sweep to primed then just carries on from here.
    entry_bar_deg_ = get_bar_angle();
    // Run the clearance choreography (raise -> sweep -> lower) rather than moving
    // straight to the primed pose, so the lift is up while the bar sweeps the long
    // way from loading and clears the obstacle on the short path.
    mode_ = Mode::PRIMING;
    prime_step_ = PrimeStep::RAISE;
    chainbar_pid_.reset();
    winch_pid_.reset();
}

void Lift::back_prime() {
    // Already in the back cycle: this is the pose toggle. Restarting the move at RAISE
    // is right from ANY step, including part way through a move onto the other pose --
    // the driver has just changed their mind, and the raise/swing/lower path handles
    // being entered from anywhere (each step passes straight through if it has nothing
    // to do). The claw stays as it is: a toggle is not a re-grab.
    if (mode_ == Mode::BACK) {
        back_pose_ = (back_pose_ == BackPose::BOTTOM) ? BackPose::TOP : BackPose::BOTTOM;
        Logger::getInstance()->log(
            "LIFTCMD back toggle -> %s (bar %.1f -> %.1f, winch %.1f -> %.0f) via %s",
            back_pose_name(back_pose_), get_bar_angle(), back_bar_deg(),
            get_winch_position(), back_winch_pos(), back_step_name(BackStep::RAISE));
        back_step_ = BackStep::RAISE;
        entry_bar_deg_ = get_bar_angle();  // hold here while the cascade climbs
        bar_target_deg_ = entry_bar_deg_;
        chainbar_pid_.reset();
        winch_pid_.reset();
        return;
    }
    // Otherwise this is the back PRIME, and like prime() it is only valid from a
    // resting pose -- it closes the claw on a piece that is sitting in the mechanism.
    // Both resting poses work, and for the same reason prime() takes both: the move
    // onto BOTTOM holds the bar wherever the pose left it while the cascade climbs
    // (entry_bar_deg_), so it never detours through the other one on the way. Coming
    // back out to either pose is the ordinary return -- see the header doc.
    if (mode_ != Mode::STOW && mode_ != Mode::MATCHLOAD) {
        Logger::getInstance()->log("LIFTCMD back_prime IGNORED (mode=%s, not STOW/MATCHLOAD)",
                                   mode_name(mode_));
        return;
    }
    Logger::getInstance()->log(
        "LIFTCMD back_prime from %s -> BOTTOM (bar %.1f, winch %.0f, ee %s %.0f)",
        mode_name(mode_), CHAINBAR_BACK_BOTTOM_DEG, WINCH_BACK_BOTTOM_POS,
        ee_alt_ ? "ALT" : "default", ee_alt_ ? CHAIN_BACK_ALT_DEG : CHAIN_BACK_DEG);
    claw_.set_state(CLAW_CLOSED_STATE);  // grab the piece before moving
    // Switch the raised-orientation pair to the back one. Only the PAIR changes -- the
    // driver's default/alternate selection (ee_alt_) is left alone, so the flip button
    // means the same thing on both sides and a choice made before priming survives.
    ee_family_ = EeFamily::BACK;
    mode_ = Mode::BACK;
    back_pose_ = BackPose::BOTTOM;
    back_step_ = BackStep::RAISE;
    entry_bar_deg_ = get_bar_angle();
    bar_target_deg_ = entry_bar_deg_;
    back_hold_bar_ = CHAINBAR_BACK_BOTTOM_DEG;
    back_hold_winch_ = WINCH_BACK_BOTTOM_POS;
    chainbar_pid_.reset();
    winch_pid_.reset();
}

void Lift::toggle_matchload() {
    if (mode_ == Mode::MATCHLOAD) {
        // Matchload -> loading is the return choreography, unchanged: raise, sweep
        // the bar back past loading, lower, settle. It already handles starting from
        // part way round (the sweep is capped at the low-zone edge until the cascade
        // is up) and it is the tuned path, so there is no second copy of it here.
        Logger::getInstance()->log("LIFTCMD matchload toggle -> back to loading");
        begin_return(Mode::STOW);
        return;
    }
    if (mode_ != Mode::STOW) {
        // Anywhere else -- primed, part way up the sequence, mid-score, or already
        // returning -- this means "come back, to matchload". Same choreography as the
        // loading return, aimed at the other pose; no need to detour through loading
        // and re-ask. STARTUP is the one exception: it owns the mechanism until it
        // has a valid reference and a known pose.
        if (mode_ == Mode::STARTUP) {
            Logger::getInstance()->log("LIFTCMD matchload IGNORED (mode=STARTUP)");
            return;
        }
        return_to_matchload();
        return;
    }
    Logger::getInstance()->log(
        "LIFTCMD matchload -> clearance raise, swing bar to %.1f / EE to %.1f, lower to %.0f",
        CHAINBAR_MATCHLOAD_DEG, CHAIN_MATCHLOAD_DEG, WINCH_MATCHLOAD_POS);
    mode_ = Mode::TO_MATCHLOAD;
    matchload_step_ = MatchloadStep::RAISE;
    entry_bar_deg_ = get_bar_angle();  // hold here while the cascade climbs
    bar_target_deg_ = entry_bar_deg_;
    chainbar_pid_.reset();
    winch_pid_.reset();
}

void Lift::move_lift_up() {
    // Consumed in update(); only actually advances while in the raise sequence.
    advancing_ = true;
}

void Lift::move_lift_down() {
    // Consumed in update(); only actually retreats while in the raise sequence.
    // A plain manual reverse of the sequence -- no release / auto-return.
    retreating_ = true;
}

void Lift::score() {
    // Release where the mechanism already is. The macro does not move anything onto
    // the tower and does not try to detect the piece meeting it: the driver has put
    // the mechanism where the piece should go, so the button just lets it go.
    //
    // Valid in the same two places it always was -- the raise sequence, and the back
    // cycle once PARKED on a pose -- plus, now, a score state once it has ARRIVED
    // (@ref run_score_state). Mid-move is still refused in both cycles: the
    // alternate-orientation retreat below would swing the bar while the cascade is in
    // transit.
    bool from_back = mode_ == Mode::BACK && back_step_ == BackStep::HOLD;
    bool from_score_state = mode_ == Mode::SCORE_STATE && score_step_ == ScoreStep::HOLD;
    if (mode_ != Mode::SEQUENCE && !from_back && !from_score_state) {
        Logger::getInstance()->log(
            "LIFTCMD score IGNORED (mode=%s%s, not in the sequence, a back pose or a score state)",
            mode_name(mode_),
            (mode_ == Mode::BACK || mode_ == Mode::SCORE_STATE) ? " mid-move" : "");
        return;
    }
    double bar = get_bar_angle();
    // A score state names its own end-effector angle, so the driver's default/alternate
    // SELECTION (ee_alt_) does not describe how the EE is actually oriented here and
    // must not decide whether the retreat runs. Releasing from a score state is
    // therefore always the plain release: claw open, hold the pose, let the caller
    // decide what happens next. If a score state is ever given a FLIPPED orientation
    // (CHAIN_SCORING_ALT_DEG / CHAIN_BACK_ALT_DEG) it will want the retreat, and the way
    // to say so is a per-row flag on ScoreState -- not ee_alt_.
    bool retreat = ee_alt_ && !from_score_state;
    Logger::getInstance()->log(
        "LIFTCMD score (%s) release @ p=%.2f bar=%.1f winch=%.1f, ee %s (tgt %.0f)%s",
        from_score_state ? current_score_state() : (from_back ? "back" : "front"),
        raise_progress_, bar, get_winch_position(),
        from_score_state ? "score-state" : (ee_alt_ ? "ALT" : "default"),
        from_score_state ? score_target_->ee_deg : raised_ee_deg(),
        retreat ? " -> retreat" : " -> hold");
    claw_.set_state(CLAW_OPEN_STATE);
    if (!retreat) {
        // DEFAULT orientation (or any score state): the end effector is clear of the
        // piece as soon as the claw opens, so there is nothing to do but hold. The mode
        // is left alone -- the driver keeps the raise/lower buttons, autonomous keeps
        // its pose, and either decides when to come home.
        return;
    }
    // ALTERNATE orientation: the end effector is flipped half a turn, so the piece has
    // just been dropped out of an EE that is itself still sitting in the structure it
    // was dropped into. Raise the cascade WINCH_ALT_RETREAT_LIFT_DEG and then swing
    // the bar up to SCORE_ALT_RETREAT_BAR_DEG to lift it clear, and park there -- see
    // the SCORE_RETREAT case in update() for the ordering, the clearance interlock and
    // the park. That mode does not take itself home either; the driver leaves it
    // with Y/X.
    //
    // The lift goes first so the EE comes up out of the structure rather than dragging
    // round through it, and it is clamped to the travel: released at or near full
    // extension there is no room for it and the swing starts straight away.
    double winch_now = get_winch_position();
    retreat_winch_target_ = std::clamp(winch_now + WINCH_ALT_RETREAT_LIFT_DEG,
                                       WINCH_DOWN_POS, WINCH_MAX_POS);
    Logger::getInstance()->log(
        "LIFTEVT score retreat armed: lift winch %.1f -> %.0f (+%.0f, capped %.0f), "
        "then swing bar %.1f -> %.1f",
        winch_now, retreat_winch_target_, WINCH_ALT_RETREAT_LIFT_DEG, WINCH_MAX_POS,
        bar, SCORE_ALT_RETREAT_BAR_DEG);
    mode_ = Mode::SCORE_RETREAT;
    score_retreat_parked_ = false;  // arm the park for this retreat
    entry_bar_deg_ = bar;           // angle to sit at while the cascade lifts / waits
    chainbar_pid_.reset();
    winch_pid_.reset();
}

void Lift::open_claw() {
    // Manual release, deliberately ungated: it only touches the solenoid, so there
    // is no pose it is unsafe from, and the primed / raise / score states -- where
    // the macros otherwise keep the claw clamped -- are exactly where it is needed.
    Logger::getInstance()->log("LIFTCMD open_claw (mode=%s p=%.2f bar=%.1f claw was %d)",
                               mode_name(mode_), raise_progress_, get_bar_angle(),
                               claw_.get_state() ? 1 : 0);
    if (claw_.get_state() == 1){
        claw_.set_state(CLAW_OPEN_STATE);
    } else {
        claw_.set_state(CLAW_CLOSED_STATE);
    }
}

void Lift::toggle_scoring_orientation() {
    // Flip which raised orientation the end effector holds. Only the SELECTION is
    // stored -- default or alternate -- never an angle: which pair of angles that
    // selection resolves to is the cycle's business (raised_ee_deg()), so this one
    // button flips the EE in the front sequence and in the back cycle alike. Every
    // mode that commands a raised pose reads raised_ee_deg() each tick, so an
    // in-flight sequence turns immediately and the choice carries into the next prime.
    ee_alt_ = !ee_alt_;
    Logger::getInstance()->log(
        "LIFTCMD flip scoring orientation -> %s %.0f (mode=%s ee=%.1f)",
        ee_alt_ ? "ALT" : "default", raised_ee_deg(), mode_name(mode_), get_ee_angle());
}

// ---- Autonomous interface (keyed macros) -----------------------------------

const Lift::ScoreState* Lift::find_score_state(Side side, const char* key) {
    if (key == nullptr) {
        return nullptr;
    }
    const ScoreState* table = (side == Side::BACK) ? BACK_SCORE_STATES : FRONT_SCORE_STATES;
    int count = (side == Side::BACK) ? BACK_SCORE_STATE_COUNT : FRONT_SCORE_STATE_COUNT;
    for (int i = 0; i < count; ++i) {
        if (std::strcmp(table[i].key, key) == 0) {
            return &table[i];
        }
    }
    return nullptr;
}

bool Lift::run_macro(Side side, const char* key) {
    // Pure dispatch: every branch calls the macro that already exists, which keeps its
    // own gating and its own LIFTCMD line. Nothing here decides anything -- if a macro
    // is invalid from the current mode it says so itself, exactly as it does for the
    // driver's button. Written as a strcmp chain rather than a map to match the repo's
    // other string dispatch (Odometry::disableSensor, Intake::start_color_sorting) and
    // to keep it off the heap.
    if (key == nullptr) {
        Logger::getInstance()->logError("LIFTCMD run_macro REFUSED (null key)");
        return false;
    }
    const char* side_name = (side == Side::BACK) ? "BACK" : "FRONT";
    if (std::strcmp(key, "prime") == 0) {
        // The one key the side actually selects between: the front raise sequence and
        // the back cycle are entered by two different macros.
        Logger::getInstance()->log("LIFTCMD run_macro %s prime", side_name);
        if (side == Side::BACK) { back_prime(); } else { prime(); }
        return true;
    }
    // Everything below is side-agnostic -- the cycle already in force determines what
    // it means (that is the whole design of score() and the returns). The side is
    // logged so a route that passes the wrong one is visible, not silently honoured.
    Logger::getInstance()->log("LIFTCMD run_macro %s %s (side not used by this macro)",
                               side_name, key);
    if (std::strcmp(key, "matchload") == 0)         { toggle_matchload(); }
    else if (std::strcmp(key, "score") == 0)        { score(); }
    else if (std::strcmp(key, "return") == 0)       { return_to_loading(); }
    else if (std::strcmp(key, "return_matchload") == 0) { return_to_matchload(); }
    else if (std::strcmp(key, "claw") == 0)         { open_claw(); }
    else if (std::strcmp(key, "flip") == 0)         { toggle_scoring_orientation(); }
    else if (std::strcmp(key, "up") == 0)           { move_lift_up(); }
    else if (std::strcmp(key, "down") == 0)         { move_lift_down(); }
    else {
        Logger::getInstance()->logError(
            "LIFTCMD run_macro REFUSED: unknown key '%s' (expected prime / matchload / "
            "score / return / return_matchload / claw / flip / up / down)", key);
        return false;
    }
    return true;
}

bool Lift::run_score_state(Side side, const char* key) {
    const ScoreState* target = find_score_state(side, key);
    if (target == nullptr) {
        Logger::getInstance()->logError(
            "LIFTCMD score_state REFUSED: no %s state named '%s'",
            side == Side::BACK ? "BACK" : "FRONT", key ? key : "(null)");
        return false;
    }
    // STARTUP is the one refusal, for the same reason toggle_matchload() refuses it:
    // that choreography owns the mechanism until it has a known pose, and moving off a
    // pose we have not established yet would swing the bar from an unresolved angle.
    // Every other mode is fair game -- the RAISE step below holds the bar wherever it
    // is while the cascade climbs, so this can be entered from a resting pose, part way
    // up the raise sequence, from the back cycle, or from another score state.
    if (mode_ == Mode::STARTUP) {
        Logger::getInstance()->log("LIFTCMD score_state '%s' IGNORED (mode=STARTUP)", key);
        return false;
    }
    Logger::getInstance()->log(
        "LIFTCMD score_state %s '%s' from %s -> raise, swing bar %.1f -> %.1f, "
        "EE -> %.1f, lower to %.0f",
        side == Side::BACK ? "BACK" : "FRONT", target->key, mode_name(mode_),
        get_bar_angle(), target->bar_deg, target->ee_deg, target->winch_pos);
    // The claw is deliberately untouched: a score state is a POSE, not a grab and not a
    // release. prime() grabs, score() releases.
    score_target_ = target;
    // Which raised-orientation pair a later flip resolves against. The pose itself does
    // not use it (it names its own EE angle), but the cycle it hands over to does.
    ee_family_ = (side == Side::BACK) ? EeFamily::BACK : EeFamily::FRONT;
    mode_ = Mode::SCORE_STATE;
    score_step_ = ScoreStep::RAISE;
    // Hold the bar at whatever angle the previous pose left it while the cascade
    // climbs, rather than dragging it toward the destination at the current height --
    // the same reason prime(), back_prime() and the matchload hop all latch this.
    entry_bar_deg_ = get_bar_angle();
    bar_target_deg_ = entry_bar_deg_;
    chainbar_pid_.reset();
    winch_pid_.reset();
    return true;
}

bool Lift::is_busy() const {
    switch (mode_) {
        case Mode::STARTUP:
            return start_step_ != StartStep::IDLE;
        case Mode::PRIMING:
        case Mode::RETURNING:
        case Mode::TO_MATCHLOAD:
            // Single-purpose choreographies: they exist only while running and land in
            // another mode when they finish, so being in one IS being busy.
            return true;
        case Mode::BACK:
            return back_step_ != BackStep::HOLD;
        case Mode::SCORE_STATE:
            return score_step_ != ScoreStep::HOLD;
        case Mode::SCORE_RETREAT:
            // The retreat is done when the bar has parked at the retreat angle; it
            // holds there indefinitely afterwards, waiting for a return.
            return !score_retreat_parked_;
        case Mode::STOW:
        case Mode::MATCHLOAD:
            return false;
        case Mode::SEQUENCE:
            // Walked by the driver's buttons, with no destination of its own -- there is
            // nothing here for a caller to wait on, so it is never "busy".
            return false;
    }
    return false;
}

bool Lift::is_settled() const {
    if (is_busy()) {
        return false;
    }
    // Not moving anywhere AND actually on the pose. The targets read here are the ones
    // update() resolved on the last tick, so this is only meaningful once the caller has
    // ticked update() at least once since the command.
    return std::abs(get_winch_position() - winch_manual_target_) < WINCH_POS_TOL &&
           std::abs(get_bar_angle() - bar_target_deg_) < CHAINBAR_POS_TOL &&
           std::abs(get_ee_angle() - chain_target_deg_) < CHAIN_POS_TOL;
}

const char* Lift::current_score_state() const {
    if (mode_ != Mode::SCORE_STATE || score_target_ == nullptr) {
        return nullptr;
    }
    return score_target_->key;
}

void Lift::begin_return(Mode dest) {
    return_dest_ = (dest == Mode::MATCHLOAD) ? Mode::MATCHLOAD : Mode::STOW;
    bool to_match = return_dest_ == Mode::MATCHLOAD;
    Logger::getInstance()->log(
        "LIFTCMD return to %s -> raise + sweep to %.1f (gated at %.1f, catch %.1f), "
        "lower to %.0f, then settle bar to %.1f / EE to %.1f",
        mode_name(return_dest_),
        to_match ? CHAINBAR_MATCHLOAD_DEG : CHAINBAR_LOWER_CLEAR_DEG,
        CHAINBAR_LOW_ZONE_DEG,
        to_match ? CHAINBAR_MATCHLOAD_DEG + (CHAINBAR_COAST_CATCH_DEG - CHAINBAR_LOWER_CLEAR_DEG)
                 : CHAINBAR_COAST_CATCH_DEG,
        to_match ? WINCH_MATCHLOAD_POS : WINCH_LOADING_POS,
        to_match ? CHAINBAR_MATCHLOAD_DEG : CHAINBAR_LOADING_DEG,
        to_match ? CHAIN_MATCHLOAD_DEG : CHAIN_LOADING_DEG);
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
    begin_return(Mode::STOW);
}

void Lift::return_to_matchload() {
    begin_return(Mode::MATCHLOAD);
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
            // An ABSOLUTE height, not start-pose plus a delta: the swing room the bar
            // needs is a physical height, the same one every other clearance move
            // targets, and it does not shift just because the mechanism powered on
            // sitting lower. (This used to read WINCH_START_POS + ...; that only
            // agreed with the other clearance moves while WINCH_START_POS was 0, and
            // silently lowered this one the moment it was set to a real offset.)
            winch_manual_target_ = WINCH_START_CLEARANCE_RAISE;
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
            if (settle_bar_arrived(bar, CHAINBAR_LOADING_DEG)) {
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
            // Raise the lift for clearance; hold the bar at the pose the prime was
            // started from (loading or matchload) while the mechanism is still down
            // low, so the bar never detours between the two resting angles on the
            // way. The end effector tucks to CHAIN_PRIME_RAISE_DEG for the climb
            // rather than staying at the resting orientation.
            winch_manual_target_ = WINCH_PRIME_CLEARANCE_POS;
            bar_target_deg_ = entry_bar_deg_;
            chain_target_deg_ = CHAIN_PRIME_RAISE_DEG;
            // Hand over to TUCK part way up. A plain threshold, not the arrival
            // tolerance the pose steps use: this is a height the cascade passes
            // THROUGH on its way to the clearance height, not one it stops at, so
            // there is nothing to be tolerant of -- and the winch target below is
            // unchanged across the hand-over, which is what keeps the climb going.
            if (winch >= WINCH_PRIME_TUCK_POS) {
                prime_step_ = PrimeStep::TUCK;
                chainbar_pid_.reset();  // hold -> profiled move: start the PID clean
            }
            break;
        }
        case PrimeStep::TUCK: {
            // Same winch target as RAISE, so the lift carries straight on up to the
            // clearance height -- the bar and the end effector move onto the tuck
            // pose UNDERNEATH it, not after it.
            //
            // The bar move is SLEWED, like the matchload hop and the final settles:
            // it is a short (~40 deg) move onto a pose, climbing away from the
            // loading angle, so it wants the profiled / feed-forward regime rather
            // than the step-target law the long sweeps run on. If the lift reaches
            // the clearance height before the bar lands, SWEEP_LOWER simply takes
            // over mid-move and carries the target on in the SAME direction (round to
            // primed) -- there is no pose here that has to be arrived at first, which
            // is why the exit below tests the winch only.
            winch_manual_target_ = WINCH_PRIME_CLEARANCE_POS;
            bar_target_deg_ = slew_bar_toward(bar_target_deg_, CHAINBAR_PRIME_TUCK_DEG, bar,
                                              bar_target_vel_dps_);
            chain_target_deg_ = CHAIN_PRIME_TUCK_DEG;
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
            //
            // SLEWED, at its own rate: this is the one long sweep that has to LAND on
            // a pose. It used to be a step target with nothing but a drive cap between
            // it and the rail, so its speed was whatever the cap happened to allow and
            // its arrival was whatever the shrinking position error happened to leave
            // (it reached 516 deg/s on the gravity-fed half and rang for ~150 ms on
            // arrival, log 07262026-2). Running it on the profiled law instead makes
            // the speed COMMANDED -- tracked by the velocity feed-forward -- and tapers
            // it over CHAINBAR_PRIME_SWEEP_DECEL_DEG so the bar decelerates into the
            // primed pose. Its own rate, not the CHAINBAR_APPROACH_* defaults: those
            // are sized for short hops onto a pose (~40 deg), and this is a ~265 deg
            // sweep that would crawl at them.
            //
            // TWO rates, split at the top of the swing -- the same split the drive cap
            // makes, because it is the same distinction. CLIMBING (above
            // CHAINBAR_PRIME_CAP_START_DEG) the bar is fighting gravity with nothing to
            // arrive at, so it is commanded faster than it can go: the profile
            // saturates, the target sits at its full lead, and the bar climbs on
            // essentially the whole rail, as it did before this sweep was profiled.
            // Past the top gravity is feeding the move and there IS a pose to land on,
            // so the rate drops to the one the taper decelerates from. The handover
            // needs no special case: the commanded rate simply falls, the bar outruns
            // its target for a moment, and the resulting braking command (uncapped, by
            // design) is what sheds the speed.
            //
            // The gravity feed-forward, which the primed pose needs (gravity pulls the
            // bar further negative there, so without it the bar settles past the
            // target and the step never completes), is added either way -- it is
            // outside the velocity/breakaway split in update(). What the profile
            // replaces is the BREAKAWAY term, and it does so only while it is actually
            // commanding motion.
            winch_manual_target_ = bar_clear_of_low_zone(bar) ? primed_winch
                                                             : WINCH_PRIME_CLEARANCE_POS;
            double sweep_dps = bar > CHAINBAR_PRIME_CAP_START_DEG
                                   ? CHAINBAR_PRIME_CLIMB_DPS
                                   : CHAINBAR_PRIME_SWEEP_DPS;
            bar_target_deg_ = slew_bar_toward(bar_target_deg_, primed_bar, bar,
                                              bar_target_vel_dps_, sweep_dps,
                                              CHAINBAR_PRIME_SWEEP_DECEL_DEG);
            // Stay tucked at the raise orientation until the bar is far enough round
            // to flip to the raised one -- going back to CHAIN_LOADING_DEG here would
            // swing the EE back to 29 for a moment and then straight on to scoring.
            chain_target_deg_ = CHAIN_PRIME_RAISE_DEG;
            if (bar < CHAIN_FLIP_BAR_DEG){
                chain_target_deg_ = raised_ee_deg();
            }
            if (std::abs(bar - primed_bar) < CHAINBAR_POS_TOL &&
                std::abs(winch - primed_winch) < WINCH_POS_TOL) {
                // Land the target ON the pose before handing over. The sequence holds
                // whatever bar target it inherits until the driver drives it (see the
                // SEQUENCE case: it only writes bar_target_deg_ while advancing /
                // retreating, or on the release edge), and the slewed target may still
                // be a couple of degrees short inside the arrival tolerance -- which
                // the bar would then sit at indefinitely. The old step target was
                // always exactly primed_bar, so this was free.
                bar_target_deg_ = primed_bar;
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
    // One choreography, two destinations. Matchload sits PART WAY round the return
    // travel (-45 vs the -1 the loading return carries on to), so returning there is
    // a shorter sweep that stops early -- every step below just aims at a different
    // end pose. The low-clearance gate, the coast and the feed-forward finish are
    // the same machinery either way; sweep_catch_angle() keeps the hand-off the same
    // distance short of whichever pose is being aimed at.
    const bool to_match = return_dest_ == Mode::MATCHLOAD;
    const double sweep_goal = to_match ? CHAINBAR_MATCHLOAD_DEG : CHAINBAR_LOWER_CLEAR_DEG;
    const double rest_bar   = to_match ? CHAINBAR_MATCHLOAD_DEG : CHAINBAR_LOADING_DEG;
    const double rest_winch = to_match ? WINCH_MATCHLOAD_POS : WINCH_LOADING_POS;
    const double rest_ee    = to_match ? CHAIN_MATCHLOAD_DEG : CHAIN_LOADING_DEG;
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
            // The loading sweep lands just PAST loading (CHAINBAR_LOWER_CLEAR_DEG) so
            // the bar is clear of the cascade for the descent in the next step; the
            // matchload sweep stops on its pose instead and descends on that.
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
            double goal = lift_clear ? sweep_goal
                                     : std::max(bar_target_deg_, CHAINBAR_LOW_ZONE_DEG);
            bar_target_deg_ = goal;
            chain_target_deg_ = raised_ee_deg();
            if (bar > CHAIN_FLIP_BAR_DEG){
                chain_target_deg_ = rest_ee;
            }
            if (std::abs(bar - sweep_goal) < CHAINBAR_POS_TOL) {
                return_step_ = ReturnStep::LOWER;
            }
            break;
        }
        case ReturnStep::LOWER: {
            // Bar is parked on the sweep goal; lower the lift onto the destination
            // height with the bar held there. (For matchload that descends with the
            // bar inside the nominal low-clearance zone -- same assumption the
            // STOW->matchload choreography makes, see CHAINBAR_MATCHLOAD_DEG.)
            winch_manual_target_ = rest_winch;
            bar_target_deg_ = sweep_goal;
            chain_target_deg_ = CHAIN_PRIME_RAISE_DEG;
            if (std::abs(winch - rest_winch) < WINCH_POS_TOL) {
                return_step_ = ReturnStep::BAR_SETTLE;
                chainbar_pid_.reset();
            }
            break;
        }
        case ReturnStep::BAR_SETTLE: {
            // Lift is down: bring the bar the last few degrees onto the resting
            // angle and turn the EE to its resting orientation, then settle in the
            // destination mode -- which is what arms the winch homing/respool pass,
            // so the respool always runs with the bar already parked on a pose.
            // Returning to matchload the sweep already stopped ON the pose, so this
            // is a no-op move there and only the EE turn is left.
            winch_manual_target_ = rest_winch;
            bar_target_deg_ = slew_bar_toward(bar_target_deg_, rest_bar, bar,
                                              bar_target_vel_dps_);
            chain_target_deg_ = rest_ee;
            if (settle_bar_arrived(bar, rest_bar)) {
                mode_ = return_dest_;
            }
            break;
        }
    }
    bool arrived = mode_ == Mode::STOW || mode_ == Mode::MATCHLOAD;
    if (return_step_ != step_before || arrived) {
        Logger::getInstance()->log("LIFTEVT return step %s -> %s (bar=%.1f winch=%.1f)",
                                   return_step_name(step_before),
                                   arrived ? mode_name(mode_) : return_step_name(return_step_),
                                   bar, winch);
    }
}

void Lift::advance_matchload_sequence(double bar, double winch) {
    MatchloadStep step_before = matchload_step_;
    switch (matchload_step_) {
        case MatchloadStep::RAISE: {
            // Cascade up first. The bar has to cross the low-clearance zone to reach
            // the matchload angle, so nothing swings until the lift is clear -- same
            // interlock the prime and startup choreographies open with, but only to
            // WINCH_MATCHLOAD_CLEARANCE_POS: this is a short arc at the loading end
            // of the travel, not a sweep round to the primed side, so it does not
            // need the full clearance height. Hold the bar where the pose left it and
            // tuck the EE for the climb.
            winch_manual_target_ = WINCH_MATCHLOAD_CLEARANCE_POS;
            bar_target_deg_ = entry_bar_deg_;
            chain_target_deg_ = CHAIN_PRIME_RAISE_DEG;
            if (winch >= WINCH_MATCHLOAD_CLEARANCE_POS - WINCH_POS_TOL) {
                matchload_step_ = MatchloadStep::BAR_SWEEP;
                chainbar_pid_.reset();
            }
            break;
        }
        case MatchloadStep::BAR_SWEEP: {
            // Swing onto the matchload pose with the cascade held up. SLEWED, not a
            // step target: this is a short move that has to LAND on a pose (~48 deg
            // from loading), and it climbs against gravity the whole way, so it is
            // the profiled/feed-forward regime -- the same one BAR_SETTLE uses -- not
            // the long FF-free sweeps. A step target here would arrive at speed with
            // nothing to decelerate it, which is exactly how the startup sweep
            // overran its pose (log 07262026-3).
            winch_manual_target_ = WINCH_MATCHLOAD_CLEARANCE_POS;
            bar_target_deg_ = slew_bar_toward(bar_target_deg_, CHAINBAR_MATCHLOAD_DEG, bar,
                                              bar_target_vel_dps_);
            chain_target_deg_ = CHAIN_MATCHLOAD_DEG;
            if (std::abs(bar - CHAINBAR_MATCHLOAD_DEG) < CHAINBAR_POS_TOL) {
                matchload_step_ = MatchloadStep::LOWER;
            }
            break;
        }
        case MatchloadStep::LOWER: {
            // Bar is on the pose; bring the cascade back down onto the matchload
            // height with it held there. (See CHAINBAR_MATCHLOAD_DEG: this descends
            // with the bar inside the nominal low-clearance zone, which is the one
            // assumption in this choreography that the geometry has to bear out.)
            winch_manual_target_ = WINCH_MATCHLOAD_POS;
            bar_target_deg_ = CHAINBAR_MATCHLOAD_DEG;
            chain_target_deg_ = CHAIN_MATCHLOAD_DEG;
            if (std::abs(winch - WINCH_MATCHLOAD_POS) < WINCH_POS_TOL) {
                mode_ = Mode::MATCHLOAD;
                claw_.set_state(CLAW_OPEN_STATE);  // resting pose accepts a piece
            }
            break;
        }
    }
    if (matchload_step_ != step_before || mode_ == Mode::MATCHLOAD) {
        Logger::getInstance()->log("LIFTEVT matchload step %s -> %s (bar=%.1f winch=%.1f)",
                                   matchload_step_name(step_before),
                                   mode_ == Mode::MATCHLOAD ? "MATCHLOAD"
                                                            : matchload_step_name(matchload_step_),
                                   bar, winch);
    }
}

void Lift::advance_back_sequence(double bar, double winch, double dt) {
    // One path onto either back pose, entering the cycle included: raise the cascade
    // clear, swing the bar (and EE) onto the pose, lower onto its height, hold. The
    // steps are ordered so that the direction takes care of itself --
    //
    //   BOTTOM -> TOP : RAISE does the work (the destination is ABOVE the clearance
    //                   height, so the climb just carries on through SWING and LOWER
    //                   finds itself already there),
    //   TOP -> BOTTOM : RAISE passes straight through (already at full extension),
    //                   the bar swings up high, and LOWER brings the cascade down onto
    //                   the low pose with the bar parked out of the way,
    //   entry         : the cascade comes up off the resting pose, the bar crosses to
    //                   the back angle, and the cascade drops onto the pose height.
    //
    // -- so there is one choreography here, not three, and the low-clearance interlock
    // is satisfied in all of them by construction: the bar only ever swings in SWING,
    // and SWING only runs once the cascade is at the clearance height.
    const double goal_bar = back_bar_deg();
    const double goal_winch = back_winch_pos();
    // Height to hold during RAISE and SWING: the clearance height, or the destination
    // if that is higher (going to TOP there is no reason to stop at 600 and set off
    // again). Never the destination when the destination is LOW -- that is what LOWER
    // is for, after the bar is out of the way.
    const double clear_winch = std::max(WINCH_CLEARANCE_POS, goal_winch);
    BackStep step_before = back_step_;
    switch (back_step_) {
        case BackStep::RAISE: {
            // Cascade up first, bar held wherever the previous pose left it. Same
            // interlock the matchload hop and the prime open with: nothing swings until
            // the lift is clear. The EE tucks for the climb rather than turning to the
            // back orientation down low.
            winch_manual_target_ = clear_winch;
            bar_target_deg_ = entry_bar_deg_;
            chain_target_deg_ = CHAIN_PRIME_RAISE_DEG;
            if (winch >= WINCH_CLEARANCE_POS - WINCH_POS_TOL) {
                back_step_ = BackStep::SWING;
                chainbar_pid_.reset();
            }
            break;
        }
        case BackStep::SWING: {
            // Swing onto the pose with the cascade held up. SLEWED, like the matchload
            // sweep and the final settle of the long ones: this is a move that has to
            // LAND on a pose (~105 deg between the two back poses), and it climbs
            // against gravity in one direction and descends in the other, so it wants
            // the profiled / feed-forward regime rather than the step-target law the
            // long sweeps use. The slew's decel taper is what keeps the downhill
            // direction from arriving hot.
            winch_manual_target_ = clear_winch;
            bar_target_deg_ = slew_bar_toward(bar_target_deg_, goal_bar, bar,
                                              bar_target_vel_dps_);
            chain_target_deg_ = raised_ee_deg();
            if (std::abs(bar - goal_bar) < CHAINBAR_POS_TOL) {
                back_step_ = BackStep::LOWER;
            }
            break;
        }
        case BackStep::LOWER: {
            // Bar is on the pose; bring the cascade onto the pose's height with it held
            // there. Going to TOP this is already satisfied and passes straight through.
            winch_manual_target_ = goal_winch;
            bar_target_deg_ = goal_bar;
            chain_target_deg_ = raised_ee_deg();
            if (std::abs(winch - goal_winch) < WINCH_POS_TOL) {
                back_step_ = BackStep::HOLD;
                // Open the hold on the pose itself, in its own state rather than read
                // off the pose each tick: the driver's cascade jog moves the held
                // height, so the hold has to be able to differ from the pose.
                back_hold_bar_ = goal_bar;
                back_hold_winch_ = goal_winch;
                winch_pid_.reset();
            }
            break;
        }
        case BackStep::HOLD: {
            // Parked. The bar and EE hold their pose; the CASCADE is the driver's, via
            // the same two buttons that walk the front raise sequence. The jog moves
            // the held height rather than adding a transient offset, so letting go
            // leaves the cascade where it was rather than springing back to the pose.
            // Raising wins if both are held, matching the sequence.
            if (advancing_) {
                back_hold_winch_ += WINCH_BACK_JOG_DPS * dt;
            } else if (retreating_) {
                back_hold_winch_ -= WINCH_BACK_JOG_DPS * dt;
            }
            back_hold_winch_ = std::clamp(back_hold_winch_, WINCH_DOWN_POS, WINCH_MAX_POS);
            winch_manual_target_ = back_hold_winch_;
            bar_target_deg_ = back_hold_bar_;
            chain_target_deg_ = raised_ee_deg();
            break;
        }
    }
    if (back_step_ != step_before) {
        Logger::getInstance()->log(
            "LIFTEVT back step %s -> %s (pose=%s bar=%.1f winch=%.1f)",
            back_step_name(step_before), back_step_name(back_step_),
            back_pose_name(back_pose_), bar, winch);
    }
}

void Lift::advance_score_state_sequence(double bar, double winch) {
    // The back cycle's choreography with the destination read from a table instead of
    // from BackPose -- raise the cascade clear, swing the bar (and EE) onto the pose,
    // lower onto its height, hold. Written as its own function rather than folded into
    // advance_back_sequence() because the two differ in what HOLD means (see below) and
    // the back cycle is tuned; the SHAPE is deliberately identical, and the reason it
    // has to be is the same: the bar may have to cross the low-clearance zone
    // (CHAINBAR_LOW_ZONE_DEG) to reach an arbitrary pose, and it cannot do that with the
    // cascade down. Swinging only in SWING, and only once the cascade is up, satisfies
    // that for every row in the table without the table having to know about it.
    //
    // As there, each step's arrival test passes straight through when it has nothing to
    // do -- a pose above the clearance height makes LOWER a no-op, a pose reached from
    // full extension makes RAISE one -- so one path covers every direction and every
    // entry mode.
    if (score_target_ == nullptr) {
        // Can't happen: run_score_state() is the only way into this mode and it always
        // sets the target. Bail to the loading pose rather than command garbage.
        Logger::getInstance()->logError("LIFTERR SCORE_STATE with no target -- returning to loading");
        begin_return(Mode::STOW);
        return;
    }
    const double goal_bar   = score_target_->bar_deg;
    const double goal_winch = score_target_->winch_pos;
    const double goal_ee    = score_target_->ee_deg;
    // Height to hold during RAISE and SWING: the clearance height, or the destination if
    // that is higher (going somewhere high there is no reason to stop at 600 and set off
    // again). Never the destination when the destination is LOW -- that is what LOWER is
    // for, after the bar is out of the way.
    const double clear_winch = std::max(WINCH_CLEARANCE_POS, goal_winch);
    ScoreStep step_before = score_step_;
    switch (score_step_) {
        case ScoreStep::RAISE: {
            // Cascade up first, bar held wherever the previous pose left it. The EE
            // tucks for the climb rather than turning to the pose's orientation down
            // low, exactly as the back cycle's RAISE does.
            winch_manual_target_ = clear_winch;
            bar_target_deg_ = entry_bar_deg_;
            chain_target_deg_ = CHAIN_PRIME_RAISE_DEG;
            if (winch >= WINCH_CLEARANCE_POS - WINCH_POS_TOL) {
                score_step_ = ScoreStep::SWING;
                chainbar_pid_.reset();
            }
            break;
        }
        case ScoreStep::SWING: {
            // Swing onto the pose with the cascade held up. SLEWED: this is a move that
            // has to LAND on a pose, and depending on the row it climbs against gravity
            // or descends with it, so it wants the profiled / feed-forward regime rather
            // than the step-target law the long sweeps use. The decel taper is what
            // keeps the downhill direction from arriving hot.
            winch_manual_target_ = clear_winch;
            bar_target_deg_ = slew_bar_toward(bar_target_deg_, goal_bar, bar,
                                              bar_target_vel_dps_);
            // The pose's OWN end-effector angle, not raised_ee_deg(): a score state
            // names its orientation outright, so the driver's flip selection does not
            // apply to it (and score() will not read ee_alt_ here either).
            chain_target_deg_ = goal_ee;
            if (std::abs(bar - goal_bar) < CHAINBAR_POS_TOL) {
                score_step_ = ScoreStep::LOWER;
            }
            break;
        }
        case ScoreStep::LOWER: {
            // Bar is on the pose; bring the cascade onto the pose's height with it held
            // there. For a pose at or above the clearance height this is already
            // satisfied and passes straight through.
            winch_manual_target_ = goal_winch;
            bar_target_deg_ = goal_bar;
            chain_target_deg_ = goal_ee;
            if (std::abs(winch - goal_winch) < WINCH_POS_TOL) {
                score_step_ = ScoreStep::HOLD;
                winch_pid_.reset();
            }
            break;
        }
        case ScoreStep::HOLD: {
            // Arrived. Unlike the back cycle's HOLD there is no cascade jog: a score
            // state is commanded by autonomous, which has no buttons, so advancing_ /
            // retreating_ are ignored and all three joints simply hold the pose. This is
            // where is_settled() goes true and where score() is armed.
            winch_manual_target_ = goal_winch;
            bar_target_deg_ = goal_bar;
            chain_target_deg_ = goal_ee;
            break;
        }
    }
    if (score_step_ != step_before) {
        Logger::getInstance()->log(
            "LIFTEVT score_state step %s -> %s (state=%s bar=%.1f winch=%.1f)",
            score_step_name(step_before), score_step_name(score_step_),
            score_target_->key, bar, winch);
    }
}

// ---- Helpers ---------------------------------------------------------------

double Lift::unwrap_angle_near(double raw_deg, double expected_deg) {
    double turns = std::round((raw_deg - expected_deg) / 360.0);
    return raw_deg - turns * 360.0;
}

double Lift::apply_soft_limit(double output_mv, double angle,
                              double min_deg, double max_deg) {
    if (angle >= max_deg && output_mv > 0.0) return 0.0;  // can't rotate further up
    if (angle <= min_deg && output_mv < 0.0) return 0.0;  // can't rotate further down
    return output_mv;
}

double Lift::raised_ee_deg() const {
    // The flip button stores a selection, the cycle supplies the pair. Nothing else in
    // the file should ever compare an EE angle to decide "is the alternate selected" --
    // ee_alt_ is that answer, in both families.
    if (ee_family_ == EeFamily::BACK) {
        return ee_alt_ ? CHAIN_BACK_ALT_DEG : CHAIN_BACK_DEG;
    }
    return ee_alt_ ? CHAIN_SCORING_ALT_DEG : CHAIN_SCORING_DEG;
}

double Lift::back_bar_deg() const {
    return back_pose_ == BackPose::TOP ? CHAINBAR_BACK_TOP_DEG : CHAINBAR_BACK_BOTTOM_DEG;
}

double Lift::back_winch_pos() const {
    return back_pose_ == BackPose::TOP ? WINCH_BACK_TOP_POS : WINCH_BACK_BOTTOM_POS;
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
           bar < sweep_catch_angle() &&
           winch >= WINCH_CLEARANCE_POS - WINCH_POS_TOL;
}

bool Lift::alt_return_hold_active(double bar) const {
    // Only the ALTERNATE raised orientation needs this: coming back from a score in
    // that orientation the end effector is flipped half a turn, so until the bar has
    // swung round to CHAIN_FLIP_BAR_DEG it is neither safe to drive the chain (it
    // would rotate the EE through the structure) nor to bring the cascade down on
    // top of it. The default orientation returns as it always has.
    return mode_ == Mode::RETURNING &&
           ee_alt_ &&
           bar < CHAIN_FLIP_BAR_DEG;
}

double Lift::sweep_catch_angle() const {
    // The catch is a fixed LEAD-IN short of whatever the sweep is aiming at, not a
    // fixed angle: -10 was only ever "9 deg before the -1 loading goal". Expressing
    // it as that distance lets the matchload return hand over 9 deg before ITS pose
    // (-54) instead of blowing past it and stalling short on P alone, which is what
    // the loading return used to do before the hand-off existed (log 07262026-2).
    constexpr double LEAD_IN = CHAINBAR_COAST_CATCH_DEG - CHAINBAR_LOWER_CLEAR_DEG;
    if (mode_ == Mode::RETURNING && return_dest_ == Mode::MATCHLOAD) {
        return CHAINBAR_MATCHLOAD_DEG + LEAD_IN;
    }
    return CHAINBAR_COAST_CATCH_DEG;
}

bool Lift::settle_bar_arrived(double bar, double rest_bar) const {
    // The settle onto loading is only (CHAINBAR_LOADING_DEG - CHAINBAR_LOWER_CLEAR_DEG)
    // = 4 deg -- already inside CHAINBAR_POS_TOL -- so a position-only check would
    // pass on the very first tick and skip the settle entirely. Also require the
    // SLEWED TARGET to have arrived at @p rest_bar, so the move is always commanded
    // in full. (Nothing can hold the target back here: the return path slews
    // unclamped, and the startup path's lead clamp only binds past
    // CHAINBAR_MAX_LEAD_DEG of lag, which a 4 deg move starting inside the tolerance
    // cannot reach. So this always completes. Returning to matchload the sweep
    // already stopped on the pose, so both halves are true on the first tick and the
    // step passes straight through -- which is correct, there is nothing to settle.)
    bool target_arrived = std::abs(bar_target_deg_ - rest_bar) < 1e-3;
    return target_arrived && std::abs(bar - rest_bar) < CHAINBAR_POS_TOL;
}

const char* Lift::mode_name(Mode m) {
    switch (m) {
        case Mode::STARTUP:  return "STARTUP";
        case Mode::STOW:     return "STOW";
        case Mode::MATCHLOAD:    return "MATCHLD";
        case Mode::TO_MATCHLOAD: return "TOMATCH";
        case Mode::PRIMING:  return "PRIMING";
        case Mode::SEQUENCE: return "SEQ";
        case Mode::BACK:     return "BACK";
        case Mode::SCORE_STATE: return "SCORESTATE";
        case Mode::SCORE_RETREAT: return "RETREAT";
        case Mode::RETURNING: return "RETURN";
    }
    return "?";
}

const char* Lift::prime_step_name(PrimeStep s) {
    switch (s) {
        case PrimeStep::RAISE:       return "RAISE";
        case PrimeStep::TUCK:        return "TUCK";
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

const char* Lift::matchload_step_name(MatchloadStep s) {
    switch (s) {
        case MatchloadStep::RAISE:     return "RAISE";
        case MatchloadStep::BAR_SWEEP: return "BAR_SWEEP";
        case MatchloadStep::LOWER:     return "LOWER";
    }
    return "?";
}

const char* Lift::back_step_name(BackStep s) {
    switch (s) {
        case BackStep::RAISE: return "RAISE";
        case BackStep::SWING: return "SWING";
        case BackStep::LOWER: return "LOWER";
        case BackStep::HOLD:  return "HOLD";
    }
    return "?";
}

const char* Lift::score_step_name(ScoreStep s) {
    switch (s) {
        case ScoreStep::RAISE: return "RAISE";
        case ScoreStep::SWING: return "SWING";
        case ScoreStep::LOWER: return "LOWER";
        case ScoreStep::HOLD:  return "HOLD";
    }
    return "?";
}

const char* Lift::back_pose_name(BackPose p) {
    switch (p) {
        case BackPose::BOTTOM: return "BOTTOM";
        case BackPose::TOP:    return "TOP";
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
    // Computed here, before the mode switch, because the drive-cap block and the
    // retreat's park both read it -- and the raw sensor velocity they would otherwise
    // use is the aliased one this window exists to replace.
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
    // Drive cap (mV) the bar-only sequence swing wants this tick, or 0 for "not a
    // capped swing". That swing is a big step target with nothing but the cap setting
    // its speed, and the cap is DIRECTIONAL: SEQ_SWING_UP_MAX_MV climbing out of the
    // primed pose, SEQ_SWING_DOWN_MAX_MV coming back down -- which is the direction a
    // held piece comes down on whatever is under it, hence the much softer limit.
    // Applied in the drive-cap block further down.
    double bar_swing_cap_mv = 0.0;
    // Set by the parked score retreat: hand the bar to its brake instead of driving
    // it (see the SCORE_RETREAT case and SCORE_RETREAT_PARK_DEG).
    bool bar_park_hold = false;
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
        case Mode::MATCHLOAD:
            // The other resting pose. Plain PID hold, exactly like STOW -- prime()
            // and toggle_matchload() are the only ways out.
            winch_manual_target_ = WINCH_MATCHLOAD_POS;
            bar_target_deg_ = CHAINBAR_MATCHLOAD_DEG;
            chain_target_deg_ = CHAIN_MATCHLOAD_DEG;
            break;
        case Mode::TO_MATCHLOAD:
            // Clearance choreography onto the matchload pose: raise, swing, lower.
            advance_matchload_sequence(bar, winch);
            break;
        case Mode::BACK:
            // The back cycle: the raise/swing/lower move onto the selected pose, then
            // the hold that keeps it there with the cascade under the driver's thumb.
            advance_back_sequence(bar, winch, dt);
            break;
        case Mode::SCORE_STATE:
            // The autonomous counterpart of the back cycle: the same raise/swing/lower
            // move onto a pose, but the pose comes from a table and the hold is plain
            // (no cascade jog -- there is no driver here).
            advance_score_state_sequence(bar, winch);
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
        case Mode::SEQUENCE: {
            // Two movements, driven differently because only one of them couples two
            // actuators:
            //
            //  1. KF0<->KF1 is the bar alone. Nothing has to stay in step with it, so
            //     it is NOT walked on the clock -- the target is simply the end of
            //     the swing and the position PID takes the bar there, speed set by
            //     SEQ_SWING_UP_MAX_MV / SEQ_SWING_DOWN_MAX_MV (the cap is directional:
            //     the way back down brings a held piece onto whatever is under it, so
            //     it is far softer). raise_progress_ is read back off the bar so it
            //     still means something, and movement 2 starts when the bar has
            //     really arrived rather than when a timer says so.
            //  2. KF1<->KF2 runs the cascade and the bar together, so it keeps the
            //     timed interpolation -- that is the only way to hold their relative
            //     positions through the move.
            //
            // Raising wins if both buttons are held; releasing freezes wherever it
            // is. The junction is unambiguous: while advancing, progress 1.0 belongs
            // to movement 2; otherwise it belongs to the swing, so retreating out of
            // movement 2 flows straight back into it. Both keyframes sit at the same
            // winch height, so nothing steps at the boundary.
            bool in_swing = advancing_ ? (raise_progress_ < 1.0) : (raise_progress_ <= 1.0);
            if (in_swing) {
                winch_manual_target_ = SEQ_WINCH_DEG[0];
                bool driving = advancing_ || retreating_;
                if (driving) {
                    bar_target_deg_ = advancing_ ? SEQ_BAR_DEG[1] : SEQ_BAR_DEG[0];
                } else if (seq_swing_driving_) {
                    bar_target_deg_ = bar;  // just released: hold this angle
                }
                seq_swing_driving_ = driving;
                // Directional cap: the swing out to KF1 is a climb, the swing back to
                // KF0 is the gravity-fed one a held piece comes down on.
                // advancing_ wins if both buttons are held, exactly as the target does.
                bar_swing_cap_mv = !driving           ? 0.0
                                 : advancing_         ? SEQ_SWING_UP_MAX_MV
                                                      : SEQ_SWING_DOWN_MAX_MV;
                raise_progress_ = seq_swing_progress(bar);
                if (advancing_ && std::abs(bar - SEQ_BAR_DEG[1]) < CHAINBAR_POS_TOL) {
                    raise_progress_ = 1.0;  // arrived: hand over to movement 2
                }
            } else {
                seq_swing_driving_ = false;
                if (advancing_) {
                    raise_progress_ = std::min(raise_progress_ + SEQ_RATE_PER_S * dt,
                                               static_cast<double>(SEQ_KEYFRAMES - 1));
                } else if (retreating_) {
                    raise_progress_ = std::max(raise_progress_ - SEQ_RATE_PER_S * dt, 1.0);
                }
                seq_targets(raise_progress_, winch_manual_target_, bar_target_deg_);
            }
            chain_target_deg_ = raised_ee_deg();
            break;
        }
        case Mode::SCORE_RETREAT: {
            // The alternate-orientation tail of the score macro (see
            // SCORE_ALT_RETREAT_BAR_DEG). The piece has been released, but the flipped
            // end effector is still in the structure it was dropped into, so the bar
            // swings back up to lift it out rather than holding the pose. The default
            // orientation never enters this mode -- there the score moves nothing.
            //
            // chain_target_deg_ deliberately keeps the value the score was released
            // at, and the cascade only ever RISES here: with the
            // EE flipped and the bar still on the far side of the swing this is the one
            // stretch of travel where the cascade must not be brought down on top of it
            // and the chain must not be turned through the structure -- the same
            // interlock alt_return_hold_active() enforces on the return. So the winch
            // holds whatever height the release happened at -- anywhere in the raise
            // sequence, or a back pose plus the driver's jog -- and the EE holds the
            // orientation it was released in.
            //
            // Driven like movement 1 of the raise sequence, which is the same travel
            // 50 deg shorter: a step target under the position PID, on that swing's UP
            // cap -- it climbs the same way out of the primed pose, so it takes the
            // same limit rather than the soft scoring one, which could not lift it.
            // Uncapped this is a ~125 deg step over the top of the swing at the full
            // rail. The gravity
            // feed-forward is on the bar's side the whole way -- it points up-swing
            // across -305..-180 and crosses zero exactly at the retreat pose.
            //
            // ARRIVAL IS A PARK, NOT A HOLD (see SCORE_RETREAT_PARK_DEG). The retreat
            // angle is straight up, so there is no gravity restoring term to settle
            // against and the drive itself was feeding a +/-20 deg limit cycle. Once
            // the bar is inside the park band the command is cut to 0 and the motor's
            // brake HOLD takes the pose instead; the PID only comes back if the bar
            // escapes past SCORE_RETREAT_WAKE_DEG, and re-parks on the way through.
            //
            // It ENDS here: the bar stays at the retreat angle and the mode does not
            // change itself. The driver picks the next move -- Y/X (return to loading /
            // matchload) both work from this mode, and the retreat pose is where that
            // return's alternate-orientation hold releases, so it comes down cleanly.
            // The raise/lower buttons are inert: the bar is parked outside the
            // sequence's own travel and re-raising a flipped EE from here is not a
            // motion the sequence can express.
            // CLEARANCE FIRST, if the retreat has to cross the low-clearance zone.
            // A FRONT release happens between -305 and -230, well clear of it, and
            // this is inert there. The BACK poses sit inside the zone
            // (CHAINBAR_BACK_BOTTOM_DEG), so the swing to -180 would otherwise drag the
            // bar across the zone at whatever height the pose was released at -- as low
            // as WINCH_BACK_BOTTOM_POS. There is no waiting at the zone edge to be done
            // here the way the return sweep does it: the bar is already INSIDE the zone,
            // so the only safe move is not to move it at all until the cascade is up.
            // The cascade may only RISE for this, never fall -- the EE is flipped.
            // LIFT FIRST, always: rise WINCH_ALT_RETREAT_LIFT_DEG off the height the
            // piece was released at (latched, and already clamped to the travel, by
            // score()) so the swing carries the flipped EE up and out of the structure
            // instead of round through it. The bar holds where it was released until
            // the cascade has arrived.
            winch_manual_target_ = retreat_winch_target_;
            bool lifted = winch >= retreat_winch_target_ - WINCH_POS_TOL;
            bool lift_clear = winch >= WINCH_CLEARANCE_POS - WINCH_POS_TOL;
            bool wait_for_clearance = !lift_clear && !bar_clear_of_low_zone(bar);
            if (wait_for_clearance) {
                winch_manual_target_ = std::max(winch_manual_target_, WINCH_CLEARANCE_POS);
            }
            // While waiting -- for the lift, or for clearance -- sit on the angle the
            // retreat started at (entry_bar_deg_, latched by score()) rather than on
            // the pose the sequence was holding.
            bar_target_deg_ = (wait_for_clearance || !lifted) ? entry_bar_deg_
                                                              : SCORE_ALT_RETREAT_BAR_DEG;
            bar_swing_cap_mv = SEQ_SWING_UP_MAX_MV;
            double retreat_err = std::abs(bar - SCORE_ALT_RETREAT_BAR_DEG);
            bool park_now = score_retreat_parked_ ? (retreat_err <= SCORE_RETREAT_WAKE_DEG)
                                                  : (retreat_err <= SCORE_RETREAT_PARK_DEG);
            if (park_now != score_retreat_parked_) {
                // Brake mode is already HOLD here (nothing in this mode coasts), but
                // the park's whole behaviour rests on it, so state it on the edge
                // rather than inheriting it. A zero command is only a hold if the
                // brake says so -- under COAST the same 0 mV would let the bar fall.
                chainbar_motor_.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
                Logger::getInstance()->log(
                    "LIFTEVT score retreat %s @ bar=%.1f (tgt %.1f err=%.1f vel=%.1f) -- "
                    "%s",
                    park_now ? "PARK" : "WAKE", bar, SCORE_ALT_RETREAT_BAR_DEG,
                    retreat_err, bar_vel_dps,
                    park_now ? "drive cut, brake HOLD" : "escaped the park band, driving back");
                score_retreat_parked_ = park_now;
            }
            bar_park_hold = score_retreat_parked_;
            break;
        }
    }
    advancing_ = false;
    retreating_ = false;

    // Clamp the winch command to its physical travel. The floor is WINCH_TARGET_FLOOR,
    // not WINCH_DOWN_POS: with a negative WINCH_START_POS the cascade powers on below
    // the taut bottom (that is what the paid-out slack means), and clamping to 0 there
    // would command the whole offset away on the very first tick -- at winch kP=300 a
    // railed 12000 mV snatch on a slack string. Letting the target sit at the boot
    // reading instead means the startup choreography's own RAISE takes the slack up
    // under a real profile. score_table_valid() checks table rows against the same
    // bound, so a pose that lands here is one the clamp will honour untouched.
    winch_manual_target_ = std::clamp(winch_manual_target_, WINCH_TARGET_FLOOR, WINCH_MAX_POS);
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

    // MATCHLOAD counts alongside STOW here: it is a settled resting pose at the same
    // bottom height, so a robot parked in it should still get its homing pass rather
    // than running on a stale winch reference until someone goes back to loading.
    bool commanded_down = WINCH_STOW_HOMING_ENABLED &&
                          (mode_ == Mode::STOW || mode_ == Mode::MATCHLOAD) &&
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
        seek_start_pos_ = winch;
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
        // Measured from where the seek STARTED, not from the taut bottom. Now that the
        // frame is seeded, a grounded lift with slack out genuinely sits a long way
        // below WINCH_DOWN_POS -- around -240 in tests/logs/07282026.txt -- so an
        // absolute backstop at WINCH_DOWN_POS - 250 would be tripped before the pre-roll
        // had paid out a single degree. What the backstop is actually for is bounding
        // how much NEW slack one seek can pay out, which is a relative quantity.
        bool over_travel = winch <= seek_start_pos_ - WINCH_SEEK_MAX_TRAVEL;
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
            // Shift the SOFTWARE reference so get_winch_position() reads rezero_to
            // here. This used to be set_zero_position_all(rezero_to), which silently
            // collapsed to ~0 whatever it was given, because one winch motor is
            // reversed -- see winch_ref_offset_. That is why every capped re-zero in
            // the logs still ended up at 0 instead of at the capped value it printed:
            // the cap has never actually bound. It does now.
            winch_ref_offset_ += rezero_to - winch;
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
    // Same config trap as the chain's clamp below -- see the comment there.
    static bool bar_clamp_warned = false;
    if (std::abs(bar_cmd - bar_target_deg_) > 1e-3) {
        if (!bar_clamp_warned) {
            Logger::getInstance()->log(
                "LIFTWARN bar target %.1f is OUTSIDE the soft limits [%.0f, %.0f] -- "
                "commanding %.1f instead; the bar will hold at the limit and fight the "
                "sequence. Fix CHAINBAR_MIN_DEG/CHAINBAR_MAX_DEG or the pose constants.",
                bar_target_deg_, CHAINBAR_MIN_DEG, CHAINBAR_MAX_DEG, bar_cmd);
            bar_clamp_warned = true;
        }
    } else {
        bar_clamp_warned = false;
    }
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
    bool bar_sweep = bar_sweeping() && bar < sweep_catch_angle();
    if (bar_park_hold) {
        // Parked score retreat: NO command at all, so the brake (HOLD, written on the
        // park edge) is what holds the bar. Not "PID with a small target error" -- at
        // the top of the swing that is precisely what oscillated, since P and the
        // breakaway feed-forward both grow as the bar drifts and neither has a gravity
        // restoring term working with it. The PID is held in reset so no integral
        // accumulates against a pose it is not driving, and so it re-engages clean if
        // the bar ever escapes the park band.
        bar_mv = 0.0;
        chainbar_pid_.reset();
    } else if (bar_coasting || bar_sweep) {
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
    // The sequence swing's cap gets the same break-out exemption, but LATCHED for the
    // swing rather than re-tested every tick. It has to be latched: the down cap
    // (SEQ_SWING_DOWN_MAX_MV = 2000) is below the bar's breakaway command, so keying it
    // on bar_moving alone would lift the limit again the moment the piece met the tower
    // and the bar stopped -- releasing the full rail into the tower for the contact
    // debounce and the whole settle, which is the one place the soft cap is FOR. Keying
    // it on nothing at all is not an option either: a sub-breakaway cap cannot start
    // the bar out of its hold at all, so the swing would never leave the pose.
    // Full authority to break out, then the cap for the rest of the swing.
    if (bar_swing_cap_mv <= 0.0) {
        bar_swing_broke_out_ = false;  // no capped swing running: re-arm for the next
    } else if (bar_moving) {
        bar_swing_broke_out_ = true;
    }
    double bar_drive_cap = 12000.0;  // no cap outside the sweeps
    if (bar_swing_cap_mv > 0.0) {
        bool needs_break_out = bar_swing_cap_mv < CHAINBAR_KS_MV && !bar_swing_broke_out_;
        bar_drive_cap = needs_break_out ? 12000.0 : bar_swing_cap_mv;
    } else if (bar_coasting || bar_sweep) {
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
        // A BACKSTOP now, not the speed knob: the sweep is slewed
        // (CHAINBAR_PRIME_SWEEP_DPS), so its speed is commanded by the velocity
        // feed-forward rather than set by whatever this allows. It still applies to
        // the FF-inclusive command, and a braking one is still exempt -- on the
        // gravity-fed half the profile is mostly holding the bar BACK, which is
        // exactly the command that must not be capped.
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
    // A pose the soft limits cannot express is a CONFIG error, and a quiet one: the
    // clamp becomes the setpoint, so the EE drives to the limit and then holds there
    // against the sequence -- which looks exactly like a reversed motor from the
    // outside (log 07262026-4, scoring -260 clamped to -110). The LIFTDBG line logs
    // the raw target, so nothing in it shows the substitution. Say so instead, once
    // per onset rather than every tick.
    static bool chain_clamp_warned = false;
    if (std::abs(chain_cmd - chain_target_deg_) > 1e-3) {
        if (!chain_clamp_warned) {
            Logger::getInstance()->log(
                "LIFTWARN chain target %.1f is OUTSIDE the soft limits [%.0f, %.0f] -- "
                "commanding %.1f instead; the EE will hold at the limit and fight the "
                "sequence. Fix CHAIN_MIN_DEG/CHAIN_MAX_DEG or the pose constants.",
                chain_target_deg_, CHAIN_MIN_DEG, CHAIN_MAX_DEG, chain_cmd);
            chain_clamp_warned = true;
        }
    } else {
        chain_clamp_warned = false;  // re-arm for the next out-of-range pose
    }
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
    // (Those two call sites are opcontrol()'s loop in main.cpp and
    // Robot::followTrajectory / Robot::waitForLift in src/hardware/robot.cpp.)
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
