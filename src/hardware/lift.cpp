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
    // Log then sleep, so the async logger flushes each line to SD before the next motor
    // op: the last LIFTINIT line in a frozen log names the call before the one that hung.
    Logger* dbg = Logger::getInstance();
    auto probe = [&](const char* msg) {
        dbg->log("LIFTINIT: %s", msg);
        pros::delay(30);
    };
    probe("enter; flushing backlog");
    pros::delay(80);  // let any earlier init backlog drain before we probe

    // The winch reads from the motor IMEs, which come up at 0 and drift, so it is seeded
    // here. The bar and EE keep their reading across a power cycle and are NOT re-zeroed,
    // but their turn count needs resolving once, since the sensor folds into [0, 360).
    winch_motors_.set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);
    probe("winch set_encoder_units_all ok");
    winch_motors_.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    probe("winch set_brake_mode_all ok");
    // Zero the encoders and hold the reference in software: 0 is the one value
    // set_zero_position_all() gets right on this reversed group (see winch_ref_offset_).
    winch_motors_.set_zero_position_all(0.0);
    winch_ref_offset_ = WINCH_START_POS;
    probe("winch encoders zeroed, reference seeded");

    chainbar_motor_.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    probe("chainbar set_brake_mode ok");
    // Orient the bar's axle sensor, then resolve its turn count.
    chainbar_rotation_.set_reversed(CHAINBAR_ROTATION_REVERSED);
    probe("chainbar rotation set_reversed ok");
    double bar_raw = chainbar_rotation_.get_position() / 100.0;
    double bar_seed = unwrap_angle_near(bar_raw, CHAINBAR_START_DEG);
    chainbar_rotation_.set_position(static_cast<std::int32_t>(bar_seed * 100.0));
    probe("chainbar rotation turn count resolved");

    chain_motor_.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    probe("chain set_brake_mode ok");
    // Same for the EE's axle sensor -- nothing here touches the chain motor's encoder.
    chain_rotation_.set_reversed(CHAIN_ROTATION_REVERSED);
    probe("chain rotation set_reversed ok");
    double ee_raw = chain_rotation_.get_position() / 100.0;
    double ee_seed = unwrap_angle_near(ee_raw, CHAIN_START_DEG);
    chain_rotation_.set_position(static_cast<std::int32_t>(ee_seed * 100.0));
    probe("chain rotation turn count resolved");

    // Start in the power-on START pose. CHAINBAR_START_DEG / CHAIN_START_DEG only
    // describe it -- nothing writes them into a sensor.
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
    // Open the bar / EE targets on their MEASURED angles, not the START_* constants, so
    // tick 1 has no standing error to lurch against.
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
    // Boot reference trace -- the line to read when a pose looks 360 deg out or the
    // mechanism sets off the wrong way on power-up.
    Logger::getInstance()->log(
        "LIFTINIT refs: bar raw=%.1f -> %.1f (START %.1f) | ee raw=%.1f -> %.1f "
        "(START %.1f) | winch raw=%.1f + offset=%.1f -> %.1f (START %.1f)",
        bar_raw, bar0, CHAINBAR_START_DEG, ee_raw, ee0, CHAIN_START_DEG,
        motor_group_position(), winch_ref_offset_, get_winch_position(),
        WINCH_START_POS);
    if (std::abs(bar0 - CHAINBAR_START_DEG) > START_POSE_SANITY_DEG ||
        std::abs(ee0 - CHAIN_START_DEG) > START_POSE_SANITY_DEG) {
        // Resolved against the START pose, so a mechanism parked elsewhere can be whole
        // turns out -- and the choreography is about to drive absolute angles on it.
        Logger::getInstance()->log(
            "LIFTWARN boot pose is NOT the START pose (bar %.1f vs %.1f, ee %.1f vs "
            "%.1f, limit %.0f) -- turn count may be resolved onto the wrong branch; "
            "power on with the mechanism in START",
            bar0, CHAINBAR_START_DEG, ee0, CHAIN_START_DEG, START_POSE_SANITY_DEG);
    }
    probe("pids reset; state seeded");

    // Kick off the START->STOW choreography; it plays out as update() is called.
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
    // accumulates past +/-360 (which CHAIN_SCORING_ALT_DEG = -425 relies on).
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
    // Taper the rate over the decel zone so the bar decelerates into the goal, with a
    // floor so the tail does not asymptote.
    double frac = 1.0;
    if (decel_deg > 0.0) {
        frac = std::clamp(dist / decel_deg, CHAINBAR_APPROACH_MIN_FRAC, 1.0);
    }
    double rate = max_dps * frac;  // profile speed magnitude (deg/s)
    double step = rate * dt;
    // The INTENDED profile rate, before the lead clamp below, so the velocity FF commands
    // the full profile even on a tick where the clamp holds the target back.
    out_vel_dps = (dist <= step) ? 0.0 : std::copysign(rate, remaining);
    double next = (dist <= step) ? goal  // close enough this tick: snap to the goal
                                 : current + std::copysign(step, remaining);
    // Never let the target lead the bar by more than CHAINBAR_MAX_LEAD_DEG, so a lagging
    // bar cannot charge the PID into an overshoot and the target is dragged along with
    // the free-swinging bar through the return coast.
    return std::clamp(next, actual_bar - CHAINBAR_MAX_LEAD_DEG,
                      actual_bar + CHAINBAR_MAX_LEAD_DEG);
}

// ---- Driver / macro entry points -------------------------------------------

void Lift::prime() {
    // Valid from either resting pose -- the clearance raise holds the bar wherever it is,
    // so the only difference is the angle the long sweep sets off from.
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
    // Hold the bar at the pose we are leaving while the lift climbs: from matchload a
    // hardcoded loading angle would swing it back across the low-clearance zone first.
    entry_bar_deg_ = get_bar_angle();
    // Raise -> sweep -> lower rather than a direct move, so the lift is up while the bar
    // sweeps the long way round the obstacle.
    mode_ = Mode::PRIMING;
    prime_step_ = PrimeStep::RAISE;
    chainbar_pid_.reset();
    winch_pid_.reset();
}

void Lift::back_prime() {
    // Already in the back cycle: this is the pose toggle. Restarting at RAISE is right
    // from any step, since raise/swing/lower handles being entered from anywhere, and the
    // claw is left alone -- a toggle is not a re-grab.
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
    // Otherwise this is the back PRIME: only valid from a resting pose, since it closes
    // the claw on a piece sitting in the mechanism. Either pose works, same as prime().
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
    // Switch the raised-orientation PAIR only -- ee_alt_ is left alone, so the flip
    // button means the same thing on both sides and a choice made before priming holds.
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
        // Matchload -> loading is just the return choreography, which already handles
        // starting part way round.
        Logger::getInstance()->log("LIFTCMD matchload toggle -> back to loading");
        begin_return(Mode::STOW);
        return;
    }
    if (mode_ != Mode::STOW) {
        // Anywhere else this means "come back, to matchload". STARTUP is the one
        // exception: it owns the mechanism until it has a known pose.
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
    // Release where the mechanism already is -- nothing moves onto the tower. Valid in the
    // raise sequence, a back pose once PARKED, and a score state once ARRIVED; mid-move is
    // refused, since the retreat below would swing the bar with the lift in transit.
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
    // A score state names its own EE angle, so ee_alt_ does not describe how the EE is
    // actually oriented and must not decide whether the retreat runs. A score state that
    // ever wants one should say so with a per-row flag, not ee_alt_.
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
        // DEFAULT orientation (or any score state): the EE is clear as soon as the claw
        // opens, so hold, and leave the mode alone for the driver / autonomous.
        return;
    }
    // ALTERNATE orientation: the flipped EE is still in the structure, so lift and then
    // swing the bar to carry it clear (see the SCORE_RETREAT case in update()). Lift
    // first, clamped to the travel, since near full extension there is no room for it.
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
    // Deliberately ungated: it only touches the solenoid, so there is no pose it is
    // unsafe from, and the states where the macros keep the claw clamped are the point.
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
    // Only the SELECTION is stored, never an angle -- raised_ee_deg() resolves it against
    // the cycle in force, and every mode re-reads that each tick, so a flip lands at once.
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
    // Pure dispatch: each branch calls the existing macro, which keeps its own gating and
    // logging. A strcmp chain to match the repo's other string dispatch and stay off the heap.
    if (key == nullptr) {
        Logger::getInstance()->logError("LIFTCMD run_macro REFUSED (null key)");
        return false;
    }
    const char* side_name = (side == Side::BACK) ? "BACK" : "FRONT";
    if (std::strcmp(key, "prime") == 0) {
        // The one key the side selects between -- two different macros.
        Logger::getInstance()->log("LIFTCMD run_macro %s prime", side_name);
        if (side == Side::BACK) { back_prime(); } else { prime(); }
        return true;
    }
    // Everything below is side-agnostic, but the side is still logged so a route passing
    // the wrong one is visible.
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
    // STARTUP is the one refusal: it owns the mechanism until it has a known pose. Every
    // other mode is fair game, since RAISE holds the bar wherever it is.
    if (mode_ == Mode::STARTUP) {
        Logger::getInstance()->log("LIFTCMD score_state '%s' IGNORED (mode=STARTUP)", key);
        return false;
    }
    Logger::getInstance()->log(
        "LIFTCMD score_state %s '%s' from %s -> raise, swing bar %.1f -> %.1f, "
        "EE -> %.1f, lower to %.0f",
        side == Side::BACK ? "BACK" : "FRONT", target->key, mode_name(mode_),
        get_bar_angle(), target->bar_deg, target->ee_deg, target->winch_pos);
    // The claw is untouched: a score state is a POSE. prime() grabs, score() releases.
    score_target_ = target;
    // Which pair a later flip resolves against -- the pose names its own EE angle, but the
    // cycle it hands over to needs this.
    ee_family_ = (side == Side::BACK) ? EeFamily::BACK : EeFamily::FRONT;
    mode_ = Mode::SCORE_STATE;
    score_step_ = ScoreStep::RAISE;
    // Hold the bar where the previous pose left it while the lift climbs, same as prime().
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
            // Done once parked at the retreat angle; it holds there awaiting a return.
            return !score_retreat_parked_;
        case Mode::STOW:
        case Mode::MATCHLOAD:
            return false;
        case Mode::SEQUENCE:
            // Walked by the driver's buttons, with no destination to wait on.
            return false;
    }
    return false;
}

bool Lift::is_settled() const {
    if (is_busy()) {
        return false;
    }
    // Not moving anywhere AND actually on the pose. These targets are the ones update()
    // resolved last tick, so it only means anything once update() has run since the command.
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
            // Raise the lift to give the bar room to swing down to loading. An ABSOLUTE
            // height, not start-pose plus a delta -- the room the bar needs is physical
            // and does not shift because the mechanism powered on sitting lower.
            winch_manual_target_ = WINCH_START_CLEARANCE_RAISE;
            if (winch >= winch_manual_target_ - WINCH_POS_TOL) {
                start_step_ = StartStep::BAR_LOADING;
                chainbar_pid_.reset();
            }
            break;
        }
        case StartStep::BAR_LOADING: {
            // Swing just PAST loading so the bar is clear for the next step's descent. The
            // return's RAISE_SWEEP over a shorter arc: a step target, not slewed, since
            // the velocity FF fights its own gravity term here (log 07262026-2).
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
            // Lift is down: bring the bar the last few degrees onto the loading angle,
            // then settle in STOW -- which is what arms the winch homing pass.
            winch_manual_target_ = WINCH_LOADING_POS;
            bar_target_deg_ = slew_bar_toward(bar_target_deg_, CHAINBAR_LOADING_DEG, bar,
                                              bar_target_vel_dps_);
            chain_target_deg_ = CHAIN_LOADING_DEG;
            if (settle_bar_arrived(bar, CHAINBAR_LOADING_DEG)) {
                start_step_ = StartStep::IDLE;
                mode_ = Mode::STOW;
                claw_.set_state(CLAW_OPEN_STATE);  // stow pose accepts a piece
                // The reference is real from here, so cap what one re-zero may move it.
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
            // Raise the lift for clearance, holding the bar at the pose the prime started
            // from so it never detours between the two resting angles down low.
            winch_manual_target_ = WINCH_PRIME_CLEARANCE_POS;
            bar_target_deg_ = entry_bar_deg_;
            chain_target_deg_ = CHAIN_PRIME_RAISE_DEG;
            // Hand over to TUCK part way up -- a plain threshold, not an arrival check,
            // since this is a height the lift passes through rather than stops at.
            if (winch >= WINCH_PRIME_TUCK_POS) {
                prime_step_ = PrimeStep::TUCK;
                chainbar_pid_.reset();  // hold -> profiled move: start the PID clean
            }
            break;
        }
        case PrimeStep::TUCK: {
            // Same winch target as RAISE, so the lift carries on up and the tuck plays out
            // underneath it. SWEEP_LOWER carries the target on in the SAME direction if
            // the lift arrives first, which is why the exit below tests the winch only.
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
            // Sweep the long way to primed, descending the moment the bar leaves the
            // low-clearance zone so the two overlap. SLEWED, at two rates split at the top
            // of the swing: saturating while climbing, then one the taper can stop.
            winch_manual_target_ = bar_clear_of_low_zone(bar) ? primed_winch
                                                             : WINCH_PRIME_CLEARANCE_POS;
            double sweep_dps = bar > CHAINBAR_PRIME_CAP_START_DEG
                                   ? CHAINBAR_PRIME_CLIMB_DPS
                                   : CHAINBAR_PRIME_SWEEP_DPS;
            bar_target_deg_ = slew_bar_toward(bar_target_deg_, primed_bar, bar,
                                              bar_target_vel_dps_, sweep_dps,
                                              CHAINBAR_PRIME_SWEEP_DECEL_DEG);
            // Stay tucked until the bar is far enough round to flip -- going back to
            // CHAIN_LOADING_DEG here would swing the EE back and then straight on again.
            chain_target_deg_ = CHAIN_PRIME_RAISE_DEG;
            if (bar < CHAIN_FLIP_BAR_DEG){
                chain_target_deg_ = raised_ee_deg();
            }
            if (std::abs(bar - primed_bar) < CHAINBAR_POS_TOL &&
                std::abs(winch - primed_winch) < WINCH_POS_TOL) {
                // Land the target ON the pose first: the sequence holds whatever target it
                // inherits until the driver drives it, and the slewed one may still be a
                // couple of degrees short inside the arrival tolerance.
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
    // One choreography, two destinations: matchload sits PART WAY round the return travel,
    // so going there is the same sweep stopping early. Every step just aims at a different
    // end pose, and sweep_catch_angle() keeps the hand-off the same distance short of it.
    const bool to_match = return_dest_ == Mode::MATCHLOAD;
    const double sweep_goal = to_match ? CHAINBAR_MATCHLOAD_DEG : CHAINBAR_LOWER_CLEAR_DEG;
    const double rest_bar   = to_match ? CHAINBAR_MATCHLOAD_DEG : CHAINBAR_LOADING_DEG;
    const double rest_winch = to_match ? WINCH_MATCHLOAD_POS : WINCH_LOADING_POS;
    const double rest_ee    = to_match ? CHAIN_MATCHLOAD_DEG : CHAIN_LOADING_DEG;
    ReturnStep step_before = return_step_;
    switch (return_step_) {
        case ReturnStep::RAISE_SWEEP: {
            // Raise and sweep at once: only the last stretch needs the lift up, so the
            // goal is capped at the low-clearance edge until it arrives. A STEP target and
            // FF-free -- this only carries the bar round, BAR_SETTLE lands it.
            winch_manual_target_ = WINCH_CLEARANCE_POS;
            if (alt_return_hold_active(bar)) {
                // Flipped EE not yet back round: the lift may still RISE but must not come
                // down on it, and after a score it usually sits above the clearance height.
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
            // Bar is parked on the sweep goal; lower onto the destination height. For
            // matchload that descends inside the low-clearance zone, as the hop does.
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
            // Lift is down: bar onto the resting angle, EE to its resting orientation,
            // then settle -- which arms the homing pass. Returning to matchload the sweep
            // already stopped on the pose, so only the EE turn is left.
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
            // Lift up first -- the bar has to cross the low-clearance zone -- but only to
            // WINCH_MATCHLOAD_CLEARANCE_POS, since a short arc at the loading end does not
            // need the full height. Bar held where the pose left it, EE tucked.
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
            // Swing onto the pose with the lift held up. SLEWED, like BAR_SETTLE: a short
            // climb that has to LAND on a pose, and a step target would arrive at speed
            // with nothing to decelerate it (log 07262026-3).
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
            // Bar is on the pose; bring the lift down onto the matchload height. This
            // descends inside the low-clearance zone -- see CHAINBAR_MATCHLOAD_DEG.
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
    // One path onto either back pose, entry included: raise clear, swing, lower, hold.
    // Each step passes through when it has nothing to do, so the direction takes care of
    // itself, and the bar only ever swings in SWING, which only runs once the lift is up.
    const double goal_bar = back_bar_deg();
    const double goal_winch = back_winch_pos();
    // Height held during RAISE and SWING: the clearance height, or the destination if that
    // is higher. Never a LOW destination -- that is what LOWER is for.
    const double clear_winch = std::max(WINCH_CLEARANCE_POS, goal_winch);
    BackStep step_before = back_step_;
    switch (back_step_) {
        case BackStep::RAISE: {
            // Lift up first, bar held wherever the previous pose left it. The EE tucks for
            // the climb rather than turning to the back orientation down low.
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
            // Swing onto the pose with the lift held up. SLEWED, like the matchload sweep
            // -- the decel taper is what keeps the downhill direction from arriving hot.
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
            // Bar is on the pose; bring the lift onto its height. Going to TOP this is
            // already satisfied and passes straight through.
            winch_manual_target_ = goal_winch;
            bar_target_deg_ = goal_bar;
            chain_target_deg_ = raised_ee_deg();
            if (std::abs(winch - goal_winch) < WINCH_POS_TOL) {
                back_step_ = BackStep::HOLD;
                // Held in its own state, not read off the pose each tick, since the
                // driver's jog moves the held height off it.
                back_hold_bar_ = goal_bar;
                back_hold_winch_ = goal_winch;
                winch_pid_.reset();
            }
            break;
        }
        case BackStep::HOLD: {
            // Parked: bar and EE hold the pose, the lift is the driver's on L1/L2. The jog
            // moves the held height, so letting go leaves the lift where it is.
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
    // The back cycle's choreography with the destination read from a table. Kept separate
    // because the two differ in what HOLD means; the shape is identical because swinging
    // only in SWING is what keeps an arbitrary row clear of the low-clearance zone.
    if (score_target_ == nullptr) {
        // Can't happen: run_score_state() always sets the target. Bail rather than
        // command garbage.
        Logger::getInstance()->logError("LIFTERR SCORE_STATE with no target -- returning to loading");
        begin_return(Mode::STOW);
        return;
    }
    const double goal_bar   = score_target_->bar_deg;
    const double goal_winch = score_target_->winch_pos;
    const double goal_ee    = score_target_->ee_deg;
    // Height held during RAISE and SWING: the clearance height, or the destination if that
    // is higher. Never a LOW destination -- that is what LOWER is for.
    const double clear_winch = std::max(WINCH_CLEARANCE_POS, goal_winch);
    ScoreStep step_before = score_step_;
    switch (score_step_) {
        case ScoreStep::RAISE: {
            // Lift up first, bar held wherever the previous pose left it, EE tucked for
            // the climb -- exactly as the back cycle's RAISE does.
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
            // Swing onto the pose with the lift held up. SLEWED, since it has to LAND on
            // the pose whichever way the row has it climbing.
            winch_manual_target_ = clear_winch;
            bar_target_deg_ = slew_bar_toward(bar_target_deg_, goal_bar, bar,
                                              bar_target_vel_dps_);
            // The pose's OWN EE angle, not raised_ee_deg() -- it names its orientation
            // outright, so the flip selection does not apply.
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
            // Arrived. No jog here, unlike the back cycle's HOLD -- autonomous has no
            // buttons. This is where is_settled() goes true and score() is armed.
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
    // The flip button stores a selection, the cycle supplies the pair. Nothing else should
    // compare an EE angle to decide "is the alternate selected" -- ee_alt_ is that answer.
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
    // The bar sweeps NEGATIVE from loading to primed, and only needs the lift up over the
    // near-loading end of that travel. Past the zone the winch may move independently.
    return bar < CHAINBAR_LOW_ZONE_DEG;
}

bool Lift::bar_sweeping() const {
    // The return's RAISE_SWEEP and the startup's BAR_LOADING are the same move at different
    // lengths, so they share one control law: step target, FF-free, capped, coasting
    // through the middle (see update()).
    return (mode_ == Mode::RETURNING && return_step_ == ReturnStep::RAISE_SWEEP) ||
           (mode_ == Mode::STARTUP && start_step_ == StartStep::BAR_LOADING);
}

bool Lift::bar_coast_active(double bar, double winch) const {
    // Past the top of the swing, not yet at the catch angle, motor in COAST. Two guards
    // beyond that: the lift must be up, since a coasting bar carries past the low-zone
    // edge, and bar_coast_done_, so a bounce cannot start a second coast.
    return bar_sweeping() &&
           !bar_coast_done_ &&
           bar >= CHAINBAR_COAST_START_DEG &&
           bar < sweep_catch_angle() &&
           winch >= WINCH_CLEARANCE_POS - WINCH_POS_TOL;
}

bool Lift::alt_return_hold_active(double bar) const {
    // Only the ALTERNATE orientation needs this: until the flipped EE has swung back round
    // it is unsafe to drive the chain or to bring the lift down on top of it.
    return mode_ == Mode::RETURNING &&
           ee_alt_ &&
           bar < CHAIN_FLIP_BAR_DEG;
}

double Lift::sweep_catch_angle() const {
    // A fixed LEAD-IN short of whatever the sweep aims at, not a fixed angle, so the
    // matchload return hands over the same distance before ITS pose (log 07262026-2).
    constexpr double LEAD_IN = CHAINBAR_COAST_CATCH_DEG - CHAINBAR_LOWER_CLEAR_DEG;
    if (mode_ == Mode::RETURNING && return_dest_ == Mode::MATCHLOAD) {
        return CHAINBAR_MATCHLOAD_DEG + LEAD_IN;
    }
    return CHAINBAR_COAST_CATCH_DEG;
}

bool Lift::settle_bar_arrived(double bar, double rest_bar) const {
    // The settle onto loading is only ~4 deg, already inside CHAINBAR_POS_TOL, so a
    // position-only check would pass on the first tick and skip the move. Requiring the
    // SLEWED TARGET to have arrived too means it is always commanded in full.
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

// One flat, greppable line per throttled tick, key=value so the Python log tools can
// parse it. Angles are output deg (bar) / motor deg (winch, EE); velocities are motor RPM
// (winch) / output deg/s (bar, EE).
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
    // Fixed control step -- deriving dt from wall-clock would drift on a loop overrun.
    double dt = Config::DT / 1000.0;
    last_update_ms_ = now;

    double bar = get_bar_angle();
    double winch = get_winch_position();

    // Bar rate as a windowed slope: the sensor updates slower than the loop, so a per-tick
    // delta aliases into a 0/spike sawtooth (log 07242026) with nothing for kD to damp
    // against. Computed here because the drive caps and the retreat's park both read it.
    for (int i = CHAINBAR_RATE_WINDOW - 1; i > 0; --i) {
        bar_angle_hist_[i] = bar_angle_hist_[i - 1];
    }
    bar_angle_hist_[0] = bar;
    double bar_vel_dps = (bar_angle_hist_[0] - bar_angle_hist_[CHAINBAR_RATE_WINDOW - 1])
                       / ((CHAINBAR_RATE_WINDOW - 1) * dt);

    // ---- Resolve this tick's winch / bar / chain targets by mode. ----
    // No profiled bar motion by default; the slewed moves overwrite this for the velocity FF.
    bar_target_vel_dps_ = 0.0;
    // Drive cap (mV) the bar-only sequence swing wants this tick, 0 if it is not running.
    // Directional, since the down swing is the one a held piece comes down on.
    double bar_swing_cap_mv = 0.0;
    // Set by the parked score retreat: hand the bar to its brake instead of driving it.
    bool bar_park_hold = false;
    switch (mode_) {
        case Mode::STARTUP:
            // Scripts every target step by step and lands in STOW.
            advance_start_sequence(bar, winch);
            break;
        case Mode::STOW:
            winch_manual_target_ = WINCH_LOADING_POS;
            bar_target_deg_ = CHAINBAR_LOADING_DEG;
            chain_target_deg_ = CHAIN_LOADING_DEG;
            break;
        case Mode::MATCHLOAD:
            // The other resting pose: a plain hold, like STOW.
            winch_manual_target_ = WINCH_MATCHLOAD_POS;
            bar_target_deg_ = CHAINBAR_MATCHLOAD_DEG;
            chain_target_deg_ = CHAIN_MATCHLOAD_DEG;
            break;
        case Mode::TO_MATCHLOAD:
            // Raise, swing, lower onto the matchload pose.
            advance_matchload_sequence(bar, winch);
            break;
        case Mode::BACK:
            // The move onto the selected back pose, then the hold that keeps it there
            // with the lift under the driver's thumb.
            advance_back_sequence(bar, winch, dt);
            break;
        case Mode::SCORE_STATE:
            // The same move onto a pose from the table, then a plain hold -- no jog.
            advance_score_state_sequence(bar, winch);
            break;
        case Mode::PRIMING:
            // Raise, sweep the long way to primed, lower again once the bar is clear;
            // hands off to SEQUENCE at progress 0.
            advance_prime_sequence(bar, winch);
            break;
        case Mode::RETURNING:
            // The reverse: raise while sweeping back, then lower onto the resting pose.
            advance_return_sequence(bar, winch);
            break;
        case Mode::SEQUENCE: {
            // KF0<->KF1 is the bar alone, so it is NOT walked on the clock: the target is
            // the end of the swing and the caps set the speed. KF1<->KF2 moves both, so it
            // keeps the timed interpolation. Raising wins if both buttons are held.
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
                // Out to KF1 is a climb, back to KF0 is the gravity-fed one a held piece
                // comes down on. advancing_ wins if both are held, as the target does.
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
            // Swing the bar up to carry the flipped EE out of the structure. LIFT FIRST,
            // and CLEARANCE FIRST if the bar starts inside the low-clearance zone as a
            // back pose does. Arrival is a PARK, not a hold, and the mode never ends itself.
            winch_manual_target_ = retreat_winch_target_;
            bool lifted = winch >= retreat_winch_target_ - WINCH_POS_TOL;
            bool lift_clear = winch >= WINCH_CLEARANCE_POS - WINCH_POS_TOL;
            bool wait_for_clearance = !lift_clear && !bar_clear_of_low_zone(bar);
            if (wait_for_clearance) {
                winch_manual_target_ = std::max(winch_manual_target_, WINCH_CLEARANCE_POS);
            }
            // While waiting, sit on the angle the retreat started at rather than the pose
            // the sequence was holding.
            bar_target_deg_ = (wait_for_clearance || !lifted) ? entry_bar_deg_
                                                              : SCORE_ALT_RETREAT_BAR_DEG;
            bar_swing_cap_mv = SEQ_SWING_UP_MAX_MV;
            double retreat_err = std::abs(bar - SCORE_ALT_RETREAT_BAR_DEG);
            bool park_now = score_retreat_parked_ ? (retreat_err <= SCORE_RETREAT_WAKE_DEG)
                                                  : (retreat_err <= SCORE_RETREAT_PARK_DEG);
            if (park_now != score_retreat_parked_) {
                // Already HOLD here, but stated rather than inherited: a zero command is
                // only a hold if the brake says so.
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

    // Clamp to the physical travel. The floor is WINCH_TARGET_FLOOR, not WINCH_DOWN_POS:
    // the lift powers on below the taut bottom, and clamping to 0 would command the whole
    // offset away on the first tick -- a railed snatch on a slack string.
    winch_manual_target_ = std::clamp(winch_manual_target_, WINCH_TARGET_FLOOR, WINCH_MAX_POS);
    double winch_target = winch_manual_target_;

    // --- Winch: PID hold, plus a home-to-bottom pass once settled at the bottom. ---
    // HOLD -> SEEK (short down pre-roll to ground the lift) -> RESPOOL (reel up until the
    // string goes taut and re-zero) -> HOLD. Gated to the resting poses, once per descent.
    double winch_vel_rpm = winch_motors_.get_actual_velocity();
    double winch_vel_dps = winch_vel_rpm * RPM_TO_DPS;
    double winch_current = winch_motors_.get_current_draw();  // mA (diagnostic)

    // MATCHLOAD counts alongside STOW -- same bottom height, so a robot parked there
    // should still get its homing pass rather than run on a stale reference.
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
        // Per-tick trace: shows whether the lift grounds and how much slack pays out.
        Logger::getInstance()->log("LIFTSEEK t=%u winch=%.1f vel=%.1f cur=%.0f mv=%.0f",
                                   now - seek_start_ms_, winch, winch_vel_rpm,
                                   winch_current, WINCH_SEEK_MV);
        // Fixed pre-roll, no bottom detection -- this lift just unspools slack once
        // grounded and gives no clean down-signature.
        bool done = (now - seek_start_ms_) > WINCH_SEEK_MS;
        // Measured from where the seek STARTED: a grounded lift with slack out already
        // sits well below WINCH_DOWN_POS, so an absolute backstop would trip immediately.
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
        // Per-tick trace: current should be LOW while reeling in slack then SPIKE as the
        // string goes taut. Sustained HIGH means it is hauling the lift up instead.
        Logger::getInstance()->log("LIFTRESPOOL t=%u winch=%.1f vel=%.1f cur=%.0f mv=%.0f",
                                   now - respool_start_ms_, winch, winch_vel_rpm,
                                   winch_current, WINCH_RESPOOL_MV);
        bool past_grace = (now - respool_start_ms_) > WINCH_RESPOOL_GRACE_MS;
        bool stalled = std::abs(winch_vel_rpm) < WINCH_RESPOOL_STALL_RPM;
        bool timed_out = (now - respool_start_ms_) > WINCH_RESPOOL_TIMEOUT_MS;
        if (past_grace && stalled) {
            // String taut: anchor the reference here. Capped once startup has finished,
            // since a false stall (snag, stiff spool, glitch) would otherwise rewrite the
            // reference and leave the hold chasing a phantom error into the hard stop.
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
            // Shift the SOFTWARE reference, NOT set_zero_position_all(): one winch motor
            // is reversed, so that call collapses to ~0 -- see winch_ref_offset_.
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
    // Mid-sweep the bar swings home on gravity, with the PID still ENGAGED; everywhere
    // else it is the usual PID with the gravity + motion feed-forward.
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
        // Brake mode follows the window even though the PID drives throughout: a 0 mV
        // command under a HOLD brake would fight the swing, since the brake is itself a
        // command. Written on the edge only.
        chainbar_motor_.set_brake_mode(bar_coasting ? pros::E_MOTOR_BRAKE_COAST
                                                    : pros::E_MOTOR_BRAKE_HOLD);
        if (!bar_coasting) {
            // Catch: the drive hands over to the feed-forward law here, so clear the
            // integral the long railed sweep leaves behind rather than carry it in.
            chainbar_pid_.reset();
            bar_coast_done_ = true;  // one coast per sweep
        }
        Logger::getInstance()->log(
            "LIFTEVT bar coast %s, brake=%s (bar=%.1f vel=%.1f tgt=%.1f winch=%.1f)",
            bar_coasting ? "START" : "CATCH", bar_coasting ? "COAST" : "HOLD",
            bar, bar_vel_dps, bar_target_deg_, winch);
        chainbar_coasting_ = bar_coasting;
    }
    // The long part of a bar sweep, up to the catch angle -- a big-error law with nothing
    // left to give at the end of the travel, where P alone loses to stiction and the bar
    // parks itself short (log 07262026-2). Past the catch the FF branch below takes over.
    bool bar_sweep = bar_sweeping() && bar < sweep_catch_angle();
    if (bar_park_hold) {
        // Parked score retreat: NO command, so the brake holds the bar. Not "PID with a
        // small error" -- at the top of the swing that is exactly what oscillated. Held in
        // reset so it re-engages clean if the bar escapes the park band.
        bar_mv = 0.0;
        chainbar_pid_.reset();
    } else if (bar_coasting || bar_sweep) {
        // The long stretch of a sweep, coast included: plain position PID, NO feed-forward.
        // Gravity pulls the bar TOWARD the goal here, so the gravity FF would point
        // backwards the whole way and cost 7031 mV to break out of (log 07262026-2).
        bar_mv = chainbar_pid_.calculate(bar_cmd, bar, bar_vel_dps, 0.0);
    } else {
        // Gravity feed-forward: the standing holding effort, so the PID does not need a
        // lingering error to balance gravity. Largest horizontal, zero at vertical rest.
        static constexpr double DEG_TO_RAD = 3.14159265358979323846 / 180.0;
        bar_gravity_ff = CHAINBAR_GRAVITY_FF_MV *
            std::sin((bar - CHAINBAR_GRAVITY_ZERO_DEG) * DEG_TO_RAD);

        // Motion feed-forward: the term that actually moves the bar, so its speed is
        // COMMANDED rather than emergent from P vs. friction. Two regimes:
        double bar_err = bar_cmd - bar;
        if (std::abs(bar_target_vel_dps_) > CHAINBAR_PROFILE_EPS_DPS) {
            // Profiled move: kV tracks the commanded speed and the kinetic term keeps low
            // speeds from being eaten by friction. Both scale with the profile velocity,
            // so nothing is left over at the pose to push the bar past it.
            double kinetic = CHAINBAR_KINETIC_FF_MV *
                std::clamp(bar_target_vel_dps_ / CHAINBAR_KS_MOVING_DPS, -1.0, 1.0);
            bar_static_ff = kinetic + CHAINBAR_KV_MV_PER_DPS * bar_target_vel_dps_;
        } else {
            // Step target / hold: position-error BREAKAWAY, only while the bar is stuck.
            // Fades inside the deadband so it does not chatter or shove the bar through
            // loading, and with speed, since on a moving bar it oscillates (log 07242026-3).
            double ks_dir_scale = std::clamp(bar_err / CHAINBAR_KS_DEADBAND_DEG, -1.0, 1.0);
            double ks_stuck_scale =
                std::clamp(1.0 - std::abs(bar_vel_dps) / CHAINBAR_KS_MOVING_DPS, 0.0, 1.0);
            bar_static_ff = CHAINBAR_KS_MV * ks_dir_scale * ks_stuck_scale;
        }
        double bar_ff = bar_gravity_ff + bar_static_ff;

        // Feed kD the VELOCITY ERROR, not the raw rate: the setpoint moves during a
        // profiled move, so raw-rate damping would brake the cruise and fight the velocity
        // FF. For holds and step targets the profile velocity is 0, so it is the raw rate.
        double bar_vel_err = bar_vel_dps - bar_target_vel_dps_;

        // FF goes INTO the PID, not added after, so the anti-windup sees the true
        // post-FF command and the output clamp covers the sum.
        bar_mv = chainbar_pid_.calculate(bar_cmd, bar, bar_vel_err, bar_ff);
    }

    // --- Drive cap for the long sweeps. -------------------------------------
    // Two carve-outs: BRAKING is never capped, since a command opposing the bar is what
    // these sweeps must never be short of; and the LOW caps only apply once the bar is
    // moving, since breaking it out of rest takes ~7000 mV (log 07262026-2).
    bool bar_moving = std::abs(bar_vel_dps) > CHAINBAR_BRAKE_MIN_DPS;
    // The sequence swing gets the same exemption, but LATCHED for the whole swing: keyed
    // on bar_moving alone the limit would lift the moment the piece met the tower and the
    // bar stopped, which is the one place the soft cap is FOR.
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
        // Capped only PAST THE TOP of the swing: climbing the bar needs the full rail,
        // past it the cap stops the motor pumping energy into a descent gravity is already
        // accelerating. A BACKSTOP -- the sweep is slewed, so the velocity FF sets speed.
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
    // Windowed-slope rate, same reason as the bar: the raw velocity swings +/-150 deg/s
    // tick to tick with the EE at rest, which kD turns into ~850 mV of buzz (log 07242026-5).
    for (int i = CHAIN_RATE_WINDOW - 1; i > 0; --i) {
        chain_angle_hist_[i] = chain_angle_hist_[i - 1];
    }
    chain_angle_hist_[0] = ee;
    double chain_vel_dps = (chain_angle_hist_[0] - chain_angle_hist_[CHAIN_RATE_WINDOW - 1])
                         / ((CHAIN_RATE_WINDOW - 1) * dt);

    double chain_cmd = std::clamp(chain_target_deg_, CHAIN_MIN_DEG, CHAIN_MAX_DEG);
    // A pose the soft limits cannot express is a CONFIG error, and a quiet one: the clamp
    // becomes the setpoint and the EE holds at the limit, looking like a reversed motor
    // (log 07262026-4). LIFTDBG logs the raw target, so warn here instead, once per onset.
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
    // Alternate-orientation return: kill the chain until the bar is back round. Not "hold
    // where you are" -- no command at all, and the PID reset so it re-engages clean.
    bool alt_hold = alt_return_hold_active(bar);
    if (alt_hold != alt_return_gate_) {
        // Brake mode has to follow: at 0 mV a HOLD-braked motor still fights rotation,
        // which is what this hold is trying to avoid. Edge-written only.
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
        // Speed cap (see CHAIN_MAX_MV), applied outside the PID so near-target behaviour
        // is untouched. The integral is worth ~200 mV at most, so it cannot wind up here.
        chain_mv = std::clamp(chain_mv, -CHAIN_MAX_MV, CHAIN_MAX_MV);
        // Soft rotation limit: never over-rotate the chain / end effector.
        chain_mv = apply_soft_limit(chain_mv, ee, CHAIN_MIN_DEG, CHAIN_MAX_DEG);
    }
    chain_motor_.move_voltage(static_cast<int>(chain_mv));

    // Full debug line to the SD log, throttled to ~20 Hz so the async logger keeps up.
    static uint32_t dbg_log_counter = 0;
    if (++dbg_log_counter % 5 == 0) {
        log_debug_state(winch_target, winch, winch_mv, winch_vel_rpm,
                        bar, bar_mv, bar_vel_dps, bar_gravity_ff, bar_static_ff,
                        ee, chain_mv, chain_vel_dps);
    }

    // Brain-screen telemetry, throttled to ~5 Hz: pros::lcd::print wraps LVGL, and calling
    // it every 10 ms races the render daemon and intermittently freezes the program.
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
