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
      back_state_(0),
      back_target_winch_(WINCH_BACK_ENTRY_POS),
      state_step_(StateStep::HOLD),
      bar_pose_step_(BarPoseStep::HOLD),
      bar_pose_winch_(WINCH_START_POS),
      state_target_(nullptr),
      return_dest_(Mode::STOW),
      entry_bar_deg_(CHAINBAR_START_DEG),
      entry_ee_deg_(CHAIN_PRIME_RAISE_DEG),
      startup_complete_(false),
      chainbar_coasting_(false),
      bar_coast_done_(false),
      alt_return_gate_(false),
      size_keep_zone_(-1),
      return_hold_winch_(WINCH_START_POS),
      raise_progress_(0.0),
      bar_swing_broke_out_(false),
      bar_stalled_ticks_(0),
      chain_(nullptr),
      chain_step_(0),
      chain_step_ms_(0),
      chain_step_settled_(false),
      chain_dispatching_(false),
      seq_swing_driving_(false),
      advancing_(false),
      retreating_(false),
      winch_jogging_(false),
      prime_grab_pending_(false),
      prime_grab_ms_(0),
      prime_from_rest_(false),
      skip_clearance_raise_(false),
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
      untension_start_ms_(0),
      winch_slack_pos_(WINCH_DOWN_POS),
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
    bar_pose_step_ = BarPoseStep::HOLD;
    bar_pose_winch_ = WINCH_START_POS;
    return_dest_ = Mode::STOW;   // the startup choreography lands in STOW
    startup_complete_ = false;  // re-zero cap arms when the choreography finishes
    chainbar_coasting_ = false;
    bar_coast_done_ = false;
    bar_kill = false;
    prime_grab_pending_ = false;
    prime_grab_ms_ = 0;
    prime_from_rest_ = false;
    skip_clearance_raise_ = false;
    alt_return_gate_ = false;
    // Cleared, not seeded from the boot angle: it is only the hysteresis latch, and the
    // first update() re-reads the bands anyway. Starting off means a boot INSIDE one logs
    // its LIFTEVT line rather than engaging silently.
    size_keep_zone_ = -1;
    return_hold_winch_ = WINCH_START_POS;
    winch_mode_ = WinchMode::HOLD;
    winch_respooled_ = false;
    winch_slack_pos_ = WINCH_DOWN_POS;  // no homing pass has run yet
    raise_progress_ = 0.0;
    bar_swing_broke_out_ = false;
    bar_stalled_ticks_ = 0;
    chain_ = nullptr;  // a re-init abandons any chain the previous run left mid-flight
    chain_step_ = 0;
    chain_step_settled_ = false;
    chain_dispatching_ = false;
    seq_swing_driving_ = false;
    advancing_ = false;
    retreating_ = false;
    winch_jogging_ = false;
    winch_manual_target_ = WINCH_START_POS;
    // Open the bar / EE targets on their MEASURED angles, not the START_* constants, so
    // tick 1 has no standing error to lurch against.
    double bar0 = get_bar_angle();
    double ee0 = get_ee_angle();
    bar_target_deg_ = bar0;
    chain_target_deg_ = ee0;
    entry_bar_deg_ = bar0;
    entry_ee_deg_ = CHAIN_PRIME_RAISE_DEG;
    retreat_winch_target_ = WINCH_START_POS;
    score_retreat_parked_ = false;
    // Back cycle idle, on its first state, and the raised orientation back to the front
    // family's default.
    back_step_ = BackStep::HOLD;
    back_state_ = 0;
    back_target_winch_ = WINCH_BACK_ENTRY_POS;
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

    probe("initialize DONE (startup choreography deferred to driver)");
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

void Lift::seq_targets(double progress, const double* bar_kf, const double* winch_kf,
                       int n_kf, double& winch_out, double& bar_out) {
    // Piecewise-linear interpolation of the keyframes. progress in [0, N-1]; the
    // integer part selects the segment, the fraction blends within it.
    double p = std::clamp(progress, 0.0, static_cast<double>(n_kf - 1));
    int i = static_cast<int>(std::floor(p));
    if (i > n_kf - 2) {
        i = n_kf - 2;  // clamp so i+1 stays in range at the top end
    }
    double f = p - static_cast<double>(i);
    winch_out = winch_kf[i] + (winch_kf[i + 1] - winch_kf[i]) * f;
    bar_out = bar_kf[i] + (bar_kf[i + 1] - bar_kf[i]) * f;
}

double Lift::seq_swing_progress(double bar, double bar_from, double bar_to) {
    // Fraction of ONE SEGMENT covered, normalised by its own span -- so which WAY it
    // travels is the table's business: the front goes positive out of primed, the back
    // negative out of its entry pose, and the same law walks both.
    double span = bar_to - bar_from;
    if (std::abs(span) < 1e-9) return 0.0;
    return std::clamp((bar - bar_from) / span, 0.0, 1.0);
}

double Lift::seq_progress_from_bar(double bar, const double* bar_kf, int n_kf) {
    // Where the whole table sits, read off the bar: the segment the angle falls in, plus
    // how far across it. This is the ONLY thing that advances the walk now -- there is no
    // clock -- which is what makes the cascade follow the bar rather than run beside it on
    // its own schedule.
    //
    // Safe to walk segment by segment because seq_bar_walkable() has already asserted the
    // table is strictly monotonic in the bar: a segment reporting a full 1.0 is one the bar
    // is genuinely past, not one it never entered.
    for (int i = 0; i < n_kf - 2; ++i) {
        double f = seq_swing_progress(bar, bar_kf[i], bar_kf[i + 1]);
        if (f < 1.0) {
            return static_cast<double>(i) + f;
        }
    }
    // The last segment takes whatever is left, and its clamp is what holds progress at the
    // top end once the bar is there (or past it).
    return static_cast<double>(n_kf - 2) +
           seq_swing_progress(bar, bar_kf[n_kf - 2], bar_kf[n_kf - 1]);
}

void Lift::advance_raise_sequence(double bar, const double* bar_kf,
                                  const double* winch_kf, int n_kf,
                                  double& bar_swing_cap_mv, bool& bar_swing_down) {
    const double top = static_cast<double>(n_kf - 1);
    const bool driving = advancing_ || retreating_;
    if (driving) {
        const double goal = advancing_ ? bar_kf[n_kf - 1] : bar_kf[0];
        bar_target_deg_ = slew_bar_toward(bar_target_deg_, goal, bar, bar_target_vel_dps_,
                                          advancing_ ? SEQ_SWING_UP_MAX_DPS
                                                     : SEQ_SWING_DOWN_MAX_DPS,
                                          SEQ_SWING_DECEL_DEG);
    } else if (seq_swing_driving_) {
        bar_target_deg_ = bar;  // just released: hold this angle
    }
    // No drive cap: the profile is the speed limit now. Left set, a cap sized for the old
    // step target would bind against the feed-forward and put the bang-bang back.
    bar_swing_cap_mv = 0.0;
    // The descent is the one travel gravity PAYS for, so it takes its own gravity, velocity
    // and breakaway amplitudes rather than the climb's (see update()).
    bar_swing_down = driving && !advancing_;
    raise_progress_ = seq_progress_from_bar(bar, bar_kf, n_kf);
    // Arrived at either end: snap onto the boundary rather than idling on the last fraction
    // of a degree, so the cascade is commanded the end pose's height outright.
    if (advancing_ && std::abs(bar - bar_kf[n_kf - 1]) < CHAINBAR_POS_TOL) {
        raise_progress_ = top;
    } else if (retreating_ && std::abs(bar - bar_kf[0]) < CHAINBAR_POS_TOL) {
        raise_progress_ = 0.0;
    }
    // The cascade height, read at the progress the bar just reported. Only while the walk is
    // being DRIVEN, plus the tick it is released on: left live it would follow the bar's
    // every sag, and with ~10 winch deg to each bar deg on the back table that is a lot of
    // cascade for a couple of degrees of droop. Frozen, the pose stays where the driver let
    // go of it.
    if (driving || seq_swing_driving_) {
        double seq_bar_unused;
        seq_targets(raise_progress_, bar_kf, winch_kf, n_kf, winch_manual_target_,
                    seq_bar_unused);
    }
    seq_swing_driving_ = driving;
    chain_target_deg_ = raised_ee_deg();
}

double Lift::slew_bar_toward(double current, double goal, double actual_bar,
                             double& out_vel_dps, double max_dps, double decel_deg) {
    double dt = Config::DT / 1000.0;
    double remaining = goal - current;
    double dist = std::abs(remaining);
    double frac = 1.0;
    if (decel_deg > 0.0) {
        frac = std::clamp(std::abs(goal - actual_bar) / decel_deg,
                          CHAINBAR_APPROACH_MIN_FRAC, 1.0);
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
    bool from_rest = mode_ == Mode::STOW || mode_ == Mode::MATCHLOAD;
    // PRIMING is crossable for back_prime() -- that is a change of destination -- but not
    // for this one: it is already the destination, and restarting would stop the sweep
    // dead and throw it again from wherever the bar had got to.
    if (mode_ == Mode::PRIMING || (!from_rest && !crossable_from(mode_))) {
        Logger::getInstance()->log(
            "LIFTCMD prime IGNORED (mode=%s, %s)", mode_name(mode_),
            mode_ == Mode::PRIMING ? "already priming"
                                   : "not a resting pose or a raised cycle");
        return;
    }
    cancel_chain_if_external("prime commanded");
    prime_from_rest_ = from_rest;  // arms the intake assist feed, see is_priming_from_rest()
    Logger::getInstance()->log(
        "LIFTCMD prime from %s (%s) -> clearance raise (bar held at %.1f, tuck to bar %.1f / "
        "EE %.1f above winch %.0f), then sweep to KF0 (bar %.1f) climb <=%.0f mV to %.1f "
        "then <=%.0f mV, coasting from %.1f, catch %.1f, then lower + settle",
        mode_name(mode_), from_rest ? "grab" : "crossing", get_bar_angle(),
        CHAINBAR_PRIME_TUCK_DEG, CHAIN_PRIME_TUCK_DEG,
        WINCH_PRIME_TUCK_POS, SEQ_BAR_DEG[0], CHAINBAR_PRIME_CLIMB_MAX_MV,
        CHAINBAR_PRIME_CLIMB_END_DEG, CHAINBAR_PRIME_DESCENT_MAX_MV,
        CHAINBAR_PRIME_COAST_START_DEG,
        SEQ_BAR_DEG[0] + CHAINBAR_PRIME_CATCH_LEAD_DEG);
    if (!from_rest) {
        // Crossing: the claw is left exactly as it is -- a crossing is not a re-grab, the
        // same rule the back cycle's state cycle follows -- and there is nothing to wait
        // for, so go straight into the choreography.
        begin_prime();
        return;
    }
    claw_.set_state(CLAW_CLOSED_STATE);  // grab the piece before raising
    // ...but do NOT start moving on the same tick. The solenoid needs time to actually
    // clamp, and pulling the piece off the loader before it has is how it gets dropped.
    // Nothing else changes here: mode_ stays on the resting pose, so the pose keeps its own
    // hold on the bar for the wait, and update() starts the choreography when it expires.
    prime_grab_pending_ = true;
    prime_grab_ms_ = pros::millis();
}

void Lift::begin_prime() {
    // Reads the bar where it is standing, so this has to happen before anything moves.
    // Crossing in from the back cycle OR priming out of MATCHLOAD, the whole sweep is
    // already past the low-clearance zone and the cascade has nothing to climb over.
    skip_clearance_raise_ = move_skips_clearance(SEQ_BAR_DEG[0]);
    if (skip_clearance_raise_) {
        Logger::getInstance()->log(
            "LIFTEVT prime SKIPS the clearance raise (from %s, bar %.1f -> %.1f is clear of "
            "the low zone %.0f) -- cascade goes straight for %.0f",
            mode_name(mode_), get_bar_angle(), SEQ_BAR_DEG[0], CHAINBAR_LOW_ZONE_DEG,
            SEQ_WINCH_DEG[0]);
    }
    ee_family_ = EeFamily::FRONT;  // front pair of raised orientations; ee_alt_ untouched
    raise_progress_ = 0.0;
    // Hold the bar (and, crossing in from a raised pose, the EE) at the pose we are
    // leaving while the lift climbs: from matchload a hardcoded loading angle would swing
    // it back across the low-clearance zone first.
    latch_entry_pose();
    // Raise -> sweep -> lower rather than a direct move, so the lift is up while the bar
    // sweeps the long way round the obstacle.
    mode_ = Mode::PRIMING;
    prime_step_ = PrimeStep::RAISE;
    bar_coast_done_ = false;  // re-arm the coast-through for this sweep
    chainbar_pid_.reset();
    winch_pid_.reset();
}

void Lift::back_prime() {
    bar_kill = false;
    if (mode_ == Mode::BACK) {
        cancel_chain_if_external("back state cycle commanded");
        const int from_state = back_state_;
        // Wraps: the last state's next is the first, so one button walks the whole cycle.
        back_state_ = (back_state_ + 1) % BACK_STATE_COUNT;
        double hold_winch = get_winch_position();
        Logger::getInstance()->log(
            "LIFTCMD back cycle state %d -> %d (bar %.1f -> %.1f, winch HELD at %.1f)",
            from_state, back_state_, get_bar_angle(), back_state_bar_deg(), hold_winch);
        begin_back_move(hold_winch);
        return;
    }
    // Otherwise this is the back PRIME: ENTERING the cycle, always at state 0. From a
    // RESTING pose it closes the claw on a piece sitting in the mechanism, either pose,
    // same as prime(). From a RAISED one -- the front sequence, a running prime, a lift
    // state, the retreat -- it is a CROSSING: the piece is already held, so the claw is
    // left alone and this is just the move.
    // START is a resting pose for this move, and the move out of it is DIRECT -- see
    // leaves_start_backward(). The claw still closes: a preload sits in the mechanism at
    // power-on, which is exactly the piece the grab is for.
    // The entry state, read from the table rather than through back_state_ -- that is not
    // moved until the move is known to be going ahead, so a refusal below leaves the cycle's
    // selection exactly as it was.
    const double entry_state_bar = CHAINBAR_BACK_STATE_DEG[0];
    const bool from_start = leaves_start_backward(mode_, entry_state_bar);
    bool from_rest = from_start || mode_ == Mode::STOW || mode_ == Mode::MATCHLOAD;
    if (!from_rest && !crossable_from(mode_)) {
        Logger::getInstance()->log(
            "LIFTCMD back_prime IGNORED (mode=%s, not a resting pose or a raised cycle)",
            mode_name(mode_));
        return;
    }
    back_state_ = 0;
    Logger::getInstance()->log(
        "LIFTCMD back_prime from %s (%s%s) -> state 0 (bar %.1f, winch %.0f, ee %s %.0f)",
        mode_name(mode_), from_rest ? "grab" : "crossing",
        from_start ? ", DIRECT: no clearance raise" : "", back_state_bar_deg(),
        WINCH_BACK_ENTRY_POS,
        ee_alt_ ? "ALT" : "default", ee_alt_ ? CHAIN_BACK_ALT_DEG : CHAIN_BACK_DEG);
    if (from_rest) {
        claw_.set_state(CLAW_CLOSED_STATE);  // grab the piece before moving
    }
    cancel_chain_if_external("back_prime commanded");
    supersede_startup();  // no-op unless this is the move out of the power-on pose
    // Switch the raised-orientation PAIR only -- ee_alt_ is left alone, so the flip
    // button means the same thing on both sides and a choice made before priming holds.
    ee_family_ = EeFamily::BACK;
    // The one winch command the cycle makes on its own: entering puts the cascade at a
    // known height, and everything after that is the driver's jog.
    begin_back_move(WINCH_BACK_ENTRY_POS);
    if (from_start) {
        // LOWER is the step that simply commands the pose, so entering there IS the direct
        // move. Everything before it exists to climb over the low zone, which this never
        // crosses. Set after begin_back_move(), which arms the ordinary path.
        back_step_ = BackStep::LOWER;
    }
}

double Lift::back_state_bar_deg() const {
    return back_state_bar_deg(get_winch_position());
}

double Lift::back_state_bar_deg(double winch) const {
    // Guarded rather than trusted: an index out of range would read past the table, and the
    // states are indexed from two places (the cycle and the entry).
    int i = std::clamp(back_state_, 0, BACK_STATE_COUNT - 1);
    if (i != BACK_TOP_STATE) {
        return CHAINBAR_BACK_STATE_DEG[i];
    }
    // The top state is a RANGE, not a number: the bar leans with the cascade as it climbs.
    // Read off the height the same way the front sequence reads its cascade height off the
    // bar angle -- a measured position, not a clock -- so the two axes stay consistent
    // however the driver moves the winch, including not at all.
    //
    // Clamped at both ends, which is what makes the band independent of the cycle's travel:
    // the cascade can be jogged anywhere from WINCH_TARGET_FLOOR to WINCH_MAX_POS, and
    // outside the band the angle simply holds the end it is nearest.
    double f = std::clamp(
        (winch - WINCH_BACK_TOP_LERP_BOTTOM) /
            (WINCH_BACK_TOP_LERP_TOP - WINCH_BACK_TOP_LERP_BOTTOM),
        0.0, 1.0);
    return CHAINBAR_BACK_TOP_LOW_DEG +
           (CHAINBAR_BACK_TOP_HIGH_DEG - CHAINBAR_BACK_TOP_LOW_DEG) * f;
}

void Lift::begin_back_move(double target_winch) {
    // The single path into the back choreography: entering the cycle and cycling between
    // states run the SAME raise -> tuck -> swing -> lower, and differ only in the height
    // passed here (and in the claw, which the caller has already dealt with).
    const double goal_bar = back_state_bar_deg();
    back_target_winch_ = std::clamp(target_winch, WINCH_TARGET_FLOOR, WINCH_MAX_POS);
    // Reads the bar where it is standing, so this has to happen before anything moves. A
    // crossing in from the front, a move out of MATCHLOAD, and every state-to-state cycle
    // are clear of the low zone at both ends and need no clearance raise -- the cascade
    // goes straight for its target instead of climbing to WINCH_CLEARANCE_POS and back
    // down past it. Only entering out of STOW still climbs.
    skip_clearance_raise_ = move_skips_clearance(goal_bar);
    if (skip_clearance_raise_) {
        Logger::getInstance()->log(
            "LIFTEVT back move SKIPS the clearance raise (from %s, bar %.1f -> %.1f is "
            "clear of the low zone %.0f) -- cascade goes straight for %.0f",
            mode_name(mode_), get_bar_angle(), goal_bar, CHAINBAR_LOW_ZONE_DEG,
            back_target_winch_);
    }
    mode_ = Mode::BACK;
    back_step_ = BackStep::RAISE;
    latch_entry_pose();  // hold the bar here while the cascade climbs
    // Nothing in the back cycle walks the raise table any more, so the progress readout is
    // not the back's to leave stale on the telemetry.
    raise_progress_ = 0.0;
    seq_swing_driving_ = false;
    chainbar_pid_.reset();
    winch_pid_.reset();
}

void Lift::toggle_matchload() {
    bar_kill = false;
    if (mode_ == Mode::MATCHLOAD) {
        // Matchload -> loading is just the return choreography, which already handles
        // starting part way round.
        Logger::getInstance()->log("LIFTCMD matchload toggle -> back to loading");
        begin_return(Mode::STOW);
        return;
    }
    if (mode_ != Mode::STOW) {
        // From START this is a DIRECT move -- see leaves_start_backward(). No startup
        // choreography, no clearance raise: just command the pose, which is what the
        // hop's LOWER step does.
        if (leaves_start_backward(mode_, CHAINBAR_MATCHLOAD_DEG)) {
            begin_matchload_hop(MatchloadStep::LOWER);
            return;
        }
        if (mode_ == Mode::STARTUP) {
            // Only reachable if the matchload pose is moved in front of the start angle,
            // which would put the poly back in the way -- refuse rather than swing into it.
            Logger::getInstance()->log(
                "LIFTCMD matchload IGNORED (mode=STARTUP, matchload bar %.1f is not behind "
                "the start pose %.1f -- run start_to_loading() first)",
                CHAINBAR_MATCHLOAD_DEG, CHAINBAR_START_DEG);
            return;
        }
        // Anywhere else this means "come back, to matchload" -- which is the same hop with
        // the claw released first, not the return choreography. See return_to_matchload().
        return_to_matchload();
        return;
    }
    begin_matchload_hop(MatchloadStep::RAISE);
}

void Lift::begin_matchload_hop(MatchloadStep from) {
    // Reads the bar where it is standing, so this has to happen before anything moves. The
    // hop is the LAST move to get this gate, and the reason it needs it is that matchload is
    // no longer only reached from STOW: coming from a raised pose -- the back cycle, the
    // sequence, a lift state -- both ends of the swing are past the low-clearance zone, and
    // the climb to WINCH_MATCHLOAD_CLEARANCE_POS and straight back down to the pose would be
    // several hundred degrees of cascade travel bought for nothing. Only the hop out of
    // STOW, which starts AT loading, still climbs.
    skip_clearance_raise_ = move_skips_clearance(CHAINBAR_MATCHLOAD_DEG);
    if (from == MatchloadStep::LOWER) {
        Logger::getInstance()->log(
            "LIFTCMD matchload DIRECT from %s -> bar %.1f / EE %.1f / winch %.0f "
            "(no clearance raise: nothing crosses the low zone)",
            mode_name(mode_), CHAINBAR_MATCHLOAD_DEG, CHAIN_MATCHLOAD_DEG,
            WINCH_MATCHLOAD_POS);
    } else {
        Logger::getInstance()->log(
            "LIFTCMD matchload from %s -> %s, swing bar to %.1f / EE to %.1f at <=%.0f dps, "
            "lower to %.0f",
            mode_name(mode_),
            skip_clearance_raise_ ? "NO clearance raise (both ends clear of the low zone)"
                                  : "clearance raise",
            CHAINBAR_MATCHLOAD_DEG, CHAIN_MATCHLOAD_DEG, CHAINBAR_MATCHLOAD_SWING_MAX_DPS,
            WINCH_MATCHLOAD_POS);
    }
    cancel_chain_if_external("matchload commanded");
    supersede_startup();  // no-op unless this is the move out of the power-on pose
    mode_ = Mode::TO_MATCHLOAD;
    matchload_step_ = from;
    latch_entry_pose();  // what RAISE holds while the cascade climbs; unread entering at LOWER
    chainbar_pid_.reset();
    winch_pid_.reset();
}

void Lift::move_lift_up() {
    // Consumed in update(); only actually advances while in the raise sequence.
    bar_kill = false;
    advancing_ = true;
}

void Lift::move_lift_down() {
    // Consumed in update(); only actually retreats while in the raise sequence.
    bar_kill = false;
    retreating_ = true;
}

void Lift::score() {
    bar_kill = false;
    // Release where the mechanism already is -- nothing moves onto the tower. Valid in the
    // raise sequence, in a back state from ANY step, and in a lift state once ARRIVED.
    //
    // The back cycle scores MID-MOVE on purpose: the driver releases when the piece is over
    // the target, and on a slow swing -- or one the EE is still fighting its way round --
    // that is well before the choreography calls itself arrived. The release itself is one
    // solenoid and is safe from anywhere. What is NOT safe mid-move is the alt-orientation
    // RETREAT below, which takes the mode over and swings the bar with the cascade still in
    // transit, so a mid-move score releases, holds, and lets the back move it interrupted
    // carry on to its state. The lift-state path still waits for HOLD: those rows are walked
    // by auton and a chain, which have no driver watching the piece.
    bool from_back = mode_ == Mode::BACK;
    bool from_lift_state = mode_ == Mode::LIFT_STATE && state_step_ == StateStep::HOLD;
    if (mode_ != Mode::SEQUENCE && !from_back && !from_lift_state) {
        Logger::getInstance()->log(
            "LIFTCMD score IGNORED (mode=%s%s, not in the sequence, a back state or a lift state)",
            mode_name(mode_),
            mode_ == Mode::LIFT_STATE ? " mid-move" : "");
        return;
    }
    // Arrived-ness is no longer what admits a back score, but it still decides the retreat.
    bool back_mid_move = from_back && back_step_ != BackStep::HOLD;
    double bar = get_bar_angle();
    // A lift state names its own EE angle, so ee_alt_ does not describe how the EE is
    // actually oriented and must not decide whether the retreat runs. A lift state that
    // ever wants one should say so with a per-row flag, not ee_alt_.
    bool retreat = ee_alt_ && !from_lift_state && !back_mid_move;
    Logger::getInstance()->log(
        "LIFTCMD score (%s%s) release @ p=%.2f bar=%.1f winch=%.1f, ee %s (tgt %.0f)%s",
        from_lift_state ? current_lift_state() : (from_back ? "back" : "front"),
        back_mid_move ? " mid-move" : "",
        raise_progress_, bar, get_winch_position(),
        from_lift_state ? "lift-state" : (ee_alt_ ? "ALT" : "default"),
        from_lift_state ? state_target_->ee_deg : raised_ee_deg(),
        retreat ? " -> retreat"
                : (ee_alt_ && back_mid_move
                       ? " -> hold (ALT retreat SKIPPED: mid-move, the back swing keeps the bar)"
                       : " -> hold"));
    cancel_chain_if_external("score commanded");
    claw_.set_state(CLAW_OPEN_STATE);
    if (!retreat) {
        // DEFAULT orientation, any lift state, or a mid-move back score: the EE is clear as
        // soon as the claw opens -- or, mid-move, the swing that is still running owns the
        // bar -- so hold, and leave the mode alone for the driver / autonomous.
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

void Lift::bar_pose() {
    bar_kill = false;
    skip_clearance_raise_ = move_skips_clearance(CHAINBAR_BAR_POSE_DEG);
    bar_pose_winch_ = std::clamp(get_winch_position(), WINCH_TARGET_FLOOR, WINCH_MAX_POS);
    Logger::getInstance()->log(
        "LIFTCMD bar_pose from %s -> swing bar %.1f -> %.1f at <=%.0f dps, %s "
        "(claw %s UNTOUCHED, EE held at %.1f, cascade back to %.1f)",
        mode_name(mode_), get_bar_angle(), CHAINBAR_BAR_POSE_DEG,
        CHAINBAR_STATE_SWING_MAX_DPS,
        skip_clearance_raise_ ? "NO clearance raise (both ends clear of the low zone): the "
                                "cascade is never commanded off its height"
                              : "clearance raise + lower back (the swing crosses the low "
                                "zone, so the lift has to be up to pass the intake poly)",
        claw_.get_state() == CLAW_CLOSED_STATE ? "CLOSED" : "OPEN",
        get_ee_angle(), bar_pose_winch_);
    cancel_chain_if_external("bar_pose commanded");
    supersede_startup();  // no-op unless this is the move out of the power-on pose
    mode_ = Mode::BAR_POSE;
    bar_pose_step_ = BarPoseStep::RAISE;
    // NOT latch_entry_pose(): that tucks the EE to CHAIN_PRIME_RAISE_DEG out of a low pose,
    // which is a change of orientation, and this move does not make any.
    entry_bar_deg_ = get_bar_angle();
    bar_target_deg_ = entry_bar_deg_;
    entry_ee_deg_ = get_ee_angle();
    chainbar_pid_.reset();
    winch_pid_.reset();
}

void Lift::toggle_kill_switch() {
    bar_kill = true;
}

void Lift::open_claw() {
    bar_kill = false;
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
    bar_kill = false;
    // Only the SELECTION is stored, never an angle -- raised_ee_deg() resolves it against
    // the cycle in force, and every mode re-reads that each tick, so a flip lands at once.
    ee_alt_ = !ee_alt_;
    Logger::getInstance()->log(
        "LIFTCMD flip scoring orientation -> %s %.0f (mode=%s ee=%.1f)",
        ee_alt_ ? "ALT" : "default", raised_ee_deg(), mode_name(mode_), get_ee_angle());
}

// ---- Autonomous interface (keyed macros) -----------------------------------

const Lift::LiftState* Lift::find_lift_state(Side side, const char* key) {
    if (key == nullptr) {
        return nullptr;
    }
    const LiftState* table = (side == Side::BACK) ? BACK_LIFT_STATES : FRONT_LIFT_STATES;
    int count = (side == Side::BACK) ? BACK_LIFT_STATE_COUNT : FRONT_LIFT_STATE_COUNT;
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
    else if (std::strcmp(key, "bar_pose") == 0)     { bar_pose(); }
    else if (std::strcmp(key, "up") == 0)           { move_lift_up(); }
    else if (std::strcmp(key, "down") == 0)         { move_lift_down(); }
    else {
        Logger::getInstance()->logError(
            "LIFTCMD run_macro REFUSED: unknown key '%s' (expected prime / matchload / "
            "score / return / return_matchload / claw / flip / bar_pose / up / down)", key);
        return false;
    }
    return true;
}

bool Lift::run_lift_state(Side side, const char* key) {
    const LiftState* target = find_lift_state(side, key);
    if (target == nullptr) {
        Logger::getInstance()->logError(
            "LIFTCMD lift_state REFUSED: no %s state named '%s'",
            side == Side::BACK ? "BACK" : "FRONT", key ? key : "(null)");
        return false;
    }
    // No mode is refused, STARTUP included: auton commands a pose straight off the power-on
    // pose, and this choreography reads no startup state (RAISE holds the bar wherever it
    // is, and the STARTUP-only sweep gates all test mode_).
    // Whether the swing crosses the low-clearance zone, i.e. whether the clearance raise
    // runs at all. Decided per move in advance_lift_state_sequence(); logged here so a
    // move that skipped the raise says so at the point it was commanded.
    bool needs_clearance = !bar_clear_of_low_zone(get_bar_angle()) ||
                           !bar_clear_of_low_zone(target->bar_deg);
    Logger::getInstance()->log(
        "LIFTCMD lift_state %s '%s' from %s -> claw %s (was %s), %s, swing bar %.1f -> %.1f, "
        "EE -> %.1f, lower to %.0f",
        side == Side::BACK ? "BACK" : "FRONT", target->key, mode_name(mode_),
        target->claw_state == CLAW_CLOSED_STATE ? "CLOSED" : "OPEN",
        claw_.get_state() == CLAW_CLOSED_STATE ? "CLOSED" : "OPEN",
        needs_clearance ? "raise for clearance"
                        : "NO raise (both ends clear of the low zone)",
        get_bar_angle(), target->bar_deg, target->ee_deg, target->winch_pos);
    // The claw is part of the state, and it is written HERE -- before anything moves --
    // so the whole choreography runs with the claw already where the row asks for it. A
    // state that opens it therefore releases where the mechanism is STANDING, exactly as
    // score() does; one that closes it does NOT wait out CLAW_GRAB_SETTLE_MS, since this
    // is not the pull off the loader that wait exists for (see prime()).
    // A pose commanded from anywhere ELSE supersedes a running chain. The guard inside
    // is what lets the chain command its own steps through here.
    cancel_chain_if_external("single lift state commanded");
    claw_.set_state(target->claw_state);
    state_target_ = target;
    // Which pair a later flip resolves against -- the pose names its own EE angle, but the
    // cycle it hands over to needs this.
    ee_family_ = (side == Side::BACK) ? EeFamily::BACK : EeFamily::FRONT;
    supersede_startup();
    mode_ = Mode::LIFT_STATE;
    state_step_ = StateStep::RAISE;
    // Hold the bar where the previous pose left it while the lift climbs, same as prime().
    latch_entry_pose();
    chainbar_pid_.reset();
    winch_pid_.reset();
    return true;
}

// ---- Chains ----------------------------------------------------------------

const Lift::LiftChain* Lift::find_chain(double id) {
    // Tolerance rather than ==: the id arrives as a double parsed out of the route JSON,
    // and a chain that silently fails to match because of a representation wobble is a
    // very expensive thing to debug on a field.
    for (int i = 0; i < LIFT_CHAIN_COUNT; ++i) {
        if (std::abs(LIFT_CHAINS[i].id - id) < 1e-6) {
            return &LIFT_CHAINS[i];
        }
    }
    return nullptr;
}

const Lift::LiftChain* Lift::find_chain(const char* name) {
    if (name == nullptr) {
        return nullptr;
    }
    for (int i = 0; i < LIFT_CHAIN_COUNT; ++i) {
        if (std::strcmp(LIFT_CHAINS[i].name, name) == 0) {
            return &LIFT_CHAINS[i];
        }
    }
    return nullptr;
}

bool Lift::run_lift_chain(double id) {
    const LiftChain* chain = find_chain(id);
    if (chain == nullptr) {
        Logger::getInstance()->logError(
            "LIFTCHAIN REFUSED: no chain with id %.2f (%d defined)", id, LIFT_CHAIN_COUNT);
        return false;
    }
    return start_chain(chain);
}

bool Lift::run_lift_chain(const char* name) {
    const LiftChain* chain = find_chain(name);
    if (chain == nullptr) {
        Logger::getInstance()->logError(
            "LIFTCHAIN REFUSED: no chain named '%s' (%d defined)",
            name ? name : "(null)", LIFT_CHAIN_COUNT);
        return false;
    }
    return start_chain(chain);
}

bool Lift::start_chain(const LiftChain* chain) {
    if (chain_ != nullptr) {
        Logger::getInstance()->log(
            "LIFTCHAIN '%s' SUPERSEDED at step %d/%d by '%s'",
            chain_->name, chain_step_ + 1, chain_->step_count, chain->name);
    }
    Logger::getInstance()->log(
        "LIFTCHAIN '%s' (id %.1f) START -- %d steps, from mode=%s bar=%.1f winch=%.1f",
        chain->name, chain->id, chain->step_count, mode_name(mode_),
        get_bar_angle(), get_winch_position());
    chain_ = chain;
    chain_step_ = 0;

    dispatch_chain_step(pros::millis());

    return chain_ != nullptr;
}

void Lift::dispatch_chain_step(uint32_t now) {
    const ChainStep& step = chain_->steps[chain_step_];
    chain_step_ms_ = now;
    chain_step_settled_ = false;
    if (step.key == nullptr) {
        // Pure dwell: hold whatever the previous step commanded for hold_ms.
        Logger::getInstance()->log(
            "LIFTCHAIN '%s' step %d/%d DWELL %u ms (holding the previous pose)",
            chain_->name, chain_step_ + 1, chain_->step_count, step.hold_ms);
        return;
    }
    Logger::getInstance()->log(
        "LIFTCHAIN '%s' step %d/%d -> %s '%s' (%s, then hold %u ms)",
        chain_->name, chain_step_ + 1, chain_->step_count,
        step.side == Side::BACK ? "BACK" : "FRONT", step.key,
        step.wait_settled ? "wait for arrival" : "no wait", step.hold_ms);
    // Through the PUBLIC entry point, so a chained pose is choreographed exactly like a
    // route-commanded one. The guard is what stops its cancel hook killing us.
    chain_dispatching_ = true;
    bool armed = run_lift_state(step.side, step.key);
    chain_dispatching_ = false;
    if (!armed) {
        // The table is key-checked at build time, so this is unreachable short of memory
        // corruption -- but a chain that silently stops is worse than one that says so.
        Logger::getInstance()->logError(
            "LIFTCHAIN '%s' step %d/%d FAILED to arm '%s' -- abandoning the chain",
            chain_->name, chain_step_ + 1, chain_->step_count, step.key);
        chain_ = nullptr;
    }
}

void Lift::advance_chain(uint32_t now) {
    if (chain_ == nullptr) {
        return;
    }
    const ChainStep& step = chain_->steps[chain_step_];
    uint32_t elapsed = now - chain_step_ms_;

    // Phase 1: the arrival gate, if this step asked for one.
    if (step.wait_settled && !chain_step_settled_) {
        // mode_busy(), NOT is_busy(): the chain is itself busy, so is_busy() would never
        // go false and the step would sit here until its timeout, every time.
        if (!mode_busy() && pose_arrived()) {
            Logger::getInstance()->log(
                "LIFTCHAIN '%s' step %d/%d ARRIVED after %u ms -- holding %u ms",
                chain_->name, chain_step_ + 1, chain_->step_count, elapsed, step.hold_ms);
        } else if (elapsed >= step.settle_timeout_ms) {
            Logger::getInstance()->log(
                "LIFTCHAIN '%s' step %d/%d settle TIMED OUT after %u ms "
                "(bar=%.1f/%.1f winch=%.1f/%.1f ee=%.1f/%.1f) -- carrying on",
                chain_->name, chain_step_ + 1, chain_->step_count, elapsed,
                get_bar_angle(), bar_target_deg_, get_winch_position(), winch_manual_target_,
                get_ee_angle(), chain_target_deg_);
        } else {
            return;  // still travelling
        }
        // Either way the dwell starts HERE, so hold_ms means "on the pose for this long"
        // rather than "this long after we set off for it".
        chain_step_settled_ = true;
        chain_step_ms_ = now;
        elapsed = 0;
    }

    // Phase 2: the dwell.
    if (elapsed < step.hold_ms) {
        return;
    }

    // Phase 3: on to the next step, or done.
    ++chain_step_;
    if (chain_step_ >= chain_->step_count) {
        // Whatever the last step commanded is LEFT commanded -- the chain scripts the
        // sequence, it does not own the mechanism afterwards.
        bool on_state = mode_ == Mode::LIFT_STATE && state_target_ != nullptr;
        Logger::getInstance()->log(
            "LIFTCHAIN '%s' DONE (%d steps) -- holding %s (bar=%.1f winch=%.1f)",
            chain_->name, chain_->step_count,
            on_state ? state_target_->key : mode_name(mode_),
            get_bar_angle(), get_winch_position());
        chain_ = nullptr;
        chain_step_ = 0;
        chain_step_settled_ = false;
        return;
    }
    dispatch_chain_step(now);
}

void Lift::cancel_chain(const char* why) {
    if (chain_ == nullptr) {
        return;
    }
    Logger::getInstance()->log(
        "LIFTCHAIN '%s' CANCELLED at step %d/%d (%s) -- the pose it last commanded stands",
        chain_->name, chain_step_ + 1, chain_->step_count, why ? why : "superseded");
    chain_ = nullptr;
    chain_step_ = 0;
    chain_step_settled_ = false;
}

void Lift::cancel_chain_if_external(const char* why) {
    // dispatch_chain_step() drives the mechanism through run_lift_state(), which calls
    // this. Without the guard a chain would cancel itself on its own first step.
    if (!chain_dispatching_) {
        cancel_chain(why);
    }
}

bool Lift::chain_active() const {
    return chain_ != nullptr;
}

const char* Lift::current_chain() const {
    return chain_ != nullptr ? chain_->name : nullptr;
}

int Lift::current_chain_step() const {
    return chain_ != nullptr ? chain_step_ + 1 : 0;
}

bool Lift::is_busy() const {
    // A chain is busy for its WHOLE run, dwells included: between two steps nothing is
    // moving and mode_busy() below is false, so without this a waitForLift() would return
    // in the gap and the rest of the chain would fire underneath whatever ran next.
    return chain_active() || mode_busy();
}

bool Lift::mode_busy() const {
    // A prime waiting out its claw grab has not moved yet and mode_ is still a resting pose,
    // which reads as idle below -- so say so here, or a macro's wait sails straight past the
    // whole prime.
    if (prime_grab_pending_) return true;
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
        case Mode::LIFT_STATE:
            return state_step_ != StateStep::HOLD;
        case Mode::BAR_POSE:
            return bar_pose_step_ != BarPoseStep::HOLD;
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

bool Lift::is_priming_from_rest() const {
    // The claw grab counts: the piece is being clamped in the intake right then, which is
    // the first half of what the feed is for. A CROSSING out of a raised cycle is not one
    // of these -- the piece is nowhere near the intake, so there is nothing to feed.
    return prime_from_rest_ && (prime_grab_pending_ || mode_ == Mode::PRIMING);
}

bool Lift::is_matchloading() const {
    // The pose only. TO_MATCHLOAD is still swinging, and its own choreography commands the
    // claw, so a claw toggle there would be overwritten anyway.
    return mode_ == Mode::MATCHLOAD;
}

bool Lift::is_settled() const {
    return !is_busy() && pose_arrived();
}

bool Lift::pose_arrived() const {
    // Actually on the pose. These targets are the ones update() resolved last tick, so it
    // only means anything once update() has run since the command.
    // The bar is exempt in STOW while the constant loading hold is in force: it is not
    // position-controlled there, so its rest angle is whatever the mechanism allows and
    // testing it against CHAINBAR_LOADING_DEG could keep waitForLift() waiting forever.
    double bar = get_bar_angle();
    bool bar_settled = (mode_ == Mode::STOW && CHAINBAR_LOADING_HOLD_MV != 0.0) ||
                       std::abs(bar - bar_target_deg_) < CHAINBAR_POS_TOL;
    // The EE is judged against what is actually COMMANDED, not what the pose asked for: in a
    // size-keep band the override is holding it at CHAIN_SIZE_KEEP_EE_DEG and the pose's own
    // angle is one it will not be given until the bar leaves. Tested against the target, a
    // chain step with wait_settled would sit out its whole settle timeout every time it
    // stopped in a band.
    double ee_goal = (size_keep_zone_at(bar) >= 0)
                         ? unwrap_angle_near(CHAIN_SIZE_KEEP_EE_DEG, get_ee_angle())
                         : chain_target_deg_;
    return std::abs(get_winch_position() - winch_manual_target_) < WINCH_POS_TOL &&
           bar_settled &&
           std::abs(get_ee_angle() - ee_goal) < CHAIN_POS_TOL;
}

const char* Lift::current_lift_state() const {
    if (mode_ != Mode::LIFT_STATE || state_target_ == nullptr) {
        return nullptr;
    }
    return state_target_->key;
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
    cancel_chain_if_external("return commanded");
    start_step_ = StartStep::IDLE;  // supersede any running startup sequence
    raise_progress_ = 0.0;
    claw_.set_state(CLAW_OPEN_STATE);  // release the piece before moving
    // The other half of move_skips_clearance(): a return to MATCHLOAD stops at
    // CHAINBAR_MATCHLOAD_DEG, short of the low-clearance zone, so the sweep home never
    // enters it and the cascade has nothing to climb over. A return to STOW lands ON
    // loading, inside the zone, so that one always raises -- the geometry decides, same as
    // the outbound moves. Read the bar before anything moves.
    skip_clearance_raise_ = move_skips_clearance(
        to_match ? CHAINBAR_MATCHLOAD_DEG : CHAINBAR_LOWER_CLEAR_DEG);
    if (skip_clearance_raise_) {
        Logger::getInstance()->log(
            "LIFTEVT return SKIPS the clearance raise (bar %.1f -> %.1f is clear of the low "
            "zone %.0f) -- cascade goes straight for %.0f",
            get_bar_angle(), CHAINBAR_MATCHLOAD_DEG, CHAINBAR_LOW_ZONE_DEG,
            WINCH_MATCHLOAD_POS);
    }
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
    bar_kill = false;
    begin_return(Mode::STOW);
}

void Lift::return_to_matchload() {
    // NOT the return choreography, despite the name. That one sweeps HOME -- toward a less
    // negative bar angle -- and every gate in it is written that way: sweep_dir(), the
    // CHAIN_FLIP_BAR_DEG hand-over, alt_return_hold_active(), the coast window. The
    // matchload pose sits at CHAINBAR_MATCHLOAD_DEG, well past the raised cycles, so the bar
    // travels the OTHER way to reach it from anywhere the mechanism can be commanded from,
    // and running that travel through the return's law reads every one of those gates
    // backwards.
    //
    // The hop is the move that actually goes there, and now that it skips its clearance
    // raise when nothing crosses the low zone, coming from a raised pose is just the swing:
    // no climb, and paced by CHAINBAR_MATCHLOAD_SWING_MAX_DPS so the EE has the length of it
    // to turn round in.
    //
    // The claw still opens HERE rather than at the pose, which is the one thing this kept
    // from the return: "return" means release what is held, and the hop's own open on
    // arrival is then a no-op.
    claw_.set_state(CLAW_OPEN_STATE);
    begin_matchload_hop(MatchloadStep::RAISE);
}

bool Lift::needs_startup() const {
    // Nothing has commanded the lift yet: still in the power-on mode with no startup
    // sequence running. Anything auton does moves mode_ off STARTUP (or arms start_step_),
    // so this is false once the lift has been used.
    return mode_ == Mode::STARTUP && start_step_ == StartStep::IDLE;
}

void Lift::supersede_startup() {
    start_step_ = StartStep::IDLE;  // supersede any running startup sequence
    // Taking over from STARTUP means the choreography that would have set this never runs,
    // and the reference it would have vouched for is the same one initialize() seeded. Arm
    // the re-zero cap here instead, so a false respool stall can't rewrite it by any amount.
    startup_complete_ = true;
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
    // This choreography IS the clearance move: it lands on loading, inside the zone.
    skip_clearance_raise_ = false;
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
    // Height held through RAISE / TUCK / SWEEP. When the bar is past the low-clearance zone
    // at both ends -- crossing in from the back cycle, or priming out of MATCHLOAD -- there
    // is nothing to hold the cascade up FOR: it takes the primed height from the first tick
    // and never makes the round trip up to WINCH_PRIME_CLEARANCE_POS and back. Only a prime
    // out of STOW still climbs. See move_skips_clearance().
    const double clear_winch = skip_clearance_raise_ ? primed_winch
                                                     : WINCH_PRIME_CLEARANCE_POS;
    PrimeStep step_before = prime_step_;
    switch (prime_step_) {
        case PrimeStep::RAISE: {
            // Raise the lift for clearance, holding the bar at the pose the prime started
            // from so it never detours between the two resting angles down low.
            winch_manual_target_ = clear_winch;
            bar_target_deg_ = entry_bar_deg_;
            chain_target_deg_ = entry_ee_deg_;
            if (skip_clearance_raise_) {
                // No climb for the tuck to play out underneath, and nothing to tuck FOR --
                // the bar and claw are already past the zone, which is the only thing the
                // tuck gets them out of the way of. The sweep is the whole move.
                prime_step_ = PrimeStep::SWEEP;
                chainbar_pid_.reset();
            } else if (winch >= WINCH_PRIME_TUCK_POS) {
                // Hand over to TUCK part way up -- a plain threshold, not an arrival check,
                // since this is a height the lift passes through rather than stops at.
                prime_step_ = PrimeStep::TUCK;
                chainbar_pid_.reset();  // hold -> profiled move: start the PID clean
            }
            break;
        }
        case PrimeStep::TUCK: {
            // Same winch target as RAISE, so the lift carries on up and the tuck plays out
            // underneath it. The exit tests the winch only: SWEEP needs the lift up before
            // it throws the bar, and the tuck angle is a waypoint it passes through.
            winch_manual_target_ = clear_winch;
            if (bar > CHAINBAR_PRIME_TUCK_DEG) {
                bar_target_deg_ = slew_bar_toward(bar_target_deg_, CHAINBAR_PRIME_TUCK_DEG,
                                                  bar, bar_target_vel_dps_);
                chain_target_deg_ = CHAIN_PRIME_TUCK_DEG;
            } else {
                // Crossing in from a raised pose that is ALREADY past the tuck: it is a
                // waypoint on the way round, not a pose to come back to, so hold here and
                // let SWEEP carry on from this angle rather than swing back up to it.
                bar_target_deg_ = entry_bar_deg_;
                chain_target_deg_ = entry_ee_deg_;
            }
            if (winch >= WINCH_PRIME_CLEARANCE_POS - WINCH_POS_TOL) {
                prime_step_ = PrimeStep::SWEEP;
                chainbar_pid_.reset();
            }
            break;
        }
        case PrimeStep::SWEEP: {
            // The return's RAISE_SWEEP the other way round: lift HELD UP, one STEP target at
            // primed, FF-free and capped, coasting over the top (see bar_sweeping()). The
            // winch stays put so the two heavy axes never rail at once -- LOWER has it.
            // Skipping the clearance raise this IS the primed height, so the cascade closes
            // whatever the move owes it (200 deg up out of the back entry pose, nothing at
            // all out of matchload) under the sweep instead of after it -- safe for the same
            // reason the raise was skippable: the bar is clear of the frame the whole way.
            winch_manual_target_ = clear_winch;
            bar_target_deg_ = primed_bar;
            // Stay where the climb left the EE until the bar is far enough round to flip --
            // going back to CHAIN_LOADING_DEG here would swing the EE back and then
            // straight on again. By CHAIN_FLIP_BAR_DEG the coast is already running, so the
            // turn lands on a free-swinging bar rather than on a control hand-over.
            chain_target_deg_ = entry_ee_deg_;
            if (bar < CHAIN_FLIP_BAR_DEG){
                chain_target_deg_ = raised_ee_deg();
            }
            if (std::abs(bar - primed_bar) < CHAINBAR_POS_TOL) {
                prime_step_ = PrimeStep::LOWER;
            }
            break;
        }
        case PrimeStep::LOWER: {
            // Bar is parked on primed; bring the lift down onto the primed height. Mirrors
            // ReturnStep::LOWER -- one axis at a time.
            winch_manual_target_ = primed_winch;
            bar_target_deg_ = primed_bar;
            chain_target_deg_ = raised_ee_deg();
            if (std::abs(winch - primed_winch) < WINCH_POS_TOL) {
                prime_step_ = PrimeStep::SETTLE;
                chainbar_pid_.reset();
            }
            break;
        }
        case PrimeStep::SETTLE: {
            // Lift is down: walk the bar the last couple of degrees onto primed, slewed, so
            // it lands rather than arrives at speed. Mirrors ReturnStep::BAR_SETTLE.
            winch_manual_target_ = primed_winch;
            bar_target_deg_ = slew_bar_toward(bar_target_deg_, primed_bar, bar,
                                              bar_target_vel_dps_);
            chain_target_deg_ = raised_ee_deg();
            if (settle_bar_arrived(bar, primed_bar)) {
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
            // A sweep that stops short of the zone (the matchload return) has no last
            // stretch to clear, so it takes the destination height from the first tick and
            // LOWER passes straight through.
            const double sweep_winch = skip_clearance_raise_ ? rest_winch
                                                             : WINCH_CLEARANCE_POS;
            winch_manual_target_ = sweep_winch;
            if (alt_return_hold_active(bar)) {
                // Flipped EE not yet back round: the lift may still RISE but must not come
                // down on it, and after a score it usually sits above the clearance height.
                // Skipping the raise this is the ONLY thing holding it up -- the height the
                // return started at, until the EE has swung back past the flip angle.
                winch_manual_target_ = std::max(sweep_winch, return_hold_winch_);
            }
            // "The bar may aim at its goal": either the lift is up, or the goal was never
            // behind the zone to begin with.
            bool lift_clear = skip_clearance_raise_ ||
                              winch >= WINCH_CLEARANCE_POS - WINCH_POS_TOL;
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
            // Bar is parked on the sweep goal; lower onto the destination height. Nothing
            // to do when the sweep already ran at that height (the raise was skipped) --
            // this passes through on the first tick, like the outbound moves' LOWER.
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
    // Height held through RAISE and BAR_SWEEP: the hop's clearance height, or the pose's own
    // when the swing never enters the low-clearance zone (see begin_matchload_hop()). In the
    // second case the cascade settles onto the pose UNDER the swing rather than after it and
    // LOWER passes straight through -- the same shape the back cycle and the lift states use
    // when they skip their raise, and safe for the same reason: the bar stays clear of the
    // frame the whole way.
    const double clear_winch = skip_clearance_raise_ ? WINCH_MATCHLOAD_POS
                                                     : WINCH_MATCHLOAD_CLEARANCE_POS;
    MatchloadStep step_before = matchload_step_;
    switch (matchload_step_) {
        case MatchloadStep::RAISE: {
            // Lift up first -- the bar has to cross the low-clearance zone -- but only to
            // WINCH_MATCHLOAD_CLEARANCE_POS, since a short arc at the loading end does not
            // need the full height. Bar held where the pose left it, EE tucked. With nothing
            // to climb over this passes straight through on the first tick and the swing
            // starts immediately.
            winch_manual_target_ = clear_winch;
            bar_target_deg_ = entry_bar_deg_;
            chain_target_deg_ = entry_ee_deg_;
            if (skip_clearance_raise_ ||
                winch >= WINCH_MATCHLOAD_CLEARANCE_POS - WINCH_POS_TOL) {
                matchload_step_ = MatchloadStep::BAR_SWEEP;
                chainbar_pid_.reset();
            }
            break;
        }
        case MatchloadStep::BAR_SWEEP: {
            // Swing onto the pose with the lift held up. SLEWED, like BAR_SETTLE: a short
            // climb that has to LAND on a pose, and a step target would arrive at speed
            // with nothing to decelerate it (log 07262026-3).
            //
            // On the hop's OWN rate, slower than the slew default: the EE is turning onto
            // CHAIN_MATCHLOAD_DEG underneath this, and the poses the hop is commanded from
            // do not all leave it room to turn where it stands -- so the bar is paced to
            // give the chain the length of the swing to get round in. See
            // CHAINBAR_MATCHLOAD_SWING_MAX_DPS; the decel taper is still the default.
            winch_manual_target_ = clear_winch;
            bar_target_deg_ = slew_bar_toward(bar_target_deg_, CHAINBAR_MATCHLOAD_DEG, bar,
                                              bar_target_vel_dps_,
                                              CHAINBAR_MATCHLOAD_SWING_MAX_DPS);
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

void Lift::advance_back_sequence(double bar, double winch, double& winch_jog_mv) {
    // The move onto the selected state: raise, tuck, swing, lower, then HOLD. Each step
    // passes through when it has nothing to do, so the direction takes care of itself, and
    // the bar only ever crosses the low-clearance zone in SWING, which only runs once the
    // lift is up (TUCK moves it only as far as the destination).
    // The height is back_target_winch_ -- the entry height when this move entered the cycle,
    // and the height the mechanism was already at when it is a state-to-state cycle, which
    // is what makes cycling leave the driver's jog alone.
    //
    // The angle is re-read from the CURRENT height every tick, not latched, because the top
    // state's is a function of it (see back_state_bar_deg()). For the other two states this
    // is a constant and the read costs nothing; for the top one it is what makes the bar
    // follow the jog. It also means the swing onto a state aims at the angle that state will
    // actually hold at the height it is being flown at, so HOLD inherits the target rather
    // than stepping to a new one.
    const double goal_bar = back_state_bar_deg(winch);
    const double goal_winch = back_target_winch_;
    // Height held during RAISE and SWING: the clearance height, or the destination if that
    // is higher. Never a LOW destination -- that is what LOWER is for. When both ends of the
    // bar travel are past the low-clearance zone, though -- crossing in from the front,
    // moving out of MATCHLOAD, or any cycle between two states -- there is nothing to hold
    // the cascade up for, so the target is the destination height from the first tick: no
    // climb to WINCH_CLEARANCE_POS and straight back down past it. See
    // move_skips_clearance().
    const double clear_winch = skip_clearance_raise_ ? goal_winch
                                                     : std::max(WINCH_CLEARANCE_POS, goal_winch);
    BackStep step_before = back_step_;
    switch (back_step_) {
        case BackStep::RAISE: {
            // Lift up first, bar held wherever the previous pose left it. Out of a LOW pose
            // the EE tucks for the climb rather than turning to the back orientation down
            // low; crossing in from a raised one it just holds (see latch_entry_pose()).
            winch_manual_target_ = clear_winch;
            bar_target_deg_ = entry_bar_deg_;
            chain_target_deg_ = entry_ee_deg_;
            if (skip_clearance_raise_) {
                // No climb for the tuck to play out underneath, and the bar and claw are
                // already past the zone the tuck exists to clear -- the swing is the move.
                back_step_ = BackStep::SWING;
                chainbar_pid_.reset();
            } else if (winch >= WINCH_PRIME_TUCK_POS) {
                // Hand over to TUCK part way up, on the same threshold the prime uses -- a
                // height the lift passes through, not one it stops at.
                back_step_ = BackStep::TUCK;
                chainbar_pid_.reset();  // hold -> profiled move: start the PID clean
            }
            break;
        }
        case BackStep::TUCK: {
            // The prime's tuck, same angles: get the bar and claw part way round while the
            // lift is still climbing, instead of dragging them up in the loading pose and
            // doing the whole move afterwards. Same winch target as RAISE, so the climb
            // carries on underneath, and the exit tests the winch only -- SWING needs the
            // lift up before it throws the bar.
            winch_manual_target_ = clear_winch;
            // Never PAST the destination: the first back state sits short of the tuck angle,
            // and tucking to CHAINBAR_PRIME_TUCK_DEG only to swing back out to it is a detour.
            const double tuck_goal = std::max(CHAINBAR_PRIME_TUCK_DEG, goal_bar);
            if (bar > tuck_goal) {
                bar_target_deg_ = slew_bar_toward(bar_target_deg_, tuck_goal, bar,
                                                  bar_target_vel_dps_);
                chain_target_deg_ = CHAIN_PRIME_TUCK_DEG;
            } else {
                // Already past it: a re-entry from further round the cycle, or a crossing
                // in from the front. Nothing to tuck -- hold, and let SWING carry on.
                bar_target_deg_ = entry_bar_deg_;
                chain_target_deg_ = entry_ee_deg_;
            }
            if (winch >= WINCH_CLEARANCE_POS - WINCH_POS_TOL) {
                back_step_ = BackStep::SWING;
                chainbar_pid_.reset();
            }
            break;
        }
        case BackStep::SWING: {
            // Swing onto the state with the lift held up. SLEWED, like the matchload sweep
            // -- the decel taper is what keeps the downhill direction from arriving hot.
            // This is the WHOLE move when cycling between two states: the raise was skipped,
            // clear_winch is the height the mechanism is already at, and LOWER below then
            // passes straight through. So a cycle press is one swing and nothing else.
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
            // Bar is on the state; bring the lift onto the move's height. Entered directly
            // out of the power-on pose this is the whole move (see back_prime()), and on a
            // state-to-state cycle it is already satisfied and passes through.
            winch_manual_target_ = goal_winch;
            bar_target_deg_ = goal_bar;
            chain_target_deg_ = raised_ee_deg();
            if (std::abs(winch - goal_winch) < WINCH_POS_TOL) {
                back_step_ = BackStep::HOLD;
                winch_pid_.reset();
            }
            break;
        }
        case BackStep::HOLD: {
            // Arrived. The cascade is the driver's: L1/L2 drive the winch at WINCH_JOG_MV,
            // and letting go parks it where it stands (see the jog block in update()).
            //
            // The bar holds the state's angle. In the two lower states that is a constant
            // and back_prime() is the only thing that moves it; in the TOP state it is the
            // interpolation, so the jog moves BOTH axes -- the winch because the button
            // drives it, the bar because goal_bar is a function of where the winch got to.
            // That is the whole coupling: no profile, no clock, just the angle the height
            // asks for, which means it also holds still the moment the jog does.
            bar_target_deg_ = goal_bar;
            chain_target_deg_ = raised_ee_deg();
            // Raising wins if both are held, the same rule the front sequence follows.
            if (advancing_) {
                winch_jog_mv = WINCH_JOG_MV;
            } else if (retreating_) {
                winch_jog_mv = -WINCH_JOG_MV;
            }
            if (winch_jog_mv != 0.0) {
                // The jog owns the cascade while a button is held, so the height it reaches
                // IS the height to hold: dragging the target along under it is what makes
                // releasing park where the driver stopped rather than snap back to where
                // they started. Clamped here as well as in update(), since this height
                // outlives the tick -- a later state cycle carries it through its swing.
                back_target_winch_ = std::clamp(winch, WINCH_TARGET_FLOOR, WINCH_MAX_POS);
            }
            winch_manual_target_ = back_target_winch_;
            break;
        }
    }
    if (back_step_ != step_before) {
        Logger::getInstance()->log(
            "LIFTEVT back step %s -> %s (state %d, bar=%.1f/%.1f winch=%.1f/%.0f)",
            back_step_name(step_before), back_step_name(back_step_), back_state_,
            bar, goal_bar, winch, goal_winch);
    }
}

void Lift::advance_lift_state_sequence(double bar, double winch) {
    // The back cycle's choreography with the destination read from a table. Kept separate
    // because the two differ in what HOLD means; the shape is identical because swinging
    // only in SWING is what keeps an arbitrary row clear of the low-clearance zone.
    // The claw is NOT stepped here -- run_lift_state() wrote it before the move started.
    if (state_target_ == nullptr) {
        // Can't happen: run_lift_state() always sets the target. Bail rather than
        // command garbage.
        Logger::getInstance()->logError("LIFTERR LIFT_STATE with no target -- returning to loading");
        begin_return(Mode::STOW);
        return;
    }
    const double goal_bar   = state_target_->bar_deg;
    const double goal_winch = state_target_->winch_pos;
    const double goal_ee    = state_target_->ee_deg;
    // Does this swing have to cross the low-clearance zone at all? The zone runs from
    // CHAINBAR_LOW_ZONE_DEG up to loading and the bar travels MONOTONICALLY from where it
    // is to the pose, so both ends being clear of it means the whole arc is -- and the
    // raise would be several hundred degrees of winch travel for nothing. Which is the
    // usual case crossing between two RAISED rows (a back pose at -118 to a front one at
    // -280, say). Read off entry_bar_deg_, latched when the state was commanded, so it
    // cannot flip part way through the move as the bar swings.
    const bool needs_clearance = !bar_clear_of_low_zone(entry_bar_deg_) ||
                                 !bar_clear_of_low_zone(goal_bar);
    // Height held during RAISE and SWING. Needing clearance that is the clearance height,
    // or the destination if that is higher -- never a LOW destination, that is what LOWER
    // is for. Not needing it there is nothing to hold above the pose FOR, so the pose's own
    // height is the target and the winch never goes anywhere it was not already headed.
    const double clear_winch = needs_clearance ? std::max(WINCH_CLEARANCE_POS, goal_winch)
                                               : goal_winch;
    StateStep step_before = state_step_;
    switch (state_step_) {
        case StateStep::RAISE: {
            // Lift up first, bar held wherever the previous pose left it, EE tucked for
            // the climb (or held, out of a raised pose) -- exactly as the back cycle's
            // RAISE does. With no zone to cross this passes straight through on the first
            // tick and SWING starts immediately, the way the back cycle's steps do when
            // they have nothing to do.
            winch_manual_target_ = clear_winch;
            bar_target_deg_ = entry_bar_deg_;
            chain_target_deg_ = entry_ee_deg_;
            if (!needs_clearance || winch >= WINCH_CLEARANCE_POS - WINCH_POS_TOL) {
                state_step_ = StateStep::SWING;
                chainbar_pid_.reset();
            }
            break;
        }
        case StateStep::SWING: {
            // Swing onto the pose with the lift held up. SLEWED, since it has to LAND on
            // the pose whichever way the row has it climbing. Skipping the raise this is
            // the only step that moves the winch, so the two axes overlap here -- safe
            // precisely because the bar stays clear of the frame the whole way.
            //
            // On the lift state's OWN speed pair, not the slew defaults: this is the one
            // swing that crosses the whole travel between the back and front rows, and it
            // used to be tuned by constants the back cycle and the matchload hop share.
            winch_manual_target_ = clear_winch;
            bar_target_deg_ = slew_bar_toward(bar_target_deg_, goal_bar, bar,
                                              bar_target_vel_dps_,
                                              CHAINBAR_STATE_SWING_MAX_DPS,
                                              CHAINBAR_STATE_SWING_DECEL_DEG);
            // The pose's OWN EE angle, not raised_ee_deg() -- it names its orientation
            // outright, so the flip selection does not apply.
            chain_target_deg_ = goal_ee;
            if (std::abs(bar - goal_bar) < CHAINBAR_POS_TOL) {
                state_step_ = StateStep::LOWER;
            }
            break;
        }
        case StateStep::LOWER: {
            // Bar is on the pose; bring the cascade onto the pose's height with it held
            // there. For a pose at or above the clearance height this is already
            // satisfied and passes straight through.
            winch_manual_target_ = goal_winch;
            bar_target_deg_ = goal_bar;
            chain_target_deg_ = goal_ee;
            if (std::abs(winch - goal_winch) < WINCH_POS_TOL) {
                state_step_ = StateStep::HOLD;
                winch_pid_.reset();
            }
            break;
        }
        case StateStep::HOLD: {
            // Arrived. No jog here, unlike the back cycle's HOLD -- autonomous has no
            // buttons. This is where is_settled() goes true and score() is armed.
            winch_manual_target_ = goal_winch;
            bar_target_deg_ = goal_bar;
            chain_target_deg_ = goal_ee;
            break;
        }
    }
    if (state_step_ != step_before) {
        Logger::getInstance()->log(
            "LIFTEVT lift_state step %s -> %s (state=%s bar=%.1f winch=%.1f claw=%s)",
            state_step_name(step_before), state_step_name(state_step_),
            state_target_->key, bar, winch,
            claw_.get_state() == CLAW_CLOSED_STATE ? "CLOSED" : "OPEN");
    }
}

void Lift::advance_bar_pose_sequence(double bar, double winch) {
    // The lift-state choreography with the axes this move does not own stripped out. The
    // cascade has no destination of its own -- every step commands bar_pose_winch_, the
    // height the move was commanded at -- and the EE is handed entry_ee_deg_ from the first
    // tick to the last, so a flip or a cycle orientation cannot pull it round underneath.
    //
    // The clearance raise is latched (skip_clearance_raise_), like every other move, so it
    // cannot flip part way through as the bar leaves the zone. In practice only LOADING
    // needs it: everywhere else the bar already sits past CHAINBAR_LOW_ZONE_DEG and both
    // RAISE and LOWER fall through on their first tick, leaving the swing as the whole move.
    const double clear_winch = skip_clearance_raise_
                                   ? bar_pose_winch_
                                   : std::max(WINCH_CLEARANCE_POS, bar_pose_winch_);
    BarPoseStep step_before = bar_pose_step_;
    switch (bar_pose_step_) {
        case BarPoseStep::RAISE: {
            // Lift up first, bar held where the previous pose left it. Nothing to climb over
            // and this passes straight through, cascade target unchanged from what it was
            // already holding.
            winch_manual_target_ = clear_winch;
            bar_target_deg_ = entry_bar_deg_;
            chain_target_deg_ = entry_ee_deg_;
            if (skip_clearance_raise_ || winch >= WINCH_CLEARANCE_POS - WINCH_POS_TOL) {
                bar_pose_step_ = BarPoseStep::SWING;
                chainbar_pid_.reset();
            }
            break;
        }
        case BarPoseStep::SWING: {
            // The move. SLEWED on the lift state's pair, since this crosses the same kind of
            // travel and has to LAND on the angle rather than arrive at speed.
            winch_manual_target_ = clear_winch;
            bar_target_deg_ = slew_bar_toward(bar_target_deg_, CHAINBAR_BAR_POSE_DEG, bar,
                                              bar_target_vel_dps_,
                                              CHAINBAR_STATE_SWING_MAX_DPS,
                                              CHAINBAR_STATE_SWING_DECEL_DEG);
            chain_target_deg_ = entry_ee_deg_;
            if (std::abs(bar - CHAINBAR_BAR_POSE_DEG) < CHAINBAR_POS_TOL) {
                bar_pose_step_ = BarPoseStep::LOWER;
            }
            break;
        }
        case BarPoseStep::LOWER: {
            // Give the cascade back: the climb out of loading was borrowed for clearance, so
            // it comes down onto the height the move started at. Satisfied on the first tick
            // whenever the raise was skipped, so the winch is never driven anywhere.
            winch_manual_target_ = bar_pose_winch_;
            bar_target_deg_ = CHAINBAR_BAR_POSE_DEG;
            chain_target_deg_ = entry_ee_deg_;
            if (std::abs(winch - bar_pose_winch_) < WINCH_POS_TOL) {
                bar_pose_step_ = BarPoseStep::HOLD;
                winch_pid_.reset();
            }
            break;
        }
        case BarPoseStep::HOLD: {
            // Arrived. A plain three-axis hold on values that are all either the pose's angle
            // or what the mechanism was already doing, so sitting here changes nothing.
            winch_manual_target_ = bar_pose_winch_;
            bar_target_deg_ = CHAINBAR_BAR_POSE_DEG;
            chain_target_deg_ = entry_ee_deg_;
            break;
        }
    }
    if (bar_pose_step_ != step_before) {
        Logger::getInstance()->log(
            "LIFTEVT bar_pose step %s -> %s (bar=%.1f/%.1f winch=%.1f/%.1f ee=%.1f)",
            bar_pose_step_name(step_before), bar_pose_step_name(bar_pose_step_),
            bar, CHAINBAR_BAR_POSE_DEG, winch, bar_pose_winch_, get_ee_angle());
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

bool Lift::leaves_start_backward(Mode m, double goal_bar) {
    // The startup choreography is for ONE obstruction: the intake poly, which sits between
    // the START pose and LOADING. A move that travels the other way -- the bar going MORE
    // negative, onto the matchload angle or the first back state -- never approaches it, so it needs
    // no choreography and no clearance raise, just the move. Both ends of that travel also
    // sit past CHAINBAR_LOW_ZONE_DEG, so there is nothing to climb over either: it is the
    // same arc the startup's own BAR_CLEAR step turns through with the lift down.
    return m == Mode::STARTUP && goal_bar <= CHAINBAR_START_DEG &&
           bar_clear_of_low_zone(goal_bar);
}

bool Lift::crossable_from(Mode m) {
    // The RAISED cycles: a move commanded out of one of these is a CROSSING between the
    // front and back sides rather than a pickup, so it neither grabs nor waits for the
    // claw. STARTUP owns the mechanism until it has a known pose, and RETURNING is refused
    // because its sweep is the one travelling HOME through the low zone with the claw
    // already released.
    // BAR_POSE belongs here for the same reason SCORE_RETREAT does: the bar is parked well
    // past the low zone and whatever was in the claw when it was commanded is still in it,
    // so a prime out of it is a crossing rather than a pickup.
    //
    // TO_MATCHLOAD is crossable even though the bar is still MOVING, which is the one entry
    // here that is not parked. Refusing it meant a driver who pressed matchload and then
    // changed their mind had to sit out the whole hop -- several hundred degrees of swing
    // plus the lower -- before the lift would answer again, and mid-match that wait is the
    // cycle. Nothing downstream needs the bar parked: begin_prime() / begin_back_move() read
    // the LIVE bar angle for both their gates (move_skips_clearance() decides the clearance
    // raise from where the bar actually is, latch_entry_pose() holds and tucks from there),
    // so a cross out of the middle of the swing is choreographed from the angle it is
    // interrupted at. The claw is right too: the hop does not release until it ARRIVES
    // (see advance_matchload_sequence()'s LOWER step), so crossing out keeps whatever is
    // held, and a hop that was already carrying nothing crosses out still carrying nothing.
    return m == Mode::SEQUENCE || m == Mode::BACK || m == Mode::PRIMING ||
           m == Mode::LIFT_STATE || m == Mode::SCORE_RETREAT || m == Mode::BAR_POSE ||
           m == Mode::TO_MATCHLOAD;
}

bool Lift::move_skips_clearance(double goal_bar) const {
    // Purely the geometry, exactly as the lift-state path reads it: the bar travels
    // monotonically from where it is to the goal, so both ends being clear of the zone
    // means the whole arc is, and there is nothing for the cascade to be held up over.
    // Which mode we are in does not come into it -- MATCHLOAD rests at
    // CHAINBAR_MATCHLOAD_DEG, already past the zone, so a prime out of it is as clear as a
    // crossing between the two raised cycles is. STOW is the pose this still refuses: it
    // sits AT loading, inside the zone, and its move is the one that climbs past the intake.
    return bar_clear_of_low_zone(get_bar_angle()) && bar_clear_of_low_zone(goal_bar);
}

void Lift::latch_entry_pose() {
    // Every move holds the bar HERE while the lift climbs, so it can be entered from any
    // pose without dragging the bar somewhere else first.
    entry_bar_deg_ = get_bar_angle();
    bar_target_deg_ = entry_bar_deg_;
    // The EE half of that, and the one thing a crossing needs: the tuck exists to keep the
    // claw inside the frame while the lift climbs out of a LOW pose. Crossing between two
    // raised cycles there is nothing to tuck for, and turning to it would spin the EE the
    // whole way round only for the swing to turn it straight back.
    entry_ee_deg_ = bar_clear_of_low_zone(entry_bar_deg_) ? get_ee_angle()
                                                         : CHAIN_PRIME_RAISE_DEG;
}

double Lift::raised_ee_deg() const {
    // The flip button stores a selection, the cycle supplies the pair. Nothing else should
    // compare an EE angle to decide "is the alternate selected" -- ee_alt_ is that answer.
    if (ee_family_ == EeFamily::BACK) {
        return ee_alt_ ? CHAIN_BACK_ALT_DEG : CHAIN_BACK_DEG;
    }
    return ee_alt_ ? CHAIN_SCORING_ALT_DEG : CHAIN_SCORING_DEG;
}

bool Lift::bar_clear_of_low_zone(double bar) {
    // The bar sweeps NEGATIVE from loading to primed, and only needs the lift up over the
    // near-loading end of that travel. Past the zone the winch may move independently.
    return bar < CHAINBAR_LOW_ZONE_DEG;
}

bool Lift::bar_sweeping() const {
    // The return's RAISE_SWEEP, the startup's BAR_LOADING and the prime's SWEEP are the same
    // move at different lengths and in two directions, so they share one control law: step
    // target, FF-free, capped, coasting through the middle (see update()). The prime used to
    // run a profiled one instead, which overran its own profile by 3x past the top of the
    // swing and answered with a 16 kV command reversal (log 07292026-2 session_003).
    return (mode_ == Mode::RETURNING && return_step_ == ReturnStep::RAISE_SWEEP) ||
           (mode_ == Mode::STARTUP && start_step_ == StartStep::BAR_LOADING) ||
           (mode_ == Mode::PRIMING && prime_step_ == PrimeStep::SWEEP);
}

double Lift::sweep_dir() const {
    // Only the prime sweep runs toward a MORE negative bar angle; everything else comes home.
    return mode_ == Mode::PRIMING ? -1.0 : 1.0;
}

double Lift::sweep_coast_start_angle() const {
    return mode_ == Mode::PRIMING ? CHAINBAR_PRIME_COAST_START_DEG : CHAINBAR_COAST_START_DEG;
}

bool Lift::bar_before_catch(double bar) const {
    return sweep_dir() * (bar - sweep_catch_angle()) < 0.0;
}

bool Lift::bar_coast_active(double bar, double winch) const {
    // Past the coast start, not yet at the catch angle, motor in COAST. Both angle tests go
    // through sweep_dir() so the window reads the same way round either direction. Two
    // guards beyond that: the coast must not be able to carry the bar into the low zone with
    // the lift down, and bar_coast_done_, so a bounce cannot start a second coast.
    //
    // The first of those is about the CARRY-PAST, not the lift height as such -- so ask the
    // question directly: a sweep whose catch angle already sits clear of the zone cannot
    // coast into it, whatever the cascade happens to be doing. That is the prime sweep
    // (catch ~-285) and the matchload return (catch ~-51), and the latter has no clearance
    // raise to satisfy a height test with -- it runs at the matchload height throughout, so
    // gating on the lift being up would leave it the one sweep driven all the way home on
    // the brake. The loading return and the startup sweep catch AT the zone edge, so they
    // still want the lift up, and for them this reads exactly as it did before.
    const bool coast_cannot_reach_zone = bar_clear_of_low_zone(sweep_catch_angle());
    return bar_sweeping() &&
           !bar_coast_done_ &&
           sweep_dir() * (bar - sweep_coast_start_angle()) >= 0.0 &&
           bar_before_catch(bar) &&
           (coast_cannot_reach_zone || winch >= WINCH_CLEARANCE_POS - WINCH_POS_TOL);
}

int Lift::size_keep_zone_at(double bar) const {
    // Hysteretic in the same way the retreat's park is, and per BAND: only the one already
    // engaged is widened, so entering takes the bar genuinely reaching an edge while leaving
    // takes a real move past it rather than the sensor wobbling across a boundary the bar is
    // parked on. The table is checked for overlap-once-widened at build time, so at most one
    // row can match and the first hit is THE hit.
    for (int i = 0; i < SIZE_KEEP_ZONE_COUNT; ++i) {
        double margin = (i == size_keep_zone_) ? CHAINBAR_SIZE_KEEP_HYST_DEG : 0.0;
        if (bar >= CHAINBAR_SIZE_KEEP_ZONES[i][0] - margin &&
            bar <= CHAINBAR_SIZE_KEEP_ZONES[i][1] + margin) {
            return i;
        }
    }
    return -1;
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
    if (mode_ == Mode::PRIMING) {
        // Short of primed in the prime's own direction, and by more -- see the constant.
        return SEQ_BAR_DEG[0] + CHAINBAR_PRIME_CATCH_LEAD_DEG;
    }
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
        case Mode::LIFT_STATE: return "LIFTSTATE";
        case Mode::SCORE_RETREAT: return "RETREAT";
        case Mode::BAR_POSE: return "BARPOSE";
        case Mode::RETURNING: return "RETURN";
    }
    return "?";
}

const char* Lift::prime_step_name(PrimeStep s) {
    switch (s) {
        case PrimeStep::RAISE:  return "RAISE";
        case PrimeStep::TUCK:   return "TUCK";
        case PrimeStep::SWEEP:  return "SWEEP";
        case PrimeStep::LOWER:  return "LOWER";
        case PrimeStep::SETTLE: return "SETTLE";
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
        case BackStep::TUCK:  return "TUCK";
        case BackStep::SWING: return "SWING";
        case BackStep::LOWER: return "LOWER";
        case BackStep::HOLD:  return "HOLD";
    }
    return "?";
}

const char* Lift::state_step_name(StateStep s) {
    switch (s) {
        case StateStep::RAISE: return "RAISE";
        case StateStep::SWING: return "SWING";
        case StateStep::LOWER: return "LOWER";
        case StateStep::HOLD:  return "HOLD";
    }
    return "?";
}

const char* Lift::bar_pose_step_name(BarPoseStep s) {
    switch (s) {
        case BarPoseStep::RAISE: return "RAISE";
        case BarPoseStep::SWING: return "SWING";
        case BarPoseStep::LOWER: return "LOWER";
        case BarPoseStep::HOLD:  return "HOLD";
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
        case WinchMode::UNTENSION: return "UNTENSION";
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
    // Fixed control step
    double dt = Config::DT / 1000.0;
    last_update_ms_ = now;

    double bar = get_bar_angle();
    double winch = get_winch_position();

    // Claw-grab wait: prime() closed the claw and left the robot resting. Start the
    // choreography once the solenoid has had CLAW_GRAB_SETTLE_MS, or drop the request if
    // something else has taken the lift off its resting pose in the meantime.
    if (prime_grab_pending_) {
        bool still_resting = mode_ == Mode::STOW || mode_ == Mode::MATCHLOAD;
        if (!still_resting) {
            Logger::getInstance()->log(
                "LIFTEVT prime grab wait CANCELLED (mode=%s took over)", mode_name(mode_));
            prime_grab_pending_ = false;
        } else if (now - prime_grab_ms_ >= CLAW_GRAB_SETTLE_MS) {
            Logger::getInstance()->log(
                "LIFTEVT prime grab wait done after %u ms -> starting (bar=%.1f winch=%.1f)",
                now - prime_grab_ms_, bar, winch);
            prime_grab_pending_ = false;
            begin_prime();
        }
    }

    // The chain runner goes here, BEFORE the mode switch below: a step commanded on this
    // tick then has its targets resolved by the same tick's advance_*_sequence() call,
    // rather than idling one tick on every step boundary.
    advance_chain(now);

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
    // ...and which WAY that swing is going, since the descent is gravity-fed and takes its
    // own feed-forward amplitudes.
    bool bar_swing_down = false;
    // Set by the parked score retreat: hand the bar to its brake instead of driving it.
    bool bar_park_hold = false;
    // Drive (mV) the back cycle's L1/L2 jog wants on the winch this tick, 0 when no button
    // is held or no cycle is live. Non-zero replaces the position hold outright.
    double winch_jog_mv = 0.0;
    // Set by the loading pose: drive the bar with a fixed voltage, not the position PID.
    bool bar_constant_hold = false;
    switch (mode_) {
        case Mode::STARTUP:
            // Scripts every target step by step and lands in STOW.
            advance_start_sequence(bar, winch);
            break;
        case Mode::STOW:
            winch_manual_target_ = WINCH_LOADING_POS;
            // Still resolved, so the telemetry and the next move's entry angle read the
            // pose, but the bar is DRIVEN by the constant below rather than to this.
            bar_target_deg_ = CHAINBAR_LOADING_DEG;
            chain_target_deg_ = CHAIN_LOADING_DEG;
            bar_constant_hold = CHAINBAR_LOADING_HOLD_MV != 0.0;
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
            // The move onto the selected back state, then the hold that keeps the bar there
            // with the cascade under the driver's thumb.
            advance_back_sequence(bar, winch, winch_jog_mv);
            break;
        case Mode::LIFT_STATE:
            // The same move onto a pose from the table, then a plain hold -- no jog.
            advance_lift_state_sequence(bar, winch);
            break;
        case Mode::BAR_POSE:
            // The bar alone onto its angle, with the cascade and EE commanded the values the
            // move was fired at -- see bar_pose().
            advance_bar_pose_sequence(bar, winch);
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
        case Mode::SEQUENCE:
            // The front table, and the only table anything walks now -- the back cycle
            // holds a state and jogs the winch instead (see advance_back_sequence()).
            advance_raise_sequence(bar, SEQ_BAR_DEG, SEQ_WINCH_DEG, SEQ_KEYFRAMES,
                                   bar_swing_cap_mv, bar_swing_down);
            break;
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
            bar_swing_cap_mv = SCORE_RETREAT_SWING_MAX_MV;
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

    // Clamp
    winch_manual_target_ = std::clamp(winch_manual_target_, WINCH_TARGET_FLOOR, WINCH_MAX_POS);

    if (winch_respooled_ && winch_manual_target_ <= WINCH_DOWN_POS + WINCH_POS_TOL) {
        winch_manual_target_ = std::min(winch_manual_target_, winch_slack_pos_);
    }
    double winch_target = winch_manual_target_;

    // --- Winch: PID hold, plus a home-to-bottom pass once settled at the bottom. ---
    // HOLD -> SEEK (short down pre-roll to ground the lift) -> RESPOOL (reel up until the
    // string goes taut and re-zero) -> UNTENSION (pay a little back out) -> HOLD. Gated to
    // the resting poses, once per descent.
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
        // No slack measured yet this pass: the bottom hold parks on the reference unless
        // an untension actually completes and moves this down.
        winch_slack_pos_ = WINCH_DOWN_POS;
        Logger::getInstance()->log("LIFTEVT seek START @ winch=%.1f", winch);
    }

    // The back cycle's jog: a held button drives the winch outright, and the position hold
    // stands down for as long as it does. Both edges reset the PID -- entering, because the
    // hold's integral has no say over a button; leaving, because BackStep::HOLD has been
    // dragging the target along under the jog and the error it hands back is ~0.
    if ((winch_jog_mv != 0.0) != winch_jogging_) {
        winch_jogging_ = winch_jog_mv != 0.0;
        winch_pid_.reset();
        Logger::getInstance()->log(
            "LIFTEVT winch jog %s (state %d, winch=%.1f mv=%.0f) -- %s",
            winch_jogging_ ? "START" : "STOP", back_state_, winch, winch_jog_mv,
            winch_jogging_ ? "hold stood down" : "holding here");
    }

    double winch_mv;
    if (winch_jogging_) {
        // Full drive, no profile and no PID: the button IS the command. The soft limit is
        // what stops it at the ends of the travel, and BackStep::HOLD keeps the hold target
        // on the height being passed through, so releasing parks the cascade right here.
        winch_mv = apply_soft_limit(winch_jog_mv, winch, WINCH_TARGET_FLOOR, WINCH_MAX_POS);
    } else if (winch_mode_ == WinchMode::SEEK) {
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

            winch_mode_ = WinchMode::UNTENSION;
            untension_start_ms_ = now;
            winch_mv = 0.0;
        } else if (timed_out) {
            // Never found resistance -- give up so we can't drive up forever.
            Logger::getInstance()->log(
                "LIFTEVT respool TIMEOUT after %u ms @ winch=%.1f vel=%.1f cur=%.0f (no resistance found)",
                now - respool_start_ms_, winch, winch_vel_rpm, winch_current);
            winch_mode_ = WinchMode::HOLD;
            winch_respooled_ = true;  // don't immediately retry; wait for a raise
            winch_mv = 0.0;
            // No untension: nothing ever went taut, so there is no tension to release and
            // no slack to park on. winch_slack_pos_ is still the reference from the seek.
        } else {
            winch_mv = WINCH_RESPOOL_MV;  // gentle raise to take up the slack
        }
    } else if (winch_mode_ == WinchMode::UNTENSION) {
        // The respool ends STALLED against a taut string, which leaves the spool holding a
        // load for as long as the robot sits there. Pay a little of it back out so the lift
        // parks on a slack string, and remember where that leaves it -- the bottom hold
        // parks on that height (see winch_slack_pos_) instead of hauling it straight back.
        // Per-tick trace, like the two phases before it: current should stay LOW throughout
        // and the position walk down by however far the string pays out.
        Logger::getInstance()->log("LIFTUNTENSION t=%u winch=%.1f vel=%.1f cur=%.0f mv=%.0f",
                                   now - untension_start_ms_, winch, winch_vel_rpm,
                                   winch_current, WINCH_UNTENSION_MV);
        bool done = (now - untension_start_ms_) >= WINCH_UNTENSION_MS;
        // Backstop: a spool that keeps paying out is unspooling, not releasing tension.
        bool over_travel = winch <= WINCH_DOWN_POS - WINCH_UNTENSION_MAX_DEG;
        if (done || over_travel) {
            winch_slack_pos_ = std::clamp(winch, WINCH_DOWN_POS - WINCH_UNTENSION_MAX_DEG,
                                          WINCH_DOWN_POS);
            Logger::getInstance()->log(
                "LIFTEVT untension done (%s) after %u ms @ winch=%.1f vel=%.1f cur=%.0f -- "
                "bottom hold parks at %.1f (reference stays %.0f)",
                over_travel ? "over-travel" : "timed", now - untension_start_ms_, winch,
                winch_vel_rpm, winch_current, winch_slack_pos_, WINCH_DOWN_POS);
            winch_mode_ = WinchMode::HOLD;
            winch_respooled_ = true;  // the pass is complete only now
            winch_pid_.reset();
            winch_mv = 0.0;
        } else {
            winch_mv = WINCH_UNTENSION_MV;  // gentle pay-out to release the tension
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
        // The brake mode itself is written at the drive site below, which owns it for the
        // whole tick -- writing it here too let the unconditional HOLD down there stomp the
        // coast window on the very same tick it opened.
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
    bool bar_sweep = bar_sweeping() && bar_before_catch(bar);
    if (bar_park_hold) {
        // Parked score retreat: NO command, so the brake holds the bar. Not "PID with a
        // small error" -- at the top of the swing that is exactly what oscillated. Held in
        // reset so it re-engages clean if the bar escapes the park band.
        bar_mv = 0.0;
        chainbar_pid_.reset();
        bar_stalled_ticks_ = 0;  // not position-controlled here: no stall to arm off
    } else if (bar_constant_hold) {
        // Loading pose
        chainbar_pid_.reset();
        bar_stalled_ticks_ = 0;  // stalled against the loading stop BY DESIGN, not stuck
    } else if (bar_coasting || bar_sweep) {
        // The long stretch of a sweep, coast included: plain position PID, NO feed-forward.
        // Gravity pulls the bar TOWARD the goal here, so the gravity FF would point
        // backwards the whole way and cost 7031 mV to break out of (log 07262026-2).
        bar_mv = chainbar_pid_.calculate(bar_cmd, bar, bar_vel_dps, 0.0);
        bar_stalled_ticks_ = 0;  // the sweep has its own cap; the breakaway stays out of it
    } else {
        // Gravity feed-forward: the standing holding effort, so the PID does not need a
        // lingering error to balance gravity. Largest horizontal, zero at vertical rest.
        //
        // TWO amplitudes, chosen by which way the scoring swing is going. Everywhere else
        // the bar is being HELD against gravity, or climbing, and the full peak is what
        // keeps the PID off a standing error. On the swing's DESCENT gravity is what moves
        // the bar, so the same term points backwards for the whole travel: the full peak
        // there is the profile spending most of its command fighting the weight that is
        // already carrying the bar down, which is what left the descent unable to hold a
        // rate. Sharing one constant meant the hold and the descent could not be tuned apart.
        static constexpr double DEG_TO_RAD = 3.14159265358979323846 / 180.0;
        const double gravity_peak_mv = bar_swing_down ? CHAINBAR_GRAVITY_FF_DOWN_MV
                                                      : CHAINBAR_GRAVITY_FF_MV;
        bar_gravity_ff = gravity_peak_mv *
            std::sin((bar - CHAINBAR_GRAVITY_ZERO_DEG) * DEG_TO_RAD);

        // Motion feed-forward: the term that actually moves the bar, so its speed is
        // COMMANDED rather than emergent from P vs. friction. Two regimes:
        double bar_err = bar_cmd - bar;
        if (std::abs(bar_target_vel_dps_) > CHAINBAR_PROFILE_EPS_DPS) {
            // FF
            double kinetic = CHAINBAR_KINETIC_FF_MV *
                std::clamp(bar_target_vel_dps_ / CHAINBAR_KS_MOVING_DPS, -1.0, 1.0);
            // Same directional split as the gravity term: kv buys the speed the profile
            // asked for, and on a gravity-fed DESCENT most of that speed is already paid
            // for. Spending the climb's rate there just pushes the bar down its own hill,
            // on top of a weight that is already doing it -- which is the descent running
            // away from its profile and arriving hot.
            const double kv = bar_swing_down ? CHAINBAR_KV_MV_PER_DPS_DOWN
                                             : CHAINBAR_KV_MV_PER_DPS;
            bar_static_ff = kinetic + kv * bar_target_vel_dps_;
            // A bar under a profile is being TOLD to move, so it is never "stuck". Clearing
            // here is also what makes the profile's end quiet: the breakaway has to earn its
            // 100 ms of stall again from scratch instead of firing on the arrival tick.
            bar_stalled_ticks_ = 0;
        } else {
            // Step target / hold: position-error BREAKAWAY, only once the bar has been
            // demonstrably STUCK for CHAINBAR_KS_STALL_TICKS. Scaling on the instantaneous
            // rate is not enough: the windowed rate reads ~0 at every turnaround, so a bar
            // reversing THROUGH the pose collected full amplitude in its new direction and
            // was flung back out again -- a relay oscillator worth two thirds of the command
            // at the reversal (log 08012026 t=17.74: err=-7.8 vel=0.0 sff=-4388 mv=-6782).
            // Latching on a sustained stall keeps the term for what it is for -- starting a
            // bar that has genuinely parked short of the pose.
            // Moving, or near enough that there is nothing left to break out toward:
            // disarm. Dropping the term the moment the bar frees itself is the point --
            // stiction is broken by then and the PID has the rest. If it re-sticks, this
            // re-arms 100 ms later and nudges again, which walks the bar in rather than
            // ringing it in.
            if (std::abs(bar_vel_dps) > CHAINBAR_KS_STALL_DPS ||
                std::abs(bar_err) < CHAINBAR_KS_MIN_ERR_DEG) {
                bar_stalled_ticks_ = 0;
            } else if (bar_stalled_ticks_ < CHAINBAR_KS_STALL_TICKS) {
                ++bar_stalled_ticks_;
            }
            if (bar_stalled_ticks_ >= CHAINBAR_KS_STALL_TICKS) {
                // Still faded across the deadband, so it does not step on/off at the band
                // edge or shove the bar through loading. Same directional split as the
                // gravity term above: a bar stalled part way DOWN the scoring swing has
                // gravity already pulling it off its stiction, so it wants a nudge, not the
                // full breakaway that starts a bar from rest against the weight.
                bar_static_ff = (bar_swing_down ? CHAINBAR_KS_DOWN_MV : CHAINBAR_KS_MV) *
                    std::clamp(bar_err / CHAINBAR_KS_DEADBAND_DEG, -1.0, 1.0);
            }
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
        bool needs_break_out = bar_swing_cap_mv < CHAINBAR_BREAKOUT_MV && !bar_swing_broke_out_;
        bar_drive_cap = needs_break_out ? 12000.0 : bar_swing_cap_mv;
    } else if (bar_coasting || bar_sweep) {
        bar_drive_cap = (mode_ == Mode::STARTUP && bar_moving) ? CHAINBAR_START_SWEEP_MAX_MV
                                                              : CHAINBAR_SWEEP_MAX_MV;
        // The prime sweep uses NEITHER of those: its two halves pull in opposite directions
        // (climb fights gravity, descent is fed by it) and sharing a cap with the return
        // meant tuning one retuned the other. This branch already implies
        // prime_step_ == SWEEP, since bar_sweeping() is what put us in it.
        if (mode_ == Mode::PRIMING) {
            bar_drive_cap = bar > CHAINBAR_PRIME_CLIMB_END_DEG
                                ? CHAINBAR_PRIME_CLIMB_MAX_MV
                                : CHAINBAR_PRIME_DESCENT_MAX_MV;
        }
    }
    bool bar_braking = bar_moving && bar_mv * bar_vel_dps < 0.0;
    if (!bar_braking) {
        bar_mv = std::clamp(bar_mv, -bar_drive_cap, bar_drive_cap);
    }

    // Soft rotation limit: never drive the bar past its cable-safe travel. EXEMPT for the
    // loading hold: CHAINBAR_MAX_DEG *is* the loading angle, so the limit would zero the
    // hold the moment it arrived and the constant drive would never hold anything. Keep
    // CHAINBAR_LOADING_HOLD_MV low enough that stalling into that stop is safe.
    if (!bar_constant_hold) {
        bar_mv = apply_soft_limit(bar_mv, bar, CHAINBAR_MIN_DEG, CHAINBAR_MAX_DEG);
    }
    // --- Brake mode + drive. -------------------------------------------------
    // COAST for the two things that want the bar free -- the kill switch and the sweep's
    // coast window -- and HOLD everywhere else. Written every tick rather than on the edge,
    // so a direct write elsewhere (the score retreat's park) cannot leave this stale.
    chainbar_motor_.set_brake_mode((bar_kill || bar_coasting) ? pros::E_MOTOR_BRAKE_COAST
                                                              : pros::E_MOTOR_BRAKE_HOLD);
    if (bar_kill) {
        // The brake mode alone releases NOTHING: the motor keeps applying the last voltage
        // it was given until it is given another, so a kill that only set the mode left the
        // bar driving at whatever the tick before it commanded. The zero command below is
        // what COAST actually applies to, and it is the whole point of the switch.
        bar_mv = 0.0;
        chainbar_pid_.reset();   // so it re-engages clean when a macro clears the kill
        bar_stalled_ticks_ = 0;  // not position-controlled here: no stall to arm off
    }
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

    // --- Size-keep override. -------------------------------------------------
    // The claw goal every mode's chain_target_deg_ is filtered through: with the bar in one
    // of the size-keep bands the EE must be at ~CHAIN_SIZE_KEEP_EE_DEG or the claw hangs
    // outside the size envelope, and that is geometry, not a preference -- so it wins over
    // the pose, the cycle's orientation and the flip button alike, for as long as the bar is
    // in there. Applied HERE rather than in each sequence because this is the one place every
    // mode's EE goal passes through: a mode added later inherits the limit rather than having
    // to remember it.
    //
    // Resolved to the turn nearest the EE's current angle: -90 and -450 are the same
    // physical orientation, so this tucks the claw the short way round instead of unwinding
    // a whole turn to arrive at the same place. Nothing is written back into
    // chain_target_deg_ -- the pose keeps asking for its own angle, and gets it again the
    // moment the bar leaves the band.
    double chain_goal = chain_target_deg_;
    const int size_keep = size_keep_zone_at(bar);
    if (size_keep >= 0) {
        chain_goal = unwrap_angle_near(CHAIN_SIZE_KEEP_EE_DEG, ee);
    }
    if (size_keep != size_keep_zone_) {
        // Which BAND, not just on/off: the bar crosses more than one of these on a long
        // sweep, and a line that only said ON/OFF would not say which edge it left by.
        const int zone = (size_keep >= 0) ? size_keep : size_keep_zone_;
        Logger::getInstance()->log(
            "LIFTEVT size-keep %s band %d [%.0f, %.0f] +/-%.0f (bar=%.1f mode=%s) -- EE "
            "commanded %.1f, the pose asked for %.1f (ee=%.1f)",
            size_keep >= 0 ? "ON" : "OFF", zone, CHAINBAR_SIZE_KEEP_ZONES[zone][0],
            CHAINBAR_SIZE_KEEP_ZONES[zone][1], CHAINBAR_SIZE_KEEP_HYST_DEG, bar,
            mode_name(mode_), chain_goal, chain_target_deg_, ee);
        size_keep_zone_ = size_keep;
        // Both edges, and the RELEASE is the one that matters: the goal swaps between the
        // size-keep angle and the pose's own in a single tick, and an integral wound against
        // the one it just left is a lurch on the tick it hands back. The pose's target has
        // been sitting in chain_target_deg_ untouched the whole time -- nothing has to be
        // restored, only the controller re-engaged clean against it.
        chain_pid_.reset();
    }

    double chain_cmd = std::clamp(chain_goal, CHAIN_MIN_DEG, CHAIN_MAX_DEG);
    // A pose the soft limits cannot express is a CONFIG error, and a quiet one: the clamp
    // becomes the setpoint and the EE holds at the limit, looking like a reversed motor
    // (log 07262026-4). LIFTDBG logs the raw target, so warn here instead, once per onset.
    static bool chain_clamp_warned = false;
    if (std::abs(chain_cmd - chain_goal) > 1e-3) {
        if (!chain_clamp_warned) {
            Logger::getInstance()->log(
                "LIFTWARN chain target %.1f is OUTSIDE the soft limits [%.0f, %.0f] -- "
                "commanding %.1f instead; the EE will hold at the limit and fight the "
                "sequence. Fix CHAIN_MIN_DEG/CHAIN_MAX_DEG or the pose constants.",
                chain_goal, CHAIN_MIN_DEG, CHAIN_MAX_DEG, chain_cmd);
            chain_clamp_warned = true;
        }
    } else {
        chain_clamp_warned = false;  // re-arm for the next out-of-range pose
    }
    // Alternate-orientation return: kill the chain until the bar is back round. Not "hold
    // where you are" -- no command at all, and the PID reset so it re-engages clean.
    //
    // This WINS over the size-keep override where the two overlap, which is the bottom half
    // of the band straddling CHAIN_FLIP_BAR_DEG: an alt return crossing it gets no chain
    // command, so the claw rides through on brake COAST rather than being turned into a
    // structure the flipped EE has not cleared yet. The override takes over the moment the
    // bar is past the flip angle, which is still inside the band.
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
        // Speed cap (see CHAIN_MAX_MV)
        chain_mv = std::clamp(chain_mv, -CHAIN_MAX_MV, CHAIN_MAX_MV);
        // Rot limit so tubing doesn't tangle with chain
        chain_mv = apply_soft_limit(chain_mv, ee, CHAIN_MIN_DEG, CHAIN_MAX_DEG);
    }
    chain_motor_.move_voltage(static_cast<int>(chain_mv));

    // Debug line
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
        const char* winch_status = (winch_mode_ == WinchMode::SEEK)      ? "SEEK"
                                 : (winch_mode_ == WinchMode::RESPOOL)   ? "RESPOOL"
                                 : (winch_mode_ == WinchMode::UNTENSION) ? "UNTENSION" : "";
        pros::lcd::print(2, "Winch t:%.0f a:%.0f %s", winch_target, winch, winch_status);
        // The back cycle walks no table, so its progress readout is meaningless -- which
        // state is selected is the thing a driver actually wants off this line.
        if (mode_ == Mode::BACK) {
            pros::lcd::print(3, "Bar t:%.1f a:%.1f BACK s:%d/%d", bar_target_deg_, bar,
                             back_state_ + 1, BACK_STATE_COUNT);
        } else {
            pros::lcd::print(3, "Bar t:%.1f a:%.1f %s p:%.2f", bar_target_deg_, bar,
                             mode_name(mode_), raise_progress_);
        }
        // The COMMANDED goal, not the pose's raw target: while the size-keep override holds
        // those two differ, and the line is read to see what the EE is actually chasing.
        pros::lcd::print(4, "EE t:%.0f a:%.0f%s", chain_cmd, ee,
                         size_keep_zone_ >= 0 ? " SIZE" : "");
    }
}
