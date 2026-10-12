#include "hardware/lift.hpp"
#include "config.hpp"
#include "utilities/logger.hpp"
#include <algorithm>
#include <cmath>
#include "pros/misc.hpp"
#include "pros/rtos.hpp"

const char *Lift::mode_name(Mode m)
{
    switch (m)
    {
    case Mode::PRIMED:    return "PRIMED";
    case Mode::SCORING:   return "SCORING";
    case Mode::MATCHLOAD: return "MATCHLOAD";
    }
    return "?";
}

const char *Lift::clamp_name(ClampPos p)
{
    switch (p)
    {
    case ClampPos::CLOSED: return "CLOSED";
    case ClampPos::OPEN:   return "OPEN";
    }
    return "?";
}

const char *Lift::state_name(State s)
{
    switch (s)
    {
    case State::IDLE:            return "IDLE";
    case State::HEIGHT_MOVE:     return "HEIGHT_MOVE";
    case State::RAISING:         return "RAISING";
    case State::SLEWING:         return "SLEWING";
    case State::LOWERING:        return "LOWERING";
    case State::LOWERING_HELD:   return "LOWERING_HELD";
    case State::SCORE_DESCEND:   return "SCORE_DESCEND";
    case State::SCORE_UNLOAD:    return "SCORE_UNLOAD";
    case State::SCORE_RELEASE:   return "SCORE_RELEASE";
    case State::MATCHLOAD_MOVE:  return "ML_MOVE";
    case State::MATCHLOAD_CLAMP: return "ML_CLAMP";
    case State::MATCHLOAD_RAISE: return "ML_RAISE";
    }
    return "?";
}

Lift::Lift(pros::Rotation &winch_rotation, pros::Motor &claw_motor,
           pros::Distance &tower_distance_sensor, pros::Distance &loader_distance_sensor,
           Pto &pto)
    : winch_rotation_(winch_rotation), claw_motor_(claw_motor),
      tower_distance_sensor_(tower_distance_sensor),
      loader_distance_sensor_(loader_distance_sensor), pto_(pto)
{
}

void Lift::initialize()
{
    winch_rotation_.set_data_rate(5);
    winch_rotation_.reset_position();
    claw_motor_.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    claw_motor_.tare_position();
    // HOLD so the claw stays put where an open move brakes it
    claw_motor_.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    // The claw starts closed and holds through the first update(); see "STARTUP"
    clamp_target_ = ClampPos::CLOSED;
    clamp_switch_ms_ = pros::millis();
    boot_hold_ = true;
    mode_ = Mode::MATCHLOAD;
    pto_.motors().set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    Logger::getInstance()->log("Lift initialized (home = %.1f deg)", get_position());
}

void Lift::move_up()
{
    up_requested_ = true;
    grab_held_ = false;
}

void Lift::hold_down()
{
    down_requested_ = true;
    grab_held_ = false;
}

void Lift::return_home()
{
    grab_held_ = false;
    if (state_ == State::LOWERING)
        return;
    if (in_macro())
        Logger::getInstance()->log("Lift macro cancelled by return_home");
    state_ = State::LOWERING;
    up_requested_ = false;
    Logger::getInstance()->log("Lift returning to MATCHLOAD (%.1f deg) from %.1f deg",
                               MATCHLOAD_HEIGHT_DEG, get_position());
}

void Lift::raise_to(double winch_deg)
{
    const double target = std::clamp(winch_deg, 0.0, MAX_HEIGHT_DEG);
    if (target <= HOME_TOLERANCE_DEG)
    {
        Logger::getInstance()->log("Lift raise_to %.1f deg is home, returning home instead", target);
        return_home();
        return;
    }
    if (in_macro())
        Logger::getInstance()->log("Lift macro cancelled by raise_to");
    up_requested_ = false;
    down_requested_ = false;
    // Same as a driver prime: hold the piece
    if (mode_ == Mode::MATCHLOAD)
        mode_ = Mode::PRIMED;
    height_target_deg_ = target;
    state_ = State::HEIGHT_MOVE;
    move_start_ms_ = pros::millis();
    Logger::getInstance()->log("Lift raise to %.1f deg from %.1f deg", target, get_position());
}

void Lift::score()
{
    if (mode_ != Mode::PRIMED)
    {
        Logger::getInstance()->log("Lift score ignored: mode is %s", mode_name(mode_));
        return;
    }
    if (is_scoring())
        return;
    if (is_matchloading())
        Logger::getInstance()->log("Lift matchload macro cancelled by score");
    // Still PRIMED (claw closed, piece held) on the way down; SCORING only starts once the
    // tower is seen in SCORE_DESCEND
    state_ = State::SCORE_DESCEND;
    score_at_deg_ = -1.0;
    up_requested_ = false;
    Logger::getInstance()->log("Lift score macro: descending from %.1f deg (tower=%d mm)",
                               get_position(), get_tower_distance_mm());
}

void Lift::score_at(double height_deg)
{
    if (mode_ != Mode::PRIMED)
    {
        Logger::getInstance()->log("Lift score ignored: mode is %s", mode_name(mode_));
        return;
    }
    if (is_scoring())
        return;
    if (is_matchloading())
        Logger::getInstance()->log("Lift matchload macro cancelled by score");
    state_ = State::SCORE_DESCEND;
    score_at_deg_ = std::max(height_deg, 0.0);
    up_requested_ = false;
    Logger::getInstance()->log("Lift score macro: descending from %.1f to %.1f deg, tower sensor ignored (tower=%d mm)",
                               get_position(), score_at_deg_, get_tower_distance_mm());
}

void Lift::matchload()
{
    if (is_matchloading())
        return;
    if (is_scoring())
        Logger::getInstance()->log("Lift score macro cancelled by matchload");
    up_requested_ = false;
    grab_held_ = false;
    mode_ = Mode::MATCHLOAD;
    state_ = State::MATCHLOAD_MOVE;
    Logger::getInstance()->log("Lift matchload macro: moving %.1f -> %.1f deg",
                               get_position(), MATCHLOAD_HEIGHT_DEG);
}

void Lift::grab()
{
    if (mode_ != Mode::MATCHLOAD)
    {
        // PRIMED and SCORING already decide the claw for themselves
        Logger::getInstance()->log("Lift grab ignored: mode is %s", mode_name(mode_));
        return;
    }
    if (grab_held_)
        return;
    grab_held_ = true;
    Logger::getInstance()->log("Lift grab: holding claw closed in %s at %.1f deg",
                               mode_name(mode_), get_position());
}

void Lift::stop()
{
    grab_held_ = false;
    if (in_macro())
        Logger::getInstance()->log("Lift macro cancelled by stop");
    state_ = State::IDLE;
    up_requested_ = false;
    down_requested_ = false;
    slew_mv_ = 0;
    hold(); // stop() can be called outside update(), so cut the motors right now
}

void Lift::hold()
{
    last_winch_mv_ = 0;
    last_winch_cmd_mv_ = 0;
    // Brake (not coast) so the HOLD brake mode actually engages against the load --
    // otherwise the motors keep whatever they were last told (this is what let a
    // released L2 keep winding the lift down). Only ours while the PTO is in LIFT mode;
    // in INTAKE mode the same motors belong to the intake, so leave them alone.
    if (pto_.mode() == Pto::Mode::LIFT)
        pto_.stop();
}

bool Lift::is_home() const
{
    return get_position() <= HOME_TOLERANCE_DEG;
}

bool Lift::is_at_max_height() const
{
    return get_position() >= MAX_HEIGHT_DEG;
}

double Lift::get_position() const
{
    // Rotation sensor reports centidegrees
    return winch_rotation_.get_position() / 100.0;
}

double Lift::get_claw_position() const
{
    return claw_motor_.get_position();
}

bool Lift::clamp_done() const
{
    const uint32_t move_ms = clamp_target_ == ClampPos::OPEN ? CLAW_OPEN_MS : CLAW_CLOSE_MS;
    return pros::millis() - clamp_switch_ms_ >= move_ms;
}

int Lift::get_tower_distance_mm() const
{
    return tower_distance_sensor_.get_distance();
}

bool Lift::tower_in_range() const
{
    // 9999 = nothing in view, negative = sensor error; neither counts as "on the tower"
    const int d = get_tower_distance_mm();
    return d > 0 && d < TOWER_DISTANCE_MM;
}

int Lift::get_loader_distance_mm() const
{
    return loader_distance_sensor_.get_distance();
}

bool Lift::loader_in_range() const
{
    // Same reading rules as the tower sensor: 9999 = nothing in view, negative = error
    const int d = get_loader_distance_mm();
    return d > 0 && d < LOADER_DISTANCE_MM;
}

void Lift::drive(int millivolts)
{
    // Soft upper limit: never push upward once at the cap (downward is always allowed)
    const bool capped = millivolts > 0 && is_at_max_height();
    if (capped && !at_cap_logged_)
        Logger::getInstance()->log("Lift at max height (%.1f deg), upward drive cut", get_position());
    at_cap_logged_ = capped;
    if (capped)
        millivolts = 0;
    // Downward speed cap: gravity is already helping, so the winch never needs full
    // voltage to lower and a string gets unspooled quicker than gravity making it get unspooled/tangeled
    millivolts = std::max(millivolts, -MAX_LOWER_MV);
    // What the state machine actually asked for, after both limits but before the PTO
    // check below can throw it away. LIFTDBG carries this rather than what reached the
    // motors, so a command lost to the PTO shows as mv!=0 with own=0 instead of looking
    // like a tick where nothing was asked for. Both caps are above it so the line never
    // reports a voltage the winch was never going to be allowed.
    last_winch_cmd_mv_ = millivolts;
    // Every winch motor lives on the PTO now, so nothing moves unless we own it
    if (!pto_.is_ready(Pto::Mode::LIFT))
        millivolts = 0;
    const int winch_mv = WINCH_DIRECTION * millivolts;
    last_winch_mv_ = winch_mv;
    pto_.run(Pto::Mode::LIFT, winch_mv);
}

bool Lift::run_to(double target_deg)
{
    pto_.request(Pto::Mode::LIFT);
    height_target_deg_ = target_deg;
    if (target_deg <= HOME_TOLERANCE_DEG)
    {
        // Home: full speed down (capped in drive()) until the sensor says we are there
        if (is_home())
        {
            drive(0);
            return true;
        }
        drive(-MAX_VOLTAGE);
        return false;
    }
    const double error = target_deg - get_position();
    if (std::fabs(error) <= HEIGHT_TOLERANCE_DEG)
    {
        drive(0);
        return true;
    }
    const int mv = std::fabs(error) <= HEIGHT_APPROACH_DEG ? HEIGHT_APPROACH_MV : MAX_VOLTAGE;
    drive(error > 0 ? mv : -mv);
    return false;
}

// One flat, greppable key=value line per tick: mode, state, who owns the PTO, target
// vs. actual vs. command for the winch, and the claw target, encoder and voltage.
// Prefixed "LIFTDBG" so tests/log_analysis/lift_parser.py can pick it out of the drive
// loop's chatter.
//
// Cheap enough for the 10 ms tick: Logger::log() only pushes onto a queue, and a
// background task does the SD write.
//
// Winch angles are winch motor deg. mV are motor millivolts -- the winch's in this
// class's convention (positive = up), so it agrees in sign with the err beside it
// rather than with what the motor was handed, and it is what drive() was asked for
// rather than what survived the PTO check. The claw's are positive = open, before
// CLAW_DIRECTION, and 0 once an open has braked. tower is the raw sensor read, where
// 9999 means nothing in view. own=0 means the winch is being commanded but the PTO is
// not ours yet, which is the one way drive() goes silently nowhere. WINCH tgt only
// means anything while a move to a height is running; elsewhere it is whatever the last
// one asked for and the err beside it is noise.
void Lift::log_debug_state() const
{
    const double winch = get_position();
    Logger::getInstance()->log(
        "LIFTDBG mode=%s state=%s pto=%s own=%d home=%d tower=%d loader=%d "
        "| WINCH tgt=%.1f act=%.1f err=%.1f mv=%d "
        "| CLAMP tgt=%s act=%.1f mv=%d done=%d",
        mode_name(mode_), state_name(state_),
        pto_.mode() == Pto::Mode::INTAKE ? "INTAKE" : "LIFT",
        pto_.is_ready(Pto::Mode::LIFT) ? 1 : 0, is_home() ? 1 : 0, get_tower_distance_mm(),
        get_loader_distance_mm(),
        height_target_deg_, winch, height_target_deg_ - winch, last_winch_cmd_mv_,
        clamp_name(clamp_target_), get_claw_position(), last_claw_mv_, clamp_done() ? 1 : 0);
}

void Lift::update()
{
    // Up button wins over everything else while it is held
    if (up_requested_)
    {
        if (in_macro())
            Logger::getInstance()->log("Lift macro cancelled by move_up");
        if (mode_ == Mode::MATCHLOAD)
        {
            // First press from the rest pose: the piece is held from here on
            mode_ = Mode::PRIMED;
            Logger::getInstance()->log("Lift PRIMED from MATCHLOAD at %.1f deg", get_position());
        }
        if (state_ != State::RAISING)
            Logger::getInstance()->log("Lift raising from %.1f deg", get_position());
        state_ = State::RAISING;
    }
    else if (down_requested_)
    {
        if (in_macro())
            Logger::getInstance()->log("Lift macro cancelled by hold_down");
        if (state_ != State::LOWERING_HELD)
            Logger::getInstance()->log("Lift lowering (held) from %.1f deg", get_position());
        state_ = State::LOWERING_HELD;
    }
    else if (state_ == State::RAISING)
    {
        // Button just released: start the slew from full voltage
        state_ = State::SLEWING;
        slew_mv_ = MAX_VOLTAGE;
    }
    else if (state_ == State::LOWERING_HELD)
    {
        state_ = State::IDLE;
        Logger::getInstance()->log("Lift holding at %.1f deg", get_position());
    }
    up_requested_ = false;
    down_requested_ = false;

    const int slew_step = static_cast<int>(MAX_VOLTAGE * Config::DT / SLEW_STOP_MS);

    switch (state_)
    {
    case State::HEIGHT_MOVE:
    {
        const bool timed_out = pros::millis() - move_start_ms_ >= HEIGHT_TIMEOUT_MS;
        if (run_to(height_target_deg_) || timed_out)
        {
            drive(0);
            state_ = State::IDLE;
            Logger::getInstance()->log("Lift height move %s at %.1f deg (target %.1f)",
                                       timed_out ? "timed out" : "done",
                                       get_position(), height_target_deg_);
        }
        break;
    }

    case State::RAISING:
        pto_.request(Pto::Mode::LIFT);
        drive(MAX_VOLTAGE);
        break;

    case State::SLEWING:
        slew_mv_ = std::max(0, slew_mv_ - slew_step);
        drive(slew_mv_);
        if (slew_mv_ == 0)
        {
            state_ = State::IDLE;
            Logger::getInstance()->log("Lift holding at %.1f deg", get_position());
        }
        break;

    case State::LOWERING:
        if (run_to(MATCHLOAD_HEIGHT_DEG))
        {
            mode_ = Mode::MATCHLOAD;
            state_ = State::IDLE;
            Logger::getInstance()->log("Lift back at %.1f deg -> MATCHLOAD", get_position());
        }
        break;

    case State::LOWERING_HELD:
        // Held down stops at the matchload pose, which is where the mode goes back to
        pto_.request(Pto::Mode::LIFT);
        if (get_position() <= MATCHLOAD_HEIGHT_DEG + HOME_TOLERANCE_DEG)
        {
            drive(0);
            if (mode_ != Mode::MATCHLOAD)
            {
                mode_ = Mode::MATCHLOAD;
                Logger::getInstance()->log("Lift back at %.1f deg -> MATCHLOAD", get_position());
            }
        }
        else
        {
            drive(-MAX_VOLTAGE); // capped to MAX_LOWER_MV in drive()
        }
        break;

    case State::SCORE_DESCEND:
    {
        pto_.request(Pto::Mode::LIFT);
        // score_at(): the height decides, not the sensor. Home counts as arrived too,
        // so a height below what the lift can reach still releases instead of stalling.
        const bool at_height = score_at_deg_ >= 0.0 &&
                               (get_position() <= score_at_deg_ || is_home());
        if (score_at_deg_ < 0.0 ? tower_in_range() : at_height)
        {
            // On the tower: the claw opens to let the piece go. No climb; the winch
            // just holds.
            mode_ = Mode::SCORING;
            state_ = State::SCORE_UNLOAD;
            drive(0);
            if (score_at_deg_ < 0.0)
                Logger::getInstance()->log("Lift score macro: on tower at %.1f deg (%d mm) -> opening claw",
                                           get_position(), get_tower_distance_mm());
            else
                Logger::getInstance()->log("Lift score macro: at %.1f deg (target %.1f, tower=%d mm ignored) -> opening claw",
                                           get_position(), score_at_deg_, get_tower_distance_mm());
        }
        else
        {
            // Never drive below home while hunting for the tower
            drive(is_home() ? 0 : SCORE_DESCEND_MV);
        }
        break;
    }

    case State::SCORE_UNLOAD:
        pto_.request(Pto::Mode::LIFT);
        hold();
        // apply_end_effector() set OPEN on the tick the mode became SCORING
        if (clamp_target_ == ClampPos::OPEN && clamp_done())
        {
            state_ = State::SCORE_RELEASE;
            release_start_ms_ = pros::millis();
            Logger::getInstance()->log("Lift score macro: claw open at %.1f deg -> released",
                                       get_position());
        }
        break;

    case State::SCORE_RELEASE:
        hold();
        if (!tower_in_range() && get_loader_distance_mm() > SCORE_CLEAR_LOADER_MM)
        {
            state_ = State::LOWERING;
            Logger::getInstance()->log("Lift score macro: tower cleared (tower=%d mm, loader=%d mm) -> returning to MATCHLOAD",
                                       get_tower_distance_mm(), get_loader_distance_mm());
        }
        else if (pros::competition::is_autonomous() &&
                 pros::millis() - release_start_ms_ >= SCORE_RELEASE_TIMEOUT_MS)
        {
            // A sensor that never clears must not strand the lift up for the rest of a route
            state_ = State::LOWERING;
            Logger::getInstance()->log("Lift score macro: release timed out after %u ms (tower=%d mm, loader=%d mm) -> returning to MATCHLOAD",
                                       static_cast<unsigned>(SCORE_RELEASE_TIMEOUT_MS),
                                       get_tower_distance_mm(), get_loader_distance_mm());
        }
        break;

    case State::MATCHLOAD_MOVE:
        if (run_to(MATCHLOAD_HEIGHT_DEG))
        {
            state_ = State::IDLE;
            Logger::getInstance()->log("Lift matchload macro: waiting on the loader at %.1f deg",
                                       get_position());
        }
        break;

    case State::MATCHLOAD_CLAMP:
        // apply_end_effector() set CLOSED on the tick the mode became PRIMED
        hold();
        if (clamp_target_ == ClampPos::CLOSED && clamp_done())
        {
            state_ = State::MATCHLOAD_RAISE;
            move_start_ms_ = pros::millis();
            Logger::getInstance()->log("Lift matchload: claw closed -> raising to %.1f deg",
                                       MATCHLOAD_PRIME_HEIGHT_DEG);
        }
        break;

    case State::MATCHLOAD_RAISE:
    {
        const bool timed_out = pros::millis() - move_start_ms_ >= HEIGHT_TIMEOUT_MS;
        if (run_to(MATCHLOAD_PRIME_HEIGHT_DEG) || timed_out)
        {
            drive(0);
            state_ = State::IDLE;
            Logger::getInstance()->log("Lift matchload macro: PRIMED at %.1f deg%s", get_position(),
                                       timed_out ? " (timed out)" : "");
        }
        break;
    }

    case State::IDLE:
        hold();
        // Resting in the matchload pose keeps the loader sensor armed, so the piece
        // after a scored matchload is taken without another B press. IDLE rather than a
        // waiting state: that would hold the PTO every tick and leave the intake dead
        // for as long as the lift sat at the loader.
        try_take_from_loader();
        break;
    }

    apply_end_effector();
    log_debug_state();
}

void Lift::try_take_from_loader()
{
    // Armed only once the jaws are fully open at the matchload pose: not mid-open
    // (straight after boot, a preload in view would otherwise be taken before the
    // route's first grab), and not while a grab holds the claw closed
    if (mode_ != Mode::MATCHLOAD || clamp_target_ != ClampPos::OPEN || !clamp_done() ||
        !loader_in_range())
        return;
    // PRIMED closes the claw
    mode_ = Mode::PRIMED;
    state_ = State::MATCHLOAD_CLAMP;
    Logger::getInstance()->log("Lift matchload: loader seen (%d mm) -> closing claw",
                               get_loader_distance_mm());
}

void Lift::set_clamp(ClampPos pos)
{
    if (pos == clamp_target_)
        return;
    clamp_target_ = pos;
    clamp_switch_ms_ = pros::millis();
    Logger::getInstance()->log("Lift clamp -> %s (%s)", clamp_name(pos), mode_name(mode_));
}

Lift::ClampPos Lift::mode_clamp() const
{
    switch (mode_)
    {
    case Mode::PRIMED:    return ClampPos::CLOSED;
    case Mode::MATCHLOAD: return grab_held_ ? ClampPos::CLOSED : ClampPos::OPEN;
    case Mode::SCORING:
    default:              return ClampPos::OPEN;
    }
}

void Lift::apply_end_effector()
{
    // A grab only means anything in the rest pose
    if (grab_held_ && mode_ != Mode::MATCHLOAD)
        grab_held_ = false;

    if (!boot_hold_)
        set_clamp(mode_clamp());
    boot_hold_ = false;

    // No position control: open spins for CLAW_OPEN_MS and then brakes where it got to;
    // closed runs in hard for CLAW_CLOSE_MS and then keeps a light squeeze on the piece
    int claw_mv = 0;
    if (clamp_target_ == ClampPos::OPEN)
        claw_mv = clamp_done() ? 0 : CLAW_OPEN_MV;
    else
        claw_mv = -(clamp_done() ? CLAW_SQUEEZE_MV : CLAW_CLOSE_MV);

    if (claw_mv == 0)
        claw_motor_.brake();
    else
        claw_motor_.move_voltage(CLAW_DIRECTION * claw_mv);
    last_claw_mv_ = claw_mv;
}
