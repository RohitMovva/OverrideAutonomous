#include "hardware/roller_mech.hpp"
#include "utilities/logger.hpp"
#include "pros/rtos.hpp"
#include <cstdlib>

RollerMech::RollerMech(Pto &pto, pros::Optical &optical, pros::Rotation &rotation)
    : pto_(pto), optical_(optical), rotation_(rotation)
{
    optical_.set_led_pwm(100);
    optical_.set_integration_time(20.0);
    rotation_.set_data_rate(5);
    rotation_.reset_position();
}

const char *RollerMech::color_name(Color c)
{
    switch (c)
    {
    case Color::RED:    return "RED";
    case Color::BLUE:   return "BLUE";
    case Color::YELLOW: return "YELLOW";
    default:            return "UNKNOWN";
    }
}

const char *RollerMech::state_name(State s)
{
    switch (s)
    {
    case State::SEEKING: return "SEEKING";
    case State::OVERRUN: return "OVERRUN";
    default:             return "IDLE";
    }
}

void RollerMech::set_target_color(Color color)
{
    if (color != Color::RED && color != Color::BLUE)
    {
        Logger::getInstance()->logError("RollerMech target must be RED or BLUE, got %s", color_name(color));
        return;
    }
    target_ = color;
    Logger::getInstance()->log("RollerMech target colour -> %s", color_name(target_));
}

void RollerMech::set_enabled(bool enabled)
{
    if (enabled == enabled_)
        return;
    enabled_ = enabled;
    Logger::getInstance()->log("RollerMech %s (target %s)", enabled_ ? "enabled" : "disabled",
                               color_name(target_));
}

bool RollerMech::needs_turning() const
{
    if (!roller_in_view())
        return false;
    const Color seen = read_color();
    return seen != Color::UNKNOWN && seen != target_;
}

bool RollerMech::roller_in_view() const
{
    return optical_.get_proximity() > PROXIMITY_THRESHOLD;
}

RollerMech::Color RollerMech::read_color() const
{
    const double hue = optical_.get_hue();
    if (hue < RED_HUE_MAX || hue > RED_HUE_MIN_WRAP)
        return Color::RED;
    if (hue >= YELLOW_HUE_MIN && hue <= YELLOW_HUE_MAX)
        return Color::YELLOW;
    if (hue >= BLUE_HUE_MIN && hue <= BLUE_HUE_MAX)
        return Color::BLUE;
    return Color::UNKNOWN;
}

double RollerMech::travel_deg() const
{
    // Rotation sensor reports centidegrees, accumulated (not wrapped)
    return rotation_.get_position() / 100.0 * spin_sign();
}

int RollerMech::drive_mv() const
{
    const int magnitude = (state_ == State::OVERRUN) ? std::abs(OVERRUN_MV) : std::abs(SPIN_MV);
    return magnitude * spin_sign();
}

void RollerMech::reset_rotation()
{
    rotation_.reset_position();
    cycle_start_deg_ = 0.0;
    Logger::getInstance()->log("RollerMech rotation zeroed");
}

void RollerMech::stop_cycle(const char *why)
{
    if (state_ == State::IDLE)
        return;
    Logger::getInstance()->log("RollerMech stop (%s) %.1f deg into the cycle, state %s",
                               why, cycle_travel_deg(), state_name(state_));
    state_ = State::IDLE;
    overrun_target_deg_ = 0.0;
    overrun_deadline_ms_ = 0;
    if (pto_.mode() == Pto::Mode::INTAKE)
        pto_.stop();
}

void RollerMech::update()
{
    update_cycle();
    log_debug_state();
}

// src=ROLLER while a cycle is driving the motors, INTAKE otherwise (the only other
// thing that runs them in INTAKE mode, and the roller is inhibited while it does). cmd
// is the last voltage sent to the PTO. rpm/mv/ma are what the three PTO motors measure
// (rpm at the cartridge output, already sign-corrected for the reversed ports), so a
// roller cycle and the intake can be compared on the same motors. AXLE is the roller
// rotation sensor: pos in deg along the spin direction, dps its own velocity reading.
void RollerMech::log_debug_state()
{
    if (pto_.mode() != Pto::Mode::INTAKE)
        return;
    const int cmd = pto_.last_command_mv();
    if (cmd == 0 && state_ == State::IDLE)
        return;
    pros::MotorGroup &m = pto_.motors();
    const auto rpm = m.get_actual_velocity_all();
    const auto mv = m.get_voltage_all();
    const auto ma = m.get_current_draw_all();
    // A group call returns fewer entries if a motor is unplugged; log 0 for it
    const auto at = [](const auto &v, std::size_t i) { return i < v.size() ? v[i] : 0; };
    Logger::getInstance()->log(
        "ROLLERDBG src=%s state=%s cmd=%d "
        "| PTO rpm=%.0f,%.0f,%.0f mv=%d,%d,%d ma=%d,%d,%d "
        "| AXLE pos=%.1f dps=%.0f",
        state_ == State::IDLE ? "INTAKE" : "ROLLER", state_name(state_), cmd,
        static_cast<double>(at(rpm, 0)), static_cast<double>(at(rpm, 1)),
        static_cast<double>(at(rpm, 2)),
        static_cast<int>(at(mv, 0)), static_cast<int>(at(mv, 1)), static_cast<int>(at(mv, 2)),
        static_cast<int>(at(ma, 0)), static_cast<int>(at(ma, 1)), static_cast<int>(at(ma, 2)),
        travel_deg(), rotation_.get_velocity() / 100.0 * spin_sign());
}

void RollerMech::update_cycle()
{
    if (!enabled_ || inhibited_)
    {
        stop_cycle(enabled_ ? "inhibited" : "disabled");
        return;
    }
    // Dormant unless the PTO is in intake mode. Only with shifts_pto_ does the mech put
    // it there itself, and only for a roller that needs turning, so an idle mech never
    // takes the motors off a lift that is holding height.
    if (!pto_.is_ready(Pto::Mode::INTAKE))
    {
        stop_cycle("PTO not in intake mode");
        if (shifts_pto_ && pto_.mode() != Pto::Mode::INTAKE)
        {
            if (needs_turning())
            {
                Logger::getInstance()->log("RollerMech shifting PTO to INTAKE: sees %s, want %s",
                                           color_name(read_color()), color_name(target_));
                pto_.request(Pto::Mode::INTAKE);
                prime_pending_ = false;
            }
            else if (prime_pending_)
            {
                Logger::getInstance()->log("RollerMech priming PTO to INTAKE ahead of the roller");
                pto_.request(Pto::Mode::INTAKE);
                prime_pending_ = false;
            }
        }
        return;
    }
    // Already in INTAKE: nothing left to prime
    prime_pending_ = false;

    switch (state_)
    {
    case State::OVERRUN:
    {
        // Colour no longer matters -- we are just running out the fixed distance
        if (!roller_in_view())
        {
            stop_cycle("roller left view");
            return;
        }
        if (travel_deg() >= overrun_target_deg_)
        {
            Logger::getInstance()->log("RollerMech done: overran %.1f deg, %.1f deg this cycle",
                                       OVERRUN_DEG, cycle_travel_deg());
            state_ = State::IDLE;
            overrun_target_deg_ = 0.0;
            overrun_deadline_ms_ = 0;
            pto_.stop();
            return;
        }
        if (pros::millis() >= overrun_deadline_ms_)
        {
            Logger::getInstance()->logError("RollerMech overrun timeout: %.1f deg short of target",
                                            overrun_target_deg_ - travel_deg());
            state_ = State::IDLE;
            overrun_target_deg_ = 0.0;
            overrun_deadline_ms_ = 0;
            pto_.stop();
            return;
        }
        break;
    }

    case State::SEEKING:
    {
        if (!roller_in_view())
        {
            stop_cycle("roller left view");
            return;
        }
        const Color seen = read_color();
        if (seen == target_)
        {
            // Overrun is measured from right here, so it is the same travel every time
            overrun_target_deg_ = travel_deg() + OVERRUN_DEG;
            overrun_deadline_ms_ = pros::millis() + OVERRUN_TIMEOUT_MS;
            state_ = State::OVERRUN;
            Logger::getInstance()->log("RollerMech sees %s, overrunning %.1f deg", color_name(seen), OVERRUN_DEG);
        }
        break;
    }

    case State::IDLE:
    {
        if (!roller_in_view())
            return;
        const Color seen = read_color();
        if (seen == Color::UNKNOWN || seen == target_)
            return;
        cycle_start_deg_ = travel_deg(); // baseline, for telemetry only
        Logger::getInstance()->log("RollerMech spin: sees %s, want %s", color_name(seen), color_name(target_));
        state_ = State::SEEKING;
        break;
    }
    }

    if (state_ != State::IDLE)
        pto_.run(Pto::Mode::INTAKE, drive_mv());
}
