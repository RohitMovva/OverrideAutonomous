#include "hardware/intake.hpp"
#include "utilities/logger.hpp"
#include "pros/rtos.hpp"

Intake::Intake(Pto &pto, EnhancedDigitalOut &lift_solenoid, pros::Distance &distance_sensor)
    : pto_(pto), lift_solenoid_(lift_solenoid), distance_sensor_(distance_sensor)
{
}

void Intake::raise_lift()
{
    if (lift_solenoid_.get_state())
        return;
    lift_solenoid_.set_state(true);
    lift_raised_ms_ = pros::millis();
    piece_seen_ms_ = 0;
    Logger::getInstance()->log("Intake lift UP (auto-drop arms in %u ms, dist=%d mm)",
                               LIFT_ARM_DELAY_MS, get_distance_mm());
}

void Intake::lower_lift()
{
    if (!lift_solenoid_.get_state())
        return;
    lift_solenoid_.set_state(false);
    piece_seen_ms_ = 0;
    Logger::getInstance()->log("Intake lift DOWN");
}

int Intake::get_distance_mm() const
{
    return distance_sensor_.get_distance();
}

void Intake::set_state(int state)
{
    if (state == state_)
        return;
    state_ = state;
    Logger::getInstance()->log("Intake state -> %d", state_);
}

void Intake::update()
{
    // Auto-drop: once armed, the first close reading starts a timer and the intake
    // drops LIFT_DROP_DELAY_MS later. Distance returns 9999 with nothing in view and a
    // negative value on error, so only a positive reading under the threshold counts.
    if (lift_solenoid_.get_state())
    {
        const uint32_t now = pros::millis();
        if (piece_seen_ms_ == 0)
        {
            if (now - lift_raised_ms_ >= LIFT_ARM_DELAY_MS)
            {
                const int d = get_distance_mm();
                if (d > 0 && d < LIFT_DROP_DISTANCE_MM)
                {
                    piece_seen_ms_ = now;
                    Logger::getInstance()->log("Intake sees piece at %d mm, dropping in %u ms",
                                               d, LIFT_DROP_DELAY_MS);
                }
            }
        }
        else if (now - piece_seen_ms_ >= LIFT_DROP_DELAY_MS)
        {
            Logger::getInstance()->log("Intake lift auto-drop");
            lift_solenoid_.set_state(false);
            piece_seen_ms_ = 0;
        }
    }

    int mv = 0;
    switch (state_)
    {
    case 1:
        mv = MAX_VOLTAGE;
        break;
    case 2:
        mv = -MAX_VOLTAGE;
        break;
    default:
        mv = 0;
        break;
    }

    if (mv == 0 || inhibited_)
    {
        // Stopped: release the motors once, on the tick we stop, and only if we still
        // own them. Braking every idle tick would fight the roller mech, which runs on
        // these motors right after us in the same tick whenever the intake is idle.
        if (driving_ && pto_.mode() == Pto::Mode::INTAKE)
            pto_.stop();
        driving_ = false;
        return;
    }

    // Shift first; Pto::run() stays at zero until the shift has settled
    pto_.request(Pto::Mode::INTAKE);
    pto_.run(Pto::Mode::INTAKE, mv);
    driving_ = true;
}
