#include "hardware/pto.hpp"
#include "pros/rtos.hpp"
#include "utilities/logger.hpp"

Pto::Pto(pros::MotorGroup &motors, EnhancedDigitalOut &solenoid, Mode initial)
    : motors_(motors), solenoid_(solenoid), mode_(initial),
      shift_started_ms_(0)
{
    apply_mode(mode_);
    motors_.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
}

void Pto::apply_mode(Mode mode)
{
    solenoid_.set_state(mode == Mode::INTAKE ? INTAKE_SOLENOID_STATE : !INTAKE_SOLENOID_STATE);
}

void Pto::request(Mode mode)
{
    if (mode == mode_)
        return;

    // brake(), not move_voltage(0): the gears are about to mesh, so the motors have to
    // actually be stopped for SHIFT_TIME_MS, not left freewheeling down from whatever
    // the outgoing subsystem was last commanding.
    motors_.brake();
    last_mv_ = 0;
    apply_mode(mode);
    mode_ = mode;
    shift_started_ms_ = pros::millis();
    Logger::getInstance()->log("PTO -> %s", mode == Mode::INTAKE ? "INTAKE" : "LIFT");
}

bool Pto::is_ready() const
{
    return pros::millis() - shift_started_ms_ >= SHIFT_TIME_MS;
}

void Pto::run(Mode mode, int millivolts)
{
    // Not our mode: the other subsystem owns the motors, so leave its command alone
    // instead of zeroing it out from under it (Lift::drive() reaches here while SLEWING
    // if the intake has taken the PTO back).
    if (mode != mode_)
        return;
    if (!is_ready())
    {
        // Our mode, but the shift has not settled: hold the motors stopped rather than
        // coasting them through the mesh.
        motors_.brake();
        last_mv_ = 0;
        return;
    }
    motors_.move_voltage(millivolts);
    last_mv_ = millivolts;
}
