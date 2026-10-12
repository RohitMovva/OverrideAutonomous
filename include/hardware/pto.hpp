#ifndef PTO_HPP
#define PTO_HPP

#include "pros/motors.hpp"
#include "hardware/enhanced_digital_out.hpp"
#include <cstdint>

/**
 * @brief Power take-off: two motors shared between the lift and the intake.
 *
 * A single solenoid shifts the gearbox. Whichever subsystem wants the motors
 * asks for its mode via @ref request(); the motors are only driven once the
 * solenoid has been in that position long enough for the shift to complete
 * (@ref is_ready). Both Lift and Intake hold a reference to one Pto and never
 * touch the motors directly.
 */
class Pto
{
public:
    enum class Mode
    {
        LIFT,
        INTAKE
    };

    /// How long after actuating the solenoid the gears are assumed meshed
    static constexpr uint32_t SHIFT_TIME_MS = 150;
    /// Solenoid level that selects the intake.
    static constexpr bool INTAKE_SOLENOID_STATE = false;

    Pto(pros::MotorGroup &motors, EnhancedDigitalOut &solenoid, Mode initial = Mode::LIFT);

    /// Shift to @p mode. No-op if already there. The motors are braked on a shift so a
    /// spinning gear never has to mesh.
    void request(Mode mode);

    Mode mode() const { return mode_; }

    /// True once the last shift has had SHIFT_TIME_MS to settle
    bool is_ready() const;

    /// True if in @p mode AND settled -- the one check a subsystem needs before driving
    bool is_ready(Mode mode) const { return mode_ == mode && is_ready(); }

    /// Drive the shared motors (mV, +-12000). Callers must own the current mode: the
    /// call does nothing at all if the PTO is in another mode (the owner's command
    /// stands), and brakes the motors if @p mode is ours but the shift is still settling.
    void run(Mode mode, int millivolts);

    /// Stop the motors using their brake mode (HOLD once the lift has initialised)
    void stop()
    {
        motors_.brake();
        last_mv_ = 0;
    }

    pros::MotorGroup &motors() { return motors_; }

    /// Last voltage sent to the PTO motors (mV), whoever sent it
    int last_command_mv() const { return last_mv_; }

private:
    /// Put the solenoid where @p mode says it belongs
    void apply_mode(Mode mode);

    pros::MotorGroup &motors_;
    EnhancedDigitalOut &solenoid_;
    Mode mode_;
    uint32_t shift_started_ms_;
    int last_mv_ = 0;
};

#endif // PTO_HPP
