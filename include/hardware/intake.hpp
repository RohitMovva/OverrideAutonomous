#pragma once

#include "hardware/pto.hpp"
#include "hardware/enhanced_digital_out.hpp"
#include "pros/distance.hpp"
#include <cstdint>

/**
 * @brief Intake driven by the PTO motors, with a pneumatic lift.
 *
 * The intake has no motors of its own: it runs off the shared PTO motors (see
 * @ref Pto). Whenever a non-zero state is commanded, @ref update() first shifts
 * the PTO to INTAKE and only drives the motors once the shift has settled. If
 * the lift takes the PTO back, the intake simply waits.
 *
 * One solenoid raises the whole intake. @ref raise_lift() puts it up and, after
 * LIFT_ARM_DELAY_MS, arms the distance sensor: from then on, once the sensor reads
 * under LIFT_DROP_DISTANCE_MM the intake drops back down on its own LIFT_DROP_DELAY_MS
 * later (see @ref update).
 *
 * State codes match the auton action-point convention:
 *   0 = stop, 1 = intake, 2 = outtake
 */
class Intake
{
public:
    static constexpr int MAX_VOLTAGE = 12000;
    /// Distance reading (mm) below which a raised intake drops itself. TUNE
    static constexpr int LIFT_DROP_DISTANCE_MM = 150;
    /// Ignore the distance sensor for this long after raising, so whatever was already
    /// under the intake when it went up doesn't drop it straight back down
    static constexpr uint32_t LIFT_ARM_DELAY_MS = 750;
    /// Once a piece is seen, wait this long before actually dropping the intake so the
    /// piece is fully under it rather than just at the sensor's edge. TUNE
    static constexpr uint32_t LIFT_DROP_DELAY_MS = 0;

    Intake(Pto &pto, EnhancedDigitalOut &lift_solenoid, pros::Distance &distance_sensor);

    void set_state(int state);
    int get_state() const { return state_; }

    /// While true the intake holds its commanded state but leaves the PTO alone
    /// (the lift is using it). Set every tick by whoever owns the arbitration.
    void set_inhibited(bool inhibited) { inhibited_ = inhibited; }

    /// Raise the intake; it lowers itself once the distance sensor trips
    void raise_lift();
    /// Force the intake down and disarm the auto-drop
    void lower_lift();
    bool is_lifted() const { return lift_solenoid_.get_state(); }
    /// Last distance-sensor reading, mm
    int get_distance_mm() const;

    /// Call every Config::DT ms
    void update();

private:
    Pto &pto_;
    EnhancedDigitalOut &lift_solenoid_;
    pros::Distance &distance_sensor_;
    int state_ = 0;
    bool inhibited_ = false;
    bool driving_ = false;        ///< we commanded the PTO motors last tick, so a stop must release them
    uint32_t lift_raised_ms_ = 0; ///< pros::millis() when raise_lift() fired
    uint32_t piece_seen_ms_ = 0;  ///< pros::millis() when the sensor first tripped; 0 = not yet
};
