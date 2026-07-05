#ifndef LIFT_HPP
#define LIFT_HPP

#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "filters/slew_rate_limiter.hpp"
#include "controllers/pid_controller.hpp"
#include "hardware/enhanced_digital_out.hpp"

/**
 * @brief Double Reverse Four-Bar (DR4B) lift subsystem with a claw.
 *
 * Control model, tailored to a band-assisted DR4B:
 *
 *  - Hold to raise: while the driver holds the lift button, @ref raise() is
 *    called each loop and the lift drives up at a fixed voltage. Raising also
 *    engages the claw (angler + clamp).
 *  - Ease + hold: on release the command slews down smoothly (no jerk) and the
 *    lift captures its current angle and holds it closed-loop (PID) so it does
 *    not sag from where the driver left it.
 *  - Return to bottom: @ref move_down() opens the claw (clamp + angler off) and
 *    eases the lift down until it stalls against the hard stop, so it settles
 *    gently instead of slamming.
 *  - Score: @ref score() lowers until it meets resistance, releases the clamp,
 *    then lifts a few inches and holds.
 *
 * @ref update() must be called every loop iteration (e.g. every Config::DT ms)
 * for the slew easing, PID hold, and stall detection to work.
 */
class Lift {
public:
    /**
     * @brief Construct a DR4B lift.
     *
     * @param lift_rotation_sensor Rotation sensor on the lift (angle + stall detection)
     * @param lift_motor           Lift drive motor
     * @param claw_angler          Claw angler solenoid (engaged while raising)
     * @param claw_clamp           Claw clamp solenoid (engaged while raising, released on score / bottom)
     * @param raise_mv             Upward drive voltage while raising (millivolts)
     * @param lower_mv             Downward drive voltage while lowering (millivolts, kept modest so it eases in)
     * @param ease_rate_mv_s       How fast the command eases toward its target (mV per second)
     * @param score_lift_deg       Angle to raise after releasing on a score (~ a few inches; tune to geometry)
     * @param hold_pid             Position-hold PID (error in degrees, output in millivolts)
     */
    Lift(pros::Rotation& lift_rotation_sensor,
         pros::Motor& lift_motor,
         EnhancedDigitalOut& claw_angler,
         EnhancedDigitalOut& claw_clamp,
         double raise_mv = 12000.0,
         double lower_mv = 5000.0,
         double ease_rate_mv_s = 48000.0,
         double score_lift_deg = 20.0,
         PIDController hold_pid = PIDController(350.0, 60.0, 1200.0, -12000.0, 12000.0));

    /**
     * @brief Request the lift to raise this iteration (and engage the claw).
     *
     * Call every loop while the driver holds the raise button. Takes priority
     * over lowering / scoring. When you stop calling it (button released) the
     * lift eases to a stop and holds its current angle.
     */
    void raise();

    /**
     * @brief Return the lift to the bottom.
     *
     * Opens the claw (clamp + angler off) and eases the lift down until it
     * stalls against its hard stop, then rests.
     */
    void move_down();

    /**
     * @brief Score: lower onto the target, release, and lift clear.
     *
     * Eases the lift down until it meets resistance, releases the claw clamp,
     * then raises a few inches (@p score_lift_deg) and holds there.
     */
    void score();

    /**
     * @brief Ease to a stop and hold the current angle.
     */
    void stop();

    /**
     * @brief Resolve the current intent, apply control + slew easing, drive the motor.
     *
     * Must be called every loop iteration.
     */
    void update();

    /**
     * @brief Current lift angle in degrees (rotation sensor reading).
     */
    double get_angle() const;

private:
    /// Internal control state.
    enum class Mode {
        HOLD,      ///< Closed-loop hold at hold_angle_
        RAISING,   ///< Open-loop drive up (driver holding button)
        LOWERING,  ///< Eased descent to the bottom hard stop
        SCORING,   ///< Eased descent until resistance, then release + lift
        REST       ///< Resting on the bottom hard stop, motor off
    };

    pros::Rotation& rotation_sensor_;
    pros::Motor& motor_;
    EnhancedDigitalOut& claw_angler_;
    EnhancedDigitalOut& claw_clamp_;

    double raise_mv_;
    double lower_mv_;
    double score_lift_deg_;

    SlewRateLimiter slew_;   ///< Eases the applied voltage between targets
    PIDController hold_pid_;  ///< Closed-loop position hold

    Mode mode_;
    bool raise_requested_;         ///< raise() called this cycle (consumed by update)
    double hold_angle_;            ///< Angle the lift is holding (deg)
    uint32_t descent_start_ms_;    ///< millis() when the current descent began
    uint32_t last_update_ms_;      ///< millis() of previous update (for dt)

    /// Below this |velocity| (centidegrees/sec) while descending counts as stalled / resistance.
    static constexpr double STALL_VELOCITY_CDPS = 200.0;
    /// Minimum descent time before stall detection is trusted (lets it start moving).
    static constexpr uint32_t STALL_GRACE_MS = 250;

    /// True once the lift has descended long enough and is no longer moving.
    bool descent_stalled(uint32_t now) const;
};

#endif // LIFT_HPP
