#ifndef ROLLER_MECH_HPP
#define ROLLER_MECH_HPP

#include "hardware/pto.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"
#include <cstdint>

/**
 * @brief Field roller spinner. Runs off the PTO motors (INTAKE mode), no driver input.
 *
 * Fully automatic off an optical sensor plus a dedicated rotation sensor on the roller
 * axle. When the optical sensor sees the roller up close (proximity above
 * PROXIMITY_THRESHOLD) showing a colour that is NOT the target, the PTO motors spin
 * until the target colour appears (SEEKING), then keep spinning a fixed
 * @ref OVERRUN_DEG further (OVERRUN) so the roller settles fully onto the colour
 * instead of stopping on the boundary between two.
 *
 * The overrun is measured by the rotation sensor from the reading taken the instant the
 * target colour appeared, so it is the same amount of roller travel every time,
 * whatever the roller's speed or how promptly the loop noticed the colour.
 *
 * If the roller leaves view mid-spin the motors stop, as does losing the PTO
 * (inhibited, or shifted to LIFT).
 *
 * Travel is measured in the spin direction: @ref OVERRUN_DEG is always counted as
 * positive regardless of the sign of @ref SPIN_MV, so the rotation sensor only has to
 * be reversed (negative port) if it reads backwards from the roller.
 *
 * Shares the PTO with the lift and intake. By default it NEVER shifts it: if the PTO
 * is in LIFT mode the mech stays dormant until the intake has put it in INTAKE mode.
 * With @ref set_shifts_pto (autonomous, where no driver is running the intake) it
 * instead requests INTAKE itself, but only once it actually sees a roller showing the
 * wrong colour, so the lift keeps the PTO (and its winch hold) the rest of the time.
 * Whoever owns arbitration also sets @ref set_inhibited every tick so the roller
 * only drives the motors when nobody else is using them.
 *
 * @ref set_enabled gates the whole mech: autonomous starts it disabled and a route's
 * `roller` action point turns it on and off (Robot::processAction).
 *
 * Every tick the PTO motors are running in INTAKE mode -- a roller cycle or the intake,
 * which turns the roller too since the two are geared together -- @ref update logs a
 * ROLLERDBG line with the PTO motors' measured speed, voltage and current and the
 * roller axle's speed, so the two cases can be compared (tests/log_analysis).
 */
class RollerMech
{
public:
    enum class Color
    {
        UNKNOWN,
        RED,
        BLUE,
        YELLOW
    };

    /// What the mech is doing right now
    enum class State
    {
        IDLE,    ///< not driving the motors
        SEEKING, ///< spinning, waiting for the target colour
        OVERRUN  ///< target seen, spinning out the fixed OVERRUN_DEG
    };

    /// Voltage while seeking the target colour. (sign = direction)
    static constexpr int SPIN_MV = 8000;
    /// Voltage during the overrun. Drop it below SPIN_MV to limit how far the roller
    /// coasts past the stop. Sign is taken from SPIN_MV, not from this.
    static constexpr int OVERRUN_MV = 8000;
    /// How far the roller keeps turning after the target colour appears, in rotation
    /// sensor degrees.
    static constexpr double OVERRUN_DEG = 0.0;
    /// Give up on the overrun after this long (roller slipping, dead rotation sensor) so
    /// the mech can never sit on the PTO forever.
    static constexpr uint32_t OVERRUN_TIMEOUT_MS = 1500;
    /// Optical proximity (0-255, higher = closer) above which the roller is "in view". TUNE
    static constexpr int PROXIMITY_THRESHOLD = 50;
    /// Hue bands (deg). TUNE against the real roller under the sensor LED
    static constexpr double RED_HUE_MAX = 20.0;    ///< red wraps: hue < RED_HUE_MAX or > RED_HUE_MIN_WRAP
    static constexpr double RED_HUE_MIN_WRAP = 340.0;
    static constexpr double YELLOW_HUE_MIN = 40.0;
    static constexpr double YELLOW_HUE_MAX = 90.0;
    static constexpr double BLUE_HUE_MIN = 190.0;
    static constexpr double BLUE_HUE_MAX = 260.0;

    RollerMech(Pto &pto, pros::Optical &optical, pros::Rotation &rotation);

    /// Colour we want the roller to show. RED or BLUE; set from main during init
    void set_target_color(Color color);
    Color get_target_color() const { return target_; }

    /// While true the roller never touches the PTO (lift or intake is using it)
    void set_inhibited(bool inhibited) { inhibited_ = inhibited; }

    /// Disabled, the mech stops any cycle in progress and ignores the sensor
    void set_enabled(bool enabled);
    bool is_enabled() const { return enabled_; }

    /// When true, a roller in view showing the wrong colour shifts the PTO to INTAKE
    /// rather than waiting for the intake to have done it. Off for driver control.
    void set_shifts_pto(bool shifts) { shifts_pto_ = shifts; }

    /// Shift the PTO to INTAKE ahead of reaching a roller, once, so a cycle can start
    /// spinning the moment the roller comes into view instead of waiting SHIFT_TIME_MS.
    /// Happens on the first tick the mech is enabled, not inhibited and allowed to shift
    /// (@ref set_shifts_pto); cleared once the PTO is in INTAKE. One-shot on purpose:
    /// it takes the motors (and the winch hold) off the lift, which the mech otherwise
    /// only does for a roller it can see. A lift command afterwards takes them back.
    void prime_pto() { prime_pending_ = true; }

    State get_state() const { return state_; }
    bool is_spinning() const { return state_ != State::IDLE; }
    bool roller_in_view() const;
    /// Colour currently under the sensor (UNKNOWN if hue is outside every band)
    Color read_color() const;

    /// Raw rotation sensor travel since the last @ref reset_rotation, measured positive
    /// in the spin direction (degrees)
    double travel_deg() const;
    /// Travel since the current cycle started, i.e. since the mech last left IDLE
    /// (degrees). After a cycle finishes it holds what that cycle turned.
    double cycle_travel_deg() const { return travel_deg() - cycle_start_deg_; }

    /// Zero the rotation sensor and the cycle baseline with it
    void reset_rotation();

    /// Call every Config::DT ms, after Intake::update() so the tick's PTO command is final
    void update();

    static const char *color_name(Color c);
    static const char *state_name(State s);

private:
    /// +1 or -1: which way travel runs relative to raw rotation sensor position
    static constexpr int spin_sign() { return SPIN_MV < 0 ? -1 : 1; }
    /// Voltage for the current state, already carrying SPIN_MV's sign
    int drive_mv() const;
    /// Stop the motors and go IDLE, logging @p why
    void stop_cycle(const char *why);
    /// A roller is in view showing a known colour that is not the target
    bool needs_turning() const;
    /// The cycle state machine; @ref update runs it, then logs
    void update_cycle();
    /// One ROLLERDBG line, while the PTO motors run in INTAKE mode (see class docs)
    void log_debug_state();

    Pto &pto_;
    pros::Optical &optical_;
    bool prime_pending_ = false; ///< see prime_pto()
    pros::Rotation &rotation_;
    Color target_ = Color::RED;
    bool inhibited_ = false;
    bool enabled_ = true;
    bool shifts_pto_ = false;
    State state_ = State::IDLE;
    double cycle_start_deg_ = 0.0;    ///< travel_deg() when this cycle left IDLE
    double overrun_target_deg_ = 0.0; ///< travel_deg() at which the overrun ends
    uint32_t overrun_deadline_ms_ = 0;
};

#endif // ROLLER_MECH_HPP
