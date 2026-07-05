#include "hardware/lift.hpp"
#include "config.hpp"
#include <cmath>

// Raising should feel responsive, so the command ramps up quickly; every other
// transition (easing to a stop, easing into a descent) uses the gentler release
// rate passed to the constructor so the lift never jerks.
static constexpr double RAISE_SLEW_RATE_MV_S = 96000.0;

Lift::Lift(pros::Rotation& lift_rotation_sensor,
           pros::Motor& lift_motor,
           EnhancedDigitalOut& claw_angler,
           EnhancedDigitalOut& claw_clamp,
           double raise_mv,
           double lower_mv,
           double ease_rate_mv_s,
           double score_lift_deg,
           PIDController hold_pid)
    : rotation_sensor_(lift_rotation_sensor),
      motor_(lift_motor),
      claw_angler_(claw_angler),
      claw_clamp_(claw_clamp),
      raise_mv_(raise_mv),
      lower_mv_(lower_mv),
      score_lift_deg_(score_lift_deg),
      slew_(RAISE_SLEW_RATE_MV_S, ease_rate_mv_s),
      hold_pid_(hold_pid),
      mode_(Mode::REST),
      raise_requested_(false),
      hold_angle_(0.0),
      descent_start_ms_(0),
      last_update_ms_(0) {}

double Lift::get_angle() const {
    return rotation_sensor_.get_position() / 100.0;  // centidegrees -> degrees
}

void Lift::raise() {
    raise_requested_ = true;
}

void Lift::move_down() {
    // Open the claw on the way down.
    claw_clamp_.set_state(false);
    claw_angler_.set_state(false);
    mode_ = Mode::LOWERING;
    descent_start_ms_ = pros::millis();
}

void Lift::score() {
    mode_ = Mode::SCORING;
    descent_start_ms_ = pros::millis();
}

void Lift::stop() {
    hold_angle_ = get_angle();
    hold_pid_.reset();
    mode_ = Mode::HOLD;
}

bool Lift::descent_stalled(uint32_t now) const {
    bool past_grace = (now - descent_start_ms_) > STALL_GRACE_MS;
    bool not_moving = std::abs(static_cast<double>(rotation_sensor_.get_velocity()))
                      < STALL_VELOCITY_CDPS;
    return past_grace && not_moving;
}

void Lift::update() {
    uint32_t now = pros::millis();
    double dt = (last_update_ms_ == 0)
                    ? Config::DT / 1000.0
                    : (now - last_update_ms_) / 1000.0;
    if (dt <= 0.0) {
        dt = Config::DT / 1000.0;
    }
    last_update_ms_ = now;

    // Raising always wins: the driver can override any other motion. Engage the
    // claw while raising. When the request stops, capture the current angle and
    // switch to a closed-loop hold so the lift stays where it was left.
    if (raise_requested_) {
        if (mode_ != Mode::RAISING) {
            mode_ = Mode::RAISING;
        }
        claw_angler_.set_state(true);
        claw_clamp_.set_state(true);
    } else if (mode_ == Mode::RAISING) {
        hold_angle_ = get_angle();
        hold_pid_.reset();
        mode_ = Mode::HOLD;
    }

    double target_mv = 0.0;
    switch (mode_) {
        case Mode::RAISING:
            target_mv = raise_mv_;
            break;

        case Mode::HOLD:
            // Closed-loop hold against gravity / band imbalance.
            target_mv = hold_pid_.calculate(hold_angle_, get_angle());
            break;

        case Mode::LOWERING:
            if (descent_stalled(now)) {
                mode_ = Mode::REST;          // settled on the hard stop
                target_mv = 0.0;
            } else {
                target_mv = -lower_mv_;      // eased in by the slew limiter
            }
            break;

        case Mode::SCORING:
            if (descent_stalled(now)) {
                // Met resistance: release pin/cup, then lift a few inches and
                // hold there (the HOLD PID drives up to the raised target).
                claw_clamp_.set_state(false);
                hold_angle_ = get_angle() + score_lift_deg_;
                hold_pid_.reset();
                mode_ = Mode::HOLD;
                target_mv = hold_pid_.calculate(hold_angle_, get_angle());
            } else {
                target_mv = -lower_mv_;      // eased descent onto the target
            }
            break;

        case Mode::REST:
            target_mv = 0.0;
            break;
    }

    double output_mv = slew_.calculate(target_mv, dt);
    motor_.move_voltage(static_cast<int>(output_mv));

    // Consume the raise request; the driver must re-assert it each loop.
    raise_requested_ = false;
}
