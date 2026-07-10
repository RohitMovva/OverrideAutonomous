#include "hardware/lift.hpp"
#include "config.hpp"
#include "pros/llemu.hpp"  // pros::lcd::print for the brain-screen readout
#include <cmath>

// Raising should feel responsive, so the command ramps up quickly; every other
// transition (easing to a stop, easing into a descent) uses the gentler release
// rate passed to the constructor so the lift never jerks.
static constexpr double RAISE_SLEW_RATE_MV_S = 96000.0;

Lift::Lift(pros::Rotation& lift_rotation_sensor,
           pros::MotorGroup& lift_motors,
           EnhancedDigitalOut& claw_angler,
           EnhancedDigitalOut& claw_clamp,
           double raise_mv,
           double lower_mv,
           double score_mv,
           double ease_rate_mv_s,
           double score_lift_deg,
           PIDController hold_pid)
    : rotation_sensor_(lift_rotation_sensor),
      motors_(lift_motors),
      claw_angler_(claw_angler),
      claw_clamp_(claw_clamp),
      raise_mv_(raise_mv),
      lower_mv_(lower_mv),
      score_mv_(score_mv),
      score_lift_deg_(score_lift_deg),
      slew_(RAISE_SLEW_RATE_MV_S, ease_rate_mv_s),
      hold_pid_(hold_pid),
      mode_(Mode::REST),
      raise_requested_(false),
      hold_angle_(0.0),
      last_pid_output_(0.0),
      descent_start_ms_(0),
      last_update_ms_(0) {}

double Lift::get_angle() const {
    return rotation_sensor_.get_position() / 100.0;  // centidegrees -> degrees
}

bool Lift::is_lowering() const {
    return mode_ == Mode::LOWERING;
}

void Lift::raise() {
    raise_requested_ = true;
}

void Lift::move_down() {
    // Open the claw on the way down.
    claw_clamp_.set_state(true);
    claw_angler_.set_state(false);
    mode_ = Mode::LOWERING;
    descent_start_ms_ = pros::millis();
}

void Lift::score() {
    claw_clamp_.set_state(false);
    mode_ = Mode::SCORING;
    descent_start_ms_ = pros::millis();
}

void Lift::stop() {
    hold_angle_ = get_angle();
    hold_pid_.reset();
    mode_ = Mode::HOLD;
}

const char* Lift::mode_name(Mode m) {
    switch (m) {
        case Mode::HOLD:     return "HOLD";
        case Mode::RAISING:  return "RAISING";
        case Mode::LOWERING: return "LOWERING";
        case Mode::SCORING:  return "SCORING";
        case Mode::REST:     return "REST";
    }
    return "?";
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
        claw_clamp_.set_state(false);
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
            // Closed-loop hold against gravity / band imbalance. The derivative
            // is fed the sensor's hardware velocity (centidegrees/sec -> deg/sec)
            // rather than a numeric difference of the quantized angle, which
            // otherwise makes the command spike/toggle as the sensor steps.
            target_mv = hold_pid_.calculate(hold_angle_, get_angle(),
                                             rotation_sensor_.get_velocity() / 100.0);
            last_pid_output_ = target_mv;
            break;

        case Mode::LOWERING:
            if (descent_stalled(now)) {
                mode_ = Mode::REST;          // settled on the hard stop
                claw_clamp_.set_state(true);
                claw_angler_.set_state(false);
                target_mv = 0.0;
            } else {
                target_mv = -lower_mv_;      // eased in by the slew limiter
            }
            break;

        case Mode::SCORING:
            if (descent_stalled(now)) {
                // Met resistance: release pin/cup, then lift a few inches and
                // hold there (the HOLD PID drives up to the raised target).
                claw_clamp_.set_state(true);
                hold_angle_ = get_angle() + score_lift_deg_;
                hold_pid_.reset();
                mode_ = Mode::HOLD;
                target_mv = hold_pid_.calculate(hold_angle_, get_angle(),
                                                rotation_sensor_.get_velocity() / 100.0);
            } else {
                target_mv = -score_mv_;      // gentler eased descent onto the target
            }
            break;

        case Mode::REST:
            claw_clamp_.set_state(true);
            claw_angler_.set_state(false);
            target_mv = 0.0;
            break;
    }

    // Closed-loop HOLD drives the motor directly. Feeding the position PID
    // through the slew limiter adds phase lag inside the loop, which makes the
    // hold oscillate. Keep the slew's state synced to the applied voltage so
    // that when we hand back to an open-loop mode (RAISING / LOWERING / SCORING)
    // it ramps from the true current voltage instead of a stale one.
    double output_mv;
    if (mode_ == Mode::HOLD) {
        output_mv = target_mv;
        slew_.reset(output_mv);
    } else {
        output_mv = slew_.calculate(target_mv, dt);
    }
    pros::lcd::print(1, "output mv:%.0f", output_mv);

    motors_.move_voltage(static_cast<int>(output_mv));

    // Compact one-line lift telemetry: target angle, actual angle, PID output.
    pros::lcd::print(2, "Lift tgt:%.1f act:%.1f pid:%.0f",
                     hold_angle_, get_angle(), last_pid_output_);
    pros::lcd::print(3, "Lift mode:%s", mode_name(mode_));

    // Consume the raise request; the driver must re-assert it each loop.
    raise_requested_ = false;
}
