#include "controllers/drivetrain_controller.hpp"
#include "utilities/logger.hpp"
#include <algorithm>

DrivetrainController::DrivetrainController(
    double kS, double kV, double kA,
    double kP, double kI, double kD,
    double kSAngular, double kVAngular,
    double kADecel)
    : kS(kS), kV(kV), kA(kA)
    // A negative kADecel means "not supplied" -> fall back to kA, i.e. the old symmetric
    // behaviour. Callers that predate the braking asymmetry keep working unchanged.
    , kADecel(kADecel < 0.0 ? kA : kADecel)
    , kP(kP), kI(kI), kD(kD)
    , kSAngular(kSAngular), kVAngular(kVAngular)
    , prevLeftVelocityError(0.0)
    , prevRightVelocityError(0.0)
    , leftIntegralError(0.0)
    , rightIntegralError(0.0)
    , prevTime(0)
{
}

void DrivetrainController::reset()
{
    prevLeftVelocityError = 0.0;
    prevRightVelocityError = 0.0;
    leftIntegralError = 0.0;
    rightIntegralError = 0.0;
    prevTime = pros::millis();
}

DrivetrainController::MotorVoltages DrivetrainController::calculateVoltages(
    double leftVelocitySetpoint,
    double rightVelocitySetpoint,
    double leftVelocityActual,
    double rightVelocityActual,
    double leftAcceleration,
    double rightAcceleration)
{
    // Get current time and calculate delta.
    //
    // The upper bound is what makes a missed reset() survivable rather than silent: this
    // runs on a Config::DT (10 ms) tick, so anything past 0.1 s is a stale clock or a
    // stalled loop, not a real interval. Integrating one scales that tick's contribution
    // by up to 230x and can slam the integrator into its clamp before the robot has
    // moved. Same bound Odometry::update() uses on its own delta, for the same reason.
    uint32_t currentTime = pros::millis();
    double dt = Units::millisecondsToSeconds(currentTime - prevTime);
    if (dt <= 0 || dt > 0.1) dt = Units::millisecondsToSeconds(Config::DT);

    // Calculate errors in ticks/sec
    double leftVelocityError = leftVelocitySetpoint - leftVelocityActual;
    double rightVelocityError = rightVelocitySetpoint - rightVelocityActual;

    // Update integral terms. The clamp on its own is not anti-windup: while a wheel
    // is pinned at the rail it cannot act on any more error, so integrating it only
    // banks a correction that has to be unwound before the controller responds
    // again. Measured on arc_R24, the left integrator sat on this clamp for 760 ms
    // of a 1.64 s run (46% of the route). Integrate tentatively here and roll the
    // update back below on any wheel that turns out to be saturated the way its own
    // error is pushing -- conditional integration, decided on this tick's voltages
    // rather than the previous tick's.
    const double integralLimit = 5.0;
    const double priorLeftIntegral = leftIntegralError;
    const double priorRightIntegral = rightIntegralError;
    leftIntegralError = std::clamp(
        leftIntegralError + leftVelocityError * dt,
        -integralLimit, integralLimit);
    rightIntegralError = std::clamp(
        rightIntegralError + rightVelocityError * dt,
        -integralLimit, integralLimit);
    
    // Calculate derivative terms
    double leftDerivative = (leftVelocityError - prevLeftVelocityError) / dt;
    double rightDerivative = (rightVelocityError - prevRightVelocityError) / dt;
    
    // Calculate feedforward
    double leftFeedforward = calculateFeedforward(
        leftVelocitySetpoint, 
        leftAcceleration);
    double rightFeedforward = calculateFeedforward(
        rightVelocitySetpoint, 
        rightAcceleration);
    
    // Calculate feedback
    double leftFeedback = calculateFeedback(
        leftVelocityError, 
        leftIntegralError, 
        leftDerivative);
    double rightFeedback = calculateFeedback(
        rightVelocityError, 
        rightIntegralError,
        rightDerivative);

    // Logger::getInstance()->log("Left Feedforward: %f, Right Feedforward: %f", leftFeedforward, rightFeedforward);
    // Logger::getInstance()->log("Left Feedback: %f, Right Feedback: %f", leftFeedback, rightFeedback);

    // Combine feedforward and feedback
    double leftVoltage = leftFeedforward + leftFeedback;
    double rightVoltage = rightFeedforward + rightFeedback;
    // Logger::getInstance()->log("Left Voltage: %f, Right Voltage: %f", leftVoltage, rightVoltage);

    // Turning scrub resists yaw: it drags the outer wheel back and pushes the inner
    // one forward, so the delivered differential falls short of the commanded one.
    // Correct it antisymmetrically -- the term carries the sign of w, so adding it to
    // the right wheel and subtracting it from the left always reinforces the commanded
    // turn, whichever way the robot is going, and leaves forward speed untouched.
    double angularVelocitySetpoint =
        (rightVelocitySetpoint - leftVelocitySetpoint) / Config::WHEEL_BASE_WIDTH;
    double angularFeedforward = calculateAngularFeedforward(angularVelocitySetpoint);
    leftVoltage -= angularFeedforward;
    rightVoltage += angularFeedforward;

    // Conditional integration: a wheel whose demand does not fit on the rail, and
    // whose error points the same way as that demand, cannot use any more integral,
    // so it does not get to bank any. Judged on the pre-desaturation demand, which
    // is what "the controller asked for more than 127" actually means.
    if (std::abs(leftVoltage) > MOTOR_VOLTAGE_LIMIT &&
        leftVelocityError * leftVoltage > 0.0) {
        leftIntegralError = priorLeftIntegral;
    }
    if (std::abs(rightVoltage) > MOTOR_VOLTAGE_LIMIT &&
        rightVelocityError * rightVoltage > 0.0) {
        rightIntegralError = priorRightIntegral;
    }

    // Update previous values
    prevLeftVelocityError = leftVelocityError;
    prevRightVelocityError = rightVelocityError;
    prevTime = currentTime;

    // Desaturate by giving up forward speed before giving up the turn.
    //
    // Scaling both wheels by one factor, which is what this did before, holds the
    // voltage RATIO but not the velocity differential -- kS is an offset, so equal
    // scaling shrinks the difference the wheels actually deliver. Measured, the
    // robot delivered only 76-81% of its commanded rotation on every arc. Splitting
    // the pair into common mode (forward) and differential (turn) and spending the
    // headroom on the differential first keeps the commanded curvature: arriving
    // late on the right arc beats arriving on time on a wider one.
    double common = 0.5 * (leftVoltage + rightVoltage);
    double differential = 0.5 * (rightVoltage - leftVoltage);
    if (std::abs(differential) > MOTOR_VOLTAGE_LIMIT) {
        differential = std::copysign(static_cast<double>(MOTOR_VOLTAGE_LIMIT), differential);
    }
    const double headroom = MOTOR_VOLTAGE_LIMIT - std::abs(differential);
    common = std::clamp(common, -headroom, headroom);
    leftVoltage = common - differential;
    rightVoltage = common + differential;

    return {static_cast<int>(leftVoltage), static_cast<int>(rightVoltage)};
}

double DrivetrainController::calculateFeedforward(double velocityTicks, double accelerationTicks) {
    // Braking is when the commanded acceleration OPPOSES the commanded velocity. Testing
    // the product rather than the sign of the acceleration is what makes this correct when
    // driving backwards too: at v < 0 it is a POSITIVE acceleration that slows the robot,
    // and that is still braking. Reverse nodes make that case routine, not exotic.
    //
    // Friction fights the motors on the way up and helps them on the way down, so braking
    // costs less voltage per unit of |acceleration| than accelerating does. Charging both
    // at kA over-drives every braking ramp: the robot decelerated ~17% harder than
    // commanded and stopped short of the goal.
    const bool braking = velocityTicks * accelerationTicks < 0.0;
    const double kAEffective = braking ? kADecel : kA;

    return std::copysign(kS, velocityTicks) +
           kV * velocityTicks +
           kAEffective * accelerationTicks;
}

double DrivetrainController::calculateFeedback(double error, double integral, double derivative) {
    return kP * error + kI * integral + kD * derivative;
}

double DrivetrainController::calculateAngularFeedforward(double angularVelocitySetpoint) {
    // Straight-line driving has no scrub torque to cancel, and the copysign term
    // below would dither on the sign of a near-zero number.
    if (std::abs(angularVelocitySetpoint) < Config::ANGULAR_FF_DEADBAND) {
        return 0.0;
    }
    return std::copysign(kSAngular, angularVelocitySetpoint) +
           kVAngular * angularVelocitySetpoint;
}
