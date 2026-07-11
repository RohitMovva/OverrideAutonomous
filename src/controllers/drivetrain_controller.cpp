#include "controllers/drivetrain_controller.hpp"
#include "utilities/logger.hpp"
#include <algorithm>

DrivetrainController::DrivetrainController(
    double kS, double kV, double kA,
    double kP, double kI, double kD)
    : kS(kS), kV(kV), kA(kA)
    , kP(kP), kI(kI), kD(kD)
    , prevLeftVelocityError(0.0)
    , prevRightVelocityError(0.0)
    , leftIntegralError(0.0)
    , rightIntegralError(0.0)
    , prevTime(0)
{
}

DrivetrainController::MotorVoltages DrivetrainController::calculateVoltages(
    double leftVelocitySetpoint,
    double rightVelocitySetpoint,
    double leftVelocityActual,
    double rightVelocityActual,
    double leftAcceleration,
    double rightAcceleration)
{
    // Get current time and calculate delta
    uint32_t currentTime = pros::millis();
    double dt = Units::millisecondsToSeconds(currentTime - prevTime);
    if (dt <= 0) dt = Units::millisecondsToSeconds(Config::DT);

    // Calculate errors in ticks/sec
    double leftVelocityError = leftVelocitySetpoint - leftVelocityActual;
    double rightVelocityError = rightVelocitySetpoint - rightVelocityActual;

    // Update integral terms (with anti-windup)
    const double integralLimit = 5.0;
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

    // Update previous values
    prevLeftVelocityError = leftVelocityError;
    prevRightVelocityError = rightVelocityError;
    prevTime = currentTime;

    // Desaturate while preserving curvature
    double maxMagnitude = std::max(std::abs(leftVoltage), std::abs(rightVoltage));
    if (maxMagnitude > MOTOR_VOLTAGE_LIMIT) {
        double scale = MOTOR_VOLTAGE_LIMIT / maxMagnitude;
        leftVoltage *= scale;
        rightVoltage *= scale;
    }

    return {static_cast<int>(leftVoltage), static_cast<int>(rightVoltage)};
}

double DrivetrainController::calculateFeedforward(double velocityTicks, double accelerationTicks) {
    // Logger::getInstance()->log("Velocity: %f, Acceleration: %f", velocityTicks, accelerationTicks);
    return std::copysign(kS, velocityTicks) + 
           kV * velocityTicks + 
           kA * accelerationTicks;
}

double DrivetrainController::calculateFeedback(double error, double integral, double derivative) {
    return kP * error + kI * integral + kD * derivative;
}
