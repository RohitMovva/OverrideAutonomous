#include "controllers/pid_controller.hpp"
#include "utilities/math/units.hpp"
#include "config.hpp"
#include "utilities/logger.hpp"
#include <algorithm>

PIDController::PIDController(double kP, double kI, double kD,
                           double minOutput, double maxOutput,
                           double integralLimit,
                           double derivativeFilterAlpha,
                           double integralActiveBand)
    : kP(kP), kI(kI), kD(kD)
    , minOutput(minOutput), maxOutput(maxOutput)
    , integralTerm(0.0)
    , lastError(0.0)
    , lastOutput(0.0)
    , lastTime(0)
    , firstRun(true)
    , integralLimit(integralLimit)
    , integralActiveBand(integralActiveBand)
    , derivativeFilterAlpha(derivativeFilterAlpha)
    , filteredDerivative(0.0)
{
    Logger::getInstance()->log("PID Controller initialized - kP: %f, kI: %f, kD: %f", kP, kI, kD);
}

double PIDController::calculate(double setpoint, double processVariable) {
    double error = setpoint - processVariable;
    return calculate(error);
}

double PIDController::calculate(double setpoint, double processVariable, double measuredRate,
                                double feedforward) {
    double error = setpoint - processVariable;

    uint32_t currentTime = pros::millis();
    double dt = Units::millisecondsToSeconds(currentTime - lastTime);

    bool isFirstRun = firstRun;
    if (firstRun || dt <= 0) {
        dt = Units::millisecondsToSeconds(Config::DT);
        firstRun = false;
    }

    // Derivative on measurement. For a (locally) constant setpoint,
    // d(error)/dt = -d(processVariable)/dt, so we drive the derivative with the
    // negated measured rate. This avoids differentiating a quantized position
    // signal, whose per-tick steps make a numeric derivative spike. Still
    // low-pass filtered to smooth any residual velocity noise. Skipped on the
    // first run after a reset (no valid history yet).
    double derivative = 0.0;
    if (!isFirstRun) {
        double errorRate = -measuredRate;
        filteredDerivative = derivativeFilterAlpha * errorRate
                           + (1.0 - derivativeFilterAlpha) * filteredDerivative;
        derivative = kD * filteredDerivative;
    }

    return composeOutput(error, derivative, feedforward, dt, currentTime);
}

double PIDController::calculate(double error) {
    // Get current time and calculate delta
    uint32_t currentTime = pros::millis();
    double dt = Units::millisecondsToSeconds(currentTime - lastTime);

    // Handle first run or invalid time step
    bool isFirstRun = firstRun;
    if (firstRun || dt <= 0) {
        dt = Units::millisecondsToSeconds(Config::DT);
        firstRun = false;
    }

    // Calculate derivative term. Skip it on the first run after a reset:
    // lastError is 0 there but the true error may be large, which would
    // otherwise produce a huge one-tick derivative "kick". The raw derivative
    // is low-pass filtered to suppress noise amplification from quantized /
    // noisy sensors (a high kD on a coarse sensor is a major source of jitter).
    double derivative = 0.0;
    if (!isFirstRun && dt > MIN_DELTA_TIME) {
        double rawDerivative = (error - lastError) / dt;
        filteredDerivative = derivativeFilterAlpha * rawDerivative
                           + (1.0 - derivativeFilterAlpha) * filteredDerivative;
        derivative = kD * filteredDerivative;
    }

    return composeOutput(error, derivative, 0.0, dt, currentTime);
}

double PIDController::composeOutput(double error, double derivative, double feedforward,
                                    double dt, uint32_t currentTime) {
    double proportional = kP * error;

    // Conditional-integration anti-windup: tentatively advance the integral,
    // but only commit it if the resulting output would not push further into
    // saturation. When railed in the direction the error is driving, we hold
    // the integral steady so it cannot wind up; it is still free to unwind (or
    // grow the other way) once the error reverses. The integral is also hard
    // clamped to +/-integralLimit as a backstop. The feedforward is included in
    // the saturation check so a caller-side feedforward (e.g. gravity holding
    // voltage) cannot mask saturation and let the integral wind up behind it.
    double candidateIntegral = clamp(integralTerm + error * dt, -integralLimit, integralLimit);
    double candidateOutput = proportional + kI * candidateIntegral + derivative + feedforward;
    bool worsensHighSaturation = candidateOutput > maxOutput && error > 0.0;
    bool worsensLowSaturation  = candidateOutput < minOutput && error < 0.0;
    // Integral separation: only accumulate near the target. A large error (a long
    // slew, a stiction stall) must NOT charge the integral, or it slings the
    // output past the goal once the mechanism finally moves. Steady-state trim
    // near the target still works because the error there is inside the band.
    bool withinActiveBand = std::abs(error) <= integralActiveBand;
    if (withinActiveBand && !worsensHighSaturation && !worsensLowSaturation) {
        integralTerm = candidateIntegral;
    }
    double integral = kI * integralTerm;

    double output = clamp(proportional + integral + derivative + feedforward, minOutput, maxOutput);

    // Log key values periodically (not every calculation)
    static uint32_t logCounter = 0;
    if (++logCounter % 50 == 0) {  // Log every 50 calculations
        Logger::getInstance()->log("PID Output: %f (P: %f, I: %f, D: %f)",
                                 output, proportional, integral, derivative);
    }

    // Update state
    lastError = error;
    lastOutput = output;
    lastTime = currentTime;

    return output;
}

void PIDController::reset() {
    integralTerm = 0.0;
    lastError = 0.0;
    lastOutput = 0.0;
    lastTime = 0;
    firstRun = true;
    filteredDerivative = 0.0;

    Logger::getInstance()->log("PID Controller reset");
}

void PIDController::setGains(double kP, double kI, double kD) {
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;
    
    Logger::getInstance()->log("PID gains updated - kP: %f, kI: %f, kD: %f", kP, kI, kD);
}

void PIDController::setOutputLimits(double minOutput, double maxOutput) {
    this->minOutput = minOutput;
    this->maxOutput = maxOutput;
    
    Logger::getInstance()->log("PID output limits updated - min: %f, max: %f", minOutput, maxOutput);
}

void PIDController::setIntegralLimit(double integralLimit) {
    this->integralLimit = integralLimit;
    
    // Clamp current integral term to new limit
    integralTerm = clamp(integralTerm, -integralLimit, integralLimit);
    
    Logger::getInstance()->log("PID integral limit updated: %f", integralLimit);
}

void PIDController::setDerivativeFilter(double alpha) {
    derivativeFilterAlpha = alpha;

    Logger::getInstance()->log("PID derivative filter alpha updated: %f", alpha);
}

double PIDController::clamp(double value, double min, double max) {
    return std::clamp(value, min, max);
}