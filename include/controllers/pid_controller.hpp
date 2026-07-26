#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include "api.h"
#include <cmath>

/**
 * @brief General-purpose PID controller implementation
 * 
 * Provides proportional, integral, and derivative control for any process
 * variable. Includes integral windup protection, output clamping, and
 * optional derivative filtering for robust performance.
 */
class PIDController {
public:
    /**
     * @brief Construct a new PID Controller
     * 
     * @param kP Proportional gain
     * @param kI Integral gain
     * @param kD Derivative gain
     * @param minOutput Minimum output value (default: -127)
     * @param maxOutput Maximum output value (default: 127)
     * @param integralLimit Maximum integral accumulation (default: 50.0)
     * @param derivativeFilterAlpha Low-pass smoothing factor for the derivative
     *        term, in (0, 1]. 1.0 = no filtering (raw derivative); smaller values
     *        smooth harder. Use this to tame derivative noise from quantized /
     *        noisy sensors (default: 1.0, i.e. disabled for backward compat).
     * @param integralActiveBand Integral separation: the integral only
     *        accumulates while |error| <= this band, so it trims steady-state
     *        error near the target but cannot wind up during a large-error
     *        transient (a long slew, a stiction stall) and sling past the goal.
     *        Default: effectively infinite (integrate always), for back-compat.
     */
    PIDController(double kP, double kI, double kD,
                  double minOutput = -127.0, double maxOutput = 127.0,
                  double integralLimit = 50.0,
                  double derivativeFilterAlpha = 1.0,
                  double integralActiveBand = 1.0e9);

    /**
     * @brief Calculate PID output based on current error
     * 
     * @param setpoint Desired target value
     * @param processVariable Current measured value
     * @return double Control output clamped to output limits
     */
    double calculate(double setpoint, double processVariable);

    /**
     * @brief Calculate PID output using a measured rate for the derivative
     *
     * Derivative-on-measurement: instead of numerically differentiating the
     * (often quantized / stepwise) process variable — which spikes every time
     * the sensor updates — the derivative term is driven by an externally
     * supplied rate, e.g. a hardware velocity. Units of @p measuredRate must be
     * (process-variable units) per second; kD is then "output per (unit/sec)".
     *
     * A @p feedforward term (e.g. gravity holding voltage) may be supplied; it is
     * added to the output AND folded into the anti-windup saturation check, so the
     * integrator sees the true, post-feedforward command when deciding whether it
     * is railed. Without this, a feedforward added by the caller *after* the PID
     * hides saturation from the anti-windup and lets the integral wind up.
     *
     * @param setpoint Desired target value
     * @param processVariable Current measured value
     * @param measuredRate Rate of change of the process variable (units/sec)
     * @param feedforward Extra command added to the output before clamping and
     *        accounted for in the anti-windup check (default: 0.0, units = output)
     * @return double Control output clamped to output limits
     */
    double calculate(double setpoint, double processVariable, double measuredRate,
                     double feedforward = 0.0);

    /**
     * @brief Calculate PID output with explicit error input
     *
     * @param error Direct error value (setpoint - processVariable)
     * @return double Control output clamped to output limits
     */
    double calculate(double error);

    /**
     * @brief Reset controller state
     * 
     * Clears integral accumulation and derivative history.
     * Call when changing setpoints or after extended periods of inactivity.
     */
    void reset();

    /**
     * @brief Set new PID gains
     * 
     * @param kP New proportional gain
     * @param kI New integral gain
     * @param kD New derivative gain
     */
    void setGains(double kP, double kI, double kD);

    /**
     * @brief Set output limits
     * 
     * @param minOutput New minimum output value
     * @param maxOutput New maximum output value
     */
    void setOutputLimits(double minOutput, double maxOutput);

    /**
     * @brief Set integral windup limit
     * 
     * @param integralLimit Maximum absolute value for integral accumulation
     */
    void setIntegralLimit(double integralLimit);

    /**
     * @brief Set the derivative low-pass smoothing factor
     *
     * @param alpha Smoothing factor in (0, 1]. 1.0 disables filtering (raw
     *        derivative); smaller values smooth the derivative term harder.
     */
    void setDerivativeFilter(double alpha);

    /**
     * @brief Get current integral term value
     * 
     * @return double Current integral accumulation
     */
    double getIntegral() const { return integralTerm; }

    /**
     * @brief Get last calculated error
     * 
     * @return double Most recent error value
     */
    double getError() const { return lastError; }

    /**
     * @brief Get last calculated output
     * 
     * @return double Most recent control output
     */
    double getOutput() const { return lastOutput; }

private:
    // PID gains
    double kP;  ///< Proportional gain
    double kI;  ///< Integral gain
    double kD;  ///< Derivative gain

    // Output limits
    double minOutput;  ///< Minimum output value
    double maxOutput;  ///< Maximum output value

    // State variables
    double integralTerm;     ///< Accumulated integral error
    double lastError;        ///< Previous error for derivative calculation
    double lastOutput;       ///< Previous output value
    uint32_t lastTime;       ///< Previous calculation timestamp
    bool firstRun;           ///< Flag for first calculation

    // Integral windup protection
    double integralLimit;    ///< Maximum integral accumulation
    double integralActiveBand; ///< Integrate only while |error| <= this (integral separation)

    // Derivative filtering
    double derivativeFilterAlpha;  ///< Low-pass smoothing factor in (0, 1]; 1.0 = off
    double filteredDerivative;     ///< Low-pass filtered raw derivative (error/sec)

    // Timing constants
    static constexpr double MIN_DELTA_TIME = 0.001;  ///< Minimum time step (seconds)

    /**
     * @brief Combine P/I/D into a clamped output, with conditional-integration
     *        anti-windup, logging, and state bookkeeping.
     *
     * Shared by every calculate() overload once the derivative term has been
     * computed. Accumulates the integral only when doing so would not drive the
     * output further into saturation, so the integral cannot wind up while the
     * output is railed.
     *
     * @param error Current error (setpoint - processVariable)
     * @param derivative Fully-formed derivative term (already scaled by kD)
     * @param feedforward Extra command folded into the output and the anti-windup
     *        saturation check (units = output)
     * @param dt Time step used for integration (seconds)
     * @param currentTime Timestamp of this calculation (ms) for state bookkeeping
     * @return double Control output clamped to output limits
     */
    double composeOutput(double error, double derivative, double feedforward,
                         double dt, uint32_t currentTime);

    /**
     * @brief Clamp value to specified range
     * 
     * @param value Input value to clamp
     * @param min Minimum allowed value
     * @param max Maximum allowed value
     * @return double Clamped value
     */
    double clamp(double value, double min, double max);

    /**
     * @brief Convert milliseconds to seconds
     * 
     * @param timeMs Time in milliseconds
     * @return double Time in seconds
     */
    double msToSeconds(uint32_t timeMs);
};

#endif // PID_CONTROLLER_H