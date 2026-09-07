#ifndef DRIVETRAIN_CONTROLLER_HPP
#define DRIVETRAIN_CONTROLLER_HPP

#include "api.h"
#include "config.hpp"
#include "utilities/math/units.hpp"
#include <cmath>

/**
 * @brief Controller for differential drivetrain motor outputs
 * 
 * Implements feedforward and PID feedback control for velocity control
 * of a differential drivetrain. Handles conversion between real-world units
 * and motor units, and provides voltage outputs suitable for V5 motors.
 */
class DrivetrainController {
public:
    /**
     * @brief Struct to hold motor voltage outputs
     */
    struct MotorVoltages {
        int left;   ///< Left motor voltage (-127 to 127)
        int right;  ///< Right motor voltage (-127 to 127)
    };

    /**
     * @brief Construct a new Drivetrain Controller
     * 
     * @param kS Static friction compensation (motor units)
     * @param kV Velocity feedforward gain (motor units)/(inches/sec)
     * @param kA Acceleration feedforward gain (motor units)/(inches/sec²)
     * @param kP Proportional gain for PID
     * @param kI Integral gain for PID
     * @param kD Derivative gain for PID
     * @param kSAngular Angular static friction compensation (motor units)
     * @param kVAngular Angular velocity feedforward gain (motor units)/(rad/sec)
     * @param kADecel Acceleration feedforward while braking; defaults to kA (symmetric)
     */
    DrivetrainController(double kS, double kV, double kA,
                        double kP, double kI, double kD,
                        double kSAngular = 0.0, double kVAngular = 0.0,
                        double kADecel = -1.0);

    double getKS() const { return kS; }
    double getKV() const { return kV; }
    double getKA() const { return kA; }
    double getKADecel() const { return kADecel; }
    double getKP() const { return kP; }
    double getKI() const { return kI; }
    double getKD() const { return kD; }
    double getKSAngular() const { return kSAngular; }
    double getKVAngular() const { return kVAngular; }

    /**
     * @brief Calculate motor voltages based on desired velocities and accelerations
     * 
     * @param leftVelocitySetpoint Desired left velocity (inches/sec)
     * @param rightVelocitySetpoint Desired right velocity (inches/sec)
     * @param leftVelocityActual Current left velocity (inches/sec)
     * @param rightVelocityActual Current right velocity (inches/sec)
     * @param leftAcceleration Desired left acceleration (inches/sec²)
     * @param rightAcceleration Desired right acceleration (inches/sec²)
     * @return MotorVoltages Voltage commands for left and right motors (-127 to 127)
     */
    MotorVoltages calculateVoltages(double leftVelocitySetpoint,
                                  double rightVelocitySetpoint,
                                  double leftVelocityActual,
                                  double rightVelocityActual,
                                  double leftAcceleration,
                                  double rightAcceleration);

    /**
     * @brief Clear the PID state and re-base the dt clock. Call once before every run.
     *
     * The controller is constructed in initialize() but first USED seconds later, when
     * autonomous() starts. prevTime is constructed to 0, so without this the first
     * calculateVoltages() call measures dt as the whole time since program start --
     * 2.294 s in 08012026-3, 2.110 s in 08012026-4. That weights the opening tick's
     * integral contribution by ~230x, by an amount that varies with however long
     * initialize() happened to take. Harmless while kI was 0; a fresh run-to-run
     * variable the moment kI is not.
     *
     * The integrals and previous errors also survive across trajectory segments and
     * across a re-run without a reboot, so a segment would inherit whatever disturbance
     * the last one was still fighting.
     */
    void reset();

private:
    // Feedforward constants
    const double kS;       ///< Static friction compensation (motor units)
    const double kV;       ///< Velocity feedforward gain
    const double kA;       ///< Acceleration feedforward gain (accelerating)
    const double kADecel;  ///< Acceleration feedforward gain (braking) -- see Config::KA_DECEL
    
    // PID constants
    const double kP;  ///< Proportional gain
    const double kI;  ///< Integral gain
    const double kD;  ///< Derivative gain

    // Angular (turning-scrub) feedforward constants
    const double kSAngular;  ///< Angular static friction compensation
    const double kVAngular;  ///< Angular velocity feedforward gain

    // State tracking
    double prevLeftVelocityError;
    double prevRightVelocityError;
    double leftIntegralError;
    double rightIntegralError;
    uint32_t prevTime;

    // Robot constants
    static constexpr int MOTOR_VOLTAGE_LIMIT = 127;      ///< V5 motor command range (±127)

    /**
     * @brief Calculate feedforward component of motor output
     *
     * Uses kA when speeding up and kADecel when braking. Friction opposes the motors on
     * the way up and helps them on the way down, so the same commanded |acceleration|
     * costs less voltage to brake than to accelerate; charging both at kA over-drives
     * every braking ramp and stops the robot short.
     *
     * @param velocityTicks Velocity in encoder ticks/sec
     * @param accelerationTicks Acceleration in encoder ticks/sec²
     * @return double Feedforward term in motor units
     */
    double calculateFeedforward(double velocityTicks, double accelerationTicks);

    /**
     * @brief Calculate feedback component of motor output
     * 
     * @param error Velocity error in encoder ticks/sec
     * @param integral Accumulated error term
     * @param derivative Rate of change of error
     * @return double Feedback term in motor units
     */
    double calculateFeedback(double error, double integral, double derivative);

    /**
     * @brief Feedforward for the yaw-resisting scrub torque that appears only
     *        while turning.
     *
     * Returned antisymmetrically: ADD to the outer wheel, SUBTRACT from the inner,
     * so it moves the wheel differential (and therefore rotation) without changing
     * forward speed. KS/KV/KA cannot represent this because they are fitted to
     * straight driving and applied to each wheel independently.
     *
     * @param angularVelocitySetpoint Commanded chassis rotation (rad/sec), positive CCW
     * @return double Angular feedforward term in motor units
     */
    double calculateAngularFeedforward(double angularVelocitySetpoint);
};

#endif // DRIVETRAIN_CONTROLLER_H