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
     */
    DrivetrainController(double kS, double kV, double kA,
                        double kP, double kI, double kD);

    double getKS() const { return kS; }
    double getKV() const { return kV; }
    double getKA() const { return kA; }
    double getKP() const { return kP; }
    double getKI() const { return kI; }
    double getKD() const { return kD; }

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

private:
    // Feedforward constants
    const double kS;  ///< Static friction compensation (motor units)
    const double kV;  ///< Velocity feedforward gain
    const double kA;  ///< Acceleration feedforward gain
    
    // PID constants
    const double kP;  ///< Proportional gain
    const double kI;  ///< Integral gain
    const double kD;  ///< Derivative gain

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
};

#endif // DRIVETRAIN_CONTROLLER_H