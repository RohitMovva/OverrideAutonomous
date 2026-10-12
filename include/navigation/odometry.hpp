#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "i_localization.hpp"
#include "api.h"
#include "filters/kalman_2d.hpp"
#include "navigation/tracking_arc.hpp"
#include "filters/heading_filter.hpp"
#include "filters/velocity_observer.hpp"
#include "config.hpp"
#include <cmath>
#include <vector>
#include <numeric>
#include "utilities/logger.hpp"
#include "utilities/math/units.hpp"
#include "utilities/math/angle.hpp"

/**
 * @brief Odometry implementation of localization using tracking wheels and IMU
 *
 * Translation comes from a vertical and a lateral tracking wheel, heading from the IMU.
 * The drive motor encoders supply the per-side velocities and stand in for a tracking
 * wheel that stops reading.
 *
 * Tracks robot position using advanced filtering techniques including Kalman filtering
 * and sensor fusion. Handles sensor validation, noise reduction, and uncertainty estimates.
 */
class Odometry : public ILocalization
{
private:
    // Hardware references
    pros::MotorGroup &leftDrive;
    pros::MotorGroup &rightDrive;
    pros::Rotation &verticalEncoder;
    pros::Rotation &lateralEncoder;
    pros::Imu &imu;

    // Filters
    VelocityObserver leftVelocityObserver;
    VelocityObserver rightVelocityObserver;
    HeadingFilter headingFilter;

    // Voltage last applied to each side, in motor units (+/-127). Fed in by the
    // controller via setDriveVoltages(); drives the observers' prediction step.
    double lastLeftVoltage;
    double lastRightVoltage;

    // Filter toggles
    bool useHeadingFilter;
    bool useVelocityFilters;

    // State variables.
    //
    // prevLeftPos / prevRightPos / prevVerticalPos / prevLateralPos / prevImuRotation are ABSOLUTE sensor
    // readings from the previous tick, and every delta is taken against them. Nothing is
    // tared inside update().
    //
    // The old code tared the encoders and the IMU at the end of every tick and read the
    // "delta" straight off a freshly-zeroed sensor. tare_position_all() / tare_heading()
    // are asynchronous smart-port writes: any motion that occurs between issuing the tare
    // and the device applying it lands in neither tick and is silently lost. That biases
    // both distance and heading LOW by an amount proportional to speed -- an error no
    // downstream filter can recover, because the measurement was never taken.
    Pose currentPose;
    double lastUpdateTime;
    std::vector<double> prevLeftPos;   ///< Absolute per-motor position, inches
    std::vector<double> prevRightPos;  ///< Absolute per-motor position, inches
    double prevVerticalPos;            ///< Absolute vertical tracking wheel position, inches
    double prevLateralPos;             ///< Absolute lateral tracking wheel position, inches
    bool hasPrevVertical;              ///< prevVerticalPos is a real reading (sensor was connected)
    bool hasPrevLateral;               ///< prevLateralPos is a real reading (sensor was connected)
    bool verticalFallback;             ///< Last tick replaced the vertical wheel with the drive encoders
    bool lateralFallback;              ///< Last tick assumed no sideslip in place of the lateral wheel
    double verticalChangeTime;         ///< When the vertical wheel's reading last changed, seconds
    double lateralChangeTime;          ///< When the lateral wheel's reading last changed, seconds
    double prevImuRotation;            ///< Absolute IMU rotation, degrees (continuous, unwrapped)
    bool hasPrevSensorReadings;        ///< False until the first post-reset baseline is captured
    double prevTime;

    // Velocity and acceleration tracking
    Velocity leftVelocity;
    Velocity rightVelocity;
    Velocity leftAccel;
    Velocity rightAccel;

    // Pre-observer encoder velocities, kept for tuning/logging. Retained as state so the
    // debug getters can report them without re-running (and mutating) a filter.
    double leftVelocityRaw;
    double rightVelocityRaw;

    // Helper functions
    double ticksToInches(double ticks);
    /// Tracking wheel position in inches, or false when the sensor is unreadable.
    /// get_position() reports a missing sensor as PROS_ERR (INT32_MAX), which is a finite
    /// number, so std::isfinite cannot catch it.
    bool readTrackingWheel(pros::Rotation &encoder, double diameter, double &inches);
    double degreesToRadians(double degrees);
    /// Mean of the finite entries only. A disconnected motor reports PROS_ERR_F
    /// (infinity); averaging that in would make the mean infinite, the delta NaN, and the
    /// pose NaN forever after.
    double getAveragePosition(const std::vector<double> &positions);
    /// Absolute per-motor position in inches. No longer folds in a previous offset --
    /// that only existed to undo the per-tick tare.
    ///
    /// `trustedIndex` selects a single motor's encoder (-1 keeps every readable one).
    /// Selection happens HERE, at the read, so that prevLeftPos/prevRightPos and the
    /// reading they are differenced against always describe the same channels. Doing it
    /// later -- seeding the baseline from every channel but averaging one for the
    /// current position -- makes the first tick report the gap between the two
    /// conventions as motion.
    std::vector<double> getMotorPositionsInches(const std::vector<double> &motorPositions,
                                               int trustedIndex);
    /// Capture the current absolute sensor readings as the baseline for the next delta.
    void seedSensorBaseline();
    bool isValidEncoderReading(double newPos, double oldPos, double deltaTime);
    double validateAndFilterEncoders(const std::vector<double> &positions,
                                     const std::vector<double> &prevPositions,
                                     double deltaTime);

public:
    /**
     * @brief Debug information structure for tuning
     */
    struct DebugInfo
    {
        double leftVelocityRaw;
        double rightVelocityRaw;
        double leftVelocityFiltered;
        double rightVelocityFiltered;
        double headingRaw;
        double headingFiltered;
        std::pair<double, double> uncertainty;
    };

    /**
     * @brief Construct a new Odometry Localization object
     *
     * @param left Left drive motors
     * @param right Right drive motors
     * @param vertical Vertical (forward/back) tracking wheel rotation sensor
     * @param lateral Lateral (sideways) tracking wheel rotation sensor
     * @param imuSensor IMU sensor
     * @param enable_heading_filter Enable heading filter
     * @param enable_velocity_filters Enable velocity filters
     */
    Odometry(pros::MotorGroup &left, pros::MotorGroup &right,
             pros::Rotation &vertical, pros::Rotation &lateral, pros::Imu &imuSensor,
             bool enable_heading_filter = false,
             bool enable_velocity_filters = false);

    // Implement core interface
    void update() override;
    void reset() override;
    Pose getPose() const override;
    void setPose(const Pose &newPose) override;
    double getHeading() const override;
    double getX() const override;
    double getY() const override;
    bool isReliable() const override;
    Velocity getLeftVelocity() const override;
    Velocity getRightVelocity() const override;
    void setDriveVoltages(double leftVoltage, double rightVoltage) override;

    LocalizationType getType() const override { return LocalizationType::ODOMETRY; }
    std::string getTypeName() const override { return "Odometry"; }

    // Odometry-specific methods
    Velocity getLeftAcceleration() const;
    Velocity getRightAcceleration() const;
    double getAngularVelocity() const;
    double getHeadingUncertainty() const;
    std::pair<double, double> getFilteredVelocities() const;
    std::pair<double, double> getPositionUncertainty() const;
    void setFiltersEnabled(bool heading, bool velocity);
    void disableSensor(const std::string &sensor);
    DebugInfo getOdometryDebugInfo();

    // Debug interface implementation
    std::unordered_map<std::string, double> getDebugData() const override;
};

#endif // ODOMETRY_H