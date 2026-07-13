#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "i_localization.hpp"
#include "api.h"
#include "filters/kalman_2d.hpp"
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
 * @brief Odometry implementation of localization using wheel encoders and IMU
 *
 * Tracks robot position using advanced filtering techniques including Kalman filtering
 * and sensor fusion. Handles sensor validation, noise reduction, and uncertainty estimates.
 */
class Odometry : public ILocalization
{
private:
    // Chassis measurements
    double track_width;
    double odom_wheel_offset;

    // Hardware references
    pros::MotorGroup &leftDrive;
    pros::MotorGroup &rightDrive;
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
    // prevLeftPos / prevRightPos / prevLateralPos / prevImuRotation are ABSOLUTE sensor
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
    double prevLateralPos;             ///< Absolute lateral encoder position, inches
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
    double latTicksToInches(int ticks);
    double degreesToRadians(double degrees);
    /// Mean of the finite entries only. A disconnected motor reports PROS_ERR_F
    /// (infinity); averaging that in would make the mean infinite, the delta NaN, and the
    /// pose NaN forever after.
    double getAveragePosition(const std::vector<double> &positions);
    /// Absolute per-motor position in inches. No longer folds in a previous offset --
    /// that only existed to undo the per-tick tare.
    std::vector<double> getMotorPositionsInches(const std::vector<double> &motorPositions);
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
     * @param lateral Lateral tracking encoder
     * @param imuSensor IMU sensor
     * @param chassis_track_width Distance between left and right wheels
     * @param lateral_wheel_offset Distance from tracking center to lateral wheel
     * @param enable_heading_filter Enable heading filter
     * @param enable_velocity_filters Enable velocity filters
     */
    Odometry(pros::MotorGroup &left, pros::MotorGroup &right,
             pros::Rotation &lateral, pros::Imu &imuSensor,
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