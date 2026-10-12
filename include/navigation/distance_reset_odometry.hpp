#ifndef DISTANCE_RESET_ODOMETRY_H
#define DISTANCE_RESET_ODOMETRY_H

#include "odometry.hpp"
#include "api.h"
#include <array>
#include <cstddef>
#include <cstdint>

/**
 * @brief Extended odometry with distance sensor resets
 * 
 * Adds functionality to reset robot position using distance sensors based on field walls, has to be triggered manually
 */
class DistanceResetOdometry : public Odometry
{
public:
    /**
     * @brief Structure to hold distance sensor and its offset
     *
     * Offsets are measured relative to the sensor's own beam, not the robot's axes,
     * from the tracking centre to the sensor face, in inches:
     *  - offsetX: along the beam. Positive when the face is ahead of the centre in the
     *    beam's direction, which is the normal mounting (face on the outside of the robot).
     *  - offsetY: across the beam, positive to the RIGHT of the beam when looking along it.
     *    For the front sensor, -4.5 means 4.5 in to the robot's left.
     */
    struct DistanceSensor
    {
        pros::Distance* sensor;
        double offsetX; ///< Along the beam, tracking centre to sensor face (in)
        double offsetY; ///< Across the beam, positive to the beam's right (in)

        DistanceSensor(pros::Distance* s = nullptr, double oX = 0.0, double oY = 0.0) 
            : sensor(s), offsetX(oX), offsetY(oY) {}
    };

    /**
     * @brief Structure to organize all distance sensors
     */
    struct DistanceSensors
    {
        DistanceSensor front;  // Y-offset from center
        DistanceSensor back;   // Y-offset from center  
        DistanceSensor left;   // X-offset from center
        DistanceSensor right;  // X-offset from center
        
        DistanceSensors() = default;
        DistanceSensors(DistanceSensor f, DistanceSensor b, DistanceSensor l, DistanceSensor r)
            : front(f), back(b), left(l), right(r) {}
    };

    /// Outcome of one reset attempt
    enum class ResetResult
    {
        Applied,  ///< Pose corrected
        Moving,   ///< Refused only because the robot was moving; worth retrying once still
        Rejected, ///< Refused for anything else (no sensor, noisy window, beam angle, jump)
    };

private:
    /// Ring buffer of one sensor's recent valid readings, filled every update() tick so a
    /// reset can use the median of the last DISTANCE_RESET_WINDOW_MS instead of one sample.
    struct ReadingHistory
    {
        static constexpr std::size_t CAPACITY = 32; ///< > DISTANCE_RESET_WINDOW_MS / DT
        struct Sample
        {
            uint32_t t_ms;
            double mm;
        };
        std::array<Sample, CAPACITY> samples{};
        std::size_t next = 0;
        std::size_t count = 0;

        void push(uint32_t t_ms, double mm);
    };

    // Distance sensors
    DistanceSensors sensors;
    ReadingHistory frontHistory, backHistory, leftHistory, rightHistory;

    /// Record the current reading of one sensor, skipping missing sensors and no-object
    /// reads. Returns the raw reading in mm, or -1 when there is no sensor.
    static int32_t sample(const DistanceSensor& sensor, ReadingHistory& history, uint32_t now_ms);

    /**
     * @brief Reset position for any sensor
     * 
     * @param sensor The sensor to reset
     * @param history That sensor's recent readings
     * @param sensor_angle Beam direction relative to the robot's heading, degrees CCW
     * @param checkMotion Apply the DISTANCE_RESET_MAX_MOTION_ERROR gate. False when the
     *        caller has already waited for isStill().
     */
    ResetResult reset(const DistanceSensor& sensor, const ReadingHistory& history, double sensor_angle,
                      bool checkMotion);

    enum class Side
    {
        Front,
        Back,
        Left,
        Right,
    };

    /// The sensor, its history and its beam angle (degrees CCW from the heading)
    struct SideInfo
    {
        const DistanceSensor& sensor;
        const ReadingHistory& history;
        double angle_deg;
    };
    SideInfo side(Side which) const;

    /**
     * @brief Position error (in) the current motion would put into a reset from this sensor
     *
     * Over half the reading window: the speed toward/away from the beam's wall, plus the
     * turn rate times d(wall distance)/d(heading) for this sensor at this reading.
     * Wheel-measured motion only: a robot sliding sideways against something is not seen.
     */
    double motionError(const DistanceSensor& sensor, double reading_in, double sensor_angle,
                       double& rangeRate, double& angularRate) const;

public:
    /**
     * @brief Construct a new Distance Reset Odometry object
     * 
     * @param left Left drive motors
     * @param right Right drive motors
     * @param vertical Vertical (forward/back) tracking wheel rotation sensor
     * @param lateral Lateral (sideways) tracking wheel rotation sensor
     * @param imuSensor IMU sensor
     * @param distance_sensors Structure containing all distance sensors and their offsets
     * @param enable_heading_filter Enable heading filter
     * @param enable_velocity_filters Enable velocity filters
     */
    DistanceResetOdometry(pros::MotorGroup &left, pros::MotorGroup &right,
                         pros::Rotation &vertical, pros::Rotation &lateral, pros::Imu &imuSensor,
                         const DistanceSensors& distance_sensors,
                         bool enable_heading_filter = false,
                         bool enable_velocity_filters = false);

    /**
     * @brief Reset position using front distance sensor
     * Robot's local Y position is corrected based on distance to front wall
     */
    ResetResult resetFront(bool checkMotion = true);

    /**
     * @brief Reset position using back distance sensor
     * Robot's local Y position is corrected based on distance to back wall
     */
    ResetResult resetBack(bool checkMotion = true);

    /**
     * @brief Reset position using left distance sensor
     * Robot's local X position is corrected based on distance to left wall
     */
    ResetResult resetLeft(bool checkMotion = true);

    /**
     * @brief Reset position using right distance sensor
     * Robot's local X position is corrected based on distance to right wall
     */
    ResetResult resetRight(bool checkMotion = true);

    /// True when the wheels are inside DISTANCE_RESET_STILL_*: the robot has stopped,
    /// not just stopped closing on the wall. What a reset deferred to a hold waits for.
    bool isStill() const;

    /// Odometry update plus one reading from every reset sensor
    void update() override;

    LocalizationType getType() const override { return LocalizationType::DISTANCE_RESET_ODOMETRY; }
    std::string getTypeName() const override { return "DistanceResetOdometry"; }
};

#endif // DISTANCE_RESET_ODOMETRY_H