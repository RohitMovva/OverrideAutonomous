#include "navigation/distance_reset_odometry.hpp"
#include "utilities/math/angle.hpp"
#include <algorithm>
#include <cstdio>

DistanceResetOdometry::DistanceResetOdometry(pros::MotorGroup &left, pros::MotorGroup &right,
                                           pros::Rotation &vertical, pros::Rotation &lateral, pros::Imu &imuSensor,
                                           const DistanceSensors& distance_sensors,
                                           bool enable_heading_filter,
                                           bool enable_velocity_filters)
    : Odometry(left, right, vertical, lateral, imuSensor, enable_heading_filter, enable_velocity_filters),
      sensors(distance_sensors)
{
    static_assert(ReadingHistory::CAPACITY * Config::DT > Config::DISTANCE_RESET_WINDOW_MS,
                  "ReadingHistory too small for DISTANCE_RESET_WINDOW_MS");
}

void DistanceResetOdometry::ReadingHistory::push(uint32_t t_ms, double mm)
{
    samples[next] = {t_ms, mm};
    next = (next + 1) % CAPACITY;
    count = std::min(count + 1, CAPACITY);
}

int32_t DistanceResetOdometry::sample(const DistanceSensor& sensor, ReadingHistory& history, uint32_t now_ms)
{
    if (sensor.sensor == nullptr) {
        return -1;
    }
    // 9999 is "no object"; PROS_ERR (INT32_MAX) is a missing device. Neither is a wall.
    const int32_t mm = sensor.sensor->get();
    if (mm < 9999) {
        history.push(now_ms, mm);
    }
    return mm;
}

void DistanceResetOdometry::update()
{
    Odometry::update();
    const uint32_t now = pros::millis();
    const int32_t front = sample(sensors.front, frontHistory, now);
    const int32_t back = sample(sensors.back, backHistory, now);
    const int32_t left = sample(sensors.left, leftHistory, now);
    const int32_t right = sample(sensors.right, rightHistory, now);
    // Raw mm every tick, so the log shows what each wall reset would have read anywhere
    // on the path (tests/log_analysis/parser.py keys on this line). -1 = no sensor,
    // 9999 = no object, 2147483647 = device missing.
    Logger::getInstance()->log("DISTREAD front=%ld back=%ld left=%ld right=%ld",
                               (long)front, (long)back, (long)left, (long)right);
}

DistanceResetOdometry::SideInfo DistanceResetOdometry::side(Side which) const
{
    switch (which) {
    case Side::Front: return {sensors.front, frontHistory, 0.0};
    case Side::Back: return {sensors.back, backHistory, 180.0};
    case Side::Left: return {sensors.left, leftHistory, 90.0};
    case Side::Right:
    default: return {sensors.right, rightHistory, 270.0};
    }
}

double DistanceResetOdometry::motionError(const DistanceSensor& sensor, double reading_in,
                                          double sensor_angle, double& rangeRate,
                                          double& angularRate) const
{
    // Wheel velocities are in in/s; the heading filter's angular rate is off by default,
    // so the turn rate comes from the wheel difference.
    const double vl = getLeftVelocity().linear;
    const double vr = getRightVelocity().linear;
    const double theta = getPose().theta;
    const double beam = Angles::normalizeAngle(theta + sensor_angle * M_PI / 180.0);
    const double normal = std::lround(beam / (M_PI / 2)) * (M_PI / 2);
    const double offNormal = Angles::angleDifference(beam, normal);
    // Velocity is along the heading; only its component along the wall normal changes
    // the distance. Driving along the wall leaves it alone.
    rangeRate = 0.5 * (vl + vr) * std::cos(theta - normal);
    angularRate = (vr - vl) / Config::WHEEL_BASE_WIDTH;
    // d(wallToCenter)/d(heading), from wallToCenter in reset(); offNormal moves 1:1 with it
    const double lever = std::fabs(-(reading_in + sensor.offsetX) * std::sin(offNormal) +
                                   sensor.offsetY * std::cos(offNormal));
    const double lag_s = 0.5 * Config::DISTANCE_RESET_WINDOW_MS / 1000.0;
    return (std::fabs(rangeRate) + lever * std::fabs(angularRate)) * lag_s;
}

bool DistanceResetOdometry::isStill() const
{
    const double vl = getLeftVelocity().linear;
    const double vr = getRightVelocity().linear;
    return std::fabs(0.5 * (vl + vr)) <= Config::DISTANCE_RESET_STILL_LINEAR_SPEED &&
           std::fabs((vr - vl) / Config::WHEEL_BASE_WIDTH) <= Config::DISTANCE_RESET_STILL_ANGULAR_SPEED;
}

DistanceResetOdometry::ResetResult DistanceResetOdometry::reset(const DistanceSensor& sensor,
                                                                const ReadingHistory& history,
                                                                double sensor_angle,
                                                                bool checkMotion) {
    if (sensor.sensor == nullptr) {
        Logger::getInstance()->logWarning("Distance sensor not available for reset");
        return ResetResult::Rejected;
    }

    // Valid readings from the last DISTANCE_RESET_WINDOW_MS. update() ran earlier this
    // tick, so the newest one is current.
    const uint32_t now = pros::millis();
    std::array<double, ReadingHistory::CAPACITY> window{};
    std::size_t n = 0;
    // Oldest first, so the logged readings below are in time order.
    const std::size_t oldest = (history.next + ReadingHistory::CAPACITY - history.count) % ReadingHistory::CAPACITY;
    for (std::size_t k = 0; k < history.count; ++k) {
        const auto& s = history.samples[(oldest + k) % ReadingHistory::CAPACITY];
        if (now - s.t_ms <= Config::DISTANCE_RESET_WINDOW_MS) {
            window[n++] = s.mm / 25.4;
        }
    }

    // Raw readings, oldest first, before sorting: shows whether a wide window is drift
    // (the robot moving) or noise (values jumping).
    char readings[ReadingHistory::CAPACITY * 8 + 1] = "";
    std::size_t used = 0;
    for (std::size_t i = 0; i < n && used < sizeof(readings); ++i) {
        const int w = std::snprintf(readings + used, sizeof(readings) - used, i ? " %.2f" : "%.2f", window[i]);
        if (w < 0) {
            break;
        }
        used += (std::size_t)w;
    }

    if (n < Config::DISTANCE_RESET_MIN_SAMPLES) {
        // Covers a sensor that sees no object (it reads 9999 and nothing is stored).
        Logger::getInstance()->logWarning("Invalid distance sensor reading: %f (%u valid readings in %u ms, need %u)",
                                          sensor.sensor->get(), (unsigned)n,
                                          (unsigned)Config::DISTANCE_RESET_WINDOW_MS,
                                          (unsigned)Config::DISTANCE_RESET_MIN_SAMPLES);
        return ResetResult::Rejected;
    }

    std::sort(window.begin(), window.begin() + n);
    const double sensorReadingInches =
        (n % 2) ? window[n / 2] : 0.5 * (window[n / 2 - 1] + window[n / 2]);
    // Interquartile range: a lone spike moves neither the median nor this, so it is
    // filtered rather than vetoing the reset; many disagreeing readings still widen it.
    const double spread = window[(3 * n) / 4] - window[n / 4];
    const double maxSpread = Config::DISTANCE_RESET_MAX_SPREAD_BASE +
                             Config::DISTANCE_RESET_MAX_SPREAD_FRACTION * sensorReadingInches;

    // tests/log_analysis/parser.py keys on these two lines; keep their wording.
    Logger::getInstance()->log("DISTANCE RESET");
    Logger::getInstance()->log("Distance sensor reading: %f in", sensorReadingInches);
    Logger::getInstance()->log("Distance window: %u readings, min %f, max %f, IQR %f in (max %f): %s",
                               (unsigned)n, window[0], window[n - 1], spread, maxSpread, readings);

    if (spread > maxSpread) {
        Logger::getInstance()->logWarning("Distance reset rejected: readings IQR %f in (max %f)",
                                          spread, maxSpread);
        return ResetResult::Rejected;
    }

    double rangeRate, angularRate;
    const double motion = motionError(sensor, sensorReadingInches, sensor_angle, rangeRate, angularRate);
    if (checkMotion && motion > Config::DISTANCE_RESET_MAX_MOTION_ERROR) {
        Logger::getInstance()->logWarning("Distance reset rejected: moving at %f in/s toward the wall, "
                                          "%f rad/s: ~%f in of error (max %f)",
                                          rangeRate, angularRate, motion,
                                          Config::DISTANCE_RESET_MAX_MOTION_ERROR);
        return ResetResult::Moving;
    }

    Pose currentPose = getPose();

    // Beam direction in the field frame (CCW+, 0 = +x), then snap it to the nearest
    // wall normal: 0 = +x (right), 1 = +y (top), 2 = -x (left), 3 = -y (bottom).
    const double beam = Angles::normalizeAngle(currentPose.theta + sensor_angle * M_PI / 180.0);
    const int wall = ((int)std::lround(beam / (M_PI / 2)) % 4 + 4) % 4;
    const double normal = wall * (M_PI / 2);
    const double offNormal = Angles::angleDifference(beam, normal);
    const bool resettingX = (wall == 0 || wall == 2);
    const double wallSign = (wall == 0 || wall == 1) ? 1.0 : -1.0;

    Logger::getInstance()->log("Beam heading: %f deg, wall %d, %f deg off normal",
                               beam * 180.0 / M_PI, wall, offNormal * 180.0 / M_PI);

    if (std::fabs(offNormal) > Config::DISTANCE_RESET_MAX_BEAM_ANGLE_DEG * M_PI / 180.0) {
        Logger::getInstance()->logWarning("Distance reset rejected: beam %f deg off wall normal (max %f)",
                                          offNormal * 180.0 / M_PI, Config::DISTANCE_RESET_MAX_BEAM_ANGLE_DEG);
        return ResetResult::Rejected;
    }

    // Perpendicular distance from the tracking centre to the wall: the beam length plus
    // the along-beam offset, projected on the wall normal, plus the across-beam offset
    // (positive to the beam's right) projected on the same normal. The beam-angle gate
    // above keeps cos(offNormal) >= cos(20 deg), so this is positive and needs no abs.
    const double wallToCenter = (sensorReadingInches + sensor.offsetX) * std::cos(offNormal) +
                                sensor.offsetY * std::sin(offNormal);
    const double actualPos = wallSign * (Config::DISTANCE_RESET_WALL_POSITION - wallToCenter);
    const double currentPos = resettingX ? currentPose.x : currentPose.y;
    const double correction = actualPos - currentPos;

    Logger::getInstance()->log("resettingX: %d, wallSign: %f, wallToCenter: %f, correction: %f",
                               resettingX, wallSign, wallToCenter, correction);

    if (std::fabs(correction) > Config::DISTANCE_RESET_MAX_CORRECTION) {
        Logger::getInstance()->logWarning("Distance reset rejected: %s correction %f in exceeds %f",
                                          resettingX ? "X" : "Y", correction, Config::DISTANCE_RESET_MAX_CORRECTION);
        return ResetResult::Rejected;
    }

    // No brain-screen print here: this runs inside followTrajectory's control loop, and
    // pros::lcd::print from a non-LVGL task can freeze the program (lv_lock is a no-op).
    if (resettingX) {
        setPose({actualPos, currentPose.y, currentPose.theta});
    } else {
        setPose({currentPose.x, actualPos, currentPose.theta});
    }
    return ResetResult::Applied;
}

DistanceResetOdometry::ResetResult DistanceResetOdometry::resetFront(bool checkMotion)
{
    const SideInfo info = side(Side::Front);
    return reset(info.sensor, info.history, info.angle_deg, checkMotion);
}

DistanceResetOdometry::ResetResult DistanceResetOdometry::resetBack(bool checkMotion)
{
    const SideInfo info = side(Side::Back);
    return reset(info.sensor, info.history, info.angle_deg, checkMotion);
}

DistanceResetOdometry::ResetResult DistanceResetOdometry::resetLeft(bool checkMotion)
{
    const SideInfo info = side(Side::Left);
    return reset(info.sensor, info.history, info.angle_deg, checkMotion);
}

DistanceResetOdometry::ResetResult DistanceResetOdometry::resetRight(bool checkMotion)
{
    const SideInfo info = side(Side::Right);
    return reset(info.sensor, info.history, info.angle_deg, checkMotion);
}