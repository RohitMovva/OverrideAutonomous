#include "navigation/odometry.hpp"
#include <algorithm>

Odometry::Odometry(pros::MotorGroup &left, pros::MotorGroup &right,
                   pros::Rotation &vertical, pros::Rotation &lateral, pros::Imu &imuSensor,
                   bool enable_heading_filter,
                   bool enable_velocity_filters)
    : leftDrive(left), rightDrive(right), verticalEncoder(vertical), lateralEncoder(lateral), imu(imuSensor),
      leftVelocityObserver(Config::KS, Config::KV, Config::KA, Config::VELOCITY_OBSERVER_GAIN,
                           Config::VELOCITY_OBSERVER_DISTURBANCE_GAIN),
      rightVelocityObserver(Config::KS, Config::KV, Config::KA, Config::VELOCITY_OBSERVER_GAIN,
                            Config::VELOCITY_OBSERVER_DISTURBANCE_GAIN),
      headingFilter(), useHeadingFilter(enable_heading_filter), useVelocityFilters(enable_velocity_filters)
{
    reset();
}

void Odometry::reset()
{
    currentPose = Pose();
    lastUpdateTime = Units::millisecondsToSeconds(pros::millis());
    prevTime = lastUpdateTime;

    leftVelocityObserver.reset(0.0);
    rightVelocityObserver.reset(0.0);
    leftVelocity = Velocity();
    rightVelocity = Velocity();
    leftVelocityRaw = 0.0;
    rightVelocityRaw = 0.0;
    lastLeftVoltage = 0.0;
    lastRightVoltage = 0.0;
    headingFilter.reset();

    // Prevents race condition with taring sensors then reading
    seedSensorBaseline();
}

void Odometry::seedSensorBaseline()
{
    prevLeftPos = getMotorPositionsInches(leftDrive.get_position_all(),
                                          Config::LEFT_ODOM_MOTOR_INDEX);
    prevRightPos = getMotorPositionsInches(rightDrive.get_position_all(),
                                           Config::RIGHT_ODOM_MOTOR_INDEX);

    verticalFallback = false;
    lateralFallback = false;
    hasPrevVertical = readTrackingWheel(verticalEncoder, Config::VERTICAL_TRACKING_WHEEL_DIAMETER,
                                        prevVerticalPos);
    hasPrevLateral = readTrackingWheel(lateralEncoder, Config::LATERAL_TRACKING_WHEEL_DIAMETER,
                                       prevLateralPos);
    verticalChangeTime = lateralChangeTime = Units::millisecondsToSeconds(pros::millis());

    const double rotation = imu.get_rotation();
    prevImuRotation = std::isfinite(rotation) ? rotation : 0.0;

    hasPrevSensorReadings = std::isfinite(rotation);
}

void Odometry::update()
{
    double currentTime = Units::millisecondsToSeconds(pros::millis());
    double deltaTime = currentTime - lastUpdateTime;
    if (deltaTime <= 0 || deltaTime > 0.1) // Avoid too small or too large time steps
    {
        Logger::getInstance()->logWarning("Invalid deltaTime: %f, reverting to default", deltaTime);
        deltaTime = Units::millisecondsToSeconds(Config::DT);
    }

    // No valid prev sensor readings which are required to find delta in movements
    if (!hasPrevSensorReadings)
    {
        Logger::getInstance()->logWarning("No valid sensor baseline; reseeding and skipping tick");
        seedSensorBaseline();
        lastUpdateTime = currentTime;
        return;
    }

    const double imuRotation = imu.get_rotation(); // Continuous rotation
    double deltaTheta = 0.0;
    if (std::isfinite(imuRotation))
    {
        // We use CCW+ so we have to negate IMU readings
        deltaTheta = Angles::degreesToRadians(-(imuRotation - prevImuRotation) * Config::IMU_HEADING_SCALE);
        prevImuRotation = imuRotation;
    }
    else
    {
        Logger::getInstance()->logWarning("Invalid IMU reading; holding heading this tick");
    }

    const auto rawLeftPositions = leftDrive.get_position_all();
    const auto rawRightPositions = rightDrive.get_position_all();
    std::vector<double> leftPositions =
        getMotorPositionsInches(rawLeftPositions, Config::LEFT_ODOM_MOTOR_INDEX);
    std::vector<double> rightPositions =
        getMotorPositionsInches(rawRightPositions, Config::RIGHT_ODOM_MOTOR_INDEX);

    double verticalPos = 0.0;
    double lateralPos = 0.0;
    const bool verticalRead = readTrackingWheel(verticalEncoder, Config::VERTICAL_TRACKING_WHEEL_DIAMETER,
                                                verticalPos);
    const bool lateralRead = readTrackingWheel(lateralEncoder, Config::LATERAL_TRACKING_WHEEL_DIAMETER,
                                               lateralPos);

    // Log the same snapshot used above, and only motors that exist. This drive
    // has two motors per side; indexing [2] was an out-of-bounds read.
    const auto logPositions = [](const char *side, const std::vector<double> &positions) {
        std::string values;
        for (double position : positions) {
            if (!values.empty()) values += ' ';
            values += std::to_string(position);
        }
        Logger::getInstance()->log("%s Positions: %s", side, values.c_str());
    };
    logPositions("Left", rawLeftPositions);
    logPositions("Right", rawRightPositions);

    // getMotorPositionsInches ignores unreadable motors. If a whole side dced (worst case), 
    // hold its previous position so the delta is zero
    if (leftPositions.empty())
    {
        Logger::getInstance()->logWarning("No readable left drive encoders; holding position");
        leftPositions = prevLeftPos;
    }
    if (rightPositions.empty())
    {
        Logger::getInstance()->logWarning("No readable right drive encoders; holding position");
        rightPositions = prevRightPos;
    }

    double currentLeftPos = getAveragePosition(leftPositions);
    double currentRightPos = getAveragePosition(rightPositions);
    Logger::getInstance()->log("Left: %f, Right: %f", currentLeftPos, currentRightPos);

    // Calculate position changes
    double deltaLeft = currentLeftPos - getAveragePosition(prevLeftPos);
    double deltaRight = currentRightPos - getAveragePosition(prevRightPos);
    Logger::getInstance()->log("DeltaLeft: %f, DeltaRight: %f", deltaLeft, deltaRight);

    // Validate readings to ignore jumps in IME readings
    const double maxPlausibleDelta = Config::FREE_WHEEL_VELOCITY * deltaTime * 1.5;
    if (!std::isfinite(deltaLeft) || !std::isfinite(deltaRight) ||
        std::abs(deltaLeft) > maxPlausibleDelta || std::abs(deltaRight) > maxPlausibleDelta)
    {
        Logger::getInstance()->logWarning(
            "Implausible encoder delta (L=%f R=%f in over %f s, max %f); dropping tick",
            deltaLeft, deltaRight, deltaTime, maxPlausibleDelta);
        deltaLeft = 0.0;
        deltaRight = 0.0;
    }

    // Tracking wheel deltas. A wheel needs a reading now AND last tick; when one is
    // missing, or the delta is a jump no drivetrain can make, fall back for this tick:
    // the vertical wheel to the drive-encoder average, the lateral wheel to "no
    // sideslip". The baseline is re-taken from whatever reads now, so a sensor that
    // reconnects is used again from the following tick.
    const double driveForward = (deltaLeft + deltaRight) / 2.0;
    double deltaVertical = verticalPos - prevVerticalPos;
    double deltaLateral = lateralPos - prevLateralPos;
    // A wheel's delta may hold the travel of every tick since its reading last changed
    // (see Config::TRACKING_WHEEL_MAX_STALE_TICKS), so gate it over that span.
    const auto wheelMaxDelta = [&](double changeTime) {
        const double span = std::clamp(currentTime - changeTime, deltaTime,
                                       Config::TRACKING_WHEEL_MAX_STALE_TICKS * deltaTime);
        return Config::FREE_WHEEL_VELOCITY * span * 1.5;
    };
    const bool verticalOk = verticalRead && hasPrevVertical &&
                            std::abs(deltaVertical) <= wheelMaxDelta(verticalChangeTime);
    const bool lateralOk = lateralRead && hasPrevLateral &&
                           std::abs(deltaLateral) <= wheelMaxDelta(lateralChangeTime);
    if (verticalRead && deltaVertical != 0.0)
    {
        verticalChangeTime = currentTime;
    }
    if (lateralRead && deltaLateral != 0.0)
    {
        lateralChangeTime = currentTime;
    }
    // Warn once per outage; the TRACKWHEEL line below records every tick of it
    if (!verticalOk && !verticalFallback)
    {
        Logger::getInstance()->logWarning(
            "Vertical tracking wheel unusable (read=%d delta=%f); using drive encoders",
            verticalRead, deltaVertical);
    }
    if (!lateralOk && !lateralFallback)
    {
        Logger::getInstance()->logWarning(
            "Lateral tracking wheel unusable (read=%d delta=%f); assuming no sideslip",
            lateralRead, deltaLateral);
    }
    verticalFallback = !verticalOk;
    lateralFallback = !lateralOk;
    if (!verticalOk)
    {
        deltaVertical = driveForward;
    }
    // Raw deltas (nan when unusable) beside the drive-encoder estimate, for
    // log_analysis.metrics.tracking_wheel_agreement.
    Logger::getInstance()->log(
        "TRACKWHEEL vert=%f lat=%f dvert=%f dlat=%f ddrive=%f vok=%d lok=%d",
        verticalRead ? verticalPos : NAN, lateralRead ? lateralPos : NAN,
        verticalOk ? deltaVertical : NAN, lateralOk ? deltaLateral : NAN,
        driveForward, verticalOk, lateralOk);

    // Update velocities with optional filtering.
    const double prevLeftVelocity = leftVelocity.linear;
    const double prevRightVelocity = rightVelocity.linear;

    leftVelocityRaw = deltaLeft / deltaTime;
    rightVelocityRaw = deltaRight / deltaTime;

    if (useVelocityFilters)
    {
        leftVelocity.linear = leftVelocityObserver.update(leftVelocityRaw, lastLeftVoltage, deltaTime);
        rightVelocity.linear = rightVelocityObserver.update(rightVelocityRaw, lastRightVoltage, deltaTime);
    }
    else
    {
        leftVelocity.linear = leftVelocityRaw;
        rightVelocity.linear = rightVelocityRaw;
    }

    // Acceleration = d(velocity)/dt 
    leftAccel.linear = (leftVelocity.linear - prevLeftVelocity) / deltaTime;
    rightAccel.linear = (rightVelocity.linear - prevRightVelocity) / deltaTime;

    // Arc odometry in the robot frame. The fallbacks above already describe the tracking
    // centre (the drive average is centred, "no sideslip" is zero), so they skip the
    // wheel-offset correction by passing zero offsets.
    const LocalDisplacement local = trackingArc(
        deltaVertical, lateralOk ? deltaLateral : 0.0, deltaTheta,
        verticalOk ? Config::VERTICAL_WHEEL_OFFSET : 0.0,
        lateralOk ? Config::LATERAL_WHEEL_OFFSET : 0.0);

    // Rotate into the field frame at the arc's mean heading
    const double midTheta = currentPose.theta + deltaTheta / 2.0;
    const double deltaX = local.x * cos(midTheta) - local.y * sin(midTheta);
    const double deltaY = local.x * sin(midTheta) + local.y * cos(midTheta);

    // Update position with optional filtering
    currentPose.x += deltaX;
    currentPose.y += deltaY;

    currentPose.theta = Angles::normalizeAngle(currentPose.theta + deltaTheta);

    prevLeftPos = leftPositions;
    prevRightPos = rightPositions;
    prevVerticalPos = verticalPos;
    prevLateralPos = lateralPos;
    hasPrevVertical = verticalRead;
    hasPrevLateral = lateralRead;
    prevTime = lastUpdateTime;
    lastUpdateTime = currentTime;
}

// Core interface implementation
Pose Odometry::getPose() const { return currentPose; }
void Odometry::setPose(const Pose &newPose)
{
    currentPose = newPose;
}
double Odometry::getHeading() const { return currentPose.theta; }
double Odometry::getX() const { return currentPose.x; }
double Odometry::getY() const { return currentPose.y; }
Velocity Odometry::getLeftVelocity() const { return leftVelocity; }
Velocity Odometry::getRightVelocity() const { return rightVelocity; }

void Odometry::setDriveVoltages(double leftVoltage, double rightVoltage)
{
    if (std::isfinite(leftVoltage) && std::isfinite(rightVoltage))
    {
        lastLeftVoltage = leftVoltage;
        lastRightVoltage = rightVoltage;
    }
}

bool Odometry::isReliable() const
{
    // False until the IMU gives a finite rotation to take deltas from; update() skips
    // every tick until then, so the pose is frozen
    return hasPrevSensorReadings;
}

// Helper function implementations
double Odometry::ticksToInches(double ticks)
{
    return Units::ticksToDistance(ticks, Config::TICKS_PER_ROTATION, Config::WHEEL_DIAMETER, Config::GEAR_RATIO);
}

bool Odometry::readTrackingWheel(pros::Rotation &encoder, double diameter, double &inches)
{
    const int32_t centidegrees = encoder.get_position();
    if (centidegrees == PROS_ERR)
    {
        return false;
    }
    inches = Units::ticksToDistance(centidegrees, Units::Constants::V5_ROTATION_SENSOR_TICKS_PER_ROTATION,
                                    diameter, Units::Constants::GEAR_RATIO_1_1);
    return true;
}

double Odometry::getAveragePosition(const std::vector<double> &positions)
{
    double sum = 0.0;
    int count = 0;
    for (double pos : positions)
    {
        if (std::isfinite(pos))
        {
            sum += pos;
            ++count;
        }
    }
    return (count > 0) ? (sum / count) : 0.0;
}

std::vector<double> Odometry::getMotorPositionsInches(const std::vector<double> &motorPositions,
                                                      int trustedIndex)
{
    // Ignores dced motors to not make everything NaN
    std::vector<double> positions;
    positions.reserve(motorPositions.size());
    for (std::size_t i = 0; i < motorPositions.size(); ++i)
    {
        // Pin the side to one encoder when a motor's is known bad. Filtering at the read
        // keeps this in one place: every consumer of prevLeftPos/prevRightPos then sees
        // the same channels the current reading came from.
        if (trustedIndex >= 0 && static_cast<int>(i) != trustedIndex)
        {
            continue;
        }
        if (std::isfinite(motorPositions[i]))
        {
            positions.push_back(ticksToInches(motorPositions[i]));
        }
    }
    return positions;
}

// Odometry-specific methods
Velocity Odometry::getLeftAcceleration() const { return leftAccel; }
Velocity Odometry::getRightAcceleration() const { return rightAccel; }
double Odometry::getAngularVelocity() const
{
    return useHeadingFilter ? headingFilter.getAngularVelocity() : 0.0;
}
double Odometry::getHeadingUncertainty() const
{
    return useHeadingFilter ? headingFilter.getHeadingUncertainty() : 0.0;
}

std::pair<double, double> Odometry::getFilteredVelocities() const
{
    return {leftVelocity.linear, rightVelocity.linear};
}

void Odometry::setFiltersEnabled(bool heading, bool velocity)
{
    useHeadingFilter = heading;
    useVelocityFilters = velocity;
}

void Odometry::disableSensor(const std::string &sensor)
{
    if (sensor == "left")
    {
        leftVelocityObserver.reset();
    }
    else if (sensor == "right")
    {
        rightVelocityObserver.reset();
    }
    else if (sensor == "vertical")
    {
        hasPrevVertical = readTrackingWheel(verticalEncoder, Config::VERTICAL_TRACKING_WHEEL_DIAMETER,
                                            prevVerticalPos);
    }
    else if (sensor == "lateral")
    {
        hasPrevLateral = readTrackingWheel(lateralEncoder, Config::LATERAL_TRACKING_WHEEL_DIAMETER,
                                           prevLateralPos);
    }
}

Odometry::DebugInfo Odometry::getOdometryDebugInfo()
{
    // Reports the estimates update() already produced.
    return {
        leftVelocityRaw,
        rightVelocityRaw,
        leftVelocity.linear,
        rightVelocity.linear,
        Angles::degreesToRadians(imu.get_heading() * -1),
        currentPose.theta,
        getPositionUncertainty()};
}

std::unordered_map<std::string, double> Odometry::getDebugData() const
{
    std::unordered_map<std::string, double> debug;
    debug["left_velocity"] = leftVelocity.linear;
    debug["right_velocity"] = rightVelocity.linear;
    debug["left_velocity_raw"] = leftVelocityRaw;
    debug["right_velocity_raw"] = rightVelocityRaw;
    debug["left_acceleration"] = leftAccel.linear;
    debug["right_acceleration"] = rightAccel.linear;
    debug["heading_raw"] = Angles::degreesToRadians(imu.get_heading() * -1);
    debug["heading_filtered"] = currentPose.theta;
    debug["reliable"] = isReliable() ? 1.0 : 0.0;

    if (useHeadingFilter)
    {
        debug["angular_velocity"] = getAngularVelocity();
        debug["heading_uncertainty"] = getHeadingUncertainty();
    }

    return debug;
}
