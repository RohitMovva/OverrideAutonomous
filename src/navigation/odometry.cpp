#include "navigation/odometry.hpp"

Odometry::Odometry(pros::MotorGroup &left, pros::MotorGroup &right,
                   pros::Rotation &lateral, pros::Imu &imuSensor,
                   bool enable_heading_filter,
                   bool enable_velocity_filters)
    : leftDrive(left), rightDrive(right), lateralEncoder(lateral), imu(imuSensor),
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
    prevLeftPos = getMotorPositionsInches(leftDrive.get_position_all());
    prevRightPos = getMotorPositionsInches(rightDrive.get_position_all());

    const double lateral = latTicksToInches(lateralEncoder.get_position());
    prevLateralPos = std::isfinite(lateral) ? lateral : 0.0;

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

    // Absolute encoder positions
    std::vector<double> leftPositions = getMotorPositionsInches(leftDrive.get_position_all());
    std::vector<double> rightPositions = getMotorPositionsInches(rightDrive.get_position_all());

    double lateralPos = latTicksToInches(lateralEncoder.get_position());
    if (!std::isfinite(lateralPos))
    {
        lateralPos = prevLateralPos;
    }

    // Only using drive IMEs so we don't need the lateral chord
    lateralPos = 0;

    Logger::getInstance()->log("Left Positions: %f %f %f", leftDrive.get_position_all()[0],
                               leftDrive.get_position_all()[1], leftDrive.get_position_all()[2]);
    Logger::getInstance()->log("Right Positions: %f %f %f", rightDrive.get_position_all()[0],
                               rightDrive.get_position_all()[1], rightDrive.get_position_all()[2]);

    // getMotorPositionsInches drops unreadable motors. If a whole side dced (kinda cooked), 
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
    double deltaLateral = lateralPos - prevLateralPos;

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

    // Calculate forward displacement
    double forwardDisplacement = (deltaLeft + deltaRight) / 2.0;

    // Local position update
    double deltaX, deltaY;

    if (std::abs(deltaTheta) > 0.00001)
    {
        // Arc odometry
        double turnRadius = forwardDisplacement / deltaTheta;
        double centralChord = 2.0 * turnRadius * sin(deltaTheta / 2.0);
        double lateralChord = deltaLateral - (Config::LATERAL_WHEEL_OFFSET * deltaTheta);
        lateralChord = 0; // Disable for now

        deltaX = centralChord * cos(currentPose.theta + deltaTheta / 2.0) -
                 lateralChord * sin(currentPose.theta + deltaTheta / 2.0);
        deltaY = centralChord * sin(currentPose.theta + deltaTheta / 2.0) +
                 lateralChord * cos(currentPose.theta + deltaTheta / 2.0);
    }
    else
    {
        // Straight line approximation when dtheta is small
        deltaX = forwardDisplacement * cos(currentPose.theta);
        deltaY = forwardDisplacement * sin(currentPose.theta);
    }

    // Update position with optional filtering
    currentPose.x += deltaX;
    currentPose.y += deltaY;

    currentPose.theta = Angles::normalizeAngle(currentPose.theta + deltaTheta);

    prevLeftPos = leftPositions;
    prevRightPos = rightPositions;
    prevLateralPos = lateralPos;
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
    return true; // TODO update based on if sensors are outputting valid 
}

// Helper function implementations
double Odometry::ticksToInches(double ticks)
{
    return Units::ticksToDistance(ticks, Config::TICKS_PER_ROTATION, Config::WHEEL_DIAMETER, Config::GEAR_RATIO);
}

double Odometry::latTicksToInches(int ticks)
{
    return Units::ticksToDistance(ticks, Units::Constants::V5_ROTATION_SENSOR_TICKS_PER_ROTATION, Config::WHEEL_DIAMETER, Units::Constants::GEAR_RATIO_1_1);
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

std::vector<double> Odometry::getMotorPositionsInches(const std::vector<double> &motorPositions)
{
    // Ignores dced motors to not make everything NaN
    std::vector<double> positions;
    positions.reserve(motorPositions.size());
    for (double pos : motorPositions)
    {
        if (std::isfinite(pos))
        {
            positions.push_back(ticksToInches(pos));
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
    else if (sensor == "lateral")
    {
        const double lateral = latTicksToInches(lateralEncoder.get_position());
        prevLateralPos = std::isfinite(lateral) ? lateral : prevLateralPos;
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