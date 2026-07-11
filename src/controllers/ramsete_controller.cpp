#include "controllers/ramsete_controller.hpp"

RamseteController::RamseteController(double b, double zeta, double scale_factor)
    : b_(b)
    , zeta_(zeta)
    , scale_factor_(scale_factor)
{
}



std::vector<double> RamseteController::calculate_global_error(double x, double y, double theta,
                                                            double goal_x, double goal_y, double goal_theta) {
    std::vector<double> error;
    // Simple difference in global coordinates
    error.push_back(goal_x - x);  // Changed from (x - goal_x) to (goal_x - x)
    error.push_back(goal_y - y);  // Changed from (y - goal_y) to (goal_y - y)
    
    // Normalize theta error to [-π, π]
    double theta_error = goal_theta - theta;
    while (theta_error > M_PI) theta_error -= 2 * M_PI;
    while (theta_error < -M_PI) theta_error += 2 * M_PI;
    error.push_back(theta_error);
    
    return error;
}

std::vector<double> RamseteController::calculate_local_error(std::vector<double> global_error, double theta) {
    std::vector<double> local_error;
    
    // Transform global error to robot frame
    local_error.push_back(global_error[0] * std::cos(theta) + global_error[1] * std::sin(theta));
    local_error.push_back(-global_error[0] * std::sin(theta) + global_error[1] * std::cos(theta));
    local_error.push_back(global_error[2]);  // Angular error remains the same
    
    return local_error;
}

std::vector<double> RamseteController::calculate_wheel_velocities(double linear_velocity,
                                                                double angular_velocity,
                                                                double track_width,
                                                                double max_wheel_velocity) {
    // Calculate wheel speed difference based on angular velocity
    // angular_velocity (rad/s) * track_width/2 (inches) = inches/sec
    double wheel_speed_diff = angular_velocity * track_width / 2.0;

    double left = linear_velocity - wheel_speed_diff;
    double right = linear_velocity + wheel_speed_diff;

    // Desaturate. If either wheel is asked for more than the drivetrain can hold,
    // scale BOTH by the same factor. That keeps left:right -- and therefore the
    // commanded curvature -- intact, so the robot drives the same arc, just slower.
    //
    // Without this the excess silently became under-rotation. A RAMSETE correction
    // stacked on top of a fast reference pushed the outside wheel to 59.4 in/s
    // against a ~55 in/s ceiling; the motor pinned at 127, the wheel fell ~5 in/s
    // short, and because rotation is the DIFFERENCE of two large wheel speeds that
    // shortfall ate most of the turn. initial_tuning.txt delivered only 69% of the
    // commanded turn differential and ran wide off the path for the entire run.
    //
    // Clamping the wheels independently would NOT work: it would cap the outside
    // wheel while leaving the inside one alone, shrinking the differential and
    // flattening the turn -- exactly the failure this exists to prevent. Scale, do
    // not clip.
    if (max_wheel_velocity > 0.0) {
        double max_magnitude = std::max(std::abs(left), std::abs(right));
        if (max_magnitude > max_wheel_velocity) {
            double scale = max_wheel_velocity / max_magnitude;
            left *= scale;
            right *= scale;
        }
    }

    return {left, right};
}

std::vector<double> RamseteController::calculate(double x, double y, double theta,
                                               double goal_x, double goal_y, double goal_theta,
                                               double v_ref, double w_ref) {

    // Scale inputs to meters
    x *= scale_factor_;
    y *= scale_factor_;
    goal_x *= scale_factor_;
    goal_y *= scale_factor_;
    v_ref *= scale_factor_;

    // Calculate global errors and transform to robot frame
    double cos_theta = std::cos(theta);
    double sin_theta = std::sin(theta);
    double e_x = (goal_x - x) * cos_theta + (goal_y - y) * sin_theta;
    double e_y = -(goal_x - x) * sin_theta + (goal_y - y) * cos_theta;
    
    // Normalize theta error to [-π, π]
    double e_theta = goal_theta - theta;
    while (e_theta > M_PI) e_theta -= 2 * M_PI;
    while (e_theta < -M_PI) e_theta += 2 * M_PI;

    // Calculate distance error for gain adjustment
    double distance_error = std::sqrt(e_x * e_x + e_y * e_y);
    
    // Calculate gains with enhanced stability
    bool is_low_speed = std::abs(v_ref) < 0.5;
    double k = 2.0 * zeta_ * std::sqrt(w_ref * w_ref + b_ * std::abs(v_ref) * std::abs(v_ref));
    
    // Calculate velocities with safe handling of small angles
    double v = v_ref * std::cos(e_theta) + k * e_x;
    double w = w_ref;
    
    if (std::abs(e_theta) < 1e-6) {
        w += b_ * v_ref * e_y + k * e_theta;
    } else {
        w += b_ * v_ref * (std::sin(e_theta) / e_theta) * e_y + k * e_theta;
    }

    // Convert linear velocity back to user units
    v /= scale_factor_;
    
    return {v, w};
}
