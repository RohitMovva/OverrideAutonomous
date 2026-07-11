#ifndef RAMSETE_CONTROLLER_HPP
#define RAMSETE_CONTROLLER_HPP

#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>

/**
 * @brief RAMSETE Controller for trajectory tracking
 * 
 * Implements the RAMSETE (Rapid and Asymptotically Maximum-likelihood Estimation)
 * nonlinear time-varying feedback controller for trajectory tracking.
 * Reference: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/ramsete.html
 */
class RamseteController {
private:
    // apply_velocity_limits() used to live here. It clamped v and w independently,
    // which changes the v:w ratio and so flattens the commanded curvature -- the
    // same mistake as clipping each wheel voltage on its own. It was also dead: the
    // only caller was get_output_velocities(), which nothing used, so RAMSETE output
    // was in fact unbounded. Saturation is now handled where it belongs, in wheel
    // space, by calculate_wheel_velocities(), which scales both wheels together.

    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }

    double prev_x;
    double prev_y;
    double prev_time;
public:
    /**
     * @brief Construct a new RAMSETE Controller
     *
     * This is the only constructor. The previous RamseteController() and
     * RamseteController(b, zeta) overloads left scale_factor_ uninitialized, and
     * calculate() multiplies every input by it -- constructing either would have fed
     * the controller garbage. Nothing used them, so they were removed.
     *
     * max_v/max_w are gone too: they were a chassis-level clamp (see the note on
     * apply_velocity_limits above) and are superseded by the wheel-space
     * desaturation in calculate_wheel_velocities().
     *
     * @param b Tuning parameter for convergence
     * @param zeta Damping factor
     * @param scale_factor Scale factor for inputs and outputs (in relation to meters)
     */
    RamseteController(double b, double zeta, double scale_factor);

    /// Gains actually in use, for logging. Do not read Config:: to log these --
    /// the log must describe the controller, not the config file.
    double getB() const { return b_; }
    double getZeta() const { return zeta_; }
    double getScaleFactor() const { return scale_factor_; }

    /**
     * @brief Calculate the global error between current and goal states
     * 
     * @param x Current x position
     * @param y Current y position
     * @param theta Current heading
     * @param goal_x Goal x position
     * @param goal_y Goal y position
     * @param goal_theta Goal heading
     * @return std::vector<double> Global error [x_error, y_error, theta_error]
     */
    std::vector<double> calculate_global_error(double x, double y, double theta,
                                             double goal_x, double goal_y, double goal_theta);

    /**
     * @brief Transform global error to local robot frame
     * 
     * @param global_error Global error vector
     * @param goal_theta Goal heading
     * @return std::vector<double> Local error in robot frame
     */
    std::vector<double> calculate_local_error(std::vector<double> global_error, double goal_theta);

    /**
     * @brief Convert desired chassis velocities to wheel velocities, desaturated
     *
     * If either wheel exceeds max_wheel_velocity, BOTH are scaled by the same
     * factor, preserving left:right and therefore the commanded curvature. The
     * robot drives the same arc, just slower. Clamping the wheels independently
     * would cap the outside wheel while leaving the inside one alone, shrinking the
     * differential and flattening the turn.
     *
     * wheel_diameter and gear_ratio used to be parameters here but were never read
     * -- the conversion only needs the track width.
     *
     * @param linear_velocity Desired linear velocity (in/s)
     * @param angular_velocity Desired angular velocity (rad/s)
     * @param track_width Track width in inches
     * @param max_wheel_velocity Wheel-speed budget (in/s); pass <= 0 to disable
     * @return std::vector<double> Wheel velocities [left_velocity, right_velocity]
     */
    std::vector<double> calculate_wheel_velocities(double linear_velocity,
                                                 double angular_velocity,
                                                 double track_width,
                                                 double max_wheel_velocity);

    /**
     * @brief Calculate control output for trajectory tracking
     * 
     * @param x Current x position
     * @param y Current y position
     * @param theta Current heading
     * @param goal_x Goal x position
     * @param goal_y Goal y position
     * @param goal_theta Goal heading
     * @param v Reference linear velocity
     * @param w Reference angular velocity
     * @return std::vector<double> Control outputs [linear_velocity, angular_velocity]
     */
    std::vector<double> calculate(double x, double y, double theta,
                                double goal_x, double goal_y, double goal_theta,
                                double v, double w);
    /**
     * @brief Helper function to calculate the sinc function
     * @param x Input value
     */
    double sinc(double x) {
        if (std::abs(x) < 1e-6) {
            return 1.0 - x * x / 6.0;
        } else {
            return std::sin(x) / x;
        }
    }

    /** 
     * @brief Helper function to normalize angle
     * @param angle Input angle in radians
     * @return double Normalized angle in range [-pi, pi]
     */
    double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }

private:
    double b_;      ///< Convergence tuning parameter
    double zeta_;   ///< Damping factor
    double scale_factor_;  ///< Scale factor for inputs and outputs
};

#endif // RAMSETE_CONTROLLER_H