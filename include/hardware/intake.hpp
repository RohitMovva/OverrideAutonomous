#pragma once

#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/rtos.hpp"
#include "hardware/enhanced_digital_out.hpp"
#include <atomic>
#include <cstring>

/**
 * @brief Structure containing color detection parameters
 */
struct ColorInfo {
    int32_t red_hue_min;              ///< Minimum hue value for red detection
    int32_t red_hue_max;              ///< Maximum hue value for red detection
    int32_t blue_hue_min;             ///< Minimum hue value for blue detection
    int32_t blue_hue_max;             ///< Maximum hue value for blue detection
    double_t saturation_threshold;    ///< Minimum saturation for valid color detection
};

/**
 * @brief Integrated intake system with color sorting functionality
 * 
 * This class manages three intake motors (indexer, intake, top_intake) and provides
 * color-based sorting that can temporarily override the current motor state to eject
 * unwanted game pieces.
 */
class Intake {
private:
    // Motors
    pros::MotorGroup& intake_motors_; ///< Lower intake motor reference
    EnhancedDigitalOut& intake_widener_;

    // Color detection
    pros::Optical& color_sort_os_;   ///< Optical sensor reference
    pros::Distance& color_sort_ds_;   ///< Optical sensor reference
    const char* target_color_;       ///< Target color for sorting ("red" or "blue")
    ColorInfo color_params_;         ///< Color detection parameters
    
    // State management
    int intake_state_;               ///< Current intake state (0-10 range)
    int intake_decay_ms_ = 0;        ///< Duration to hold temporary states
    int intake_decay_state_ = 0;     ///< State to revert to after decay
    int time_since_state_set_ = 0;   ///< Time since last state change
    bool color_sorting_active_ = false; ///< Flag indicating if color sorting is running

    bool skills_;

    int intake_max_speed_ = 12000;

    /// Voltage (mV) used for a gentle outtake (state 4), e.g. while the lift lowers.
    static constexpr int OUTTAKE_SLOW_SPEED = 4000;

    // Auto-widener: when something sits close to the distance sensor long enough,
    // pull the intake widener in for a fixed time.
    int widener_detect_ms_ = 0;      ///< How long the distance has been below the threshold (ms)
    int widener_close_ms_ = 0;       ///< Remaining time to hold the widener closed (ms)
    int widener_cooldown_ms_ = 0;    ///< Remaining cooldown after opening before it can re-trigger (ms)

    /// Distance (mm, smaller = closer) below this counts as an object present.
    static constexpr int32_t WIDENER_DISTANCE_THRESHOLD = 150;
    /// Distance must stay below the threshold this long before the widener closes.
    static constexpr int WIDENER_TRIGGER_MS = 20;
    /// How long to hold the widener closed once triggered.
    static constexpr int WIDENER_HOLD_MS = 1000;
    /// Cooldown after the widener opens back out before it can trigger again.
    static constexpr int WIDENER_COOLDOWN_MS = 500;

    /**
     * @brief Apply motor speeds based on current intake state
     */
    void apply_state_motors();

public:
    /**
     * @brief Constructor for Intake
     * @param intake_motors Reference to intake motor group
     * @param optical_sensor Reference to optical sensor
     */
    Intake(pros::MotorGroup& intake_motors, 
                EnhancedDigitalOut& intake_widener, pros::Optical& optical_sensor, pros::Distance& distance_sensor);
    
    /**
     * @brief Destructor - ensures proper cleanup
     */
    ~Intake();
    
    /**
     * @brief Set the intake state
     * @param state Integer state value (0-10 range)
     */
    void set_state(int state);

    /**
     * @brief Temporarily set a state, wait, then revert to another state
     * @param start_state Initial state to set
     * @param end_state State to revert to after delay
     * @param delay_ms Duration to wait in milliseconds
     */
    void state_decay(int start_state, int end_state, int delay_ms);

    /**
     * 
     */
    void set_intake_speed(int speed) {
        intake_max_speed_ = speed;
        apply_state_motors();
    }

    /**
     * 
     */
    void update();
    
    /**
     * @brief Get the current intake state
     * @return Current state value
     */
    int get_state() const;
    
    /**
     * @brief Start color sorting functionality
     * @param target_color Target color to keep ("red" or "blue")
     * @param colors Color detection parameters (optional, uses defaults if not provided)
     */
    void start_color_sorting(const char* target_color, const ColorInfo& colors = {
        310, 10,   // Red hue range
        200, 270,  // Blue hue range  
        0.0        // Minimum saturation threshold
    });
    
    /**
     * @brief Stop color sorting functionality
     */
    void stop_color_sorting();

    void set_shift(bool state) {
        if (state){
            intake_max_speed_ = 6000;
        } else {
            intake_max_speed_ = 12000;
        }
    }
    
    /**
     * @brief Check if color sorting is currently active
     * @return true if color sorting is running, false otherwise
     */
    bool is_color_sorting_active() const;

    void park();
};