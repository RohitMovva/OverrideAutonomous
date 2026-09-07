#include "intake.hpp"

Intake::Intake(pros::MotorGroup& intake_motors,
               pros::Optical& optical_sensor, pros::Distance& distance_sensor)
    : intake_motors_(intake_motors),
      color_sort_os_(optical_sensor), color_sort_ds_(distance_sensor),
      target_color_(nullptr), intake_state_(0), skills_(false),
      color_sorting_active_(false) {

    color_sort_os_.set_integration_time(20.0);
    color_sort_os_.set_led_pwm(100);
    target_color_ = "none";
}

Intake::~Intake() {
    stop_color_sorting();
}

void Intake::set_state(int state) {
    if (intake_state_ == 5 && state == 0) { // don't stop intaking when parking
        return;
    }
    if (state != 3 && (intake_state_ == 3 || intake_state_ == 13)){
        intake_state_ = state;
        intake_decay_ms_ = 0;
        time_since_state_set_ = 0;
    }

    if (intake_decay_ms_ > 0 && state != 8) {
        return; // Ignore state changes during decay
    }

    if (intake_state_ != state) {
        time_since_state_set_ = 0;
    }
    intake_state_ = state;
}

void Intake::set_assist_feed(bool on) {
    assist_feed_ = on;
}

void Intake::state_decay(int start_state, int end_state, int delay_ms) {
    if (start_state == 3 && (intake_state_ == 3 || intake_state_ == 13) && intake_decay_ms_ > 0){
        return;
    }
    intake_decay_state_ = end_state;
    intake_decay_ms_ = delay_ms;
    set_state(start_state);
}

int Intake::get_state() const {
    return intake_state_;
}

void Intake::apply_state_motors() {
    // Assist feed wins over the commanded state while it is set, and only while it is set:
    // intake_state_ is never written here, so dropping the flag hands the intake straight
    // back to whatever the driver / auton had it doing.
    if (assist_feed_) {
        intake_motors_.move_voltage(ASSIST_FEED_SPEED);
        return;
    }
    switch (intake_state_) {
        case 0:
            intake_motors_.move_voltage(0);
            break;

        case 1: // Intake
            intake_motors_.move_voltage(intake_max_speed_);
            break;

        case 2: // Outtake to low goal
            intake_motors_.move_voltage(-intake_max_speed_);
            break;

        case 4: // Outtake slowly (e.g. while the lift lowers)
            intake_motors_.move_voltage(-4000);
            break;

        default:
            intake_motors_.move_voltage(0);
            break;
    }
}

void Intake::start_color_sorting(const char* target_color, const ColorInfo& colors) {
    target_color_ = target_color;
    color_params_ = colors;
    color_sorting_active_ = true;
}

void Intake::stop_color_sorting() {
    color_sorting_active_ = false;
}

bool Intake::is_color_sorting_active() const {
    return color_sorting_active_;
}

void Intake::update(){
    // double hue = color_sort_os_.get_hue();
    // double saturation = color_sort_os_.get_saturation();

    bool detected = false;


    time_since_state_set_ += 10;

    apply_state_motors();
}
