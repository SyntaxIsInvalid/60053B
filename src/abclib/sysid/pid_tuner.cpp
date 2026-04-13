#include "abclib/sysid/pid_tuner.hpp"

namespace abclib::sysid
{
    PIDTuner::PIDTuner(hardware::Chassis &chassis, pros::Controller &controller)
        : chassis_(chassis),
          controller_(controller),
          lateral_constants_(chassis.get_lateral_pid_constants()),
          angular_constants_(chassis.get_angular_pid_constants()),
          lateral_defaults_(chassis.get_lateral_pid_constants()),
          angular_defaults_(chassis.get_angular_pid_constants())
    {
        build_gain_list();
    }

    void PIDTuner::set_test_move(std::function<void()> test_move)
    {
        test_move_ = test_move;
    }

    void PIDTuner::build_gain_list()
    {
        gains_ = {
            {"LAT KP", &lateral_constants_.kP, 0.1,  0.0},
            {"LAT KI", &lateral_constants_.kI, 0.01, 0.0},
            {"LAT KD", &lateral_constants_.kD, 0.05, 0.0},
            {"ANG KP", &angular_constants_.kP, 1.0,  0.0},
            {"ANG KI", &angular_constants_.kI, 0.01, 0.0},
            {"ANG KD", &angular_constants_.kD, 0.1,  0.0},
        };
    }

    void PIDTuner::apply_current_gain(double delta)
    {
        TunableGain &gain = gains_[cursor_];
        *gain.value += delta;
        if (*gain.value < gain.min)
            *gain.value = gain.min;
    }

    void PIDTuner::push_to_chassis()
    {
        if (cursor_ <= 2)
            chassis_.set_lateral_pid_constants(lateral_constants_);
        else
            chassis_.set_angular_pid_constants(angular_constants_);
    }

    bool PIDTuner::check_chord()
    {
        return controller_.get_digital(pros::E_CONTROLLER_DIGITAL_L1) &&
               controller_.get_digital(pros::E_CONTROLLER_DIGITAL_L2) &&
               controller_.get_digital(pros::E_CONTROLLER_DIGITAL_R1) &&
               controller_.get_digital(pros::E_CONTROLLER_DIGITAL_R2) &&
               controller_.get_digital(pros::E_CONTROLLER_DIGITAL_Y)  &&
               controller_.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
    }

    void PIDTuner::render()
    {
        char buf[17]; // 16 chars + null terminator

        switch (state_)
        {
            case TunerState::INACTIVE:
                controller_.clear_line(0);
                break;

            case TunerState::TUNING:
                snprintf(buf, sizeof(buf), "TUNE %s %d",
                         gains_[cursor_].name,
                         move_count_);
                controller_.print(0, 0, buf);
                break;

            case TunerState::TESTING:
                controller_.print(0, 0, "IN MOTION       ");
                break;
        }
    }

    void PIDTuner::handle_inactive()
    {
        bool chord_now = check_chord();

        if (chord_now && !chord_was_held_)
        {
            // Activate tuner - reset session state
            cursor_ = 0;
            move_count_ = 0;
            chord_was_held_ = true;
            state_ = TunerState::TUNING;
            render();
            return;
        }

        chord_was_held_ = chord_now;
    }

    void PIDTuner::handle_tuning()
    {
        // Check chord to deactivate
        bool chord_now = check_chord();
        if (chord_now && !chord_was_held_)
        {
            chord_was_held_ = true;
            state_ = TunerState::INACTIVE;
            render();
            return;
        }
        chord_was_held_ = chord_now;

        // Navigate cursor
        if (controller_.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
        {
            cursor_++;
            if (cursor_ > static_cast<int>(gains_.size()) - 1)
                cursor_ = 0;
            render();
        }
        else if (controller_.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT))
        {
            cursor_--;
            if (cursor_ < 0)
                cursor_ = static_cast<int>(gains_.size()) - 1;
            render();
        }

        // Modify gain
        if (controller_.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
        {
            apply_current_gain(gains_[cursor_].increment);
            push_to_chassis();
            render();
        }
        else if (controller_.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
        {
            apply_current_gain(-gains_[cursor_].increment);
            push_to_chassis();
            render();
        }

        // Reset to defaults
        if (controller_.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
        {
            reset_to_defaults();
            render();
        }

        // Trigger test move
        if (controller_.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
        {
            if (test_move_)
            {
                state_ = TunerState::TESTING;
                render();
                // Transition to testing handled in handle_testing
            }
        }
    }

    void PIDTuner::handle_testing()
    {
        // Run the blocking test move
        test_move_();

        // Move completed - increment counter and return to tuning
        move_count_++;
        state_ = TunerState::TUNING;
        render();
    }

    void PIDTuner::update()
    {
        switch (state_)
        {
            case TunerState::INACTIVE:
                handle_inactive();
                break;
            case TunerState::TUNING:
                handle_tuning();
                break;
            case TunerState::TESTING:
                handle_testing();
                break;
        }
    }

    void PIDTuner::reset_to_defaults()
    {
        lateral_constants_ = lateral_defaults_;
        angular_constants_ = angular_defaults_;
        chassis_.set_lateral_pid_constants(lateral_constants_);
        chassis_.set_angular_pid_constants(angular_constants_);
        cursor_ = 0;
        build_gain_list();
    }
}