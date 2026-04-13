#pragma once

#include "abclib/hardware/chassis.hpp"
#include "abclib/control/pid/pid.hpp"
#include "pros/misc.h"
#include <vector>
#include <functional>

namespace abclib::hardware
{
    class Chassis;
}

namespace abclib::sysid
{
    enum class TunerState
    {
        INACTIVE,
        TUNING,
        TESTING
    };

    struct TunableGain
    {
        const char *name;
        double *value;
        double increment;
        double min;
    };

    class PIDTuner
    {
    private:
        hardware::Chassis &chassis_;
        pros::Controller &controller_;

        control::PIDConstants lateral_constants_;
        control::PIDConstants angular_constants_;

        const control::PIDConstants lateral_defaults_;
        const control::PIDConstants angular_defaults_;

        std::vector<TunableGain> gains_;
        int cursor_ = 0;
        int move_count_ = 0;

        TunerState state_ = TunerState::INACTIVE;
        std::function<void()> test_move_;
        bool chord_was_held_ = false;

        void build_gain_list();
        void apply_current_gain(double delta);
        void push_to_chassis();
        void render();
        bool check_chord();
        void handle_inactive();
        void handle_tuning();
        void handle_testing();

    public:
        PIDTuner(hardware::Chassis &chassis, pros::Controller &controller);

        void set_test_move(std::function<void()> test_move);
        void update();
        void reset_to_defaults();
        bool is_inactive() const { return state_ == TunerState::INACTIVE; }
        TunerState get_state() const { return state_; }

        const control::PIDConstants &get_lateral_constants() const { return lateral_constants_; }
        const control::PIDConstants &get_angular_constants() const { return angular_constants_; }
        int get_cursor() const { return cursor_; }
        int get_move_count() const { return move_count_; }
        const std::vector<TunableGain> &get_gains() const { return gains_; }
    };
}