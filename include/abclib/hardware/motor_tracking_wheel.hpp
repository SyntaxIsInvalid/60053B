#pragma once

#include "motor_group.hpp"
#include "tracking_wheel_interface.hpp"
#include "abclib/units/units.hpp"
#include <cmath>

namespace abclib::hardware
{
    class MotorTrackingWheel : public ITrackingWheel
    {
    private:
        AdvancedMotorGroup *motor_group_;
        units::Length wheel_diameter_;
        units::Length offset_;

    public:
        MotorTrackingWheel(AdvancedMotorGroup *motors,
                           units::Length diameter,
                           units::Length wheel_offset = units::Length::from_inches(0.0));

        units::Length get_distance() override;
        units::Length get_offset() override;
        void reset() override;
    };
}