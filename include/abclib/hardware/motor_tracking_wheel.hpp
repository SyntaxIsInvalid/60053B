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
        units::Distance wheel_diameter_;
        units::Distance offset_;

    public:
        MotorTrackingWheel(AdvancedMotorGroup *motors,
                           units::Distance diameter,
                           units::Distance wheel_offset = units::Distance::from_inches(0.0));

        units::Distance get_distance() override;
        units::Distance get_offset() override;
        void reset() override;
    };
}