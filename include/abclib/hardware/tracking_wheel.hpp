#pragma once
#include "api.h"
#include "tracking_wheel_interface.hpp"
#include "abclib/units/units.hpp"

namespace abclib::hardware
{
    class TrackingWheel : public ITrackingWheel
    {
    private:
        pros::Rotation *rotation_sensor;
        units::Distance diameter;
        units::Distance radius;
        units::Distance offset;

    public:
        TrackingWheel(pros::Rotation *rotation_sensor,
                      units::Distance diameter,
                      units::Distance offset);

        units::Distance get_distance() override;
        units::Distance get_offset() override;
        void reset() override;
    };
}