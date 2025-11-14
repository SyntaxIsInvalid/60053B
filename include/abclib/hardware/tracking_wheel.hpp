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
        units::Length diameter;
        units::Length radius;
        units::Length offset;

    public:
        TrackingWheel(pros::Rotation *rotation_sensor,
                      units::Length diameter,
                      units::Length offset);

        units::Length get_distance() override;
        units::Length get_offset() override;
        void reset() override;
    };
}