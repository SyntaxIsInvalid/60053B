#pragma once
#include "abclib/units/units.hpp"

namespace abclib::hardware
{
    class ITrackingWheel
    {
    public:
        virtual ~ITrackingWheel() = default;
        virtual units::Length get_distance() = 0;
        virtual units::Length get_offset() = 0;
        virtual void reset() = 0;
    };
}