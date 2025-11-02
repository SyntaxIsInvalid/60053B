#pragma once

#include "abclib/units/units.hpp"
#include <cmath>

namespace abclib::estimation
{
    struct LocalMotion
    {
        units::Distance x;
        units::Distance y;
        units::Radians theta;
    };
    
    class ArcLengthDifferentialDrive
    {
    public:
        static LocalMotion compute_local_motion(
            units::Distance delta_vertical,
            units::Distance delta_horizontal,
            units::Radians delta_heading,
            units::Distance vertical_offset,
            units::Distance horizontal_offset)
        {
            LocalMotion motion;
            motion.theta = delta_heading;
            
            if (std::abs(delta_heading.value) < 1e-6)
            {
                motion.x = delta_horizontal;
                motion.y = delta_vertical;
            }
            else
            {
                double sin_half = std::sin(delta_heading.value / 2.0);
                double arc_factor = 2.0 * sin_half / delta_heading.value;
                
                motion.x = units::Distance::from_inches(
                    arc_factor * (delta_horizontal.inches + horizontal_offset.inches * delta_heading.value));
                motion.y = units::Distance::from_inches(
                    arc_factor * (delta_vertical.inches + vertical_offset.inches * delta_heading.value));
            }
            
            return motion;
        }
    };
}