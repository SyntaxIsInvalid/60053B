#pragma once

#include "abclib/units/units.hpp"
#include <cmath>

namespace abclib::estimation
{
    struct LocalMotion
    {
        units::Length x;
        units::Length y;
        units::Angle theta;
    };
    
    class ArcLengthDifferentialDrive
    {
    public:
        static LocalMotion compute_local_motion(
            units::Length delta_vertical,
            units::Length delta_horizontal,
            units::Angle delta_heading,
            units::Length vertical_offset,
            units::Length horizontal_offset)
        {
            LocalMotion motion;
            motion.theta = delta_heading;
            
            // Use .value() to get the SI value (radians)
            if (std::abs(delta_heading.value()) < 1e-6)
            {
                motion.x = delta_horizontal;
                motion.y = delta_vertical;
            }
            else
            {
                double sin_half = std::sin(delta_heading.value() / 2.0);
                double arc_factor = 2.0 * sin_half / delta_heading.value();
                
                // Convert to inches, do calculation, then convert back
                motion.x = units::Length::from_inches(
                    arc_factor * (delta_horizontal.to_inches() + 
                                  horizontal_offset.to_inches() * delta_heading.value()));
                motion.y = units::Length::from_inches(
                    arc_factor * (delta_vertical.to_inches() + 
                                  vertical_offset.to_inches() * delta_heading.value()));
            }
            
            return motion;
        }
    };
}