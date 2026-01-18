#pragma once

#include "abclib/units/units.hpp"
#include "abclib/math/SE2.hpp"
#include <cmath>

namespace abclib::estimation
{
    class ArcLengthDifferentialDrive
    {
    public:
        /**
         * @brief Compute SE2 transformation from odometry measurements
         * 
         * @param delta_vertical Forward motion of tracking wheel
         * @param delta_horizontal Lateral motion of tracking wheel
         * @param delta_heading Change in heading
         * @param vertical_offset Distance from tracking center to vertical wheel
         * @param horizontal_offset Distance from tracking center to horizontal wheel
         * @return SE2 transformation in local robot frame
         */
        static math::SE2 compute_local_transformation(
            units::Length delta_vertical,
            units::Length delta_horizontal,
            units::Angle delta_heading,
            units::Length vertical_offset,
            units::Length horizontal_offset)
        {
            const double dtheta = delta_heading.to_radians();
            
            // Handle near-zero rotation case
            if (std::abs(dtheta) < 1e-6)
            {
                // Pure translation - no arc correction needed
                return math::SE2(
                    delta_horizontal.to_inches(),
                    delta_vertical.to_inches(),
                    0.0
                );
            }
            
            // Arc-length correction for wheels offset from tracking center
            // When robot rotates, offset wheels travel in arcs
            const double sin_half = std::sin(dtheta / 2.0);
            const double arc_factor = 2.0 * sin_half / dtheta;
            
            // Compute local displacement accounting for wheel offsets
            const double dx_local = arc_factor * (
                delta_horizontal.to_inches() + 
                horizontal_offset.to_inches() * dtheta
            );
            
            const double dy_local = arc_factor * (
                delta_vertical.to_inches() + 
                vertical_offset.to_inches() * dtheta
            );
            
            return math::SE2(dx_local, dy_local, dtheta);
        }
    };
}