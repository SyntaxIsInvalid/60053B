#pragma once

#include "abclib/units/units.hpp"

namespace abclib::profiling
{
    /**
     * @brief Abstract interface for velocity profiles
     */
    class IMotionProfile
    {
    public:
        virtual ~IMotionProfile() = default;

        // Core query methods
        virtual units::Length get_position(units::Time time) const = 0;
        virtual units::Velocity get_velocity(units::Time time) const = 0;
        virtual units::Acceleration get_acceleration(units::Time time) const = 0;
        
        virtual units::Time get_total_time() const = 0;
        virtual units::Length get_total_distance() const = 0;
    };

} // namespace abclib::profiling