// trapezoidal.hpp
#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include "abclib/units/units.hpp"  // ADD THIS

namespace abclib::profiling
{
    /**
     * @brief Trapezoidal velocity profile for path following
     * 
     * Generates a smooth velocity profile with constant acceleration/deceleration
     * phases and an optional constant velocity cruise phase.
     * 
     * All inputs and outputs use typed units for safety.
     */
    class TrapezoidalProfile
    {
    public:
        /**
         * @brief Constructs a trapezoidal velocity profile.
         *
         * @param total_distance Total distance to travel (arc length)
         * @param max_velocity Maximum velocity constraint
         * @param max_acceleration Maximum acceleration constraint
         */
        TrapezoidalProfile(units::Distance total_distance, 
                          units::BodyLinearVelocity max_velocity, 
                          double max_acceleration);  // inches/s² (dimensionless for now)

        // Query methods
        units::Distance get_position(units::Time time) const;
        units::BodyLinearVelocity get_velocity(units::Time time) const;
        double get_acceleration(units::Time time) const;  // inches/s²
        
        units::Time get_total_time() const { return total_time_; }
        units::BodyLinearVelocity get_max_velocity() const { return max_velocity_; }
        double get_max_acceleration() const { return max_acceleration_; }
        units::Distance get_total_distance() const { return total_distance_; }

        bool is_trapezoidal() const { return is_trapezoidal_; }

    private:
        void calculate_profile_parameters();

        // Member Variables
        units::Distance total_distance_;
        units::BodyLinearVelocity max_velocity_;
        double max_acceleration_;  // inches/s²

        // Profile timing
        units::Time accel_time_;   // t1: end of acceleration phase
        units::Time cruise_time_;  // t2: end of cruise phase
        units::Time total_time_;   // t3: total profile time

        // Profile distances
        units::Distance accel_distance_;   // distance covered during acceleration
        units::Distance cruise_distance_;  // distance covered during cruise
        units::Distance decel_distance_;   // distance covered during deceleration

        // Profile type
        bool is_trapezoidal_;  // true if reaches max_velocity
        units::BodyLinearVelocity peak_velocity_;  // actual peak velocity achieved
    };

    // Implementation
    inline TrapezoidalProfile::TrapezoidalProfile(
        units::Distance total_distance,
        units::BodyLinearVelocity max_velocity,
        double max_acceleration)
        : total_distance_(total_distance),
          max_velocity_(max_velocity),
          max_acceleration_(max_acceleration)
    {
        if (total_distance.inches <= 0)
        {
            throw std::invalid_argument("Total distance must be positive");
        }
        if (max_velocity.inches_per_sec <= 0)
        {
            throw std::invalid_argument("Max velocity must be positive");
        }
        if (max_acceleration <= 0)
        {
            throw std::invalid_argument("Max acceleration must be positive");
        }

        calculate_profile_parameters();
    }

    inline units::Distance TrapezoidalProfile::get_position(units::Time time) const
    {
        double t = std::clamp(time.seconds, 0.0, total_time_.seconds);

        if (t <= accel_time_.seconds)
        {
            // Acceleration phase: s = 0.5 * a * t²
            return units::Distance::from_inches(
                0.5 * max_acceleration_ * t * t
            );
        }
        else if (t <= cruise_time_.seconds)
        {
            // Cruise phase: s = s1 + v_max * (t - t1)
            double cruise_elapsed = t - accel_time_.seconds;
            return units::Distance::from_inches(
                accel_distance_.inches + peak_velocity_.inches_per_sec * cruise_elapsed
            );
        }
        else
        {
            // Deceleration phase
            double decel_elapsed = t - cruise_time_.seconds;
            return units::Distance::from_inches(
                accel_distance_.inches + cruise_distance_.inches +
                peak_velocity_.inches_per_sec * decel_elapsed -
                0.5 * max_acceleration_ * decel_elapsed * decel_elapsed
            );
        }
    }

    inline units::BodyLinearVelocity TrapezoidalProfile::get_velocity(units::Time time) const
    {
        double t = std::clamp(time.seconds, 0.0, total_time_.seconds);

        if (t <= accel_time_.seconds)
        {
            // Acceleration phase: v = a * t
            return units::BodyLinearVelocity(max_acceleration_ * t);
        }
        else if (t <= cruise_time_.seconds)
        {
            // Cruise phase: v = v_max (constant)
            return peak_velocity_;
        }
        else
        {
            // Deceleration phase: v = v_max - a * (t - t2)
            double decel_elapsed = t - cruise_time_.seconds;
            return units::BodyLinearVelocity(
                peak_velocity_.inches_per_sec - max_acceleration_ * decel_elapsed
            );
        }
    }

    inline double TrapezoidalProfile::get_acceleration(units::Time time) const
    {
        double t = std::clamp(time.seconds, 0.0, total_time_.seconds);

        if (t >= total_time_.seconds)
        {
            return 0.0;
        }

        if (t <= accel_time_.seconds)
        {
            return max_acceleration_;
        }
        else if (t <= cruise_time_.seconds)
        {
            return 0.0;
        }
        else
        {
            return -max_acceleration_;
        }
    }

    inline void TrapezoidalProfile::calculate_profile_parameters()
    {
        // Check if we can reach max velocity
        double distance_to_reach_max_vel = 
            max_velocity_.inches_per_sec * max_velocity_.inches_per_sec / max_acceleration_;

        if (distance_to_reach_max_vel <= total_distance_.inches)
        {
            // Trapezoidal profile - reaches max velocity
            is_trapezoidal_ = true;
            peak_velocity_ = max_velocity_;

            double accel_time_raw = max_velocity_.inches_per_sec / max_acceleration_;
            accel_time_ = units::Time::from_seconds(accel_time_raw);
            
            double accel_dist = 0.5 * max_acceleration_ * accel_time_raw * accel_time_raw;
            accel_distance_ = units::Distance::from_inches(accel_dist);
            decel_distance_ = accel_distance_;
            
            cruise_distance_ = units::Distance::from_inches(
                total_distance_.inches - accel_distance_.inches - decel_distance_.inches
            );

            cruise_time_ = units::Time::from_seconds(
                accel_time_raw + (cruise_distance_.inches / max_velocity_.inches_per_sec)
            );
            total_time_ = units::Time::from_seconds(
                cruise_time_.seconds + accel_time_raw
            );
        }
        else
        {
            // Triangular profile - never reaches max velocity
            is_trapezoidal_ = false;
            cruise_distance_ = units::Distance::from_inches(0);

            // Peak velocity: v_peak = sqrt(a * total_distance)
            double peak_vel = std::sqrt(max_acceleration_ * total_distance_.inches);
            peak_velocity_ = units::BodyLinearVelocity(peak_vel);

            double accel_time_raw = peak_vel / max_acceleration_;
            accel_time_ = units::Time::from_seconds(accel_time_raw);
            
            accel_distance_ = units::Distance::from_inches(total_distance_.inches / 2.0);
            decel_distance_ = accel_distance_;

            cruise_time_ = accel_time_;
            total_time_ = units::Time::from_seconds(2.0 * accel_time_raw);
        }
    }

} // namespace abclib::profiling