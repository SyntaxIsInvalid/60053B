// trapezoidal.hpp
#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include "abclib/units/units.hpp"

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
         * @param max_acceleration Maximum acceleration constraint (in inches/s²)
         */
        TrapezoidalProfile(units::Length total_distance,
                           units::Velocity max_velocity,
                           units::Acceleration max_acceleration);

        // Query methods
        units::Length get_position(units::Time time) const;
        units::Velocity get_velocity(units::Time time) const;
        units::Acceleration get_acceleration(units::Time time) const;

        units::Time get_total_time() const { return total_time_; }
        units::Velocity get_max_velocity() const { return max_velocity_; }
        units::Acceleration get_max_acceleration() const { return max_acceleration_; }
        units::Length get_total_distance() const { return total_distance_; }

        bool is_trapezoidal() const { return is_trapezoidal_; }

    private:
        void calculate_profile_parameters();

        // Member Variables
        units::Length total_distance_;
        units::Velocity max_velocity_;
        units::Acceleration max_acceleration_;

        // Profile timing
        units::Time accel_time_;  // t1: end of acceleration phase
        units::Time cruise_time_; // t2: end of cruise phase
        units::Time total_time_;  // t3: total profile time

        // Profile distances
        units::Length accel_distance_;  // distance covered during acceleration
        units::Length cruise_distance_; // distance covered during cruise
        units::Length decel_distance_;  // distance covered during deceleration

        // Profile type
        bool is_trapezoidal_;           // true if reaches max_velocity
        units::Velocity peak_velocity_; // actual peak velocity achieved
    };

    // Implementation
    inline TrapezoidalProfile::TrapezoidalProfile(
        units::Length total_distance,
        units::Velocity max_velocity,
        units::Acceleration max_acceleration)
        : total_distance_(total_distance),
          max_velocity_(max_velocity),
          max_acceleration_(max_acceleration)
    {
        if (total_distance.to_inches() <= 0)
        {
            throw std::invalid_argument("Total distance must be positive");
        }
        if (max_velocity.to_ips() <= 0)
        {
            throw std::invalid_argument("Max velocity must be positive");
        }
        if (max_acceleration.to_mps2() <= 0)
        {
            throw std::invalid_argument("Max acceleration must be positive");
        }

        calculate_profile_parameters();
    }

    inline units::Length TrapezoidalProfile::get_position(units::Time time) const
    {
        double t = std::clamp(time.to_seconds(), 0.0, total_time_.to_seconds());

        if (t <= accel_time_.to_seconds())
        {
            // Acceleration phase: s = 0.5 * a * t²
            double pos_meters = 0.5 * max_acceleration_.to_mps2() * t * t;
            return units::Length::from_meters(pos_meters);
        }
        else if (t <= cruise_time_.to_seconds())
        {
            // Cruise phase: s = s1 + v_max * (t - t1)
            double cruise_elapsed = t - accel_time_.to_seconds();
            double pos_meters = accel_distance_.to_meters() +
                                peak_velocity_.to_mps() * cruise_elapsed;
            return units::Length::from_meters(pos_meters);
        }
        else
        {
            // Deceleration phase
            double decel_elapsed = t - cruise_time_.to_seconds();
            double pos_meters = accel_distance_.to_meters() +
                                cruise_distance_.to_meters() +
                                peak_velocity_.to_mps() * decel_elapsed -
                                0.5 * max_acceleration_.to_mps2() * decel_elapsed * decel_elapsed;
            return units::Length::from_meters(pos_meters);
        }
    }

    inline units::Velocity TrapezoidalProfile::get_velocity(units::Time time) const
    {
        double t = std::clamp(time.to_seconds(), 0.0, total_time_.to_seconds());

        if (t <= accel_time_.to_seconds())
        {
            // Acceleration phase: v = a * t
            double vel_mps = max_acceleration_.to_mps2() * t;
            return units::Velocity::from_mps(vel_mps);
        }
        else if (t <= cruise_time_.to_seconds())
        {
            // Cruise phase: v = v_max (constant)
            return peak_velocity_;
        }
        else
        {
            // Deceleration phase: v = v_max - a * (t - t2)
            double decel_elapsed = t - cruise_time_.to_seconds();
            double vel_mps = peak_velocity_.to_mps() -
                             max_acceleration_.to_mps2() * decel_elapsed;
            return units::Velocity::from_mps(vel_mps);
        }
    }

    inline units::Acceleration TrapezoidalProfile::get_acceleration(units::Time time) const
    {
        double t = std::clamp(time.to_seconds(), 0.0, total_time_.to_seconds());

        if (t >= total_time_.to_seconds())
        {
            return units::Acceleration::from_mps2(0.0);
        }

        if (t <= accel_time_.to_seconds())
        {
            return max_acceleration_;
        }
        else if (t <= cruise_time_.to_seconds())
        {
            return units::Acceleration::from_mps2(0.0);
        }
        else
        {
            return units::Acceleration::from_mps2(-max_acceleration_.to_mps2());
        }
    }

    inline void TrapezoidalProfile::calculate_profile_parameters()
    {
        // Check if we can reach max velocity
        // Using v² = 2ad, so d = v²/(2a)
        auto vel_squared = units::Qsq(max_velocity_);
        auto distance_to_reach_max_vel = vel_squared / (max_acceleration_ * 2.0);

        if (distance_to_reach_max_vel.to_meters() <= total_distance_.to_meters())
        {
            // Trapezoidal profile - reaches max velocity
            is_trapezoidal_ = true;
            peak_velocity_ = max_velocity_;

            // t = v / a
            accel_time_ = units::Time::from_seconds(
                max_velocity_.to_mps() / max_acceleration_.to_mps2());

            // d = 0.5 * a * t²
            double accel_time_s = accel_time_.to_seconds();
            double accel_dist_m = 0.5 * max_acceleration_.to_mps2() *
                                  accel_time_s * accel_time_s;
            accel_distance_ = units::Length::from_meters(accel_dist_m);
            decel_distance_ = accel_distance_;

            cruise_distance_ = units::Length::from_meters(
                total_distance_.to_meters() - accel_distance_.to_meters() -
                decel_distance_.to_meters());

            cruise_time_ = units::Time::from_seconds(
                accel_time_s + (cruise_distance_.to_meters() / max_velocity_.to_mps()));
            total_time_ = units::Time::from_seconds(
                cruise_time_.to_seconds() + accel_time_s);
        }
        else
        {
            // Triangular profile - never reaches max velocity
            is_trapezoidal_ = false;
            cruise_distance_ = units::Length::from_meters(0);

            // Peak velocity: v_peak = sqrt(a * d)
            // Using v² = 2ad, so v = sqrt(2 * a * d)
            double peak_vel_mps = std::sqrt(2.0 * max_acceleration_.to_mps2() *
                                            total_distance_.to_meters() / 2.0);
            peak_velocity_ = units::Velocity::from_mps(peak_vel_mps);

            // t = v / a
            accel_time_ = units::Time::from_seconds(
                peak_velocity_.to_mps() / max_acceleration_.to_mps2());

            accel_distance_ = units::Length::from_meters(total_distance_.to_meters() / 2.0);
            decel_distance_ = accel_distance_;

            cruise_time_ = accel_time_;
            total_time_ = units::Time::from_seconds(2.0 * accel_time_.to_seconds());
        }
    }

} // namespace abclib::profiling