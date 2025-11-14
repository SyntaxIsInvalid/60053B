// incremental_trapezoidal.hpp
#pragma once

#include <algorithm>
#include <cmath>
#include "abclib/units/units.hpp"

namespace abclib::profiling
{
    struct ProfileState
    {
        units::Length position;
        units::Velocity velocity;
        units::Acceleration acceleration;
        
        ProfileState() 
            : position(units::Length::from_inches(0.0)),
              velocity(units::Velocity::from_ips(0.0)),
              acceleration(units::Acceleration::from_mps2(0.0)) {}
              
        ProfileState(units::Length pos, units::Velocity vel, units::Acceleration accel = units::Acceleration::from_mps2(0.0)) 
            : position(pos), velocity(vel), acceleration(accel) {}
    };
    
    struct ProfileConstraints
    {
        units::Velocity max_velocity;
        units::Acceleration max_acceleration;
        
        ProfileConstraints(units::Velocity max_vel, units::Acceleration max_accel)
            : max_velocity(max_vel), max_acceleration(max_accel) {}
    };
    
    class IncrementalTrapezoidalProfile
    {
    public:
        IncrementalTrapezoidalProfile(const ProfileConstraints& constraints);
        
        ProfileState calculate(units::Time dt, const ProfileState& current, const ProfileState& goal);
        
        void set_constraints(const ProfileConstraints& constraints);
        ProfileConstraints get_constraints() const;
        
    private:
        ProfileConstraints constraints_;
        
        units::Velocity calculate_max_reachable_velocity(
            units::Length position, 
            units::Velocity velocity, 
            units::Length goal_position, 
            units::Acceleration max_accel) const;
    };
    
    inline IncrementalTrapezoidalProfile::IncrementalTrapezoidalProfile(const ProfileConstraints& constraints)
        : constraints_(constraints)
    {
    }
    
    inline ProfileState IncrementalTrapezoidalProfile::calculate(
        units::Time dt, 
        const ProfileState& current, 
        const ProfileState& goal)
    {
        // Assume goal.velocity = 0 for endpoint
        double direction = (goal.position.to_meters() >= current.position.to_meters()) ? 1.0 : -1.0;
        
        ProfileState next;
        
        units::Length position_error(goal.position - current.position);
        units::Velocity velocity_error(goal.velocity - current.velocity);
        
        // If already at goal, stay there
        if (std::abs(position_error.to_meters()) < 1e-6 && std::abs(velocity_error.to_mps()) < 1e-6)
        {
            next.position = goal.position;
            next.velocity = goal.velocity;
            next.acceleration = units::Acceleration::from_mps2(0.0);
            return next;
        }
        
        units::Length distance_to_goal(units::Qabs(position_error));
        
        // Calculate the maximum velocity we can have and still stop at the goal
        // Using: v_f² = v_i² + 2*a*d, solving for v_i when v_f = 0 (goal velocity)
        units::Velocity max_reachable_velocity = calculate_max_reachable_velocity(
            current.position, current.velocity, goal.position, constraints_.max_acceleration
        );
        
        // Clamp to profile's max velocity
        if (max_reachable_velocity.to_mps() > constraints_.max_velocity.to_mps())
        {
            max_reachable_velocity = constraints_.max_velocity;
        }
        
        // Determine if we should accelerate or decelerate
        units::Acceleration acceleration;
        if (std::abs(current.velocity.to_mps()) < max_reachable_velocity.to_mps() - 1e-6)
        {
            // Accelerate toward max velocity
            acceleration = units::Acceleration::from_mps2(
                direction * constraints_.max_acceleration.to_mps2()
            );
        }
        else
        {
            // Decelerate to be able to stop at goal
            acceleration = units::Acceleration::from_mps2(
                -direction * constraints_.max_acceleration.to_mps2()
            );
        }
        
        // Calculate next velocity: v = v0 + a*t
        double next_vel_mps = current.velocity.to_mps() + 
                             acceleration.to_mps2() * dt.to_seconds();
        next.velocity = units::Velocity::from_mps(next_vel_mps);
        
        // Clamp velocity to constraints
        next.velocity = units::Velocity::from_mps(
            std::clamp(next.velocity.to_mps(), 
                      -constraints_.max_velocity.to_mps(), 
                      constraints_.max_velocity.to_mps())
        );
        
        // Enforce direction (don't go backwards when going forward)
        if (direction > 0)
        {
            next.velocity = units::Velocity::from_mps(
                std::max(0.0, next.velocity.to_mps())
            );
        }
        else
        {
            next.velocity = units::Velocity::from_mps(
                std::min(0.0, next.velocity.to_mps())
            );
        }
        
        // Calculate next position using average velocity over dt
        // This is more accurate than using just next.velocity
        units::Velocity avg_velocity((current.velocity + next.velocity) / 2.0);
        double next_pos_m = current.position.to_meters() + 
                           avg_velocity.to_mps() * dt.to_seconds();
        next.position = units::Length::from_meters(next_pos_m);
        
        // Store the actual acceleration used
        next.acceleration = acceleration;
        
        // Check if we've reached or passed the goal
        if ((direction > 0 && next.position.to_meters() >= goal.position.to_meters()) ||
            (direction < 0 && next.position.to_meters() <= goal.position.to_meters()))
        {
            // Clamp to goal - we've arrived
            next.position = goal.position;
            next.velocity = goal.velocity;  // Should be 0
            next.acceleration = units::Acceleration::from_mps2(0.0);
        }
        
        return next;
    }
    
    inline units::Velocity IncrementalTrapezoidalProfile::calculate_max_reachable_velocity(
        units::Length position, 
        units::Velocity velocity, 
        units::Length goal_position, 
        units::Acceleration max_accel) const
    {
        units::Length distance_to_goal(units::Qabs(goal_position - position));
        
        // v² = v_goal² + 2*a*d
        // Solving for v: v = sqrt(v_goal² + 2*a*d)
        double vel_squared = velocity.to_mps() * velocity.to_mps() + 
                            2.0 * max_accel.to_mps2() * distance_to_goal.to_meters();
        
        if (vel_squared < 0)
        {
            return units::Velocity::from_mps(0.0);
        }
        
        return units::Velocity::from_mps(std::sqrt(vel_squared));
    }
    
    inline void IncrementalTrapezoidalProfile::set_constraints(const ProfileConstraints& constraints)
    {
        constraints_ = constraints;
    }
    
    inline ProfileConstraints IncrementalTrapezoidalProfile::get_constraints() const
    {
        return constraints_;
    }

} // namespace abclib::profiling