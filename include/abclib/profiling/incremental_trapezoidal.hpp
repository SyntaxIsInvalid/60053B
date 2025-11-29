// incremental_trapezoidal.hpp
#pragma once

#include <algorithm>
#include <cmath>

namespace abclib::profiling
{
    struct ProfileState
    {
        double position;
        double velocity;
        double acceleration;
        
        ProfileState() : position(0.0), velocity(0.0), acceleration(0.0) {}
        ProfileState(double pos, double vel, double accel = 0.0) 
            : position(pos), velocity(vel), acceleration(accel) {}
    };
    
    struct ProfileConstraints
    {
        double max_velocity;
        double max_acceleration;
        
        ProfileConstraints(double max_vel, double max_accel)
            : max_velocity(max_vel), max_acceleration(max_accel) {}
    };
    
    class IncrementalTrapezoidalProfile
    {
    public:
        IncrementalTrapezoidalProfile(const ProfileConstraints& constraints);
        
        ProfileState calculate(double dt, const ProfileState& current, const ProfileState& goal);
        
        void set_constraints(const ProfileConstraints& constraints);
        ProfileConstraints get_constraints() const;
        
    private:
        ProfileConstraints constraints_;
        
        double calculate_max_reachable_velocity(double position, double velocity, 
                                                double goal_position, double max_accel) const;
    };
    
    inline IncrementalTrapezoidalProfile::IncrementalTrapezoidalProfile(const ProfileConstraints& constraints)
        : constraints_(constraints)
    {
    }
    
    inline ProfileState IncrementalTrapezoidalProfile::calculate(
        double dt, 
        const ProfileState& current, 
        const ProfileState& goal)
    {
        // Assume goal.velocity = 0 for endpoint
        double direction = (goal.position >= current.position) ? 1.0 : -1.0;
        
        ProfileState next;
        
        double position_error = goal.position - current.position;
        double velocity_error = goal.velocity - current.velocity;
        
        // If already at goal, stay there
        if (std::abs(position_error) < 1e-6 && std::abs(velocity_error) < 1e-6)
        {
            next.position = goal.position;
            next.velocity = goal.velocity;
            next.acceleration = 0.0;
            return next;
        }
        
        double distance_to_goal = std::abs(position_error);
        
        // Calculate the maximum velocity we can have and still stop at the goal
        // Using: v_f² = v_i² + 2*a*d, solving for v_i when v_f = 0 (goal velocity)
        double max_reachable_velocity = std::sqrt(
            goal.velocity * goal.velocity + 2.0 * constraints_.max_acceleration * distance_to_goal
        );
        
        // Clamp to profile's max velocity
        max_reachable_velocity = std::min(max_reachable_velocity, constraints_.max_velocity);
        
        // Determine if we should accelerate or decelerate
        double acceleration;
        if (std::abs(current.velocity) < max_reachable_velocity - 1e-6)
        {
            // Accelerate toward max velocity
            acceleration = direction * constraints_.max_acceleration;
        }
        else
        {
            // Decelerate to be able to stop at goal
            acceleration = -direction * constraints_.max_acceleration;
        }
        
        // Calculate next velocity
        next.velocity = current.velocity + acceleration * dt;
        
        // Clamp velocity to constraints
        next.velocity = std::clamp(next.velocity, 
                                   -constraints_.max_velocity, 
                                   constraints_.max_velocity);
        
        // Enforce direction (don't go backwards when going forward)
        if (direction > 0)
        {
            next.velocity = std::max(0.0, next.velocity);
        }
        else
        {
            next.velocity = std::min(0.0, next.velocity);
        }
        
        // Calculate next position using average velocity over dt
        // This is more accurate than using just next.velocity
        double avg_velocity = (current.velocity + next.velocity) / 2.0;
        next.position = current.position + avg_velocity * dt;
        
        // Store the actual acceleration used
        next.acceleration = acceleration;
        
        // Check if we've reached or passed the goal
        if ((direction > 0 && next.position >= goal.position) ||
            (direction < 0 && next.position <= goal.position))
        {
            // Clamp to goal - we've arrived
            next.position = goal.position;
            next.velocity = goal.velocity;  // Should be 0
            next.acceleration = 0.0;
        }
        
        return next;
    }
    
    inline double IncrementalTrapezoidalProfile::calculate_max_reachable_velocity(
        double position, 
        double velocity, 
        double goal_position, 
        double max_accel) const
    {
        double distance_to_goal = std::abs(goal_position - position);
        double velocity_squared = velocity * velocity;
        double max_vel_squared = velocity_squared + 2.0 * max_accel * distance_to_goal;
        
        if (max_vel_squared < 0)
        {
            return 0.0;
        }
        
        return std::sqrt(max_vel_squared);
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