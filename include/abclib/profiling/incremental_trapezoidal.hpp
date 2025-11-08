// incremental_trapezoidal.hpp
#pragma once

#include <algorithm>
#include <cmath>
#include "abclib/units/units.hpp"

namespace abclib::profiling
{
    struct ProfileState
    {
        double position;
        double velocity;
        
        ProfileState() : position(0.0), velocity(0.0) {}
        ProfileState(double pos, double vel) : position(pos), velocity(vel) {}
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
        
        bool should_flip_acceleration(const ProfileState& current, const ProfileState& goal) const;
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
        double direction = (goal.position >= current.position) ? 1.0 : -1.0;
        
        ProfileState next = current;
        
        double position_error = goal.position - current.position;
        double velocity_error = goal.velocity - current.velocity;
        
        if (std::abs(position_error) < 1e-6 && std::abs(velocity_error) < 1e-6)
        {
            return goal;
        }
        
        double max_reachable_velocity = calculate_max_reachable_velocity(
            current.position, current.velocity, goal.position, constraints_.max_acceleration);
        
        max_reachable_velocity = std::min(max_reachable_velocity, constraints_.max_velocity);
        
        double desired_velocity = direction * max_reachable_velocity;
        
        double acceleration = 0.0;
        if (std::abs(current.velocity) < max_reachable_velocity)
        {
            acceleration = direction * constraints_.max_acceleration;
        }
        else
        {
            double stopping_distance = (current.velocity * current.velocity) / (2.0 * constraints_.max_acceleration);
            double distance_to_goal = std::abs(goal.position - current.position);
            
            if (stopping_distance >= distance_to_goal - 1e-6)
            {
                acceleration = -direction * constraints_.max_acceleration;
            }
            else
            {
                acceleration = 0.0;
            }
        }
        
        next.velocity = current.velocity + acceleration * dt;
        
        next.velocity = std::clamp(next.velocity, 
                                   -constraints_.max_velocity, 
                                   constraints_.max_velocity);
        
        if (direction > 0)
        {
            next.velocity = std::max(0.0, next.velocity);
        }
        else
        {
            next.velocity = std::min(0.0, next.velocity);
        }
        
        double distance_to_goal = std::abs(goal.position - current.position);
        double stopping_distance = (next.velocity * next.velocity) / (2.0 * constraints_.max_acceleration);
        
        if (stopping_distance >= distance_to_goal)
        {
            next.position = goal.position;
            next.velocity = goal.velocity;
            return next;
        }
        
        next.position = current.position + next.velocity * dt;
        
        if ((direction > 0 && next.position >= goal.position) ||
            (direction < 0 && next.position <= goal.position))
        {
            next.position = goal.position;
            next.velocity = goal.velocity;
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
    
    inline bool IncrementalTrapezoidalProfile::should_flip_acceleration(
        const ProfileState& current, 
        const ProfileState& goal) const
    {
        double distance_to_goal = goal.position - current.position;
        double stopping_distance = (current.velocity * current.velocity) / (2.0 * constraints_.max_acceleration);
        
        return (distance_to_goal > 0 && stopping_distance > distance_to_goal) ||
               (distance_to_goal < 0 && stopping_distance < distance_to_goal);
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