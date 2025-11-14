// incremental_trapezoidal.hpp
#pragma once

#include <algorithm>
#include <cmath>
#include "abclib/units/units.hpp"

namespace abclib::profiling
{
    // Template-based ProfileState that works with any quantity type
    template<typename QuantityType>
    struct ProfileState
    {
        QuantityType position;
        decltype(QuantityType() / units::Time()) velocity;  // Automatic velocity type
        decltype(QuantityType() / units::Time() / units::Time()) acceleration;  // Automatic acceleration type
        
        ProfileState() 
            : position(QuantityType(0.0)),
              velocity(decltype(velocity)(0.0)),
              acceleration(decltype(acceleration)(0.0)) {}
              
        ProfileState(QuantityType pos, decltype(velocity) vel, decltype(acceleration) accel = decltype(acceleration)(0.0)) 
            : position(pos), velocity(vel), acceleration(accel) {}
    };
    
    template<typename QuantityType>
    struct ProfileConstraints
    {
        decltype(QuantityType() / units::Time()) max_velocity;
        decltype(QuantityType() / units::Time() / units::Time()) max_acceleration;
        
        using VelocityType = decltype(QuantityType() / units::Time());
        using AccelerationType = decltype(QuantityType() / units::Time() / units::Time());
        
        ProfileConstraints(VelocityType max_vel, AccelerationType max_accel)
            : max_velocity(max_vel), max_acceleration(max_accel) {}
    };
    
    template<typename QuantityType>
    class IncrementalTrapezoidalProfile
    {
    public:
        using VelocityType = decltype(QuantityType() / units::Time());
        using AccelerationType = decltype(QuantityType() / units::Time() / units::Time());
        using StateType = ProfileState<QuantityType>;
        using ConstraintsType = ProfileConstraints<QuantityType>;
        
        IncrementalTrapezoidalProfile(const ConstraintsType& constraints);
        
        StateType calculate(units::Time dt, const StateType& current, const StateType& goal);
        
        void set_constraints(const ConstraintsType& constraints);
        ConstraintsType get_constraints() const;
        
    private:
        ConstraintsType constraints_;
        
        VelocityType calculate_max_reachable_velocity(
            QuantityType position, 
            VelocityType velocity, 
            QuantityType goal_position, 
            AccelerationType max_accel) const;
    };
    
    template<typename QuantityType>
    inline IncrementalTrapezoidalProfile<QuantityType>::IncrementalTrapezoidalProfile(
        const ConstraintsType& constraints)
        : constraints_(constraints)
    {
    }
    
    template<typename QuantityType>
    inline typename IncrementalTrapezoidalProfile<QuantityType>::StateType 
    IncrementalTrapezoidalProfile<QuantityType>::calculate(
        units::Time dt, 
        const StateType& current, 
        const StateType& goal)
    {
        // Determine direction using value() to get underlying SI units
        double direction = (goal.position.value() >= current.position.value()) ? 1.0 : -1.0;
        
        StateType next;
        
        auto position_error = goal.position - current.position;
        auto velocity_error = goal.velocity - current.velocity;
        
        // If already at goal, stay there
        if (std::abs(position_error.value()) < 1e-9 && std::abs(velocity_error.value()) < 1e-9)
        {
            next.position = goal.position;
            next.velocity = goal.velocity;
            next.acceleration = AccelerationType(0.0);
            return next;
        }
        
        auto distance_to_goal = units::Qabs(position_error);
        
        // Calculate the maximum velocity we can have and still stop at the goal
        // Using: v_f² = v_i² + 2*a*d, solving for v_i when v_f = 0
        VelocityType max_reachable_velocity = calculate_max_reachable_velocity(
            current.position, current.velocity, goal.position, constraints_.max_acceleration
        );
        
        // Clamp to profile's max velocity
        if (max_reachable_velocity.value() > constraints_.max_velocity.value())
        {
            max_reachable_velocity = constraints_.max_velocity;
        }
        
        // Determine if we should accelerate or decelerate
        AccelerationType acceleration;
        if (std::abs(current.velocity.value()) < max_reachable_velocity.value() - 1e-9)
        {
            // Accelerate toward max velocity
            acceleration = AccelerationType(direction * constraints_.max_acceleration.value());
        }
        else
        {
            // Decelerate to be able to stop at goal
            acceleration = AccelerationType(-direction * constraints_.max_acceleration.value());
        }
        
        // Calculate next velocity: v = v0 + a*t
        next.velocity = VelocityType(
            current.velocity.value() + acceleration.value() * dt.to_seconds()
        );
        
        // Clamp velocity to constraints
        next.velocity = VelocityType(
            std::clamp(next.velocity.value(), 
                      -constraints_.max_velocity.value(), 
                      constraints_.max_velocity.value())
        );
        
        // Enforce direction (don't go backwards when going forward)
        if (direction > 0)
        {
            next.velocity = VelocityType(std::max(0.0, next.velocity.value()));
        }
        else
        {
            next.velocity = VelocityType(std::min(0.0, next.velocity.value()));
        }
        
        // Calculate next position using average velocity over dt
        auto avg_velocity = (current.velocity + next.velocity) / 2.0;
        next.position = QuantityType(
            current.position.value() + avg_velocity.value() * dt.to_seconds()
        );
        
        // Store the actual acceleration used
        next.acceleration = acceleration;
        
        // Check if we've reached or passed the goal
        if ((direction > 0 && next.position.value() >= goal.position.value()) ||
            (direction < 0 && next.position.value() <= goal.position.value()))
        {
            // Clamp to goal - we've arrived
            next.position = goal.position;
            next.velocity = goal.velocity;
            next.acceleration = AccelerationType(0.0);
        }
        
        return next;
    }
    
    template<typename QuantityType>
    inline typename IncrementalTrapezoidalProfile<QuantityType>::VelocityType 
    IncrementalTrapezoidalProfile<QuantityType>::calculate_max_reachable_velocity(
        QuantityType position, 
        VelocityType velocity, 
        QuantityType goal_position, 
        AccelerationType max_accel) const
    {
        auto distance_to_goal = units::Qabs(goal_position - position);
        
        // v² = v_goal² + 2*a*d
        double vel_squared = velocity.value() * velocity.value() + 
                            2.0 * max_accel.value() * distance_to_goal.value();
        
        if (vel_squared < 0)
        {
            return VelocityType(0.0);
        }
        
        return VelocityType(std::sqrt(vel_squared));
    }
    
    template<typename QuantityType>
    inline void IncrementalTrapezoidalProfile<QuantityType>::set_constraints(
        const ConstraintsType& constraints)
    {
        constraints_ = constraints;
    }
    
    template<typename QuantityType>
    inline typename IncrementalTrapezoidalProfile<QuantityType>::ConstraintsType 
    IncrementalTrapezoidalProfile<QuantityType>::get_constraints() const
    {
        return constraints_;
    }

} // namespace abclib::profiling