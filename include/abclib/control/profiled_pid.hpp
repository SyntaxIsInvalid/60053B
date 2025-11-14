// profiled_pid.hpp
#pragma once

#include "abclib/control/pid.hpp"
#include "abclib/units/units.hpp"
#include "abclib/profiling/incremental_trapezoidal.hpp"
#include <memory>

namespace abclib::control
{
    struct ProfiledPIDConstants
    {
        PIDConstants pid_constants;
        units::Velocity max_velocity;
        units::Acceleration max_acceleration;
        units::Length position_tolerance;
        units::Velocity velocity_tolerance;
    };

    class ProfiledPID
    {
    private:
        PID pid_;
        profiling::IncrementalTrapezoidalProfile profile_;
        profiling::ProfileState current_setpoint_;
        profiling::ProfileState goal_;
        units::Length position_tolerance_;
        units::Velocity velocity_tolerance_;
        units::Length last_measurement_;
        bool initialized_;

    public:
        explicit ProfiledPID(const ProfiledPIDConstants& constants);
        
        // Compute using units::Length for measurement and goal
        double compute(units::Length measurement, units::Length goal, units::Time dt);
        double compute(units::Length measurement, const profiling::ProfileState& goal, units::Time dt);
        
        profiling::ProfileState get_setpoint() const;
        units::Length get_setpoint_position() const;
        units::Velocity get_setpoint_velocity() const;
        profiling::ProfileState get_goal() const;
        units::Length get_error() const;
        bool at_goal() const;
        
        void reset();
        void reset(units::Length initial_position);
        void reset(const profiling::ProfileState& initial_state);
        void set_constraints(units::Velocity max_vel, units::Acceleration max_accel);
        void set_tolerance(units::Length position_tolerance, units::Velocity velocity_tolerance);
        
        PID& get_pid();
        const PID& get_pid() const;
    };
    
} // namespace abclib::control