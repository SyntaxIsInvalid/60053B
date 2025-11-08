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
        double max_velocity;
        double max_acceleration;
        double position_tolerance;
        double velocity_tolerance;
    };

    class ProfiledPID
    {
    private:
        PID pid_;
        profiling::IncrementalTrapezoidalProfile profile_;
        profiling::ProfileState current_setpoint_;
        profiling::ProfileState goal_;
        double position_tolerance_;
        double velocity_tolerance_;
        double last_measurement_;
        bool initialized_;

    public:
        explicit ProfiledPID(const ProfiledPIDConstants& constants);
        
        double compute(double measurement, double goal, double dt);
        double compute(double measurement, const profiling::ProfileState& goal, double dt);
        
        profiling::ProfileState get_setpoint() const;
        double get_setpoint_position() const;
        double get_setpoint_velocity() const;
        profiling::ProfileState get_goal() const;
        double get_error() const;
        bool at_goal() const;
        
        void reset();
        void reset(double initial_position);
        void reset(const profiling::ProfileState& initial_state);
        void set_constraints(double max_vel, double max_accel);
        void set_tolerance(double position_tolerance, double velocity_tolerance);
        
        PID& get_pid();
        const PID& get_pid() const;
    };
    
} // namespace abclib::control