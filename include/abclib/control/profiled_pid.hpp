// profiled_pid.hpp
#pragma once

#include "abclib/control/pid.hpp"
#include "abclib/units/units.hpp"
#include "abclib/profiling/incremental_trapezoidal.hpp"
#include <memory>

namespace abclib::control
{
    template<typename QuantityType>
    struct ProfiledPIDConstants
    {
        PIDConstants pid_constants;
        typename profiling::ProfileConstraints<QuantityType>::VelocityType max_velocity;
        typename profiling::ProfileConstraints<QuantityType>::AccelerationType max_acceleration;
        QuantityType position_tolerance;
        typename profiling::ProfileConstraints<QuantityType>::VelocityType velocity_tolerance;
    };

    template<typename QuantityType>
    class ProfiledPID
    {
    public:
        using VelocityType = decltype(QuantityType() / units::Time());
        using AccelerationType = decltype(QuantityType() / units::Time() / units::Time());
        using StateType = profiling::ProfileState<QuantityType>;
        using ConstraintsType = profiling::ProfileConstraints<QuantityType>;
        
    private:
        PID pid_;
        profiling::IncrementalTrapezoidalProfile<QuantityType> profile_;
        StateType current_setpoint_;
        StateType goal_;
        QuantityType position_tolerance_;
        VelocityType velocity_tolerance_;
        QuantityType last_measurement_;
        bool initialized_;

    public:
        explicit ProfiledPID(const ProfiledPIDConstants<QuantityType>& constants);
        
        double compute(QuantityType measurement, QuantityType goal, units::Time dt);
        double compute(QuantityType measurement, const StateType& goal, units::Time dt);
        
        StateType get_setpoint() const;
        QuantityType get_setpoint_position() const;
        VelocityType get_setpoint_velocity() const;
        StateType get_goal() const;
        QuantityType get_error() const;
        bool at_goal() const;
        
        void reset();
        void reset(QuantityType initial_position);
        void reset(const StateType& initial_state);
        void set_constraints(VelocityType max_vel, AccelerationType max_accel);
        void set_tolerance(QuantityType position_tolerance, VelocityType velocity_tolerance);
        
        PID& get_pid();
        const PID& get_pid() const;
    };
    
} // namespace abclib::control