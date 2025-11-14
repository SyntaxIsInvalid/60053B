// profiled_pid.cpp
#include "abclib/control/profiled_pid.hpp"
#include <algorithm>
#include <cmath>

namespace abclib::control
{
    template<typename QuantityType>
    ProfiledPID<QuantityType>::ProfiledPID(const ProfiledPIDConstants<QuantityType> &constants)
        : pid_(constants.pid_constants),
          profile_(ConstraintsType(constants.max_velocity, constants.max_acceleration)),
          current_setpoint_(),
          goal_(),
          position_tolerance_(constants.position_tolerance),
          velocity_tolerance_(constants.velocity_tolerance),
          last_measurement_(QuantityType(0.0)),
          initialized_(false)
    {
    }

    template<typename QuantityType>
    double ProfiledPID<QuantityType>::compute(QuantityType measurement, QuantityType goal, units::Time dt)
    {
        StateType goal_state(goal, VelocityType(0.0));
        return compute(measurement, goal_state, dt);
    }

    template<typename QuantityType>
    double ProfiledPID<QuantityType>::compute(QuantityType measurement, const StateType &goal, units::Time dt)
    {
        last_measurement_ = measurement;

        if (!initialized_)
        {
            current_setpoint_.position = measurement;
            current_setpoint_.velocity = VelocityType(0.0);
            current_setpoint_.acceleration = AccelerationType(0.0);
            initialized_ = true;
        }

        goal_ = goal;

        current_setpoint_ = profile_.calculate(dt, current_setpoint_, goal_);

        auto error = current_setpoint_.position - measurement;
        // PID operates on raw doubles (using value() to get SI units)
        return pid_.compute(error.value(), dt.to_seconds());
    }

    template<typename QuantityType>
    typename ProfiledPID<QuantityType>::StateType ProfiledPID<QuantityType>::get_setpoint() const
    {
        return current_setpoint_;
    }

    template<typename QuantityType>
    QuantityType ProfiledPID<QuantityType>::get_setpoint_position() const
    {
        return current_setpoint_.position;
    }

    template<typename QuantityType>
    typename ProfiledPID<QuantityType>::VelocityType ProfiledPID<QuantityType>::get_setpoint_velocity() const
    {
        return current_setpoint_.velocity;
    }

    template<typename QuantityType>
    typename ProfiledPID<QuantityType>::StateType ProfiledPID<QuantityType>::get_goal() const
    {
        return goal_;
    }

    template<typename QuantityType>
    QuantityType ProfiledPID<QuantityType>::get_error() const
    {
        return QuantityType(current_setpoint_.position.value() - last_measurement_.value());
    }

    template<typename QuantityType>
    bool ProfiledPID<QuantityType>::at_goal() const
    {
        if (!initialized_)
            return false;

        auto setpoint_to_goal_error = units::Qabs(goal_.position - current_setpoint_.position);
        bool setpoint_at_goal = setpoint_to_goal_error.value() <= position_tolerance_.value();

        auto setpoint_velocity_error = units::Qabs(current_setpoint_.velocity - goal_.velocity);
        bool setpoint_stopped = setpoint_velocity_error.value() <= velocity_tolerance_.value();

        auto tracking_error = units::Qabs(current_setpoint_.position - last_measurement_);
        bool tracking_setpoint = tracking_error.value() <= position_tolerance_.value();

        return setpoint_at_goal && setpoint_stopped && tracking_setpoint;
    }

    template<typename QuantityType>
    void ProfiledPID<QuantityType>::reset()
    {
        pid_.reset();
        current_setpoint_ = StateType();
        goal_ = StateType();
        last_measurement_ = QuantityType(0.0);
        initialized_ = false;
    }

    template<typename QuantityType>
    void ProfiledPID<QuantityType>::reset(QuantityType initial_position)
    {
        reset(StateType(initial_position, VelocityType(0.0)));
    }

    template<typename QuantityType>
    void ProfiledPID<QuantityType>::reset(const StateType &initial_state)
    {
        pid_.reset();
        current_setpoint_ = initial_state;
        goal_ = initial_state;
        last_measurement_ = initial_state.position;
        initialized_ = true;
    }

    template<typename QuantityType>
    void ProfiledPID<QuantityType>::set_constraints(VelocityType max_vel, AccelerationType max_accel)
    {
        profile_.set_constraints(ConstraintsType(max_vel, max_accel));
    }

    template<typename QuantityType>
    void ProfiledPID<QuantityType>::set_tolerance(QuantityType position_tolerance, VelocityType velocity_tolerance)
    {
        position_tolerance_ = position_tolerance;
        velocity_tolerance_ = velocity_tolerance;
    }

    template<typename QuantityType>
    PID &ProfiledPID<QuantityType>::get_pid()
    {
        return pid_;
    }

    template<typename QuantityType>
    const PID &ProfiledPID<QuantityType>::get_pid() const
    {
        return pid_;
    }

    // Explicit instantiations
    template class ProfiledPID<units::Length>;
    template class ProfiledPID<units::Angle>;

} // namespace abclib::control