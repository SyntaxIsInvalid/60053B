// profiled_pid.cpp
#include "abclib/control/profiled_pid.hpp"
#include <algorithm>
#include <cmath>

namespace abclib::control
{
    ProfiledPID::ProfiledPID(const ProfiledPIDConstants &constants)
        : pid_(constants.pid_constants),
          profile_(profiling::ProfileConstraints(constants.max_velocity, constants.max_acceleration)),
          current_setpoint_(),
          goal_(),
          position_tolerance_(constants.position_tolerance),
          velocity_tolerance_(constants.velocity_tolerance),
          last_measurement_(units::Length::from_inches(0.0)),
          initialized_(false)
    {
    }

    double ProfiledPID::compute(units::Length measurement, units::Length goal, units::Time dt)
    {
        profiling::ProfileState goal_state(goal, units::Velocity::from_ips(0.0));
        return compute(measurement, goal_state, dt);
    }

    double ProfiledPID::compute(units::Length measurement, const profiling::ProfileState &goal, units::Time dt)
    {
        last_measurement_ = measurement;

        if (!initialized_)
        {
            current_setpoint_.position = measurement;
            current_setpoint_.velocity = units::Velocity::from_ips(0.0);
            current_setpoint_.acceleration = units::Acceleration::from_mps2(0.0);
            initialized_ = true;
        }

        goal_ = goal;

        current_setpoint_ = profile_.calculate(dt, current_setpoint_, goal_);

        units::Length error(current_setpoint_.position - measurement);
        // PID operates on raw doubles (in inches per your convention)
        return pid_.compute(error.to_inches(), dt.to_seconds());
    }

    profiling::ProfileState ProfiledPID::get_setpoint() const
    {
        return current_setpoint_;
    }

    units::Length ProfiledPID::get_setpoint_position() const
    {
        return current_setpoint_.position;
    }

    units::Velocity ProfiledPID::get_setpoint_velocity() const
    {
        return current_setpoint_.velocity;
    }

    profiling::ProfileState ProfiledPID::get_goal() const
    {
        return goal_;
    }

    units::Length ProfiledPID::get_error() const
    {
        return units::Length(current_setpoint_.position - last_measurement_);
    }

    bool ProfiledPID::at_goal() const
    {
        if (!initialized_)
            return false;

        // Check if the setpoint has converged to the goal
        units::Length setpoint_to_goal_error(units::Qabs(goal_.position - current_setpoint_.position));
        bool setpoint_at_goal = setpoint_to_goal_error.to_inches() <= position_tolerance_.to_inches();

        // Check if the setpoint velocity has reached goal velocity (typically 0)
        units::Velocity setpoint_velocity_error(units::Qabs(current_setpoint_.velocity - goal_.velocity));
        bool setpoint_stopped = setpoint_velocity_error.to_ips() <= velocity_tolerance_.to_ips();

        // Check if measurement is tracking the setpoint
        units::Length tracking_error(units::Qabs(current_setpoint_.position - last_measurement_));
        bool tracking_setpoint = tracking_error.to_inches() <= position_tolerance_.to_inches();

        return setpoint_at_goal && setpoint_stopped && tracking_setpoint;
    }

    void ProfiledPID::reset()
    {
        pid_.reset();
        current_setpoint_ = profiling::ProfileState();
        goal_ = profiling::ProfileState();
        last_measurement_ = units::Length::from_inches(0.0);
        initialized_ = false;
    }

    void ProfiledPID::reset(units::Length initial_position)
    {
        reset(profiling::ProfileState(initial_position, units::Velocity::from_ips(0.0)));
    }

    void ProfiledPID::reset(const profiling::ProfileState &initial_state)
    {
        pid_.reset();
        current_setpoint_ = initial_state;
        goal_ = initial_state;
        last_measurement_ = initial_state.position;
        initialized_ = true;
    }

    void ProfiledPID::set_constraints(units::Velocity max_vel, units::Acceleration max_accel)
    {
        profile_.set_constraints(profiling::ProfileConstraints(max_vel, max_accel));
    }

    void ProfiledPID::set_tolerance(units::Length position_tolerance, units::Velocity velocity_tolerance)
    {
        position_tolerance_ = position_tolerance;
        velocity_tolerance_ = velocity_tolerance;
    }

    PID &ProfiledPID::get_pid()
    {
        return pid_;
    }

    const PID &ProfiledPID::get_pid() const
    {
        return pid_;
    }

} // namespace abclib::control