// profiled_pid.cpp
#include "abclib/control/pid/profiled_pid.hpp"
#include <algorithm>
#include <cmath>

namespace abclib::control
{
    ProfiledPID::ProfiledPID(const ProfiledPIDConstants &constants)
        : pid_(constants.pid_constants),
          profile_(profiling::ProfileConstraints(constants.max_velocity, constants.max_acceleration)),
          current_setpoint_(0.0, 0.0),
          goal_(0.0, 0.0),
          position_tolerance_(constants.position_tolerance),
          velocity_tolerance_(constants.velocity_tolerance),
          last_measurement_(0.0),
          initialized_(false)
    {
    }

    double ProfiledPID::compute(double measurement, double goal, double dt)
    {
        profiling::ProfileState goal_state(goal, 0.0);
        return compute(measurement, goal_state, dt);
    }

    double ProfiledPID::compute(double measurement, const profiling::ProfileState &goal, double dt)
    {
        last_measurement_ = measurement;

        if (!initialized_)
        {
            current_setpoint_.position = measurement;
            current_setpoint_.velocity = 0.0;
            initialized_ = true;
        }

        goal_ = goal;

        current_setpoint_ = profile_.calculate(dt, current_setpoint_, goal_);

        double error = current_setpoint_.position - measurement;
        return pid_.compute(error, dt);
    }

    profiling::ProfileState ProfiledPID::get_setpoint() const
    {
        return current_setpoint_;
    }

    double ProfiledPID::get_setpoint_position() const
    {
        return current_setpoint_.position;
    }

    double ProfiledPID::get_setpoint_velocity() const
    {
        return current_setpoint_.velocity;
    }

    profiling::ProfileState ProfiledPID::get_goal() const
    {
        return goal_;
    }

    double ProfiledPID::get_error() const
    {
        return current_setpoint_.position - last_measurement_;
    }

    bool ProfiledPID::at_goal() const
    {
        if (!initialized_)
            return false;

        // Check if the setpoint has converged to the goal
        double setpoint_to_goal_error = std::abs(goal_.position - current_setpoint_.position);
        bool setpoint_at_goal = setpoint_to_goal_error <= position_tolerance_;

        // Check if the setpoint velocity has reached goal velocity (typically 0)
        double setpoint_velocity_error = std::abs(current_setpoint_.velocity - goal_.velocity);
        bool setpoint_stopped = setpoint_velocity_error <= velocity_tolerance_;

        // Check if measurement is tracking the setpoint
        double tracking_error = std::abs(current_setpoint_.position - last_measurement_);
        bool tracking_setpoint = tracking_error <= position_tolerance_;

        return setpoint_at_goal && setpoint_stopped && tracking_setpoint;
    }

    void ProfiledPID::reset()
    {
        pid_.reset();
        current_setpoint_ = profiling::ProfileState(0.0, 0.0);
        goal_ = profiling::ProfileState(0.0, 0.0);
        last_measurement_ = 0.0;
        initialized_ = false;
    }

    void ProfiledPID::reset(double initial_position)
    {
        reset(profiling::ProfileState(initial_position, 0.0));
    }

    void ProfiledPID::reset(const profiling::ProfileState &initial_state)
    {
        pid_.reset();
        current_setpoint_ = initial_state;
        goal_ = initial_state;
        last_measurement_ = initial_state.position;
        initialized_ = true;
    }

    void ProfiledPID::set_constraints(double max_vel, double max_accel)
    {
        profile_.set_constraints(profiling::ProfileConstraints(max_vel, max_accel));
    }

    void ProfiledPID::set_tolerance(double position_tolerance, double velocity_tolerance)
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