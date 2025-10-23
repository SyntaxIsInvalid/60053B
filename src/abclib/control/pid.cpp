#include "pid.hpp"
#include <algorithm>
namespace abclib::control
{
    PID::PID(const PIDConstants &constants) : constants(constants) {}

    double PID::compute(double error, double dt)
    {
        // Proportional term
        last_p_term = constants.kP * error;

        // Integral term
        integral += error * dt;
        integral = std::clamp(integral, -constants.max_integral, constants.max_integral);
        last_i_term = constants.kI * integral;

        // Derivative term
        double derivative = (error - prev_error) / dt;
        last_d_term = constants.kD * derivative;

        prev_error = error;

        return last_p_term + last_i_term + last_d_term;
    }

    double PID::compute(double error, double dt, double kP, double kI, double kD)
    {
        // Proportional term
        last_p_term = kP * error;

        // Integral term
        integral += error * dt;
        integral = std::clamp(integral, -constants.max_integral, constants.max_integral);
        last_i_term = kI * integral;

        // Derivative term
        double derivative = (error - prev_error) / dt;
        last_d_term = kD * derivative;

        prev_error = error;

        return last_p_term + last_i_term + last_d_term;
    }

    double PID::get_p_term() const
    {
        return last_p_term;
    }

    double PID::get_i_term() const
    {
        return last_i_term;
    }

    double PID::get_d_term() const
    {
        return last_d_term;
    }

    double PID::get_integral() const
    {
        return integral;
    }

    void PID::reset()
    {
        integral = 0.0;
        prev_error = 0.0;
        last_p_term = 0.0;
        last_i_term = 0.0;
        last_d_term = 0.0;
    }

    void PID::set_constants(const PIDConstants &new_constants)
    {
        constants = new_constants;
    }

    PIDConstants PID::get_constants() const
    {
        return constants;
    }
}