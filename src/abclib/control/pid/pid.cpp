#include "pid.hpp"
#include <algorithm>
#include <cmath>

namespace abclib::control
{
    PID::PID(const PIDConstants& constants) 
        : constants_(constants) 
    {
    }

    PID::PID(const PIDConstants& constants,
             const filters::FilterConfig& filter_config)
        : constants_(constants),
          derivative_filter_(filter_config)
    {
    }

    double PID::compute(double error, double dt)
    {
        // Proportional term
        last_p_term_ = constants_.kP * error;

        // Anti-windup: reset on zero crossing
        if (constants_.integral.reset_on_zero_crossing && 
            ((error > 0) != (prev_error_ > 0) || error == 0))
        {
            integral_ = 0;
        }

        // Anti-windup: range-limited accumulation
        if (std::abs(error) <= constants_.integral.accumulation_range)
        {
            integral_ += error * dt;
            integral_ = std::clamp(integral_, -constants_.integral.max, 
                                  constants_.integral.max);
        }
        
        last_i_term_ = constants_.kI * integral_;

        // Derivative term (calculate from error history)
        double derivative = (error - prev_error_) / dt;
        
        // Apply filtering if enabled
        if (derivative_filter_ && derivative_filter_->is_enabled())
        {
            derivative = derivative_filter_->update(derivative, dt);
        }
        
        last_d_term_ = constants_.kD * derivative;

        prev_error_ = error;

        return last_p_term_ + last_i_term_ + last_d_term_;
    }

    double PID::compute(double error, double dt, double error_derivative)
    {
        // Proportional term
        last_p_term_ = constants_.kP * error;

        // Anti-windup: reset on zero crossing
        if (constants_.integral.reset_on_zero_crossing && 
            ((error > 0) != (prev_error_ > 0) || error == 0))
        {
            integral_ = 0;
        }

        // Anti-windup: range-limited accumulation
        if (std::abs(error) <= constants_.integral.accumulation_range)
        {
            integral_ += error * dt;
            integral_ = std::clamp(integral_, -constants_.integral.max, 
                                  constants_.integral.max);
        }

        last_i_term_ = constants_.kI * integral_;

        // Derivative term (use provided derivative)
        double derivative = error_derivative;
        
        // Apply filtering if enabled
        if (derivative_filter_ && derivative_filter_->is_enabled())
        {
            derivative = derivative_filter_->update(derivative, dt);
        }
        
        last_d_term_ = constants_.kD * derivative;

        prev_error_ = error;  // Still store for zero crossing detection

        return last_p_term_ + last_i_term_ + last_d_term_;
    }

    double PID::compute(double error, double dt, double kP, double kI, double kD)
    {
        // Proportional term
        last_p_term_ = kP * error;

        // Anti-windup: reset on zero crossing
        if (constants_.integral.reset_on_zero_crossing && 
            ((error > 0) != (prev_error_ > 0) || error == 0))
        {
            integral_ = 0;
        }

        // Anti-windup: range-limited accumulation
        if (std::abs(error) <= constants_.integral.accumulation_range)
        {
            integral_ += error * dt;
            integral_ = std::clamp(integral_, -constants_.integral.max, 
                                  constants_.integral.max);
        }
        
        last_i_term_ = kI * integral_;

        // Derivative term
        double derivative = (error - prev_error_) / dt;
        
        // Apply filtering if enabled
        if (derivative_filter_ && derivative_filter_->is_enabled())
        {
            derivative = derivative_filter_->update(derivative, dt);
        }
        
        last_d_term_ = kD * derivative;

        prev_error_ = error;

        return last_p_term_ + last_i_term_ + last_d_term_;
    }

    double PID::get_p_term() const
    {
        return last_p_term_;
    }

    double PID::get_i_term() const
    {
        return last_i_term_;
    }

    double PID::get_d_term() const
    {
        return last_d_term_;
    }

    double PID::get_integral() const
    {
        return integral_;
    }

    void PID::reset()
    {
        integral_ = 0.0;
        prev_error_ = 0.0;
        last_p_term_ = 0.0;
        last_i_term_ = 0.0;
        last_d_term_ = 0.0;
        
        if (derivative_filter_)
        {
            derivative_filter_->reset();
        }
    }

    void PID::set_constants(const PIDConstants& new_constants)
    {
        constants_ = new_constants;
    }

    PIDConstants PID::get_constants() const
    {
        return constants_;
    }

    void PID::set_filter_enabled(bool enabled)
    {
        if (derivative_filter_)
        {
            derivative_filter_->set_enabled(enabled);
        }
    }

    bool PID::is_filter_enabled() const
    {
        return derivative_filter_ && derivative_filter_->is_enabled();
    }

    bool PID::has_filter() const
    {
        return derivative_filter_.has_value();
    }

    filters::LowPassFilter* PID::get_filter()
    {
        return derivative_filter_ ? &(*derivative_filter_) : nullptr;
    }

    const filters::LowPassFilter* PID::get_filter() const
    {
        return derivative_filter_ ? &(*derivative_filter_) : nullptr;
    }
}