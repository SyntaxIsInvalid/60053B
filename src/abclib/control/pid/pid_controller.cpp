#include "abclib/control/pid/pid_controller.hpp"

namespace abclib::control
{
    PIDController::PIDController(
        const PIDConstants& constants,
        const filters::FilterConfig& filter_config)
        : pid_(constants),
          filter_(filter_config),
          filter_enabled_(filter_config.enabled),
          prev_error_(0.0)
    {
    }

    double PIDController::compute(double error, double dt)
    {
        // Calculate raw derivative (error units per second)
        double error_derivative = (error - prev_error_) / dt;
        
        // Apply filtering if enabled
        if (filter_enabled_)
        {
            error_derivative = filter_.update(error_derivative, dt);
        }
        
        // Store for next iteration
        prev_error_ = error;
        
        // Compute PID with filtered derivative
        return pid_.compute(error, dt, error_derivative);
    }

    double PIDController::compute(double error, double error_derivative, double dt)
    {
        // Apply filtering to pre-calculated derivative if enabled
        if (filter_enabled_)
        {
            error_derivative = filter_.update(error_derivative, dt);
        }
        
        prev_error_ = error;
        
        // Compute PID with filtered derivative
        return pid_.compute(error, dt, error_derivative);
    }

    void PIDController::reset()
    {
        pid_.reset();
        filter_.reset();
        prev_error_ = 0.0;
    }

    PID& PIDController::get_pid()
    {
        return pid_;
    }

    const PID& PIDController::get_pid() const
    {
        return pid_;
    }

    filters::LowPassFilter& PIDController::get_filter()
    {
        return filter_;
    }

    const filters::LowPassFilter& PIDController::get_filter() const
    {
        return filter_;
    }

    void PIDController::set_filter_enabled(bool enabled)
    {
        filter_enabled_ = enabled;
    }

    bool PIDController::is_filter_enabled() const
    {
        return filter_enabled_;
    }

    void PIDController::set_constants(const PIDConstants& constants)
    {
        pid_.set_constants(constants);
    }

    PIDConstants PIDController::get_constants() const
    {
        return pid_.get_constants();
    }
}