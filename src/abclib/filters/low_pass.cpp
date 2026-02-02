#include "abclib/filters/low_pass_filter.hpp"

namespace abclib::filters
{
    LowPassFilter::LowPassFilter(double time_constant)
        : time_constant_(time_constant), filtered_value_(0.0)
    {
    }

    LowPassFilter::LowPassFilter(const FilterConfig& config)
        : LowPassFilter(config.time_constant)
    {
    }

    double LowPassFilter::update(double value, double dt)
    {
        // Calculate filter coefficient: alpha = dt / (tau + dt)
        // Both dt and time_constant_ are in seconds
        // alpha is dimensionless (0 to 1), representing how much to trust new vs old value
        double alpha = dt / (time_constant_ + dt);
        
        // Exponential moving average
        // value and filtered_value_ are unitless (inherit units from input signal)
        filtered_value_ += alpha * (value - filtered_value_);
        
        return filtered_value_;
    }

    void LowPassFilter::reset()
    {
        filtered_value_ = 0.0;
    }

    double LowPassFilter::get_filtered_value() const
    {
        return filtered_value_;
    }

    void LowPassFilter::set_time_constant(double time_constant)
    {
        time_constant_ = time_constant;  // in seconds
    }

    double LowPassFilter::get_time_constant() const
    {
        return time_constant_;  // in seconds
    }
}