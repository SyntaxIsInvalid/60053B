#include "abclib/filters/low_pass_filter.hpp"

namespace abclib::filters
{
    LowPassFilter::LowPassFilter(double time_constant)
        : time_constant_(time_constant), 
          alpha_(0.0),
          filtered_value_(0.0),
          enabled_(true),
          use_fixed_alpha_(false)
    {
    }

    LowPassFilter::LowPassFilter(const FilterConfig& config)
        : time_constant_(config.time_constant),
          alpha_(0.0),
          filtered_value_(0.0),
          enabled_(config.enabled),
          use_fixed_alpha_(false)
    {
    }

    LowPassFilter LowPassFilter::from_alpha(double alpha)
    {
        LowPassFilter filter(0.0);  // dummy time_constant
        filter.alpha_ = alpha;
        filter.use_fixed_alpha_ = true;
        return filter;
    }

    LowPassFilter LowPassFilter::from_alpha(double alpha, const FilterConfig& config)
    {
        LowPassFilter filter(0.0);  // dummy time_constant
        filter.alpha_ = alpha;
        filter.enabled_ = config.enabled;
        filter.use_fixed_alpha_ = true;
        return filter;
    }

    double LowPassFilter::update(double value, double dt)
    {
        if (!enabled_)
        {
            filtered_value_ = value;
            return value;
        }

        double alpha = use_fixed_alpha_ ? alpha_ : dt / (time_constant_ + dt);
        
        // Exponential moving average
        filtered_value_ = alpha * value + (1.0 - alpha) * filtered_value_;
        
        return filtered_value_;
    }

    void LowPassFilter::reset()
    {
        filtered_value_ = 0.0;
    }

    void LowPassFilter::reset(double initial_value)
    {
        filtered_value_ = initial_value;
    }

    double LowPassFilter::get_filtered_value() const
    {
        return filtered_value_;
    }

    void LowPassFilter::set_time_constant(double time_constant)
    {
        time_constant_ = time_constant;
        use_fixed_alpha_ = false;
    }

    double LowPassFilter::get_time_constant() const
    {
        return time_constant_;
    }

    void LowPassFilter::set_alpha(double alpha)
    {
        alpha_ = alpha;
        use_fixed_alpha_ = true;
    }

    double LowPassFilter::get_alpha() const
    {
        return alpha_;
    }

    void LowPassFilter::set_enabled(bool enabled)
    {
        enabled_ = enabled;
    }

    bool LowPassFilter::is_enabled() const
    {
        return enabled_;
    }
}