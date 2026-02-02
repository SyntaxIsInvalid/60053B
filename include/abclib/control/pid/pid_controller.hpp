#pragma once

#include "pid.hpp"
#include "abclib/filters/low_pass_filter.hpp"

namespace abclib::control
{
    /**
     * @brief PID Controller with optional derivative filtering
     * 
     * This is a convenience wrapper around the core PID class that adds
     * derivative filtering to reduce noise. For basic usage without filtering,
     * use the PID class directly.
     * 
     * Note on units: The controller is unitless and works with whatever units
     * you use for error. If error is in inches, derivative is in inches/sec.
     * If error is in degrees, derivative is in degrees/sec.
     */
    class PIDController
    {
    public:
        /**
         * @brief Construct PID controller with derivative filtering
         * @param constants PID gain constants
         * @param filter_config Optional filter configuration (enabled by default)
         *                      time_constant should be in seconds
         */
        explicit PIDController(
            const PIDConstants& constants,
            const filters::FilterConfig& filter_config = filters::FilterConfig{});

        /**
         * @brief Compute PID output with filtered derivative
         * @param error Current error value (unitless - e.g., inches, degrees, etc.)
         * @param dt Time step in seconds
         * @return Control output (same units as determined by your PID gains)
         */
        double compute(double error, double dt);

        /**
         * @brief Compute PID output with pre-calculated derivative
         * @param error Current error value (unitless - e.g., inches, degrees, etc.)
         * @param error_derivative Pre-calculated error derivative (error units per second)
         * @param dt Time step in seconds
         * @return Control output (same units as determined by your PID gains)
         */
        double compute(double error, double error_derivative, double dt);

        /**
         * @brief Reset controller state (integral, derivative, filter)
         */
        void reset();

        /**
         * @brief Get the underlying PID controller
         * @return Reference to PID instance
         */
        PID& get_pid();

        /**
         * @brief Get the underlying PID controller (const)
         * @return Const reference to PID instance
         */
        const PID& get_pid() const;

        /**
         * @brief Get the derivative filter
         * @return Reference to filter instance
         */
        filters::LowPassFilter& get_filter();

        /**
         * @brief Get the derivative filter (const)
         * @return Const reference to filter instance
         */
        const filters::LowPassFilter& get_filter() const;

        /**
         * @brief Enable or disable derivative filtering
         * @param enabled True to enable filtering, false to disable
         */
        void set_filter_enabled(bool enabled);

        /**
         * @brief Check if filtering is enabled
         * @return True if filtering is enabled
         */
        bool is_filter_enabled() const;

        /**
         * @brief Set PID constants
         * @param constants New PID constants
         */
        void set_constants(const PIDConstants& constants);

        /**
         * @brief Get PID constants
         * @return Current PID constants
         */
        PIDConstants get_constants() const;

    private:
        PID pid_;
        filters::LowPassFilter filter_;
        bool filter_enabled_;
        double prev_error_;
    };
}