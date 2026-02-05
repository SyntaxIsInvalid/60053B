#pragma once
#include "abclib/units/units.hpp"
#include "abclib/filters/low_pass_filter.hpp"
#include <optional>

namespace abclib::control
{
    struct IntegralConfig
    {
        double max = 1e20;
        double accumulation_range = 1e20;
        bool reset_on_zero_crossing = false;
    };

    struct PIDConstants
    {
        double kP;
        double kI;
        double kD;
        IntegralConfig integral = {};
    };

    // Helper functions for calculating max integral
    inline double calculate_max_integral_for_output_limit(
        double max_output,  // In whatever units your output is
        double kI)          // Must match: output_units / (error_units * time)
    {
        return max_output / kI;
    }
    
    // Convenience overload for voltage specifically
    inline double calculate_max_integral_for_voltage(
        units::Voltage max_voltage,
        double kI)
    {
        return max_voltage.to_volts() / kI;
    }

    /**
     * @brief PID Controller with optional derivative filtering
     * 
     * Note on units: The controller is unitless and works with whatever units
     * you use for error. If error is in inches, derivative is in inches/sec.
     * If error is in degrees, derivative is in degrees/sec.
     */
    class PID
    {
    private:
        PIDConstants constants_;
        double integral_ = 0.0;
        double prev_error_ = 0.0;

        // Last computed terms (for telemetry)
        double last_p_term_ = 0.0;
        double last_i_term_ = 0.0;
        double last_d_term_ = 0.0;

        // Optional derivative filtering
        std::optional<filters::LowPassFilter> derivative_filter_;

    public:
        /**
         * @brief Construct PID controller without derivative filtering
         * @param constants PID gain constants
         */
        explicit PID(const PIDConstants& constants);

        /**
         * @brief Construct PID controller with derivative filtering
         * @param constants PID gain constants
         * @param filter_config Filter configuration for derivative term
         *                      time_constant should be in seconds
         */
        PID(const PIDConstants& constants,
            const filters::FilterConfig& filter_config);

        /**
         * @brief Compute PID output (calculates derivative internally)
         * @param error Current error value (unitless - e.g., inches, degrees, etc.)
         * @param dt Time step in seconds
         * @return Control output (same units as determined by your PID gains)
         */
        double compute(double error, double dt);

        /**
         * @brief Compute PID output with pre-calculated derivative
         * @param error Current error value
         * @param dt Time step in seconds
         * @param error_derivative Pre-calculated error derivative (error units per second)
         * @return Control output
         */
        double compute(double error, double dt, double error_derivative);

        /**
         * @brief Compute with custom gains (for gain scheduling)
         * @param error Current error value
         * @param dt Time step in seconds
         * @param kP Proportional gain
         * @param kI Integral gain
         * @param kD Derivative gain
         * @return Control output
         */
        double compute(double error, double dt, double kP, double kI, double kD);

        // Telemetry accessors
        double get_p_term() const;
        double get_i_term() const;
        double get_d_term() const;
        double get_integral() const;

        /**
         * @brief Reset controller state (integral, derivative history, filter)
         */
        void reset();

        /**
         * @brief Set PID constants
         * @param new_constants New PID constants
         */
        void set_constants(const PIDConstants& new_constants);

        /**
         * @brief Get current PID constants
         * @return Current PID constants
         */
        PIDConstants get_constants() const;

        /**
         * @brief Enable or disable derivative filtering
         * 
         * Only has effect if controller was constructed with a filter.
         * If no filter exists, this does nothing.
         * 
         * @param enabled True to enable filtering, false to disable
         */
        void set_filter_enabled(bool enabled);

        /**
         * @brief Check if filtering is enabled and active
         * @return True if a filter exists and is enabled
         */
        bool is_filter_enabled() const;

        /**
         * @brief Check if this controller has a filter (regardless of enabled state)
         * @return True if controller was constructed with a filter
         */
        bool has_filter() const;

        /**
         * @brief Get the derivative filter (if it exists)
         * @return Pointer to filter, or nullptr if no filter
         */
        filters::LowPassFilter* get_filter();

        /**
         * @brief Get the derivative filter (const)
         * @return Const pointer to filter, or nullptr if no filter
         */
        const filters::LowPassFilter* get_filter() const;
    };
}