#pragma once
#include "abclib/units/units.hpp"

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

    class PID
    {
    private:
        PIDConstants constants;
        double integral = 0.0;
        double prev_error = 0.0;

        // Last computed terms (for telemetry)
        double last_p_term = 0.0;
        double last_i_term = 0.0;
        double last_d_term = 0.0;

    public:
        explicit PID(const PIDConstants &constants);

        double compute(double error, double dt);

        double compute(double error, double dt, double error_derivative);

        // Compute with custom gains (for gain scheduling)
        double compute(double error, double dt, double kP, double kI, double kD);

        double get_p_term() const;
        double get_i_term() const;
        double get_d_term() const;
        double get_integral() const;

        void reset();
        void set_constants(const PIDConstants &new_constants);
        PIDConstants get_constants() const;
    };
}