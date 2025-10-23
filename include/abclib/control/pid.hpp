#pragma once
namespace abclib::control
{
    struct PIDConstants
    {
        double kP;
        double kI;
        double kD;
        double max_integral;
    };

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