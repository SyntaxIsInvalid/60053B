#pragma once

namespace abclib::filters
{
    /**
     * @brief Configuration for low-pass filter
     */
    struct FilterConfig
    {
        double time_constant = 0.015; // Time constant in seconds (tau) - controls smoothing aggressiveness
        bool enabled = true;          // Enable/disable filtering
    };

    /**
     * @brief Exponential moving average low-pass filter
     *
     * Uses a first-order low-pass filter to smooth noisy signals:
     * filtered += alpha * (new_value - filtered)
     * where alpha = dt / (tau + dt)
     *
     * The time constant (tau) controls how aggressively the filter smooths:
     * - Smaller tau (e.g., 0.005s): Fast response, less smoothing, more noise
     * - Larger tau (e.g., 0.050s): Slow response, more smoothing, cleaner signal
     *
     * Rule of thumb: tau ≈ 1-3× your loop period for good balance
     *
     * Commonly used for filtering derivative terms in PID controllers,
     * but can be used for any noisy signal.
     */
    class LowPassFilter
    {
    public:
        /**
         * @brief Construct a new Low Pass Filter
         * @param time_constant Filter time constant (tau) in seconds
         *                      Default 0.015s is good for 50-100Hz loops
         */
        explicit LowPassFilter(double time_constant = 0.015);

        /**
         * @brief Construct from config
         * @param config Filter configuration
         */
        explicit LowPassFilter(const FilterConfig &config);

        /**
         * @brief Update filter with new value
         * @param value New measurement (unitless - same units as your signal)
         * @param dt Time step in seconds
         * @return Filtered value (same units as input)
         */
        double update(double value, double dt);

        /**
         * @brief Reset filter state
         */
        void reset();
        void reset(double initial_value); // Overload for non-zero initialization
        void set_alpha(double alpha);
        double get_alpha() const;
        /**
         * @brief Get current filtered value
         * @return Current filtered value (same units as input)
         */
        double get_filtered_value() const;

        /**
         * @brief Set time constant
         * @param time_constant New time constant in seconds
         */
        void set_time_constant(double time_constant);

        /**
         * @brief Get time constant
         * @return Current time constant in seconds
         */
        double get_time_constant() const;

        void set_enabled(bool enabled);
        bool is_enabled() const;

        static LowPassFilter from_alpha(double alpha);
        static LowPassFilter from_alpha(double alpha, const FilterConfig &config);

    private:
        double time_constant_;  // Filter time constant (tau) in seconds
        double filtered_value_; // Current filtered value (unitless - inherits units from input)
        bool enabled_;
        double alpha_;
        bool use_fixed_alpha_;
    };
}