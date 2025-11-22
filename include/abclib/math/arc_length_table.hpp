#pragma once

#include "abclib/math/GL15.hpp"
#include "abclib/math/lerp.hpp"
#include <array>
#include <functional>

namespace abclib::math
{
    /// Number of samples in arc length lookup table
    inline constexpr std::size_t ARC_LENGTH_TABLE_SIZE = 64;

    /**
     * @brief Precomputed arc length lookup table for fast s <-> u conversion
     * 
     * Stores cumulative arc length at uniform u intervals.
     * Uses binary search + linear interpolation for queries.
     */
    struct ArcLengthTable
    {
        /// Cumulative arc length at each sample point
        /// samples[i] = arc length from u=0 to u=(i / (SIZE-1))
        std::array<double, ARC_LENGTH_TABLE_SIZE> samples{};
        
        /// Total arc length (cached for convenience)
        double total_length = 0.0;

        /**
         * @brief Build table by integrating ||p'(u)|| over [0,1]
         * 
         * @param speed_func Function returning ||p'(u)|| at given u
         */
        void build(const std::function<double(double)>& speed_func)
        {
            constexpr std::size_t N = ARC_LENGTH_TABLE_SIZE;
            constexpr double du = 1.0 / static_cast<double>(N - 1);
            
            samples[0] = 0.0;
            double cumulative = 0.0;
            
            for (std::size_t i = 1; i < N; ++i)
            {
                double u_prev = static_cast<double>(i - 1) * du;
                double u_curr = static_cast<double>(i) * du;
                
                cumulative += integrate_gauss15(speed_func, u_prev, u_curr);
                samples[i] = cumulative;
            }
            
            total_length = cumulative;
        }

        /**
         * @brief Convert parameter u to arc length s
         * 
         * @param u Parameter in [0, 1]
         * @return Arc length from start to u
         */
        double u_to_arc_length(double u) const
        {
            if (u <= 0.0) return 0.0;
            if (u >= 1.0) return total_length;
            
            constexpr std::size_t N = ARC_LENGTH_TABLE_SIZE;
            constexpr double du = 1.0 / static_cast<double>(N - 1);
            
            // Find which interval u falls into
            double idx_float = u / du;
            std::size_t idx = static_cast<std::size_t>(idx_float);
            
            if (idx >= N - 1) return total_length;
            
            // Interpolate within interval
            double t = idx_float - static_cast<double>(idx);
            return lerp(samples[idx], samples[idx + 1], t);
        }

        /**
         * @brief Convert arc length s to parameter u
         * 
         * @param s Arc length from start
         * @return Parameter u in [0, 1]
         */
        double arc_length_to_u(double s) const
        {
            if (s <= 0.0) return 0.0;
            if (s >= total_length) return 1.0;
            
            constexpr std::size_t N = ARC_LENGTH_TABLE_SIZE;
            constexpr double du = 1.0 / static_cast<double>(N - 1);
            
            // Binary search for interval containing s
            std::size_t lo = 0;
            std::size_t hi = N - 1;
            
            while (hi - lo > 1)
            {
                std::size_t mid = (lo + hi) / 2;
                if (samples[mid] <= s)
                    lo = mid;
                else
                    hi = mid;
            }
            
            // Interpolate within interval [lo, hi]
            double s_lo = samples[lo];
            double s_hi = samples[hi];
            double t = (s - s_lo) / (s_hi - s_lo);
            
            double u_lo = static_cast<double>(lo) * du;
            return u_lo + t * du;
        }
    };

} // namespace abclib::math