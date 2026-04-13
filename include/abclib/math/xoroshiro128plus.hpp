#pragma once

#include <cstdint>
#include <cmath>
#include <algorithm>

namespace abclib::math
{
    /**
     * @brief Xoshiro128+ PRNG
     * 
     * 128-bit state, passes all BigCrush tests.
     * The "+" scrambler is fastest and sufficient for MCL/simulation use.
     * NOT cryptographically secure.
     * 
     * Reference: Blackman & Vigna, 2018
     * https://prng.di.unimi.it/xoshiro128plus.c
     */
    struct Xoshiro128Plus
    {
    private:
        uint32_t s[4];

        // Rotate left helper
        inline uint32_t rotl(uint32_t x, int k) const
        {
            return (x << k) | (x >> (32 - k));
        }

        /**
         * @brief SplitMix32 used exclusively for seeding
         * 
         * Ensures the 4 state words are well-distributed even
         * if the raw seed is small or zero.
         */
        static uint32_t splitmix32(uint32_t &state)
        {
            uint32_t z = (state += 0x9e3779b9u);
            z = (z ^ (z >> 16)) * 0x85ebca6bu;
            z = (z ^ (z >> 13)) * 0xc2b2ae35u;
            return z ^ (z >> 16);
        }

    public:
        /**
         * @brief Construct and seed from a single uint32
         * 
         * Uses SplitMix32 to expand seed into all 4 state words.
         * Handles zero seed safely.
         * 
         * @param seed Any uint32 — pros::micros() is a good default
         */
        explicit Xoshiro128Plus(uint32_t seed = 0x12345678u)
        {
            // Avoid all-zero state (undefined behavior for xoshiro)
            if (seed == 0) seed = 0x12345678u;

            s[0] = splitmix32(seed);
            s[1] = splitmix32(seed);
            s[2] = splitmix32(seed);
            s[3] = splitmix32(seed);
        }

        /**
         * @brief Generate next raw uint32
         * 
         * Core xoshiro128+ algorithm.
         * Period: 2^128 - 1
         */
        inline uint32_t next_u32()
        {
            const uint32_t result = s[0] + s[3];

            const uint32_t t = s[1] << 9;

            s[2] ^= s[0];
            s[3] ^= s[1];
            s[1] ^= s[2];
            s[0] ^= s[3];

            s[2] ^= t;
            s[3] = rotl(s[3], 11);

            return result;
        }

        /**
         * @brief Generate float in [0, 1)
         * 
         * Drops 8 bits to get 24-bit mantissa precision,
         * matching float's actual precision. Cast happens
         * once here — callers get pure float.
         */
        inline float next_f32()
        {
            // 24-bit mantissa, single cast, no double involved
            return static_cast<float>(next_u32() >> 8) * (1.0f / static_cast<float>(1u << 24));
        }

        /**
         * @brief Generate float in [min, max)
         */
        inline float range_f32(float min, float max)
        {
            return min + (max - min) * next_f32();
        }

        /**
         * @brief Sample from N(0, std_dev) using Box-Muller transform
         * 
         * Box-Muller chosen over ziggurat for simplicity.
         * Ziggurat is ~3x faster if particle count becomes a bottleneck.
         * 
         * NOTE: Generates two samples per call, discards second.
         * Cache second sample if throughput becomes a concern.
         * 
         * @param std_dev Standard deviation of distribution
         */
        inline float gaussian(float std_dev)
        {
            // Guard against degenerate u1 = 0 (log(0) = -inf)
            const float u1 = std::max(next_f32(), 1e-12f);
            const float u2 = next_f32();

            // All float — no double promotion
            const float r = std::sqrt(-2.0f * std::log(u1));
            const float theta = 2.0f * static_cast<float>(M_PI) * u2;

            return r * std::cos(theta) * std_dev;
        }

        /**
         * @brief Jump ahead by 2^64 steps
         * 
         * Useful for generating non-overlapping streams
         * from the same seed (e.g. parallel tasks).
         * Not needed for single-threaded MCL but included for completeness.
         */
        void jump()
        {
            static constexpr uint32_t JUMP[] = {
                0x8764000bu, 0xf542d2d3u, 0x6fa035c3u, 0x77f2db5bu
            };

            uint32_t s0 = 0, s1 = 0, s2 = 0, s3 = 0;

            for (int i = 0; i < 4; i++)
            {
                for (int b = 0; b < 32; b++)
                {
                    if (JUMP[i] & (1u << b))
                    {
                        s0 ^= s[0];
                        s1 ^= s[1];
                        s2 ^= s[2];
                        s3 ^= s[3];
                    }
                    next_u32();
                }
            }

            s[0] = s0;
            s[1] = s1;
            s[2] = s2;
            s[3] = s3;
        }
    };

} // namespace abclib::math