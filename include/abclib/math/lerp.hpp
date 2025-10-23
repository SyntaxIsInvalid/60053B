#pragma once
#include "abclib/math/angles.hpp"

namespace abclib::math
{
    inline constexpr double lerp(double a, double b, double t) noexcept
    {
        return a + t * (b - a);
    }

    inline double lerp_angle(double a, double b, double t) noexcept
    {
        double delta = normalize_angle(b - a);
        return normalize_angle(a + t * delta);
    }
}