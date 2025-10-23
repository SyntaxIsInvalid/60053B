#pragma once

#include <cmath>

    namespace abclib::math
    {
        inline double normalize_angle(double angle)
        {
            angle = fmod(angle + M_PI, 2.0 * M_PI);
            if (angle < 0.0)
            {
                angle += 2.0 * M_PI;
            }
            return angle - M_PI;
        }

    inline double deg_to_rad(double degree)
    {
        return degree * M_PI / 180.0;
    }

    inline double rad_to_deg(double radian)
    {
        return radian * 180.0 / M_PI;
    }

    // General math utilities
    inline int sgn(double x)
    {
        return (x > 0) - (x < 0);
    }

    inline double sinc(double x)
    {
        if (std::abs(x) < 1e-9)
        {
            return 1.0 - (x * x) / 6.0;
        }
        return std::sin(x) / x;
    }

    inline double cosc(double x)
    {
        if (std::abs(x) < 1e-9)
        {
            return 0.5 * x - (x * x * x) / 24.0;
        }
        return (1.0 - std::cos(x)) / x;
    }
}