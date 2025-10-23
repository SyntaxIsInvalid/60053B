#pragma once

#include <array>
#include <functional>
#include <utility>

namespace abclib::math
{
    static constexpr std::array<double, 15> xi15 = {
        0.0000000000000000,
        -0.2011940939974345, 0.2011940939974345,
        -0.3941513470775634, 0.3941513470775634,
        -0.5709721726085388, 0.5709721726085388,
        -0.7244177313601701, 0.7244177313601701,
        -0.8482065834104272, 0.8482065834104272,
        -0.9372733924007060, 0.9372733924007060,
        -0.9879925180204854, 0.9879925180204854};
    static constexpr std::array<double, 15> w15 = {
        0.2025782419255613,
        0.1984314853271116, 0.1984314853271116,
        0.1861610000155622, 0.1861610000155622,
        0.1662692058169939, 0.1662692058169939,
        0.1395706779261543, 0.1395706779261543,
        0.1071592204671719, 0.1071592204671719,
        0.0703660474881081, 0.0703660474881081,
        0.0307532419961173, 0.0307532419961173};
}

/// Integrate f over [0,1] using fixed 15-point Gauss-Legendre
inline double integrate_gauss15(const std::function<double(double)> &f)
{
    double sum = 0.0;
    for (int i = 0; i < 15; ++i)
    {
        double u = 0.5 * (1.0 + abclib::math::xi15[i]); // map xi from [-1,1] to [0,1]
        sum += abclib::math::w15[i] * f(u);
    }
    return 0.5 * sum; // account for du = 1/2 dxi
}

/// Integrate f over [a,b] using fixed 15-point Gauss-Legendre
inline double integrate_gauss15(
    const std::function<double(double)> &f,
    double a, double b)
{
    double half = 0.5 * (b - a);
    double mid = 0.5 * (b + a);
    double sum = 0.0;
    for (int i = 0; i < 15; ++i)
    {
        double u = mid + half * abclib::math::xi15[i]; // map xi to [a,b]
        sum += abclib::math::w15[i] * f(u);
    }
    return half * sum; // account for du = half * dxi
}

/// Integrate f over [a,b] with error estimate via 2-panel split
inline std::pair<double, double> integrate_gauss15_with_error(
    const std::function<double(double)> &f,
    double a, double b)
{
    // Single-panel result
    double L1 = integrate_gauss15(f, a, b);
    // Two-panel split
    double m = 0.5 * (a + b);
    double L2 = integrate_gauss15(f, a, m) + integrate_gauss15(f, m, b);
    double err = std::abs(L2 - L1);
    return {L2, err};
}
