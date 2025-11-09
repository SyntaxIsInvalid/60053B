#pragma once
#include <cmath>

namespace abclib::math
{
    struct Point2D
    {
        double x;
        double y;

        constexpr Point2D() : x(0), y(0) {}
        constexpr Point2D(double x_, double y_) : x(x_), y(y_) {}

        // Distance to another point
        double distance_to(const Point2D& other) const
        {
            double dx = x - other.x;
            double dy = y - other.y;
            return std::sqrt(dx * dx + dy * dy);
        }

        // Angle to another point (returns radians)
        double angle_to(const Point2D& other) const
        {
            return std::atan2(other.y - y, other.x - x);
        }

        // Vector operations
        Point2D operator+(const Point2D& other) const
        {
            return Point2D(x + other.x, y + other.y);
        }

        Point2D operator-(const Point2D& other) const
        {
            return Point2D(x - other.x, y - other.y);
        }

        Point2D operator*(double scalar) const
        {
            return Point2D(x * scalar, y * scalar);
        }
    };
}