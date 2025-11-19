#include "abclib/path/quintic_hermite_segment.hpp"
#include <algorithm>
#include <cmath>
#include "abclib/math/GL15.hpp"

namespace abclib::path
{
    using Point = IPathSegment::Point;
    using Pose = IPathSegment::Pose;

    // Constructor with heuristic
    QuinticHermiteSegment::QuinticHermiteSegment(const Pose& start_pose, const Pose& end_pose)
        : start_pose_(start_pose), end_pose_(end_pose)
    {
        // Calculate distance between poses
        double distance = (end_pose.head<2>() - start_pose.head<2>()).norm();
        
        // Apply 1.2 scaling heuristic
        double scalar = 1.2 * distance;
        
        // Generate velocity vectors aligned with headings
        Point start_vel(scalar * std::cos(start_pose(2)), 
                       scalar * std::sin(start_pose(2)));
        Point end_vel(scalar * std::cos(end_pose(2)), 
                     scalar * std::sin(end_pose(2)));
        
        // Zero acceleration at endpoints
        Point zero_accel = Point::Zero();
        
        // Calculate coefficients
        calculate_coefficients(start_pose.head<2>(), start_vel, zero_accel,
                             end_pose.head<2>(), end_vel, zero_accel);
        
        // Calculate arc length
        segment_length_ = integrate_arc_length();
    }

    // Constructor with explicit control vectors
    QuinticHermiteSegment::QuinticHermiteSegment(const Pose& start_pose, const Pose& end_pose,
                                                 const Point& start_vel, const Point& start_accel,
                                                 const Point& end_vel, const Point& end_accel)
        : start_pose_(start_pose), end_pose_(end_pose)
    {
        calculate_coefficients(start_pose.head<2>(), start_vel, start_accel,
                             end_pose.head<2>(), end_vel, end_accel);
        
        segment_length_ = integrate_arc_length();
    }

    void QuinticHermiteSegment::calculate_coefficients(
        const Point& pos_a, const Point& vel_a, const Point& accel_a,
        const Point& pos_b, const Point& vel_b, const Point& accel_b)
    {
        // Quintic Hermite interpolation formulas (closed-form)
        // p(u) = c0 + c1*u + c2*u^2 + c3*u^3 + c4*u^4 + c5*u^5
        
        // X-dimension coefficients
        x_coeffs_(0) = pos_a(0);
        x_coeffs_(1) = vel_a(0);
        x_coeffs_(2) = 0.5 * accel_a(0);
        x_coeffs_(3) = -10.0 * pos_a(0) - 6.0 * vel_a(0) - 1.5 * accel_a(0) 
                       + 10.0 * pos_b(0) - 4.0 * vel_b(0) + 0.5 * accel_b(0);
        x_coeffs_(4) = 15.0 * pos_a(0) + 8.0 * vel_a(0) + 1.5 * accel_a(0) 
                       - 15.0 * pos_b(0) + 7.0 * vel_b(0) - 1.0 * accel_b(0);
        x_coeffs_(5) = -6.0 * pos_a(0) - 3.0 * vel_a(0) - 0.5 * accel_a(0) 
                       + 6.0 * pos_b(0) - 3.0 * vel_b(0) + 0.5 * accel_b(0);
        
        // Y-dimension coefficients
        y_coeffs_(0) = pos_a(1);
        y_coeffs_(1) = vel_a(1);
        y_coeffs_(2) = 0.5 * accel_a(1);
        y_coeffs_(3) = -10.0 * pos_a(1) - 6.0 * vel_a(1) - 1.5 * accel_a(1) 
                       + 10.0 * pos_b(1) - 4.0 * vel_b(1) + 0.5 * accel_b(1);
        y_coeffs_(4) = 15.0 * pos_a(1) + 8.0 * vel_a(1) + 1.5 * accel_a(1) 
                       - 15.0 * pos_b(1) + 7.0 * vel_b(1) - 1.0 * accel_b(1);
        y_coeffs_(5) = -6.0 * pos_a(1) - 3.0 * vel_a(1) - 0.5 * accel_a(1) 
                       + 6.0 * pos_b(1) - 3.0 * vel_b(1) + 0.5 * accel_b(1);
    }

    void QuinticHermiteSegment::calc_point(double u, double& x, double& y) const
    {
        u = std::clamp(u, 0.0, 1.0);
        
        // Horner's method for efficient polynomial evaluation
        // x(u) = c0 + c1*u + c2*u^2 + c3*u^3 + c4*u^4 + c5*u^5
        x = x_coeffs_(5);
        x = x * u + x_coeffs_(4);
        x = x * u + x_coeffs_(3);
        x = x * u + x_coeffs_(2);
        x = x * u + x_coeffs_(1);
        x = x * u + x_coeffs_(0);
        
        y = y_coeffs_(5);
        y = y * u + y_coeffs_(4);
        y = y * u + y_coeffs_(3);
        y = y * u + y_coeffs_(2);
        y = y * u + y_coeffs_(1);
        y = y * u + y_coeffs_(0);
    }

    Point QuinticHermiteSegment::calc_first_deriv(double u) const
    {
        u = std::clamp(u, 0.0, 1.0);
        
        // First derivative: dx/du = c1 + 2*c2*u + 3*c3*u^2 + 4*c4*u^3 + 5*c5*u^4
        double x = 5.0 * x_coeffs_(5);
        x = x * u + 4.0 * x_coeffs_(4);
        x = x * u + 3.0 * x_coeffs_(3);
        x = x * u + 2.0 * x_coeffs_(2);
        x = x * u + x_coeffs_(1);
        
        double y = 5.0 * y_coeffs_(5);
        y = y * u + 4.0 * y_coeffs_(4);
        y = y * u + 3.0 * y_coeffs_(3);
        y = y * u + 2.0 * y_coeffs_(2);
        y = y * u + y_coeffs_(1);
        
        return Point(x, y);
    }

    Point QuinticHermiteSegment::calc_second_deriv(double u) const
    {
        u = std::clamp(u, 0.0, 1.0);
        
        // Second derivative: d²x/du² = 2*c2 + 6*c3*u + 12*c4*u^2 + 20*c5*u^3
        double x = 20.0 * x_coeffs_(5);
        x = x * u + 12.0 * x_coeffs_(4);
        x = x * u + 6.0 * x_coeffs_(3);
        x = x * u + 2.0 * x_coeffs_(2);
        
        double y = 20.0 * y_coeffs_(5);
        y = y * u + 12.0 * y_coeffs_(4);
        y = y * u + 6.0 * y_coeffs_(3);
        y = y * u + 2.0 * y_coeffs_(2);
        
        return Point(x, y);
    }

    double QuinticHermiteSegment::calc_curvature(double u) const
    {
        const Point d1 = calc_first_deriv(u);   // (x', y')
        const Point d2 = calc_second_deriv(u);  // (x'', y'')
        
        // Curvature formula: κ = |x'y'' - y'x''| / (x'² + y'²)^(3/2)
        const double numerator = std::abs(d1.x() * d2.y() - d1.y() * d2.x());
        const double denominator = std::pow(d1.squaredNorm(), 1.5);
        
        // Avoid division by zero
        if (std::abs(denominator) < 1e-9)
        {
            return 0.0;
        }
        
        return numerator / denominator;
    }

    double QuinticHermiteSegment::get_segment_length() const
    {
        return segment_length_;
    }

    const Pose& QuinticHermiteSegment::get_start_pose() const
    {
        return start_pose_;
    }

    const Pose& QuinticHermiteSegment::get_end_pose() const
    {
        return end_pose_;
    }

    double QuinticHermiteSegment::get_start_curvature() const
    {
        // Compute curvature at u=0 from the curve geometry
        return calc_curvature(0.0);
    }

    double QuinticHermiteSegment::get_end_curvature() const
    {
        // Compute curvature at u=1 from the curve geometry
        return calc_curvature(1.0);
    }

    double QuinticHermiteSegment::get_start_curvature_derivative() const
    {
        // Quintic Hermite only guarantees G2 continuity (not G3)
        // Curvature derivative is not specified/controlled
        return 0.0;
    }

    double QuinticHermiteSegment::get_end_curvature_derivative() const
    {
        // Quintic Hermite only guarantees G2 continuity (not G3)
        // Curvature derivative is not specified/controlled
        return 0.0;
    }

    double QuinticHermiteSegment::integrate_arc_length()
    {
        // Use Gauss-Legendre 15-point quadrature (same as Eta3)
        return integrate_gauss15([this](double u) {
            const double norm = calc_first_deriv(u).norm();
            return std::max(norm, 1e-6);
        });
    }

} // namespace abclib::path