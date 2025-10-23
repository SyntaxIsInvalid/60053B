#include "straight_segment.hpp"
#include <algorithm>

namespace abclib::path
{
    using Point = IPathSegment::Point;

    StraightSegment::StraightSegment(const Pose &start_pose, const Pose &end_pose)
        : start_pose_(start_pose), end_pose_(end_pose)
    {

        // Calculate displacement and geometric properties
        double dx = end_pose(0) - start_pose(0);
        double dy = end_pose(1) - start_pose(1);

        // Store displacement vector
        displacement_ = Point(dx, dy);

        // Calculate length
        length_ = displacement_.norm();

        // Handle zero-length segment
        if (length_ < 1e-9)
        {
            throw std::invalid_argument("StraightSegment: start and end positions are identical");
        }

        // Calculate direction unit vector
        direction_ = displacement_ / length_;

        // Calculate geometric heading
        double geometric_heading = std::atan2(dy, dx);

        // Enforce heading alignment with tolerance for floating point errors
        const double heading_tolerance = 1e-6; // ~0.00006 degrees

        double start_heading_error = math::normalize_angle(start_pose(2) - geometric_heading);
        if (std::abs(start_heading_error) > heading_tolerance)
        {
            throw std::invalid_argument(
                "StraightSegment: start heading must align with line direction. "
                "Expected: " +
                std::to_string(geometric_heading * 180.0 / M_PI) + "°, "
                                                                   "Got: " +
                std::to_string(start_pose(2) * 180.0 / M_PI) + "°");
        }

        double end_heading_error = math::normalize_angle(end_pose(2) - geometric_heading);
        if (std::abs(end_heading_error) > heading_tolerance)
        {
            throw std::invalid_argument(
                "StraightSegment: end heading must align with line direction. "
                "Expected: " +
                std::to_string(geometric_heading * 180.0 / M_PI) + "°, "
                                                                   "Got: " +
                std::to_string(end_pose(2) * 180.0 / M_PI) + "°");
        }
    }

    void StraightSegment::calc_point(double u, double &x, double &y) const
    {
        u = std::clamp(u, 0.0, 1.0);

        // Linear interpolation: p(u) = start + u * (end - start)
        x = start_pose_(0) + u * displacement_(0);
        y = start_pose_(1) + u * displacement_(1);
    }

    Point StraightSegment::calc_first_deriv(double u) const
    {
        // First derivative is constant (the displacement vector)
        // dp/du = end - start
        return displacement_;
    }

    Point StraightSegment::calc_second_deriv(double u) const
    {
        // Second derivative is zero (no acceleration along straight line)
        return Point(0.0, 0.0);
    }

    double StraightSegment::calc_curvature(double u) const
    {
        // Straight lines have zero curvature everywhere
        return 0.0;
    }

    double StraightSegment::get_segment_length() const
    {
        return length_;
    }

    const StraightSegment::Pose &StraightSegment::get_start_pose() const
    {
        return start_pose_;
    }

    const StraightSegment::Pose &StraightSegment::get_end_pose() const
    {
        return end_pose_;
    }

    double StraightSegment::get_start_curvature() const
    {
        return 0.0;
    }

    double StraightSegment::get_end_curvature() const
    {
        return 0.0;
    }

    double StraightSegment::get_start_curvature_derivative() const
    {
        return 0.0;
    }

    double StraightSegment::get_end_curvature_derivative() const
    {
        return 0.0;
    }

} // namespace abclib::path