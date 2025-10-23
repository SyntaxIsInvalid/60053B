#pragma once

#include <Eigen/Dense>

namespace abclib::path
{

    class IPathSegment
    {
    public:
        // Shared type aliases for ALL path geometries
        using Point = Eigen::Vector2d;
        using Pose = Eigen::Vector3d;

        virtual ~IPathSegment() = default;

        virtual void calc_point(double u, double &x, double &y) const = 0;
        virtual Point calc_first_deriv(double u) const = 0;
        virtual Point calc_second_deriv(double u) const = 0;
        virtual double calc_curvature(double u) const = 0;
        virtual double get_segment_length() const = 0;
        virtual const Pose &get_start_pose() const = 0;
        virtual const Pose &get_end_pose() const = 0;
        virtual bool is_turn_in_place() const { return false; }
    };
    using Point = IPathSegment::Point;
    using Pose = IPathSegment::Pose;
}
