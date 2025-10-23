#pragma once

#include "path_segment_interface.hpp"
#include "abclib/math/angles.hpp"
#include <cmath>
#include <stdexcept>

namespace abclib::path {
    
class StraightSegment : public IPathSegment {
public:
    /**
     * @brief Construct a straight line segment with enforced geometric alignment
     * @param start_pose Starting pose [x, y, theta]
     * @param end_pose Ending pose [x, y, theta]
     * 
     * @throws std::invalid_argument if start or end heading doesn't align with line direction
     * 
     * Note: For proper path stitching (G1 continuity), both start and end headings
     * must match the geometric direction of the line from start to end.
     */
    StraightSegment(const Pose& start_pose, const Pose& end_pose);
    
    // IPathSegment interface
    void calc_point(double u, double& x, double& y) const override;
    Point calc_first_deriv(double u) const override;
    Point calc_second_deriv(double u) const override;
    double calc_curvature(double u) const override;
    double get_segment_length() const override;
    const Pose& get_start_pose() const override;
    const Pose& get_end_pose() const override;
    
    // Boundary conditions for path stitching
    double get_start_curvature() const;
    double get_end_curvature() const;
    double get_start_curvature_derivative() const;
    double get_end_curvature_derivative() const;
    
private:
    Pose start_pose_;
    Pose end_pose_;
    double length_;
    Point direction_;  // Unit vector from start to end (normalized)
    Point displacement_;  // Total displacement vector (not normalized)
};

} // namespace abclib::path