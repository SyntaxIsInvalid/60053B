#include "turn_in_place_segment.hpp"
#include <algorithm>

namespace abclib::path {

using Point = IPathSegment::Point;
using Pose = IPathSegment::Pose;

TurnInPlaceSegment::TurnInPlaceSegment(const Pose& center_pose, 
                                       double end_heading, 
                                       units::Distance track_width)  // CHANGED HERE
    : start_pose_(center_pose),
      end_pose_(center_pose(0), center_pose(1), end_heading),
      track_width_(track_width)
{
    // Validate track width
    if (track_width.inches <= 0) {  // CHANGED: access .inches member
        throw std::invalid_argument(
            "TurnInPlaceSegment: track_width must be positive. Got: " + 
            std::to_string(track_width.inches)  // CHANGED: access .inches
        );
    }
    
    // Calculate angular displacement (shortest path)
    angular_displacement_ = math::normalize_angle(end_heading - center_pose(2));
    
    // Enforce non-zero turn
    if (std::abs(angular_displacement_) < 1e-6) {
        throw std::invalid_argument(
            "TurnInPlaceSegment: angular displacement is too small. "
            "Start: " + std::to_string(center_pose(2) * 180.0 / M_PI) + "°, "
            "End: " + std::to_string(end_heading * 180.0 / M_PI) + "°"
        );
    }
    
    // Calculate arc length based on actual wheel separation
    // Arc length = radius * angle, where radius = track_width / 2
    double turning_radius = track_width_.inches / 2.0;  // CHANGED: access .inches
    arc_length_ = std::abs(angular_displacement_) * turning_radius;
}

void TurnInPlaceSegment::calc_point(double u, double& x, double& y) const {
    u = std::clamp(u, 0.0, 1.0);
    
    // Position remains constant during turn-in-place
    x = start_pose_(0);
    y = start_pose_(1);
}

Point TurnInPlaceSegment::calc_first_deriv(double u) const {
    // No translation - velocity in x,y is zero
    return Point(0.0, 0.0);
}

Point TurnInPlaceSegment::calc_second_deriv(double u) const {
    // No translation - acceleration in x,y is zero
    return Point(0.0, 0.0);
}

double TurnInPlaceSegment::calc_curvature(double u) const {
    // Turn-in-place has infinite curvature (zero radius)
    // Return a large finite value for numerical stability
    return std::copysign(1e9, angular_displacement_);
}

double TurnInPlaceSegment::get_segment_length() const {
    // Return arc length traced by the wheels
    return arc_length_;
}

const TurnInPlaceSegment::Pose& TurnInPlaceSegment::get_start_pose() const {
    return start_pose_;
}

const TurnInPlaceSegment::Pose& TurnInPlaceSegment::get_end_pose() const {
    return end_pose_;
}

} // namespace abclib::path