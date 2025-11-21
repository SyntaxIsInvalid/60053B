#pragma once

#include "abclib/path/path_segment_interface.hpp"
#include "abclib/units/units.hpp"
#include <optional>

namespace abclib::builder {

class PathBuilder;

class QuinticHermiteBuilder {
public:
    explicit QuinticHermiteBuilder(PathBuilder& parent);
    
    // Target pose
    QuinticHermiteBuilder& to(units::Length x, units::Length y, units::Angle theta);
    
    // Use heuristic (1.2 * distance scaling, zero accels)
    QuinticHermiteBuilder& use_heuristic();
    
    // Explicit start control vectors (in inches, inches/u)
    QuinticHermiteBuilder& with_tangent(units::Length vx, units::Length vy);
    QuinticHermiteBuilder& with_second_derivative(units::Length ax, units::Length ay);
    
    // Explicit end control vectors
    QuinticHermiteBuilder& end_tangent(units::Length vx, units::Length vy);
    QuinticHermiteBuilder& end_second_derivative(units::Length ax, units::Length ay);
    
    // Build segment, add to parent, return parent
    PathBuilder& build();

private:
    PathBuilder& parent_;
    
    std::optional<path::Pose> end_pose_;
    
    std::optional<path::Point> start_tangent_;
    std::optional<path::Point> start_accel_;
    std::optional<path::Point> end_tangent_;
    std::optional<path::Point> end_accel_;
    
    bool use_heuristic_ = false;
    
    void validate() const;
};

} // namespace abclib::builder