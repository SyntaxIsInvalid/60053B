#include "abclib/builder/spline_builder/quintic_hermite_builder.hpp"
#include "abclib/builder/path_builder.hpp"
#include "abclib/path/quintic_hermite_segment.hpp"
#include <stdexcept>
#include <cmath>

namespace abclib::builder {

QuinticHermiteBuilder::QuinticHermiteBuilder(PathBuilder& parent)
    : parent_(parent)
{
}

QuinticHermiteBuilder& QuinticHermiteBuilder::to(units::Length x, units::Length y, units::Angle theta) {
    end_pose_ = path::Pose(x.to_inches(), y.to_inches(), theta.to_radians());
    return *this;
}

QuinticHermiteBuilder& QuinticHermiteBuilder::use_heuristic() {
    use_heuristic_ = true;
    return *this;
}

QuinticHermiteBuilder& QuinticHermiteBuilder::with_tangent(units::Length vx, units::Length vy) {
    start_tangent_ = path::Point(vx.to_inches(), vy.to_inches());
    return *this;
}

QuinticHermiteBuilder& QuinticHermiteBuilder::with_second_derivative(units::Length ax, units::Length ay) {
    start_accel_ = path::Point(ax.to_inches(), ay.to_inches());
    return *this;
}

QuinticHermiteBuilder& QuinticHermiteBuilder::end_tangent(units::Length vx, units::Length vy) {
    end_tangent_ = path::Point(vx.to_inches(), vy.to_inches());
    return *this;
}

QuinticHermiteBuilder& QuinticHermiteBuilder::end_second_derivative(units::Length ax, units::Length ay) {
    end_accel_ = path::Point(ax.to_inches(), ay.to_inches());
    return *this;
}

PathBuilder& QuinticHermiteBuilder::build() {
    validate();
    
    path::Pose start_pose = parent_.get_current_pose();
    
    std::unique_ptr<path::IPathSegment> segment;
    
    if (use_heuristic_) {
        segment = std::make_unique<path::QuinticHermiteSegment>(
            start_pose, *end_pose_);
    } else {
        segment = std::make_unique<path::QuinticHermiteSegment>(
            start_pose, *end_pose_,
            *start_tangent_, *start_accel_,
            *end_tangent_, *end_accel_);
    }
    
    parent_.add_segment(std::move(segment));
    parent_.set_current_pose(*end_pose_);
    
    return parent_;
}

void QuinticHermiteBuilder::validate() const {
    if (!end_pose_) {
        throw std::runtime_error("QuinticHermiteBuilder: must call to() before build()");
    }
    
    if (!use_heuristic_) {
        if (!start_tangent_ || !start_accel_ || !end_tangent_ || !end_accel_) {
            throw std::runtime_error(
                "QuinticHermiteBuilder: must either use_heuristic() or provide all control vectors");
        }
    }
}

} // namespace abclib::builder