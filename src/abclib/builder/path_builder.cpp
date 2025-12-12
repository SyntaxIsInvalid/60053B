#include "abclib/builder/path_builder.hpp"
#include "abclib/builder/path.hpp"
#include "abclib/builder/profile_group.hpp"
#include "abclib/builder/profile_builder.hpp"
#include "abclib/builder/spline_builder/quintic_hermite_builder.hpp"
#include "abclib/path/quintic_hermite_segment.hpp"
#include "abclib/path/straight_segment.hpp"
#include "abclib/path/turn_in_place_segment.hpp"
#include <stdexcept>
#include <cmath>
#include "abclib/math/angles.hpp"  
namespace abclib::builder {

PathBuilder::PathBuilder(units::Length track_width)
    : track_width_(track_width)
    , current_pose_(path::Pose::Zero())
    , started_(false)
    , active_group_(nullptr)
{
}

PathBuilder& PathBuilder::start(units::Length x, units::Length y, units::Angle theta) {
    if (started_) {
        throw std::runtime_error("PathBuilder: start() already called");
    }
    current_pose_ << x.to_inches(), y.to_inches(), theta.to_radians();
    started_ = true;
    return *this;
}

ProfileBuilder PathBuilder::begin_profile(const std::string& name) {
    ensure_started();
    finalize_active_group();
    return ProfileBuilder(*this, name);
}

void PathBuilder::create_profile_group(const std::string& name,
                                        units::Velocity max_vel,
                                        units::Acceleration max_accel,
                                        path::ProfileType type) {
    groups_.emplace_back(name, max_vel, max_accel, type);
    active_group_ = &groups_.back();
}

PathBuilder& PathBuilder::spline_to(units::Length x, units::Length y, units::Angle theta) {
    return quintic_hermite()
        .to(x, y, theta)
        .use_heuristic()
        .build();
}

PathBuilder& PathBuilder::straight_to(units::Length x, units::Length y) {
    ensure_started();
    ensure_active_profile();
    
    double x_in = x.to_inches();
    double y_in = y.to_inches();
    
    double dx = x_in - current_pose_(0);
    double dy = y_in - current_pose_(1);
    double heading = std::atan2(dy, dx);
    
    path::Pose end_pose;
    end_pose << x_in, y_in, heading;
    
    auto segment = std::make_unique<path::StraightSegment>(current_pose_, end_pose);
    add_segment(std::move(segment));
    current_pose_ = end_pose;
    
    return *this;
}

PathBuilder& PathBuilder::straight_forward(units::Length distance) {
    ensure_started();
    ensure_active_profile();
    
    double dist_in = distance.to_inches();
    double heading = current_pose_(2);
    double x = current_pose_(0) + dist_in * std::cos(heading);
    double y = current_pose_(1) + dist_in * std::sin(heading);
    
    path::Pose end_pose;
    end_pose << x, y, heading;
    
    auto segment = std::make_unique<path::StraightSegment>(current_pose_, end_pose);
    add_segment(std::move(segment));
    current_pose_ = end_pose;
    
    return *this;
}

PathBuilder& PathBuilder::turn_in_place(units::Angle heading) {
    ensure_started();
    
    if (!active_group_) {
        throw std::runtime_error(
            "PathBuilder: turn_in_place() requires begin_profile() to be called first");
    }
    
    if (!active_group_->segments.empty()) {
        throw std::runtime_error(
            "PathBuilder: turn_in_place() requires a fresh profile group with no other segments. "
            "Call begin_profile() immediately before turn_in_place()");
    }
    
    auto segment = std::make_unique<path::TurnInPlaceSegment>(
        current_pose_, heading.to_radians(), track_width_);
    
    add_segment(std::move(segment));
    current_pose_(2) = heading.to_radians();
    
    finalize_active_group();
    active_group_ = nullptr;
    
    return *this;
}

PathBuilder& PathBuilder::break_continuity() {
    ensure_started();
    finalize_active_group();
    active_group_ = nullptr;
    return *this;
}

QuinticHermiteBuilder PathBuilder::quintic_hermite() {
    ensure_started();
    ensure_active_profile();
    return QuinticHermiteBuilder(*this);
}

path::Path PathBuilder::build() {
    ensure_started();
    finalize_active_group();
    
    if (groups_.empty()) {
        throw std::runtime_error("PathBuilder: no segments added");
    }
    
    path::Path result;
    for (auto& group : groups_) {
        result.add_profile_group(std::move(group));
    }
    
    groups_.clear();
    active_group_ = nullptr;
    started_ = false;
    
    return result;
}

void PathBuilder::add_segment(std::unique_ptr<path::IPathSegment> segment) {
    ensure_active_profile();
    
    if (!active_group_->segments.empty()) {
        const auto& prev = active_group_->segments.back();
        auto result = check_continuity(*prev, *segment);
        
        if (result.achieved < active_group_->achieved_continuity) {
            active_group_->achieved_continuity = result.achieved;
        }
    }
    
    active_group_->segments.push_back(std::move(segment));
}

void PathBuilder::ensure_started() const {
    if (!started_) {
        throw std::runtime_error("PathBuilder: must call start() first");
    }
}

void PathBuilder::ensure_active_profile() const {
    if (!active_group_) {
        throw std::runtime_error("PathBuilder: must call begin_profile() before adding segments");
    }
}

void PathBuilder::finalize_active_group() {
    if (active_group_ && !active_group_->segments.empty()) {
        active_group_->compute_arc_length();
    }
}

path::ContinuityResult PathBuilder::check_continuity(
    const path::IPathSegment& prev,
    const path::IPathSegment& next)
{
    constexpr double pos_tol = 1e-6;
    constexpr double angle_tol = 1e-6;
    constexpr double curvature_tol = 1e-4;
    constexpr double curvature_deriv_tol = 1e-3;
    
    path::ContinuityResult result;
    
    auto prev_end = prev.get_end_pose();
    auto next_start = next.get_start_pose();
    
    // G0: Position
    result.position_error = (prev_end.head<2>() - next_start.head<2>()).norm();
    if (result.position_error > pos_tol) {
        throw std::runtime_error(
            "G0 violation: position discontinuity (gap = " + 
            std::to_string(result.position_error) + " in)");
    }
    
    // G1: Heading
    result.heading_error = std::abs(math::angle_error(prev_end(2), next_start(2)));
    if (result.heading_error > angle_tol) {
        throw std::runtime_error(
            "G1 violation: heading mismatch (delta = " + 
            std::to_string(result.heading_error) + " rad)");
    }
    
    // G2: Curvature
    result.curvature_error = std::abs(
        prev.get_end_curvature() - next.get_start_curvature());
    if (result.curvature_error > curvature_tol) {
        throw std::runtime_error(
            "G2 violation: curvature mismatch (delta = " + 
            std::to_string(result.curvature_error) + ")");
    }
    
    // G3: Curvature derivative (no throw)
    result.curvature_deriv_error = std::abs(
        prev.get_end_curvature_derivative() - next.get_start_curvature_derivative());
    
    if (result.curvature_deriv_error <= curvature_deriv_tol) {
        result.achieved = path::ContinuityLevel::G3;
    } else {
        result.achieved = path::ContinuityLevel::G2;
    }
    
    result.meets_minimum = true;
    return result;
}

} // namespace abclib::builder