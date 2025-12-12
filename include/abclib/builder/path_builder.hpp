#pragma once

#include "abclib/path/path_segment_interface.hpp"
#include "abclib/builder/profile_group.hpp"  // Need ProfileType
#include "abclib/units/units.hpp"
#include <string>
#include <vector>
#include <memory>
#include "continuity.hpp"
#include "abclib/builder/profile_builder.hpp"
#include "abclib/builder/spline_builder/quintic_hermite_builder.hpp"

namespace abclib::path {
class Path;
struct ProfileGroup;
}

namespace abclib::builder {

class QuinticHermiteBuilder;
class ProfileBuilder;

class PathBuilder {
public:
    explicit PathBuilder(units::Length track_width);
    
    // Initialize starting pose
    PathBuilder& start(units::Length x, units::Length y, units::Angle theta);
    
    // Profile management - now returns fluent builder
    ProfileBuilder begin_profile(const std::string& name);
    
    // Simple spline (quintic hermite with heuristic)
    PathBuilder& spline_to(units::Length x, units::Length y, units::Angle theta);
    
    // Straight lines
    PathBuilder& straight_to(units::Length x, units::Length y);
    PathBuilder& straight_forward(units::Length distance);
    
    // Turn in place (requires dedicated empty profile)
    PathBuilder& turn_in_place(units::Angle heading);
    
    // Explicit continuity break
    PathBuilder& break_continuity();
    
    // Explicit spline builder
    QuinticHermiteBuilder quintic_hermite();
    
    // Build final path
    path::Path build();

    // For sub-builders to access
    const path::Pose& get_current_pose() const { return current_pose_; }
    void set_current_pose(const path::Pose& pose) { current_pose_ = pose; }
    void add_segment(std::unique_ptr<path::IPathSegment> segment);
    
    // For ProfileBuilder to call
    void create_profile_group(const std::string& name,
                              units::Velocity max_vel,
                              units::Acceleration max_accel,
                              path::ProfileType type);

private:
    units::Length track_width_;
    path::Pose current_pose_;
    bool started_ = false;
    
    std::vector<path::ProfileGroup> groups_;
    path::ProfileGroup* active_group_ = nullptr;
    
    void ensure_started() const;
    void ensure_active_profile() const;
    void finalize_active_group();
    
    // Add this declaration:
    static path::ContinuityResult check_continuity(
        const path::IPathSegment& prev,
        const path::IPathSegment& next);
};

} // namespace abclib::builder