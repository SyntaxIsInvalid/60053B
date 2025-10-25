#pragma once

#include "abclib/path/path_segment_interface.hpp"
#include "abclib/units/units.hpp"
#include <vector>
#include <string>
#include <memory>

namespace abclib::path
{
    struct ProfileGroup
    {
        std::string name;
        units::BodyLinearVelocity max_velocity;
        double max_acceleration; // inches/s²
        
        std::vector<std::unique_ptr<IPathSegment>> segments;
        
        // Computed during build
        double total_arc_length = 0.0;
        
        ProfileGroup(const std::string& group_name,
                    units::BodyLinearVelocity max_vel,
                    double max_accel)
            : name(group_name)
            , max_velocity(max_vel)
            , max_acceleration(max_accel)
        {}
        
        // Calculate total arc length from all segments
        void compute_arc_length()
        {
            total_arc_length = 0.0;
            for (const auto& seg : segments) {
                total_arc_length += seg->get_segment_length();
            }
        }
        
        Pose get_start_pose() const
        {
            if (segments.empty()) {
                throw std::runtime_error("ProfileGroup: no segments");
            }
            return segments.front()->get_start_pose();
        }
        
        Pose get_end_pose() const
        {
            if (segments.empty()) {
                throw std::runtime_error("ProfileGroup: no segments");
            }
            return segments.back()->get_end_pose();
        }
    };

} // namespace abclib::path