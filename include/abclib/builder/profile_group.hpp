#pragma once

#include "abclib/path/path_segment_interface.hpp"
#include "abclib/units/units.hpp"
#include "abclib/builder/continuity.hpp"
#include <vector>
#include <string>
#include <memory>

namespace abclib::path
{
    enum class ProfileType {
        Trapezoidal
        // Future: SCurve
    };

    struct ProfileGroup
    {
        std::string name;
        units::Velocity max_velocity;
        units::Acceleration max_acceleration;
        ProfileType profile_type;
        
        // Future for S-curve:
        // std::optional<units::Jerk> max_jerk;

        std::vector<std::unique_ptr<IPathSegment>> segments;

        // Computed during build
        units::Length total_arc_length = units::Length::from_inches(0.0);
        ContinuityLevel achieved_continuity = ContinuityLevel::G3;

        ProfileGroup(const std::string &group_name,
                     units::Velocity max_vel,
                     units::Acceleration max_accel,
                     ProfileType type = ProfileType::Trapezoidal)
            : name(group_name)
            , max_velocity(max_vel)
            , max_acceleration(max_accel)
            , profile_type(type)
        {
        }

        // Rest stays the same...
        void compute_arc_length()
        {
            total_arc_length = units::Length::from_inches(0.0);
            for (const auto &seg : segments)
            {
                total_arc_length = total_arc_length +
                                   units::Length::from_inches(seg->get_segment_length());
            }
        }

        void add_segment(std::unique_ptr<IPathSegment> segment)
        {
            segments.push_back(std::move(segment));
        }

        bool is_turn_in_place_group() const
        {
            if (segments.empty())
            {
                return false;
            }
            return segments.front()->is_turn_in_place();
        }

        Pose get_start_pose() const
        {
            if (segments.empty())
            {
                throw std::runtime_error("ProfileGroup: no segments");
            }
            return segments.front()->get_start_pose();
        }

        Pose get_end_pose() const
        {
            if (segments.empty())
            {
                throw std::runtime_error("ProfileGroup: no segments");
            }
            return segments.back()->get_end_pose();
        }
    };

} // namespace abclib::path