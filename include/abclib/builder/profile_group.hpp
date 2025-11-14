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
        units::Velocity max_velocity;      // Changed from BodyLinearVelocity
        units::Acceleration max_acceleration; // Changed from double (inches/s^2)

        std::vector<std::unique_ptr<IPathSegment>> segments;

        // Computed during build
        units::Length total_arc_length = units::Length::from_inches(0.0); // Changed from double

        ProfileGroup(const std::string &group_name,
                     units::Velocity max_vel,
                     units::Acceleration max_accel)
            : name(group_name), max_velocity(max_vel), max_acceleration(max_accel)
        {
        }

        // Calculate total arc length from all segments
        void compute_arc_length()
        {
            total_arc_length = units::Length::from_inches(0.0);
            for (const auto &seg : segments)
            {
                total_arc_length = total_arc_length + 
                    units::Length::from_inches(seg->get_segment_length());
            }
        }

        bool is_turn_in_place_group() const
        {
            if (segments.empty())
            {
                return false;
            }
            // Since turn-in-place always breaks continuity,
            // if ANY segment is turn-in-place, the whole group is
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