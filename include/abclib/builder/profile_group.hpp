#pragma once

#include "abclib/path/path_segment_interface.hpp"
#include "abclib/units/units.hpp"
#include "abclib/builder/continuity.hpp"
#include "abclib/profiling/trapezoidal.hpp"
#include <vector>
#include <string>
#include <memory>
#include <algorithm>
#include <optional>

namespace abclib::path
{
    enum class ProfileType {
        Trapezoidal
    };

    struct ProfileGroup
    {
        std::string name;
        units::Velocity max_velocity;
        units::Acceleration max_acceleration;
        ProfileType profile_type;

        std::vector<std::unique_ptr<IPathSegment>> segments;

        units::Length total_arc_length = units::Length::from_inches(0.0);
        ContinuityLevel achieved_continuity = ContinuityLevel::G3;
        
        std::vector<double> segment_arc_offsets;
        
        std::optional<profiling::TrapezoidalProfile> profile;

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

        void compute_arc_length()
        {
            segment_arc_offsets.clear();
            segment_arc_offsets.reserve(segments.size());
            
            double cumulative = 0.0;
            for (const auto &seg : segments)
            {
                segment_arc_offsets.push_back(cumulative);
                cumulative += seg->get_segment_length();
            }
            total_arc_length = units::Length::from_inches(cumulative);
            
            // Build the motion profile
            if (cumulative > 0.0) {
                profile.emplace(total_arc_length, max_velocity, max_acceleration);
            }
        }

        std::pair<size_t, double> arc_length_to_segment_u(double s) const
        {
            if (segments.empty()) {
                return {0, 0.0};
            }
            
            double total = total_arc_length.to_inches();
            
            if (s <= 0.0) {
                return {0, 0.0};
            }
            if (s >= total) {
                return {segments.size() - 1, 1.0};
            }
            
            auto it = std::upper_bound(
                segment_arc_offsets.begin(),
                segment_arc_offsets.end(),
                s);
            
            size_t idx = (it == segment_arc_offsets.begin()) 
                ? 0 
                : std::distance(segment_arc_offsets.begin(), it) - 1;
            
            double local_s = s - segment_arc_offsets[idx];
            double local_u = segments[idx]->arc_length_to_u(local_s);
            
            return {idx, local_u};
        }

        Point query_position(double s) const
        {
            auto [idx, u] = arc_length_to_segment_u(s);
            double x, y;
            segments[idx]->calc_point(u, x, y);
            return Point(x, y);
        }

        double query_heading(double s) const
        {
            auto [idx, u] = arc_length_to_segment_u(s);
            auto deriv = segments[idx]->calc_first_deriv(u);
            return std::atan2(deriv.y(), deriv.x());
        }

        double query_curvature(double s) const
        {
            auto [idx, u] = arc_length_to_segment_u(s);
            return segments[idx]->calc_curvature(u);
        }

        Pose query_pose(double s) const
        {
            auto [idx, u] = arc_length_to_segment_u(s);
            double x, y;
            segments[idx]->calc_point(u, x, y);
            auto deriv = segments[idx]->calc_first_deriv(u);
            double theta = std::atan2(deriv.y(), deriv.x());
            return Pose(x, y, theta);
        }

        /**
         * @brief Query path state at time t
         * @param t Time since start of this profile group
         * @return Pose (x, y, theta)
         */
        Pose query_at_time(units::Time t) const
        {
            if (!profile) {
                throw std::runtime_error("ProfileGroup: no profile built");
            }
            double s = profile->get_position(t).to_inches();
            return query_pose(s);
        }

        /**
         * @brief Get velocity at time t
         */
        units::Velocity get_velocity_at_time(units::Time t) const
        {
            if (!profile) {
                throw std::runtime_error("ProfileGroup: no profile built");
            }
            return profile->get_velocity(t);
        }

        /**
         * @brief Get total time to traverse this group
         */
        units::Time get_total_time() const
        {
            if (!profile) {
                return units::Time::from_seconds(0.0);
            }
            return profile->get_total_time();
        }

        void add_segment(std::unique_ptr<IPathSegment> segment)
        {
            segments.push_back(std::move(segment));
        }

        bool is_turn_in_place_group() const
        {
            if (segments.empty()) {
                return false;
            }
            return segments.front()->is_turn_in_place();
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