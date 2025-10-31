// trajectory.hpp
#pragma once

#include "abclib/path/eta3_segment.hpp"
#include "abclib/profiling/trapezoidal.hpp"
#include "abclib/units/units.hpp"
#include <cmath>
#include <algorithm>
#include "abclib/path/path_segment_interface.hpp"
#include "abclib/path/turn_in_place_segment.hpp"
#include "abclib/math/coordinate_frames.hpp"
#include "abclib/builder/profile_group.hpp"
namespace abclib::trajectory
{
    struct TrajectoryState
    {
        units::Time time;
        double x, y, theta; // raw positions in path frame
        double vx, vy, omega;
        double ax, ay, alpha;
        double curvature;
        double arc_length;
        units::BodyLinearVelocity arc_velocity; // TYPED
        double arc_acceleration;
    };

    class Trajectory
    {
    public:
        Trajectory(const path::IPathSegment *segment,
                   units::BodyLinearVelocity max_velocity, // TYPED
                   double max_acceleration)
            : segment_(segment),
              total_arc_length_(segment->get_segment_length()),
              profile_(units::Distance::from_inches(total_arc_length_),
                       max_velocity,
                       max_acceleration)
        {
        }

        Trajectory(const path::ProfileGroup *profile_group)
            : segment_(nullptr), // null for multi-segment mode
              profile_group_(profile_group),
              total_arc_length_(profile_group->total_arc_length),
              profile_(units::Distance::from_inches(total_arc_length_),
                       profile_group->max_velocity,
                       profile_group->max_acceleration)
        {
            build_segment_lookup_table();
        }

        TrajectoryState get_state(units::Time time) const
        {
            TrajectoryState state;
            state.time = time;

            units::Distance arc_pos = profile_.get_position(time);
            state.arc_length = arc_pos.inches;
            state.arc_velocity = profile_.get_velocity(time);
            state.arc_acceleration = profile_.get_acceleration(time);

            const path::IPathSegment *current_segment;
            double u;

            // Determine which mode we're in and get the segment + u
            if (profile_group_)
            {
                // Multi-segment mode: use lookup table
                auto location = find_segment_at_arc(state.arc_length);
                current_segment = profile_group_->segments[location.segment_index].get();

                // Convert local arc length to u parameter
                double segment_length = current_segment->get_segment_length();
                u = std::clamp(location.local_arc_length / segment_length, 0.0, 1.0);
            }
            else
            {
                // Single-segment mode: use the stored segment pointer
                current_segment = segment_;
                u = std::clamp(state.arc_length / total_arc_length_, 0.0, 1.0);
            }

            // Now use current_segment and u for all geometry calculations
            current_segment->calc_point(u, state.x, state.y);

            auto d1 = current_segment->calc_first_deriv(u);
            auto d2 = current_segment->calc_second_deriv(u);

            double ds_du = d1.norm();
            double du_ds = 1.0 / std::max(ds_du, 1e-6);

            // Calculate theta and curvature
            if (ds_du < 1e-6)
            {
                // Turn-in-place: interpolate heading directly
                double theta_start = current_segment->get_start_pose()(2);
                double theta_end = current_segment->get_end_pose()(2);
                state.theta = theta_start + u * (theta_end - theta_start);
            }
            else
            {
                state.theta = std::atan2(d1.y(), d1.x());
            }

            state.curvature = current_segment->calc_curvature(u);

            double v = state.arc_velocity.inches_per_sec;

            // Check if this is a turn-in-place segment
            if (current_segment->is_turn_in_place())
            {
                // Turn-in-place: arc velocity represents wheel velocity
                auto *turn_seg = static_cast<const path::TurnInPlaceSegment *>(current_segment);
                double turning_radius = turn_seg->get_turning_radius();

                // No translation during turn-in-place
                state.vx = 0.0;
                state.vy = 0.0;
                state.omega = v / turning_radius;

                state.ax = 0.0;
                state.ay = 0.0;
                state.alpha = state.arc_acceleration / turning_radius;
            }
            else
            {
                // Normal path: compute velocities from derivatives
                state.vx = d1.x() * du_ds * v;
                state.vy = d1.y() * du_ds * v;
                state.omega = state.curvature * v;

                double v_squared = v * v;
                double du_ds_squared = du_ds * du_ds;

                state.ax = d2.x() * du_ds_squared * v_squared +
                           d1.x() * du_ds * state.arc_acceleration;
                state.ay = d2.y() * du_ds_squared * v_squared +
                           d1.y() * du_ds * state.arc_acceleration;
                state.alpha = state.curvature * state.arc_acceleration;
            }

            return state;
        }

        units::Time get_total_time() const { return profile_.get_total_time(); }
        double get_total_distance() const { return profile_.get_total_distance().inches; }
        bool is_complete(units::Time time) const { return time >= get_total_time(); }

    private:
        const path::IPathSegment *segment_;
        double total_arc_length_;
        profiling::TrapezoidalProfile profile_;

        const path::ProfileGroup *profile_group_;
        std::vector<double> segment_arc_offsets_; // Cumulative arc lengths: [0, seg0_len, seg0_len+seg1_len, ...]
        struct SegmentLocation
        {
            size_t segment_index;
            double local_arc_length; // arc_length relative to segment start
        };

        // NEW: Build the lookup table
        void build_segment_lookup_table()
        {
            if (!profile_group_)
                return;

            segment_arc_offsets_.reserve(profile_group_->segments.size() + 1);
            segment_arc_offsets_.push_back(0.0); // Start at 0

            double cumulative = 0.0;
            for (const auto &seg : profile_group_->segments)
            {
                cumulative += seg->get_segment_length();
                segment_arc_offsets_.push_back(cumulative);
            }
        }

        SegmentLocation find_segment_at_arc(double global_arc_length) const
        {
            // Clamp to valid range
            global_arc_length = std::clamp(global_arc_length, 0.0, total_arc_length_);

            // Binary search to find the segment
            // std::upper_bound finds first element > global_arc_length
            auto it = std::upper_bound(
                segment_arc_offsets_.begin(),
                segment_arc_offsets_.end(),
                global_arc_length);

            // upper_bound returns iterator to element AFTER the one we want
            // So back up by 1 to get the segment containing this arc_length
            size_t segment_index = std::distance(segment_arc_offsets_.begin(), it) - 1;

            // Handle edge case: if we're exactly at the end
            if (segment_index >= profile_group_->segments.size())
            {
                segment_index = profile_group_->segments.size() - 1;
            }

            // Calculate local arc length within this segment
            double segment_start_arc = segment_arc_offsets_[segment_index];
            double local_arc_length = global_arc_length - segment_start_arc;

            return {segment_index, local_arc_length};
        }
    };

} // namespace abclib::trajectory