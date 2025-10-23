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

        TrajectoryState get_state(units::Time time) const // TYPED
        {
            TrajectoryState state;
            state.time = time;

            units::Distance arc_pos = profile_.get_position(time);
            state.arc_length = arc_pos.inches;
            state.arc_velocity = profile_.get_velocity(time);
            state.arc_acceleration = profile_.get_acceleration(time);

            double u = std::clamp(state.arc_length / total_arc_length_, 0.0, 1.0);

            segment_->calc_point(u, state.x, state.y);

            auto d1 = segment_->calc_first_deriv(u);
            auto d2 = segment_->calc_second_deriv(u);

            double ds_du = d1.norm();
            double du_ds = 1.0 / std::max(ds_du, 1e-6);

            // Calculate theta and curvature
            if (ds_du < 1e-6)
            {
                // Turn-in-place: interpolate heading directly
                double theta_start = segment_->get_start_pose()(2);
                double theta_end = segment_->get_end_pose()(2);
                state.theta = theta_start + u * (theta_end - theta_start);
            }
            else
            {
                state.theta = std::atan2(d1.y(), d1.x());
            }

            state.curvature = segment_->calc_curvature(u);

            double v = state.arc_velocity.inches_per_sec;

            // Check if this is a turn-in-place segment
            if (segment_->is_turn_in_place())
            {
                // Turn-in-place: arc velocity represents wheel velocity
                auto *turn_seg = static_cast<const path::TurnInPlaceSegment *>(segment_);
                double turning_radius = turn_seg->get_turning_radius();

                // No translation during turn-in-place
                state.vx = 0.0;
                state.vy = 0.0;
                state.omega = v / turning_radius; // omega = v_wheel / r

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
            const auto *eta3_seg = dynamic_cast<const path::Eta3PathSegment *>(segment_);
            if (eta3_seg != nullptr)
            {
                // Convert from math frame (Eta3 output) to body frame (RAMSETE input)
                units::BodyPose state_math_pose(state.x, state.y, units::Radians(state.theta));
                units::BodyPose state_body_pose = math::math_to_body_frame(
                    state_math_pose.x(),
                    state_math_pose.y(),
                    state_math_pose.theta());

                // Update position with body frame coordinates
                state.x = state_body_pose.x();
                state.y = state_body_pose.y();
                state.theta = state_body_pose.theta();

                // Convert velocities from math frame to body frame
                units::BodyLinearVelocity body_v;
                units::BodyAngularVelocity body_omega;
                math::math_velocity_to_body(
                    state.vx, state.vy, state.omega,
                    state.theta, // Current body heading (just converted)
                    body_v, body_omega);

                // Update velocities - decompose body v into vx, vy components
                state.vx = body_v.inches_per_sec * std::cos(state.theta);
                state.vy = body_v.inches_per_sec * std::sin(state.theta);
                state.omega = body_omega.rad_per_sec;
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
    };

} // namespace abclib::trajectory