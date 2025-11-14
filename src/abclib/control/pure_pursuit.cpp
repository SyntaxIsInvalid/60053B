#if 0
#include "abclib/control/pure_pursuit.hpp"
#include "abclib/math/angles.hpp"
#include <cmath>
#include <algorithm>

namespace abclib::control
{
    PurePursuitOutput PurePursuit::compute(
        const estimation::Pose &current_pose,
        const path::IPathSegment *segment,
        const PurePursuitConfig &config)
    {
        PurePursuitOutput output;

        // 1. Find closest point on path to robot
        double closest_arc = find_closest_arc_length(current_pose, segment);
        double segment_length = segment->get_segment_length();
        double closest_u = closest_arc / segment_length;

        // 2. Compute adaptive lookahead distance
        double adaptive_lookahead = compute_adaptive_lookahead(
            closest_u, current_pose.v, segment, config);

        output.lookahead_distance = units::Distance::from_inches(adaptive_lookahead);

        // 3. Get lookahead point
        double lookahead_x, lookahead_y;
        get_lookahead_point(closest_arc, adaptive_lookahead, segment,
                            lookahead_x, lookahead_y);

        // 4. Compute cross-track error (distance from robot to closest point)
        double closest_x, closest_y;
        segment->calc_point(closest_u, closest_x, closest_y);
        double dx_error = closest_x - current_pose.x();
        double dy_error = closest_y - current_pose.y();
        output.cross_track_error = units::Distance::from_inches(
            std::sqrt(dx_error * dx_error + dy_error * dy_error));

        // 5. Transform lookahead point to robot frame
        double dx = lookahead_x - current_pose.x();
        double dy = lookahead_y - current_pose.y();

        double cos_theta = std::cos(current_pose.theta());
        double sin_theta = std::sin(current_pose.theta());

        // Rotate to robot frame
        double lookahead_x_robot = cos_theta * dx + sin_theta * dy;
        double lookahead_y_robot = -sin_theta * dx + cos_theta * dy;

        // 6. Compute curvature to reach lookahead point
        // Pure pursuit formula: curvature = 2 * y / L^2
        // where y is lateral offset and L is lookahead distance
        double L_squared = adaptive_lookahead * adaptive_lookahead;
        double curvature = (2.0 * lookahead_y_robot) / L_squared;

        // 7. Compute velocity commands
        output.v = config.max_velocity;
        output.omega = units::BodyAngularVelocity(
            curvature * config.max_velocity.inches_per_sec);

        return output;
    }

    double PurePursuit::find_closest_arc_length(
        const estimation::Pose &current_pose,
        const path::IPathSegment *segment)
    {
        // Binary search for closest point on path
        double segment_length = segment->get_segment_length();
        double best_arc = 0.0;
        double best_distance = std::numeric_limits<double>::max();

        // Coarse search first (every 0.5 inches)
        const int num_samples = std::max(2, static_cast<int>(segment_length / 0.5) + 1);
        for (int i = 0; i < num_samples; ++i)
        {
            double arc = (i * segment_length) / (num_samples - 1);
            double u = arc / segment_length;

            double x, y;
            segment->calc_point(u, x, y);

            double dx = x - current_pose.x();
            double dy = y - current_pose.y();
            double distance = dx * dx + dy * dy;

            if (distance < best_distance)
            {
                best_distance = distance;
                best_arc = arc;
            }
        }

        // Fine search around best point (within 1 inch)
        double search_range = 1.0;
        double arc_min = std::max(0.0, best_arc - search_range);
        double arc_max = std::min(segment_length, best_arc + search_range);

        const int fine_samples = 20;
        for (int i = 0; i < fine_samples; ++i)
        {
            double arc = arc_min + (arc_max - arc_min) * i / (fine_samples - 1);
            double u = arc / segment_length;

            double x, y;
            segment->calc_point(u, x, y);

            double dx = x - current_pose.x();
            double dy = y - current_pose.y();
            double distance = dx * dx + dy * dy;

            if (distance < best_distance)
            {
                best_distance = distance;
                best_arc = arc;
            }
        }

        return best_arc;
    }

    void PurePursuit::get_lookahead_point(
        double closest_arc_length,
        double lookahead_distance,
        const path::IPathSegment *segment,
        double &lookahead_x,
        double &lookahead_y)
    {
        // Project lookahead distance forward along path
        double segment_length = segment->get_segment_length();
        double target_arc = closest_arc_length + lookahead_distance;

        // Clamp to path end
        target_arc = std::clamp(target_arc, 0.0, segment_length);

        double u = target_arc / segment_length;
        segment->calc_point(u, lookahead_x, lookahead_y);
    }

    double PurePursuit::compute_adaptive_lookahead(
        double u_param,
        units::BodyLinearVelocity current_velocity,
        const path::IPathSegment *segment,
        const PurePursuitConfig &config)
    {
        // Get curvature at lookahead point
        double curvature = segment->calc_curvature(u_param);

        // Velocity scaling: faster = longer lookahead
        double velocity_factor = current_velocity.inches_per_sec /
                                 config.max_velocity.inches_per_sec;
        velocity_factor = std::max(0.3, velocity_factor); // Don't go below 30% speed

        // Curvature scaling: sharper curves = shorter lookahead
        double curvature_factor = std::abs(curvature) * config.curvature_lookahead_gain;

        // Combined adaptive formula - extract .inches from Distance
        double adaptive = config.base_lookahead.inches * velocity_factor / (1.0 + curvature_factor);

        // Safety bounds - extract .inches from both Distance values
        return std::clamp(adaptive, config.min_lookahead.inches, config.base_lookahead.inches * 1.5);
    }
} // namespace abclib::control
#endif