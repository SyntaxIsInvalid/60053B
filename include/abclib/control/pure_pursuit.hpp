#if 0
#pragma once

#include "abclib/estimation/pose.hpp"
#include "abclib/path/path_segment_interface.hpp"
#include "abclib/units/units.hpp"

namespace abclib::control
{
    struct PurePursuitConfig
    {
        units::Distance base_lookahead;                  // Base lookahead distance, typically 12-20 inches
        units::BodyLinearVelocity max_velocity;          // Maximum forward velocity
        double curvature_lookahead_gain;                 // How aggressively to reduce lookahead on curves (0.3-0.7)
        units::Distance min_lookahead;                   // Minimum lookahead distance, safety floor (3-5 inches)
    };

    struct PurePursuitOutput
    {
        units::BodyLinearVelocity v;
        units::BodyAngularVelocity omega;
        units::Distance lookahead_distance;              // Actual lookahead used (for telemetry/tuning)
        units::Distance cross_track_error;               // Lateral error from path
    };

    class PurePursuit
    {
    public:
        PurePursuit() = default;

        /**
         * @brief Compute velocity commands using adaptive pure pursuit
         * @param current_pose Current robot pose
         * @param segment Path segment to follow
         * @param config Pure pursuit parameters
         * @return Velocity commands (v, omega) and tracking info
         */
        PurePursuitOutput compute(
            const estimation::Pose &current_pose,
            const path::IPathSegment *segment,
            const PurePursuitConfig &config);

    private:
        /**
         * @brief Find arc length parameter of closest point on path
         */
        double find_closest_arc_length(
            const estimation::Pose &current_pose,
            const path::IPathSegment *segment);

        /**
         * @brief Get lookahead point on path
         */
        void get_lookahead_point(
            double closest_arc_length,
            double lookahead_distance,
            const path::IPathSegment *segment,
            double &lookahead_x,
            double &lookahead_y);

        /**
         * @brief Compute adaptive lookahead based on path curvature
         */
        double compute_adaptive_lookahead(
            double u_param,
            units::BodyLinearVelocity current_velocity,
            const path::IPathSegment *segment,
            const PurePursuitConfig &config);
    };

} // namespace abclib::control
#endif