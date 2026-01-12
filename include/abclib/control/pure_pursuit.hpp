#pragma once

#include "abclib/path/path_segment_interface.hpp"
#include "abclib/estimation/pose.hpp"
#include "abclib/units/units.hpp"

namespace abclib::control
{
    struct PurePursuitConfig
    {
        units::Length lookahead_distance = units::Length::from_inches(15.0);
        units::Velocity target_velocity = units::Velocity::from_ips(24.0);

        // NEW: Motion profile support
        bool use_motion_profile = false;
        units::Acceleration max_acceleration = units::Acceleration::from_ips2(48.0);

        bool use_adaptive_lookahead = false;
        double lookahead_velocity_gain = 0.5;
        units::Length min_lookahead = units::Length::from_inches(6.0);
        units::Length max_lookahead = units::Length::from_inches(24.0);
    };

    /**
     * @brief Pure pursuit control law
     *
     * Calculates commanded curvature to track a lookahead point.
     * This is a stateless, pure function - no robot dependencies.
     */
    class PurePursuit
    {
    public:
        /**
         * @brief Calculate curvature command to reach lookahead point
         *
         * @param lookahead_point Target point in global frame
         * @param robot_pose Current robot pose
         * @param lookahead_distance Distance to lookahead point
         * @return Commanded curvature (1/radius)
         */
        static double calculate_curvature(
            const path::Point &lookahead_point,
            const estimation::Pose &robot_pose,
            units::Length lookahead_distance);

        /**
         * @brief Convert curvature to differential drive wheel velocities
         *
         * @param curvature Commanded curvature
         * @param linear_velocity Desired forward velocity
         * @param track_width Distance between wheels
         * @param left_velocity [out] Left wheel velocity
         * @param right_velocity [out] Right wheel velocity
         */
        static void curvature_to_wheel_velocities(
            double curvature,
            units::Velocity linear_velocity,
            units::Length track_width,
            units::Velocity &left_velocity,
            units::Velocity &right_velocity);
    };

} // namespace abclib::control