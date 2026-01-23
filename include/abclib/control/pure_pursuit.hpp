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

        bool use_motion_profile = false;
        units::Acceleration max_acceleration = units::Acceleration::from_ips2(48.0);

        bool use_adaptive_lookahead = false;
        units::Time lookahead_time = units::Time::from_seconds(0.5);
        units::Length min_lookahead = units::Length::from_inches(6.0);
        units::Length max_lookahead = units::Length::from_inches(24.0);

        bool use_heading_correction = false;
        double heading_correction_gain = 2.0;
        double heading_start_threshold = 0.8;

        // NEW: Curvature regulation
        bool use_curvature_regulation = false;
        units::Length min_radius = units::Length::from_inches(12.0); // Slow down for turns tighter than 12"

        bool use_final_turn = false;
        bool use_stateful_rotation = true;                                           // Prevent oscillation
        units::Length rotation_distance_threshold = units::Length::from_inches(3.0); // Start rotating when this close
        units::AngularVelocity rotate_to_heading_angular_vel = units::AngularVelocity::from_deg_per_sec(90);
        units::AngularAcceleration max_angular_accel = units::AngularAcceleration::from_deg_per_sec2(360);
        units::Angle final_heading_tolerance = units::Angle::from_degrees(2.0); // Exit when within this
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

        static double calculate_heading_correction(
            units::Angle current_heading,
            units::Angle target_heading,
            double progress,
            const PurePursuitConfig &config);

        /**
         * @brief Apply curvature constraint to velocity
         *
         * Slows down in tight turns to maintain tracking accuracy.
         * Linear ramp: v = v_raw * (r / r_min) for r < r_min
         *
         * @param raw_velocity Desired velocity from profile or config
         * @param curvature Path curvature at lookahead point (1/inches)
         * @param min_radius Minimum comfortable turning radius
         * @return Constrained velocity
         */
        static units::Velocity apply_curvature_constraint(
            units::Velocity raw_velocity,
            double curvature,
            units::Length min_radius);

        /**
         * @brief Calculate regulated angular velocity for rotating to heading
         *
         * Respects:
         * - Max angular velocity
         * - Kinematic acceleration limits
         * - Stopping distance (avoids overshoot)
         *
         * @param heading_error Angle to target heading (radians)
         * @param current_omega Current angular velocity
         * @param config Pure pursuit configuration
         * @param dt Control loop timestep
         * @return Commanded angular velocity (rad/s)
         */
        static double calculate_rotation_velocity(
            units::Angle heading_error,
            units::AngularVelocity current_omega,
            const PurePursuitConfig &config,
            double dt);
    };
} // namespace abclib::control