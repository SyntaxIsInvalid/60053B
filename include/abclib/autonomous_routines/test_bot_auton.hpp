#pragma once
#include "auton_selector.hpp"

namespace abclib::auton
{
    inline void test_bot_auton(RobotSubsystems &robot)
    {
        // Set starting position
        robot.chassis.set_pose(0_in, 0_in, 0_deg);
        control::PurePursuitConfig config;

        // === Feature 1: Motion Profile ===
        config.use_motion_profile = true;
        config.max_acceleration = units::Acceleration::from_ips2(60.0); // Default is fine

        // === Feature 2: Adaptive Lookahead ===
        config.use_adaptive_lookahead = true;
        config.lookahead_time = units::Time::from_seconds(0.5); // Default is fine
        config.min_lookahead = units::Length::from_inches(6.0);
        config.max_lookahead = units::Length::from_inches(24.0);

        // === Feature 3: Heading Correction ===
        config.use_heading_correction = true;
        config.heading_correction_gain = 2.0; // Default is fine
        config.heading_start_threshold = 0.8; // Start correcting at 80% progress

        // === Feature 4: Curvature Regulation ===
        config.use_curvature_regulation = true;
        config.min_radius = units::Length::from_inches(12.0); // Slow down for turns tighter than 12"

        // === Feature 5: Final Turn ===
        config.use_final_turn = true;
        config.use_stateful_rotation = true; // Already default, prevents oscillation
        config.rotation_distance_threshold = units::Length::from_inches(3.0);
        config.rotate_to_heading_angular_vel = units::AngularVelocity::from_deg_per_sec(90);
        config.max_angular_accel = units::AngularAcceleration::from_deg_per_sec2(360);
        config.final_heading_tolerance = units::Angle::from_degrees(1);

        // === Base velocity ===
        config.target_velocity = units::Velocity::from_ips(36.0); // Or slower for initial testing

        // === Run the test ===
        robot.chassis.quintic_pure_pursuit(
            units::Length::from_inches(-18),
            units::Length::from_inches(18),
            units::Angle::from_degrees(90),
            config,
            units::Time::from_seconds(10));
    }
}