#pragma once

#include "abclib/trajectory/trajectory.hpp"
#include "abclib/control/ramsete.hpp"
#include "abclib/path/path_segment_interface.hpp"
#include "abclib/telemetry/telemetry.hpp" // For PathFollowerStatus
#include "abclib/estimation/odometry.hpp" // For Pose
#include <functional>
#include "abclib/control/pid.hpp" // <-- ADD THIS LINE

namespace abclib::hardware
{
    class Chassis; // Forward declaration is fine since we only use pointers
}

namespace abclib::trajectory
{
    struct FollowerConfig
    {
        units::BodyLinearVelocity max_velocity; // TYPED
        double max_acceleration;                // inches/s² (consider typing this too)
        control::RamseteConstants ramsete_constants = {2.0, 0.7};

        // Settlement criteria
        units::Time timeout = units::Time::from_seconds(15);                           // TYPED
        units::Distance position_threshold = units::Distance::from_inches(0.3);        // TYPED
        units::BodyLinearVelocity velocity_threshold = units::BodyLinearVelocity(0.5); // TYPED
        int settle_count_required = 10;                                                // consecutive loops

        // Optional callback for actions/telemetry
        std::function<void(const TrajectoryState &)> state_callback = nullptr;
    };

    class PathFollower
    {
    public:
        /**
         * @brief Construct a PathFollower
         * @param chassis Pointer to chassis (non-owning)
         * @param constants RAMSETE controller gains
         */
        explicit PathFollower(hardware::Chassis *chassis,
                              const control::RamseteConstants &ramsete_constants);
        /**
         * @brief Follow a single path segment using RAMSETE control
         * @param segment Path segment to follow (IPathSegment interface)
         * @param config Follower configuration (velocity limits, settlement, etc.)
         *
         * This function blocks until the path is complete or timeout occurs.
         * Updates telemetry throughout execution.
         */
        void follow_segment(
            const path::IPathSegment *segment,
            const FollowerConfig &config);

        /**
         * @brief Set RAMSETE gains (can tune during runtime)
         */
        void set_ramsete_constants(const control::RamseteConstants &constants);

        /**
         * @brief Get current RAMSETE gains
         */
        control::RamseteConstants get_ramsete_constants() const;

    private:
        hardware::Chassis *chassis_;
        control::Ramsete ramsete_;

        /**
         * @brief Main control loop - executes trajectory with RAMSETE
         */
        void execute_control_loop(
            const path::IPathSegment *segment, // <-- ADD THIS PARAMETER
            const Trajectory &trajectory,
            const FollowerConfig &config);

        /**
         * @brief Check if robot has settled at target
         */
        bool check_settlement(
            const estimation::Pose &current_pose,
            const TrajectoryState &reference_state,
            const FollowerConfig &config,
            int &settle_count) const;

        /**
         * @brief Determine current trajectory phase status
         */
        PathFollowerStatus determine_trajectory_status(
            const Trajectory &trajectory,
            units::Time elapsed_time, // TYPED
            bool is_settling) const;

        /**
         * @brief Update telemetry with tracking errors and status
         */
        void update_telemetry(
            const estimation::Pose &current_pose,
            const TrajectoryState &reference_state,
            const control::RamseteOutput &ramsete_output,
            units::Voltage left_voltage,  // TYPED
            units::Voltage right_voltage, // TYPED
            PathFollowerStatus status,
            units::Time elapsed_time, // TYPED
            units::Time total_time    // TYPED
        ) const;
    };

} // namespace abclib::trajectory