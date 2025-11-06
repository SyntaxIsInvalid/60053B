#pragma once

#include "abclib/trajectory/trajectory.hpp"
#include "abclib/control/ramsete.hpp"
#include "abclib/path/path_segment_interface.hpp"
#include "abclib/builder/path.hpp"
#include "abclib/telemetry/telemetry.hpp"

namespace abclib::hardware
{
    class Chassis; // Forward declaration
}

namespace abclib::trajectory
{
    struct FollowerConfig
    {
        units::BodyLinearVelocity max_velocity;
        double max_acceleration; // inches/s²
        control::RamseteConstants ramsete_constants = {2.0, 0.7};

        // Settlement criteria
        units::Time timeout = units::Time::from_seconds(15);
        units::Distance position_threshold = units::Distance::from_inches(0.3);
        units::BodyLinearVelocity velocity_threshold = units::BodyLinearVelocity(0.5);
        int settle_count_required = 10; // consecutive loops

        // Turn-in-place control gain
        double turn_kP = 0.04; // Proportional gain for heading feedback during turns

        // Optional callback for action scheduling
        std::function<void(const TrajectoryState &)> state_callback = nullptr;
    };

    class PathFollower
    {
    public:
        /**
         * @brief Construct a PathFollower
         * @param chassis Pointer to chassis (non-owning)
         * @param ramsete_constants RAMSETE controller gains
         */
        explicit PathFollower(hardware::Chassis *chassis,
                              const control::RamseteConstants &ramsete_constants);

        /**
         * @brief Follow a single path segment
         * @param segment Path segment to follow
         * @param config Follower configuration
         *
         * Blocks until complete, timeout, or settled.
         */
        void follow_segment(const path::IPathSegment *segment,
                            const FollowerConfig &config);

        /**
         * @brief Follow a complete path from PathBuilder
         * @param path Path containing multiple ProfileGroups
         * @param timeout Maximum time for entire path execution
         *
         * Blocks until complete or timeout.
         * Automatically chains ProfileGroups sequentially.
         */
        void follow_path(const path::Path &path,
                         units::Time timeout = units::Time::from_seconds(15));

        /**
         * @brief Get the state of the path at a specific time
         * @param path Path to query
         * @param time Time from path start
         * @return TrajectoryState at the specified time
         *
         * Used for action scheduling - query where robot should be at time T.
         */
        TrajectoryState get_state_at(const path::Path &path, units::Time time) const;

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
         * @param trajectory Trajectory to execute
         * @param config Follower configuration
         *
         * Pure execution - just takes trajectory and runs it.
         * No knowledge of segments, ProfileGroups, or path structure.
         */
        void execute_trajectory(const Trajectory &trajectory,
                                const FollowerConfig &config);

        /**
         * @brief Check if robot has settled at target
         */
        bool check_settlement(
            const estimation::Pose &current_pose,
            const TrajectoryState &reference_state,
            const FollowerConfig &config,
            const control::RamseteOutput &ramsete_output,
            int &settle_count,
            const path::IPathSegment *segment) const;

        /**
         * @brief Determine current trajectory phase status
         */
        PathFollowerStatus determine_trajectory_status(const Trajectory &trajectory,
                                                       units::Time elapsed_time,
                                                       bool is_settling) const;

        /**
         * @brief Update telemetry with tracking errors and status
         */
        void update_telemetry(const estimation::Pose &current_pose,
                              const TrajectoryState &reference_state,
                              const control::RamseteOutput &ramsete_output,
                              units::Voltage left_voltage,
                              units::Voltage right_voltage,
                              PathFollowerStatus status,
                              units::Time elapsed_time,
                              units::Time total_time) const;
    };

} // namespace abclib::trajectory