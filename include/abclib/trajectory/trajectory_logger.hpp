 #if 0 
 #pragma once

#include "trajectory.hpp"
#include "abclib/builder/path.hpp"
#include "abclib/profiling/profile_factory.hpp"  // Use factory instead of direct include
#include <string>
#include <cstdio>
#include <memory>
#include "abclib/builder/profile_group.hpp"

namespace abclib::trajectory
{
    class TrajectoryLogger
    {
    public:
        /**
         * @brief Log all trajectories from a Path to a single CSV
         * @param path Path containing ProfileGroups
         * @param filename Base filename (will be prepended with /usd/)
         * @param profile_type Profile type to use for logging (nullopt = use group defaults)
         * @param time_step_seconds Time resolution for sampling (default 0.01s = 100Hz)
         */
        static void log_path_trajectories(
            const path::Path &path,
            const std::string &filename,
            std::optional<profiling::ProfileType> profile_type = std::nullopt,
            double time_step_seconds = 0.01)
        {
            std::string full_path = "/usd/" + filename + ".csv";
            FILE *file = fopen(full_path.c_str(), "w");

            if (!file)
                return;

            write_header(file);

            const auto &groups = path.get_profile_groups();
            for (size_t group_idx = 0; group_idx < groups.size(); group_idx++)
            {
                // Determine which profile type to use
                profiling::ProfileType selected_type;
                if (profile_type.has_value()) {
                    // Use override if provided
                    selected_type = profile_type.value();
                } else {
                    // Use group's default
                    selected_type = groups[group_idx].default_profile_type;
                }
                
                // Create profile for this group using factory
                auto profile = profiling::create_profile(
                    selected_type,
                    groups[group_idx].total_arc_length,
                    groups[group_idx].default_max_velocity,
                    groups[group_idx].default_max_acceleration
                );
                
                Trajectory traj(&groups[group_idx], std::move(profile));
                write_trajectory_data(file, traj, group_idx, groups[group_idx].name, time_step_seconds);
            }

            fclose(file);
        }

        /**
         * @brief Log a single segment trajectory to CSV
         * @param segment Path segment to log
         * @param max_velocity Maximum velocity constraint
         * @param max_acceleration Maximum acceleration constraint
         * @param filename Base filename (will be prepended with /usd/)
         * @param profile_type Profile type to use (default: trapezoidal)
         * @param time_step_seconds Time resolution for sampling (default 0.01s = 100Hz)
         */
        static void log_single_trajectory(
            const path::IPathSegment *segment,
            units::Velocity max_velocity,
            units::Acceleration max_acceleration,
            const std::string &filename,
            profiling::ProfileType profile_type = profiling::ProfileType::TRAPEZOIDAL,
            double time_step_seconds = 0.01)
        {
            std::string full_path = "/usd/" + filename + ".csv";
            FILE *file = fopen(full_path.c_str(), "w");

            if (!file)
                return;

            write_header(file);
            
            // Create profile for single segment using factory
            units::Length segment_length = units::Length::from_inches(segment->get_segment_length());
            auto profile = profiling::create_profile(
                profile_type,
                segment_length,
                max_velocity,
                max_acceleration
            );
            
            Trajectory trajectory(segment, std::move(profile));
            write_trajectory_data(file, trajectory, 0, "single_segment", time_step_seconds);

            fclose(file);
        }

    private:
        static void write_header(FILE *file)
        {
            fprintf(file, "group_index,group_name,");
            fprintf(file, "time_s,arc_length_inches,");
            fprintf(file, "velocity_inches_per_sec,acceleration_inches_per_sec2,");
            fprintf(file, "x_inches,y_inches,theta_rad,theta_deg,");
            fprintf(file, "vx,vy,omega,curvature\n");
        }

        static void write_trajectory_data(FILE *file,
                                          const Trajectory &trajectory,
                                          size_t group_index,
                                          const std::string &group_name,
                                          double time_step)
        {
            units::Time total_time = trajectory.get_total_time();

            for (double t = 0.0; t <= total_time.to_seconds(); t += time_step)
            {
                units::Time time = units::Time::from_seconds(t);
                TrajectoryState state = trajectory.get_state(time);

                fprintf(file, "%zu,%s,", group_index, group_name.c_str());
                fprintf(file, "%.3f,%.3f,", t, state.arc_length);
                fprintf(file, "%.3f,%.3f,",
                        state.arc_velocity.to_ips(),
                        state.arc_acceleration.to_mps2() / units::constants::INCH_TO_METER);
                fprintf(file, "%.3f,%.3f,%.6f,%.2f,",
                        state.x, state.y, state.theta, state.theta * 180.0 / M_PI);
                fprintf(file, "%.3f,%.3f,%.3f,%.6f\n",
                        state.vx, state.vy, state.omega, state.curvature);
            }
        }
    };
}
#endif