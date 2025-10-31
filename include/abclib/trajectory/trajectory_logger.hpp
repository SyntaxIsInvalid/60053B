#pragma once

#include "trajectory.hpp"
#include "abclib/builder/path.hpp"
#include <string>
#include <cstdio>
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
         * @param time_step_seconds Time resolution for sampling (default 0.01s = 100Hz)
         */
        static void log_path_trajectories(const path::Path& path,
                                          const std::string& filename,
                                          double time_step_seconds = 0.01)
        {
            std::string full_path = "/usd/" + filename + ".csv";
            FILE* file = fopen(full_path.c_str(), "w");
            
            if (!file) return;
            
            write_header(file);
            
            const auto& groups = path.get_profile_groups();
            for (size_t group_idx = 0; group_idx < groups.size(); group_idx++)
            {
                Trajectory traj(&groups[group_idx]);
                write_trajectory_data(file, traj, group_idx, groups[group_idx].name, time_step_seconds);
            }
            
            fclose(file);
        }
        
    private:
        static void write_header(FILE* file)
        {
            fprintf(file, "group_index,group_name,");
            fprintf(file, "time_s,arc_length_inches,");
            fprintf(file, "velocity_inches_per_sec,acceleration_inches_per_sec2,");
            fprintf(file, "x_inches,y_inches,theta_rad,theta_deg,");
            fprintf(file, "vx,vy,omega,curvature\n");
        }
        
        static void write_trajectory_data(FILE* file,
                                         const Trajectory& trajectory,
                                         size_t group_index,
                                         const std::string& group_name,
                                         double time_step)
        {
            units::Time total_time = trajectory.get_total_time();
            
            for (double t = 0.0; t <= total_time.seconds; t += time_step)
            {
                units::Time time = units::Time::from_seconds(t);
                TrajectoryState state = trajectory.get_state(time);
                
                fprintf(file, "%zu,%s,", group_index, group_name.c_str());
                fprintf(file, "%.3f,%.3f,", t, state.arc_length);
                fprintf(file, "%.3f,%.3f,",
                       state.arc_velocity.inches_per_sec,
                       state.arc_acceleration);
                fprintf(file, "%.3f,%.3f,%.6f,%.2f,",
                       state.x, state.y, state.theta, state.theta * 180.0 / M_PI);
                fprintf(file, "%.3f,%.3f,%.3f,%.6f\n",
                       state.vx, state.vy, state.omega, state.curvature);
            }
        }
    };
}