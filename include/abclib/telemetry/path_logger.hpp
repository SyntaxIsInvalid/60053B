#pragma once

#include "abclib/builder/profile_group.hpp"
#include "abclib/builder/path.hpp"
#include "abclib/field/coordinate_transform.hpp"
#include "abclib/field/alliance.hpp"
#include "abclib/field/field_config.hpp"
#include <fstream>
#include <string>

namespace abclib::telemetry
{
    class PathLogger
    {
    public:
        /**
         * @brief Log quintic spline path data to CSV
         * 
         * NOTE: Assumes path is in CORNER frame (default behavior of chassis.get_pose())
         * 
         * @param group ProfileGroup containing the path
         * @param filename CSV filename (e.g., "/usd/quintic_path.csv")
         * @param num_samples Number of points to sample along path
         * @param alliance Alliance color (for converting to standard frame if needed)
         * @param field_config Field configuration
         */
        static void log_profile_group_to_csv(
            const path::ProfileGroup& group,
            const std::string& filename,
            int num_samples = 100,
            field::Alliance alliance = field::Alliance::BLUE,
            const field::FieldConfig& field_config = field::FieldConfig::standard_vex())
        {
            std::ofstream file(filename);
            if (!file.is_open())
            {
                return;
            }

            // CSV Header
            file << "arc_length_in,x_corner_in,y_corner_in,theta_corner_deg,"
                 << "x_standard_in,y_standard_in,theta_standard_deg,"
                 << "curvature,velocity_ips,time_s\n";

            double total_length = group.total_arc_length.to_inches();
            
            for (int i = 0; i <= num_samples; ++i)
            {
                double s = (static_cast<double>(i) / num_samples) * total_length;
                
                // Query path state - this is in CORNER frame
                path::Pose corner_path_pose = group.query_pose(s);
                
                // Path data is already in corner frame - use directly
                double x_corner = corner_path_pose(0);
                double y_corner = corner_path_pose(1);
                double theta_corner_rad = corner_path_pose(2);
                double theta_corner_deg = theta_corner_rad * 180.0 / M_PI;
                
                // Convert to standard frame for visualization/debugging
                estimation::Pose corner_est_pose(
                    math::SE2(x_corner, y_corner, theta_corner_rad),
                    units::Velocity::from_ips(0),
                    units::AngularVelocity::from_rad_per_sec(0)
                );
                
                estimation::Pose standard_pose = field::alliance_corner_to_standard(
                    corner_est_pose,
                    alliance,
                    field_config
                );
                
                // Get curvature (frame-independent)
                double curvature = group.query_curvature(s);
                
                // Get velocity from profile
                units::Velocity velocity = group.get_velocity_at_arc_length(s);
                
                // Get time
                units::Time time = units::Time::from_seconds(0);
                if (group.profile)
                {
                    double t_max = group.profile->get_total_time().to_seconds();
                    for (double t = 0; t <= t_max; t += 0.01)
                    {
                        units::Time test_time = units::Time::from_seconds(t);
                        double pos = group.profile->get_position(test_time).to_inches();
                        if (pos >= s)
                        {
                            time = test_time;
                            break;
                        }
                    }
                }
                
                // Write CSV row: CORNER frame first (what the path actually is)
                file << s << ","
                     << x_corner << "," << y_corner << "," << theta_corner_deg << ","
                     << standard_pose.x_inches() << "," << standard_pose.y_inches() << "," 
                     << standard_pose.theta_deg() << ","
                     << curvature << ","
                     << velocity.to_ips() << ","
                     << time.to_seconds() << "\n";
            }
            
            file.close();
        }
    };

} // namespace abclib::telemetry