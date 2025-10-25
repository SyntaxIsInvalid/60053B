#pragma once

#include "path.hpp"
#include <string>
#include <cstdio>

namespace abclib::path
{
    /**
     * @brief Logs path geometry to CSV for visualization/debugging
     * 
     * Samples the path at regular intervals and exports:
     * - Position (x, y)
     * - Heading (theta)
     * - Curvature
     * - Profile group info
     * - Segment boundaries
     * 
     * This is DIFFERENT from telemetry logging - this is pure geometry analysis.
     */
    class PathLogger
    {
    public:
        /**
         * @brief Log path geometry to CSV file
         * @param path Path to analyze
         * @param filename Base filename (will be prepended with /usd/)
         * @param samples_per_inch Sample density (default: 2 samples per inch)
         */
        static void log_path(const Path& path, 
                            const std::string& filename,
                            double samples_per_inch = 2.0)
        {
            std::string full_path = "/usd/" + filename + ".csv";
            FILE* file = fopen(full_path.c_str(), "w");
            
            if (!file) {
                return; // Fail silently - robot should keep running
            }
            
            write_header(file);
            write_path_data(file, path, samples_per_inch);
            
            fclose(file);
        }
        
    private:
        static void write_header(FILE* file)
        {
            fprintf(file, "group_index,group_name,segment_index,");
            fprintf(file, "arc_length_inches,u_parameter,");
            fprintf(file, "x_inches,y_inches,theta_rad,theta_deg,");
            fprintf(file, "curvature,");
            fprintf(file, "dx_du,dy_du,");
            fprintf(file, "is_segment_start,is_segment_end,");
            fprintf(file, "is_group_start,is_group_end\n");
        }
        
        static void write_path_data(FILE* file, 
                                   const Path& path,
                                   double samples_per_inch)
        {
            double cumulative_arc_length = 0.0;
            
            for (size_t group_idx = 0; group_idx < path.get_profile_groups().size(); group_idx++)
            {
                const auto& group = path.get_profile_groups()[group_idx];
                
                for (size_t seg_idx = 0; seg_idx < group.segments.size(); seg_idx++)
                {
                    const auto& segment = group.segments[seg_idx];
                    double seg_length = segment->get_segment_length();
                    
                    // Calculate number of samples for this segment
                    int num_samples = std::max(2, static_cast<int>(seg_length * samples_per_inch));
                    
                    for (int i = 0; i < num_samples; i++)
                    {
                        double u = static_cast<double>(i) / (num_samples - 1);
                        
                        // Sample geometry at this u
                        double x, y;
                        segment->calc_point(u, x, y);
                        
                        auto deriv = segment->calc_first_deriv(u);
                        double theta = std::atan2(deriv.y(), deriv.x());
                        double curvature = segment->calc_curvature(u);
                        
                        // Arc length along this segment
                        double local_arc = u * seg_length;
                        double global_arc = cumulative_arc_length + local_arc;
                        
                        // Boundary flags
                        bool is_seg_start = (i == 0);
                        bool is_seg_end = (i == num_samples - 1);
                        bool is_group_start = (seg_idx == 0 && is_seg_start);
                        bool is_group_end = (seg_idx == group.segments.size() - 1 && is_seg_end);
                        
                        // Write row
                        fprintf(file, "%zu,%s,%zu,", 
                               group_idx, group.name.c_str(), seg_idx);
                        fprintf(file, "%.3f,%.4f,", global_arc, u);
                        fprintf(file, "%.3f,%.3f,%.6f,%.2f,", 
                               x, y, theta, theta * 180.0 / M_PI);
                        fprintf(file, "%.6f,", curvature);
                        fprintf(file, "%.3f,%.3f,", deriv.x(), deriv.y());
                        fprintf(file, "%d,%d,%d,%d\n",
                               is_seg_start, is_seg_end, is_group_start, is_group_end);
                    }
                    
                    cumulative_arc_length += seg_length;
                }
            }
        }
    };

} // namespace abclib::path