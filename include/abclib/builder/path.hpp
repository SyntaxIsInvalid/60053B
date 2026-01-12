#pragma once

#include "profile_group.hpp"
#include "abclib/path/path_segment_interface.hpp"
#include <vector>
#include <stdexcept>

namespace abclib::path
{
    class Path
    {
    public:
        Path() = default;
        
        // Add a completed profile group
        void add_profile_group(ProfileGroup&& group)
        {
            groups_.push_back(std::move(group));
        }
        
        // Access
        const std::vector<ProfileGroup>& get_profile_groups() const 
        { 
            return groups_; 
        }
        
        size_t num_groups() const { return groups_.size(); }
        
        bool empty() const { return groups_.empty(); }
        
        // Get total path length
        double get_total_arc_length() const
        {
            double total = 0.0;
            for (const auto& group : groups_) {
                total += group.total_arc_length.to_inches();
            }
            return total;
        }
        
        // NEW: Get point at specific arc length along entire path
        Point get_point_at_arc_length(double s) const
        {
            if (groups_.empty()) {
                throw std::runtime_error("Path: cannot get point from empty path");
            }
            
            // Find which group contains this arc length
            double cumulative_length = 0.0;
            
            for (const auto& group : groups_) {
                double group_length = group.total_arc_length.to_inches();
                
                if (s <= cumulative_length + group_length) {
                    // This group contains the point
                    double local_s = s - cumulative_length;
                    
                    // Find which segment in the group
                    double segment_cumulative = 0.0;
                    for (const auto& segment : group.segments) {
                        double segment_length = segment->get_segment_length();
                        
                        if (local_s <= segment_cumulative + segment_length) {
                            // This segment contains the point
                            double segment_local_s = local_s - segment_cumulative;
                            double u = segment->arc_length_to_u(segment_local_s);
                            
                            double x, y;
                            segment->calc_point(u, x, y);
                            return Point(x, y);
                        }
                        
                        segment_cumulative += segment_length;
                    }
                }
                
                cumulative_length += group_length;
            }
            
            // If we get here, s is beyond the path - return end point
            const auto& last_group = groups_.back();
            const auto& last_segment = last_group.segments.back();
            double x, y;
            last_segment->calc_point(1.0, x, y);
            return Point(x, y);
        }
        
        Pose get_start_pose() const
        {
            if (groups_.empty()) {
                throw std::runtime_error("Path: empty path has no start pose");
            }
            return groups_.front().get_start_pose();
        }
        
        Pose get_end_pose() const
        {
            if (groups_.empty()) {
                throw std::runtime_error("Path: empty path has no end pose");
            }
            return groups_.back().get_end_pose();
        }
        
    private:
        std::vector<ProfileGroup> groups_;
    };

} // namespace abclib::path