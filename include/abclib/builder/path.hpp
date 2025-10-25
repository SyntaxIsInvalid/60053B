#pragma once

#include "profile_group.hpp"
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
                total += group.total_arc_length;
            }
            return total;
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