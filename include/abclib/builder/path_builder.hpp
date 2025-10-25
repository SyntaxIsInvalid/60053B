#pragma once

#include "path.hpp"
#include "abclib/path/eta3_segment.hpp"
#include "abclib/path/straight_segment.hpp"
#include "abclib/math/angles.hpp"
#include <optional>

namespace abclib::path
{
    class PathBuilder
    {
    public:
        PathBuilder() = default;

        // Initialize starting pose
        PathBuilder &start(double x, double y, double heading)
        {
            current_pose_ = Pose(x, y, heading);
            has_start_ = true;
            return *this;
        }

        // Begin a new profile group
        PathBuilder &begin_profile(const std::string &name,
                                   units::BodyLinearVelocity max_velocity,
                                   double max_acceleration)
        {
            if (!has_start_)
            {
                throw std::runtime_error("PathBuilder: must call start() before begin_profile()");
            }

            // Finalize previous group if exists
            finalize_current_group();

            // Start new group
            current_group_ = ProfileGroup(name, max_velocity, max_acceleration);

            return *this;
        }

        // Add spline segment to current pose
        PathBuilder &spline_to(double x, double y, double heading)
        {
            ensure_current_group("spline_to");

            Pose end_pose(x, y, heading);

            // Create spline segment (uses heuristic eta, zero kappa)
            auto segment = std::make_unique<Eta3PathSegment>(
                current_pose_,
                end_pose);

            current_group_->segments.push_back(std::move(segment));
            current_pose_ = end_pose;

            return *this;
        }

        // Add straight segment
        // Version 1: Just position - validates current heading
        PathBuilder &straight_to(double x, double y)
        {
            ensure_current_group("straight_to");

            double dx = x - current_pose_(0);
            double dy = y - current_pose_(1);
            double required_heading = std::atan2(dy, dx);

            double heading_error = math::normalize_angle(current_pose_(2) - required_heading);

            if (std::abs(heading_error) > 1e-3)
            {
                throw std::invalid_argument(
                    "PathBuilder: straight_to(" + std::to_string(x) + ", " + std::to_string(y) +
                    ") requires current heading to match line direction.\n" +
                    "Current heading: " + std::to_string(current_pose_(2) * 180.0 / M_PI) + "°\n" +
                    "Required heading: " + std::to_string(required_heading * 180.0 / M_PI) + "°\n\n" +
                    "Solutions:\n" +
                    "1. Use: .straight_to_heading(" + std::to_string(x) + ", " +
                    std::to_string(y) + ", " + std::to_string(required_heading) + ")\n" +
                    "2. Use splineTo() instead (can handle any heading)\n" +
                    "3. Insert turn_in_place() or break_continuity() before this call");
            }

            Pose end_pose(x, y, required_heading);
            auto segment = std::make_unique<StraightSegment>(current_pose_, end_pose);
            current_group_->segments.push_back(std::move(segment));
            current_pose_ = end_pose;

            return *this;
        }

        // Version 2: With explicit heading - implicitly breaks continuity
        PathBuilder &straight_to_heading(double x, double y, double heading)
        {
            ensure_current_group("straight_to_heading");

            double dx = x - current_pose_(0);
            double dy = y - current_pose_(1);
            double geometric_heading = std::atan2(dy, dx);

            // Validate that requested heading matches line direction
            double heading_error = math::normalize_angle(heading - geometric_heading);
            if (std::abs(heading_error) > 1e-3)
            {
                throw std::invalid_argument(
                    std::string("PathBuilder: straight_to_heading() heading must align with line direction.\n") +
                    "Line direction: " + std::to_string(geometric_heading * 180.0 / M_PI) + "°\n" +
                    "Requested heading: " + std::to_string(heading * 180.0 / M_PI) + "°");
            }

            // Update current pose heading (breaks G2 continuity at this point)
            current_pose_(2) = heading;

            Pose end_pose(x, y, heading);
            auto segment = std::make_unique<StraightSegment>(current_pose_, end_pose);
            current_group_->segments.push_back(std::move(segment));
            current_pose_ = end_pose;

            return *this;
        }

        // Build final path
        Path build()
        {
            if (!has_start_)
            {
                throw std::runtime_error("PathBuilder: cannot build path without calling start()");
            }

            // Finalize last group
            finalize_current_group();

            if (path_.empty())
            {
                throw std::runtime_error("PathBuilder: cannot build empty path");
            }

            // Return by move
            Path result = std::move(path_);

            // Reset builder state for potential reuse
            reset();

            return result;
        }

    private:
        Pose current_pose_;
        bool has_start_ = false;

        std::optional<ProfileGroup> current_group_;
        Path path_;

        void ensure_current_group(const std::string &method_name)
        {
            if (!current_group_.has_value())
            {
                throw std::runtime_error(
                    "PathBuilder: must call begin_profile() before " + method_name + "()");
            }
        }

        void finalize_current_group()
        {
            if (current_group_.has_value())
            {
                if (!current_group_->segments.empty())
                {
                    current_group_->compute_arc_length();
                    path_.add_profile_group(std::move(*current_group_));
                }
                current_group_.reset();
            }
        }

        void reset()
        {
            has_start_ = false;
            current_group_.reset();
            path_ = Path();
        }
    };

} // namespace abclib::path