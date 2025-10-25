#pragma once

#include "path.hpp"
#include "abclib/path/eta3_segment.hpp"
#include "abclib/path/straight_segment.hpp"
#include "abclib/math/angles.hpp"
#include <optional>
#include "abclib/path/turn_in_place_segment.hpp"

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

        explicit PathBuilder(units::Distance track_width)
            : track_width_(track_width) {}

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
        PathBuilder &spline_to(double x, double y, double heading,
                               const std::optional<std::array<double, 6>> &eta = std::nullopt,
                               const std::optional<std::array<double, 4>> &kappa = std::nullopt)
        {
            ensure_current_group("spline_to");

            Pose end_pose(x, y, heading);

            // Convert arrays to Eigen vectors if provided
            std::optional<Eta3PathSegment::EtaVec> eta_vec;
            std::optional<Eta3PathSegment::KappaVec> kappa_vec;

            if (eta.has_value())
            {
                eta_vec = Eta3PathSegment::EtaVec::Map(eta->data());
            }
            if (kappa.has_value())
            {
                kappa_vec = Eta3PathSegment::KappaVec::Map(kappa->data());
            }

            auto segment = std::make_unique<Eta3PathSegment>(
                current_pose_,
                end_pose,
                eta_vec,
                kappa_vec);

            if (!current_group_->segments.empty())
            {
                validate_continuity(current_group_->segments.back().get(), segment.get());
            }

            current_group_->segments.push_back(std::move(segment));
            current_pose_ = end_pose;

            return *this;
        }

        // Add straight segment
        PathBuilder &straight_to(double x, double y, std::optional<double> heading = std::nullopt)
        {
            ensure_current_group("straight_to");

            double dx = x - current_pose_(0);
            double dy = y - current_pose_(1);
            double geometric_heading = std::atan2(dy, dx);

            double actual_heading;

            if (heading.has_value())
            {
                // Explicit heading provided - validate it matches line geometry
                double heading_error = math::normalize_angle(*heading - geometric_heading);
                if (std::abs(heading_error) > 1e-3)
                {
                    throw std::invalid_argument(
                        std::string("straightTo() heading must align with line direction.\n") +
                        "Line direction: " + std::to_string(geometric_heading * 180 / M_PI) + "°\n" +
                        "Provided heading: " + std::to_string(*heading * 180 / M_PI) + "°");
                }
                actual_heading = *heading;

                // Update current pose heading (allows breaking G2 continuity)
                current_pose_(2) = actual_heading;
            }
            else
            {
                // No heading provided - validate current heading matches line
                double heading_error = math::normalize_angle(current_pose_(2) - geometric_heading);
                if (std::abs(heading_error) > 1e-3)
                {
                    throw std::invalid_argument(
                        std::string("straightTo() target not aligned with current heading.\n") +
                        "Current: " + std::to_string(current_pose_(2) * 180 / M_PI) + "°\n" +
                        "Required: " + std::to_string(geometric_heading * 180 / M_PI) + "°\n" +
                        "Use .straightTo(x, y, heading) to override.");
                }
                actual_heading = geometric_heading;
            }

            Pose end_pose(x, y, actual_heading);
            auto segment = std::make_unique<StraightSegment>(current_pose_, end_pose);

            if (!current_group_->segments.empty())
            {
                validate_continuity(current_group_->segments.back().get(), segment.get());
            }

            current_group_->segments.push_back(std::move(segment));
            current_pose_ = end_pose;

            return *this;
        }

        // Turn in place to absolute heading (implicitly breaks continuity)
        PathBuilder &turn_in_place(double heading)
        {
            ensure_current_group("turn_in_place");

            // Finalize current profile group (turn breaks continuity)
            finalize_current_group();

            // Create turn-in-place segment
            auto segment = std::make_unique<TurnInPlaceSegment>(
                current_pose_,
                heading,
                track_width_);

            // Create a single-segment profile group for the turn
            // Use reasonable defaults for turning - user can override with beginProfile
            ProfileGroup turn_group(
                "turn_in_place",
                units::BodyLinearVelocity(12.0), // moderate angular velocity
                30.0                             // moderate angular acceleration
            );

            turn_group.segments.push_back(std::move(segment));
            turn_group.compute_arc_length();
            path_.add_profile_group(std::move(turn_group));

            // Update current pose to new heading
            current_pose_(2) = heading;

            return *this;
        }

        PathBuilder &break_continuity()
        {
            ensure_current_group("break_continuity");

            // Just finalize the current group
            // Next beginProfile() will start a new group at the same pose
            finalize_current_group();

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
        units::Distance track_width_;

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

        void validate_continuity(const IPathSegment *prev, const IPathSegment *next)
        {
            // Check position continuity (G0)
            auto prev_end = prev->get_end_pose();
            auto next_start = next->get_start_pose();

            double pos_error = (prev_end.head<2>() - next_start.head<2>()).norm();
            if (pos_error > 1e-6)
            {
                throw std::runtime_error(
                    "PathBuilder: Position discontinuity detected. "
                    "Segments must connect end-to-start.");
            }

            // Check heading continuity (G1)
            double heading_error = math::normalize_angle(
                prev_end(2) - next_start(2));
            if (std::abs(heading_error) > 1e-6)
            {
                throw std::runtime_error(
                    "PathBuilder: Heading discontinuity detected. "
                    "Use turn_in_place() or break_continuity() for sharp turns.");
            }

            // G2 continuity check is harder - would need curvature methods
            // Your segments don't expose curvature at endpoints yet
            // Can add later if needed
        }
    };

} // namespace abclib::path