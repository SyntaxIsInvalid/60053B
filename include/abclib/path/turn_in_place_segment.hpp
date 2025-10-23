#pragma once

#include "path_segment_interface.hpp"
#include "abclib/math/angles.hpp"
#include <cmath>
#include "abclib/units/units.hpp"
#include <stdexcept>

namespace abclib::path
{

    class TurnInPlaceSegment : public IPathSegment
    {
    public:
        /**
         * @brief Construct a turn-in-place segment (zero-radius turn)
         * @param center_pose Center pose where the turn happens [x, y, theta_start]
         * @param end_heading Final heading in radians
         * @param track_width Distance between left and right wheels (inches)
         *
         * The robot rotates about its center point from theta_start to end_heading.
         * Position (x, y) remains constant throughout the turn.
         * Arc length is calculated based on the actual wheel track width.
         *
         * @throws std::invalid_argument if angular displacement is zero or track_width <= 0
         */
        TurnInPlaceSegment(const Pose &center_pose, double end_heading, units::Distance track_width);

        // IPathSegment interface
        void calc_point(double u, double &x, double &y) const override;
        Point calc_first_deriv(double u) const override;
        Point calc_second_deriv(double u) const override;
        double calc_curvature(double u) const override;
        double get_segment_length() const override;
        const Pose &get_start_pose() const override;
        const Pose &get_end_pose() const override;

        // Additional getters
        double get_angular_displacement() const { return angular_displacement_; }
        units::Distance get_track_width() const { return track_width_; }
        bool is_turn_in_place() const override { return true; }
        double get_turning_radius() const { return track_width_.inches / 2.0; }

    private:
        Pose start_pose_;
        Pose end_pose_;
        double angular_displacement_; // Signed angular change (radians)
        double arc_length_;           // Arc length traced by wheels
        units::Distance track_width_; // Robot track width
    };

} // namespace abclib::path