#pragma once

#include "abclib/units/units.hpp"
#include "abclib/field/field_config.hpp"
#include "abclib/field/alliance.hpp"
#include "abclib/estimation/pose.hpp"
#include "abclib/math/SE2.hpp"
#include <cmath>
#include <algorithm>

namespace abclib::field
{
    class FieldMap
    {
    private:
        FieldConfig config_;
        Alliance alliance_;

        // Computed wall positions (assuming center at origin)
        double north_wall() const { return config_.height.to_inches() / 2.0; }
        double south_wall() const { return -config_.height.to_inches() / 2.0; }
        double east_wall() const { return config_.width.to_inches() / 2.0; }
        double west_wall() const { return -config_.width.to_inches() / 2.0; }

    public:
        enum class Wall
        {
            NORTH,
            SOUTH,
            EAST,
            WEST,
            NONE
        };

        explicit FieldMap(
            const FieldConfig &config = FieldConfig::standard_vex(),
            Alliance alliance = Alliance::BLUE)
            : config_(config), alliance_(alliance) {}

        void set_alliance(Alliance alliance) { alliance_ = alliance; }
        Alliance get_alliance() const { return alliance_; }
        const FieldConfig& get_field_config() const { return config_; }
        /**
         * Compute expected distance reading from a sensor
         * All inputs in standard math frame
         */
        units::Length compute_expected_distance(
            const estimation::Pose& robot_pose,
            units::Length sensor_offset_forward,
            units::Length sensor_offset_lateral,
            units::Angle sensor_bearing) const;

        /**
         * Determine which wall a sensor is facing
         */
        Wall get_nearest_wall(
            const estimation::Pose& robot_pose,
            units::Angle sensor_bearing) const;

        /**
         * Compute distance from sensor pose to a specific wall
         */
        units::Length compute_distance_to_wall(
            const math::SE2& sensor_pose,
            Wall wall) const;

        /**
         * Get the position of a wall (returns coordinate value)
         */
        units::Length get_wall_position(Wall wall) const;

        /**
         * Check if a pose is inside the field with optional margin
         */
        bool is_inside_field(const math::SE2& pose, units::Length margin = units::Length::from_inches(0)) const;

        /**
         * Convert wall enum to string
         */
        static const char *wall_to_string(Wall wall);

        /**
         * Compute sensor's global pose using SE2 transformations
         */
        math::SE2 compute_sensor_global_pose(
            const estimation::Pose& robot_pose,
            units::Length sensor_offset_forward,
            units::Length sensor_offset_lateral,
            units::Angle sensor_bearing) const;
    };
}