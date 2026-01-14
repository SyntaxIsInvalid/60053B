#pragma once

#include "abclib/units/units.hpp"
#include "abclib/field/field_config.hpp"
#include <cmath>
#include <algorithm>

namespace abclib::field
{
    class FieldMap
    {
    private:
        FieldConfig config_;
        
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

        explicit FieldMap(const FieldConfig& config = FieldConfig::standard_vex())
            : config_(config) {}

        double compute_expected_distance(
            double robot_x,
            double robot_y,
            double robot_theta,
            double sensor_offset_forward,
            double sensor_offset_lateral,
            double sensor_bearing);

        Wall get_nearest_wall(
            double robot_x,
            double robot_y,
            double robot_theta,
            double sensor_bearing);

        double compute_distance_to_wall(
            double x,
            double y,
            double direction,
            Wall wall);

        double get_wall_position(Wall wall);

        bool is_inside_field(double x, double y, double margin = 0.0);

        static const char* wall_to_string(Wall wall);

    private:
        void compute_sensor_global_position(
            double robot_x,
            double robot_y,
            double robot_theta,
            double sensor_offset_forward,
            double sensor_offset_lateral,
            double& sensor_x_out,
            double& sensor_y_out);
    };
}