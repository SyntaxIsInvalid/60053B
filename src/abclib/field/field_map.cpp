#include "abclib/field/field_map.hpp"
#include <limits>
#include "abclib/math/angles.hpp"
namespace abclib::field
{
    double FieldMap::compute_expected_distance(
        double robot_x,
        double robot_y,
        double robot_theta,
        double sensor_offset_forward,
        double sensor_offset_lateral,
        double sensor_bearing)
    {
        double sensor_x, sensor_y;
        compute_sensor_global_position(
            robot_x, robot_y, robot_theta,
            sensor_offset_forward, sensor_offset_lateral,
            sensor_x, sensor_y);

        double sensor_global_heading = robot_theta + sensor_bearing;
        Wall wall = get_nearest_wall(robot_x, robot_y, robot_theta, sensor_bearing);

        return compute_distance_to_wall(sensor_x, sensor_y, sensor_global_heading, wall);
    }

    FieldMap::Wall FieldMap::get_nearest_wall(
        double robot_x,
        double robot_y,
        double robot_theta,
        double sensor_bearing)
    {
        double sensor_heading = robot_theta + sensor_bearing;

        sensor_heading = math::normalize_angle(sensor_heading);
        double heading_deg = sensor_heading * 180.0 / M_PI;

        // Display coords: 0° = North, 90° = West, 180° = South, 270° = East
        if (heading_deg >= 315.0 || heading_deg < 45.0)
        {
            return Wall::NORTH; // Changed from EAST
        }
        else if (heading_deg >= 45.0 && heading_deg < 135.0)
        {
            return Wall::WEST; // This stays
        }
        else if (heading_deg >= 135.0 && heading_deg < 225.0)
        {
            return Wall::SOUTH; // Changed from WEST
        }
        else // 225-315
        {
            return Wall::EAST; // Changed from SOUTH
        }
    }

    double FieldMap::compute_distance_to_wall(
        double x, // This is the VERTICAL coordinate (display coords)
        double y, // This is the HORIZONTAL coordinate (display coords)
        double direction,
        Wall wall)
    {
        // Display coords: 0° points North (+X), so:
        // dx/dt = cos(theta) for VERTICAL motion (X-axis)
        // dy/dt = sin(theta) for HORIZONTAL motion (Y-axis)
        double dx_component = std::cos(direction); // VERTICAL (X-axis) component - FIXED
        double dy_component = std::sin(direction); // HORIZONTAL (Y-axis) component - FIXED

        const double epsilon = 1e-9;

        switch (wall)
        {
        case Wall::NORTH:               // +X direction (vertical up)
            if (dx_component > epsilon) // Moving in +X direction
            {
                return (north_wall() - x) / dx_component; // Use X not Y!
            }
            return -1.0;

        case Wall::SOUTH:                // -X direction (vertical down)
            if (dx_component < -epsilon) // Moving in -X direction
            {
                return (south_wall() - x) / dx_component; // Use X not Y!
            }
            return -1.0;

        case Wall::EAST:                // +Y direction (horizontal right)
            if (dy_component > epsilon) // Moving in +Y direction
            {
                return (east_wall() - y) / dy_component; // Use Y
            }
            return -1.0;

        case Wall::WEST:                 // -Y direction (horizontal left)
            if (dy_component < -epsilon) // Moving in -Y direction
            {
                return (west_wall() - y) / dy_component; // Use Y
            }
            return -1.0;

        case Wall::NONE:
        default:
            return -1.0;
        }
    }

    double FieldMap::get_wall_position(Wall wall)
    {
        switch (wall)
        {
        case Wall::NORTH:
            return north_wall();
        case Wall::SOUTH:
            return south_wall();
        case Wall::EAST:
            return east_wall();
        case Wall::WEST:
            return west_wall();
        case Wall::NONE:
        default:
            return 0.0;
        }
    }

    bool FieldMap::is_inside_field(double x, double y, double margin)
    {
        return (x >= west_wall() + margin &&
                x <= east_wall() - margin &&
                y >= south_wall() + margin &&
                y <= north_wall() - margin);
    }

    const char *FieldMap::wall_to_string(Wall wall)
    {
        switch (wall)
        {
        case Wall::NORTH:
            return "NORTH";
        case Wall::SOUTH:
            return "SOUTH";
        case Wall::EAST:
            return "EAST";
        case Wall::WEST:
            return "WEST";
        case Wall::NONE:
            return "NONE";
        default:
            return "UNKNOWN";
        }
    }

    void FieldMap::compute_sensor_global_position(
        double robot_x,
        double robot_y,
        double robot_theta,
        double sensor_offset_forward,
        double sensor_offset_lateral,
        double &sensor_x_out,
        double &sensor_y_out)
    {
        double cos_theta = std::cos(robot_theta);
        double sin_theta = std::sin(robot_theta);

        sensor_x_out = robot_x + sensor_offset_forward * cos_theta - sensor_offset_lateral * sin_theta;
        sensor_y_out = robot_y + sensor_offset_forward * sin_theta + sensor_offset_lateral * cos_theta;
    }
}