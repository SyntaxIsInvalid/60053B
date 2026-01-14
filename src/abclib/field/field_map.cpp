#include "abclib/field/field_map.hpp"
#include <limits>

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

        while (sensor_heading < 0.0) sensor_heading += 2.0 * M_PI;
        while (sensor_heading >= 2.0 * M_PI) sensor_heading -= 2.0 * M_PI;

        double heading_deg = sensor_heading * 180.0 / M_PI;

        if (heading_deg >= 315.0 || heading_deg < 45.0)
        {
            return Wall::EAST;
        }
        else if (heading_deg >= 45.0 && heading_deg < 135.0)
        {
            return Wall::NORTH;
        }
        else if (heading_deg >= 135.0 && heading_deg < 225.0)
        {
            return Wall::WEST;
        }
        else
        {
            return Wall::SOUTH;
        }
    }

    double FieldMap::compute_distance_to_wall(
        double x,
        double y,
        double direction,
        Wall wall)
    {
        double cos_dir = std::cos(direction);
        double sin_dir = std::sin(direction);

        const double epsilon = 1e-9;

        switch (wall)
        {
        case Wall::NORTH:
            if (sin_dir > epsilon)
            {
                return (north_wall() - y) / sin_dir;
            }
            return -1.0;

        case Wall::SOUTH:
            if (sin_dir < -epsilon)
            {
                return (south_wall() - y) / sin_dir;
            }
            return -1.0;

        case Wall::EAST:
            if (cos_dir > epsilon)
            {
                return (east_wall() - x) / cos_dir;
            }
            return -1.0;

        case Wall::WEST:
            if (cos_dir < -epsilon)
            {
                return (west_wall() - x) / cos_dir;
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

    const char* FieldMap::wall_to_string(Wall wall)
    {
        switch (wall)
        {
        case Wall::NORTH: return "NORTH";
        case Wall::SOUTH: return "SOUTH";
        case Wall::EAST: return "EAST";
        case Wall::WEST: return "WEST";
        case Wall::NONE: return "NONE";
        default: return "UNKNOWN";
        }
    }

    void FieldMap::compute_sensor_global_position(
        double robot_x,
        double robot_y,
        double robot_theta,
        double sensor_offset_forward,
        double sensor_offset_lateral,
        double& sensor_x_out,
        double& sensor_y_out)
    {
        double cos_theta = std::cos(robot_theta);
        double sin_theta = std::sin(robot_theta);
        
        sensor_x_out = robot_x + sensor_offset_forward * cos_theta - sensor_offset_lateral * sin_theta;
        sensor_y_out = robot_y + sensor_offset_forward * sin_theta + sensor_offset_lateral * cos_theta;
    }
}