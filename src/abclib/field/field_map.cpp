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
        // Compute sensor position in global frame
        double sensor_x, sensor_y;
        compute_sensor_global_position(
            robot_x, robot_y, robot_theta,
            sensor_offset_forward, sensor_offset_lateral,
            sensor_x, sensor_y);

        // Sensor's absolute heading in global frame
        double sensor_global_heading = robot_theta + sensor_bearing;

        // Determine which wall the sensor is pointing at
        Wall wall = get_nearest_wall(robot_x, robot_y, robot_theta, sensor_bearing);

        // Compute distance to that wall
        return compute_distance_to_wall(sensor_x, sensor_y, sensor_global_heading, wall);
    }

    FieldMap::Wall FieldMap::get_nearest_wall(
        double robot_x,
        double robot_y,
        double robot_theta,
        double sensor_bearing)
    {
        // Sensor's absolute heading in global frame
        double sensor_heading = robot_theta + sensor_bearing;

        // Normalize to [0, 2π)
        while (sensor_heading < 0.0) sensor_heading += 2.0 * M_PI;
        while (sensor_heading >= 2.0 * M_PI) sensor_heading -= 2.0 * M_PI;

        // Convert to degrees for easier reasoning
        double heading_deg = sensor_heading * 180.0 / M_PI;

        // Determine primary direction (with 45° sectors)
        // East: -45° to 45° (or 315° to 45°)
        // North: 45° to 135°
        // West: 135° to 225°
        // South: 225° to 315°

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
        else // 225° to 315°
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

        // Prevent division by zero
        const double epsilon = 1e-9;

        switch (wall)
        {
        case Wall::NORTH:
            // Wall at y = NORTH_WALL, need sin(direction) > 0 (pointing north)
            if (sin_dir > epsilon)
            {
                return (NORTH_WALL - y) / sin_dir;
            }
            return -1.0;

        case Wall::SOUTH:
            // Wall at y = SOUTH_WALL, need sin(direction) < 0 (pointing south)
            if (sin_dir < -epsilon)
            {
                return (SOUTH_WALL - y) / sin_dir;
            }
            return -1.0;

        case Wall::EAST:
            // Wall at x = EAST_WALL, need cos(direction) > 0 (pointing east)
            if (cos_dir > epsilon)
            {
                return (EAST_WALL - x) / cos_dir;
            }
            return -1.0;

        case Wall::WEST:
            // Wall at x = WEST_WALL, need cos(direction) < 0 (pointing west)
            if (cos_dir < -epsilon)
            {
                return (WEST_WALL - x) / cos_dir;
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
            return NORTH_WALL;
        case Wall::SOUTH:
            return SOUTH_WALL;
        case Wall::EAST:
            return EAST_WALL;
        case Wall::WEST:
            return WEST_WALL;
        case Wall::NONE:
        default:
            return 0.0;
        }
    }

    bool FieldMap::is_inside_field(double x, double y, double margin)
    {
        return (x >= WEST_WALL + margin &&
                x <= EAST_WALL - margin &&
                y >= SOUTH_WALL + margin &&
                y <= NORTH_WALL - margin);
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
        // Transform sensor offset from body frame to global frame
        // Body frame: forward = x_body, left = y_body
        // Global frame: east = x_global, north = y_global
        
        double cos_theta = std::cos(robot_theta);
        double sin_theta = std::sin(robot_theta);

        // Rotation matrix from body to global:
        // [cos(θ)  -sin(θ)] [forward]
        // [sin(θ)   cos(θ)] [lateral]
        
        sensor_x_out = robot_x + sensor_offset_forward * cos_theta - sensor_offset_lateral * sin_theta;
        sensor_y_out = robot_y + sensor_offset_forward * sin_theta + sensor_offset_lateral * cos_theta;
    }
}