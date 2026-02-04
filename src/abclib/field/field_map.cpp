#include "abclib/field/field_map.hpp"
#include "abclib/math/angles.hpp"
#include <limits>

namespace abclib::field
{
    units::Length FieldMap::compute_expected_distance(
        const estimation::Pose &robot_pose,
        units::Length sensor_offset_forward,
        units::Length sensor_offset_lateral,
        units::Angle sensor_bearing) const
    {
        // Compute sensor's global pose using SE2 composition
        math::SE2 sensor_pose = compute_sensor_global_pose(
            robot_pose,
            sensor_offset_forward,
            sensor_offset_lateral,
            sensor_bearing);

        // Determine which wall the sensor is facing
        Wall wall = get_nearest_wall(robot_pose, sensor_bearing);

        // Compute distance to that wall
        return compute_distance_to_wall(sensor_pose, wall);
    }

    math::SE2 FieldMap::compute_sensor_global_pose(
        const estimation::Pose &robot_pose,
        units::Length sensor_offset_forward,
        units::Length sensor_offset_lateral,
        units::Angle sensor_bearing) const
    {
        // Use SE2::FromBodyFrame to handle the body frame -> SE2 coordinate swap
        // Body frame: (X, Y) = (lateral, forward)
        // This factory method maps: (forward, lateral, bearing) -> SE2(x=forward, y=lateral, theta)
        math::SE2 robot_to_sensor = math::SE2::FromBodyFrame(
            sensor_offset_forward.to_inches(),
            sensor_offset_lateral.to_inches(),
            sensor_bearing.to_radians());

        // Compose transformations: world -> robot -> sensor
        return robot_pose.se2 * robot_to_sensor;
    }

    FieldMap::Wall FieldMap::get_nearest_wall(
        const estimation::Pose &robot_pose,
        units::Angle sensor_bearing) const
    {
        // Sensor's global heading = robot heading + sensor bearing
        double sensor_heading = robot_pose.theta_rad() + sensor_bearing.to_radians();
        sensor_heading = math::normalize_angle(sensor_heading);

        double heading_deg = std::fmod(sensor_heading * 180.0 / M_PI, 360.0);
        if (heading_deg < 0.0)
            heading_deg += 360.0; // Ensure positive [0, 360)

        // Standard Math Frame Convention:
        // 0° = East, 90° = North, 180° = West, 270° = South
        if (heading_deg >= 315.0 || heading_deg < 45.0)
        {
            return Wall::EAST; // 0° faces East
        }
        else if (heading_deg >= 45.0 && heading_deg < 135.0)
        {
            return Wall::NORTH; // 90° faces North
        }
        else if (heading_deg >= 135.0 && heading_deg < 225.0)
        {
            return Wall::WEST; // 180° faces West
        }
        else // 225-315
        {
            return Wall::SOUTH; // 270° faces South
        }
    }

    units::Length FieldMap::compute_distance_to_wall(
        const math::SE2 &sensor_pose,
        Wall wall) const
    {
        double x = sensor_pose.x();
        double y = sensor_pose.y();
        double direction = sensor_pose.theta();

        // Standard math frame: direction is angle from +X axis (East)
        double dx_component = std::cos(direction);
        double dy_component = std::sin(direction);

        const double min_component = 1e-5; // Avoid division by near-zero

        double dist_inches = -1.0;

        switch (wall)
        {
        case Wall::NORTH:
            if (std::abs(dy_component) > min_component)
            {
                double dist = (north_wall() - y) / dy_component;
                dist_inches = (dist >= 0) ? dist : -1.0;
            }
            break;

        case Wall::SOUTH:
            if (std::abs(dy_component) > min_component)
            {
                double dist = (south_wall() - y) / dy_component;
                dist_inches = (dist >= 0) ? dist : -1.0;
            }
            break;

        case Wall::EAST:
            if (std::abs(dx_component) > min_component)
            {
                double dist = (east_wall() - x) / dx_component;
                dist_inches = (dist >= 0) ? dist : -1.0;
            }
            break;

        case Wall::WEST:
            if (std::abs(dx_component) > min_component)
            {
                double dist = (west_wall() - x) / dx_component;
                dist_inches = (dist >= 0) ? dist : -1.0;
            }
            break;

        case Wall::NONE:
        default:
            break;
        }

        return units::Length::from_inches(dist_inches);
    }

    units::Length FieldMap::get_wall_position(Wall wall) const
    {
        double pos_inches;
        switch (wall)
        {
        case Wall::NORTH:
            pos_inches = north_wall();
            break;
        case Wall::SOUTH:
            pos_inches = south_wall();
            break;
        case Wall::EAST:
            pos_inches = east_wall();
            break;
        case Wall::WEST:
            pos_inches = west_wall();
            break;
        case Wall::NONE:
        default:
            pos_inches = 0.0;
            break;
        }
        return units::Length::from_inches(pos_inches);
    }

    bool FieldMap::is_inside_field(const math::SE2 &pose, units::Length margin) const
    {
        double x = pose.x();
        double y = pose.y();
        double m = margin.to_inches();

        return (x >= west_wall() + m &&
                x <= east_wall() - m &&
                y >= south_wall() + m &&
                y <= north_wall() - m);
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

    FieldMap::Wall FieldMap::find_wall_intersection(const math::SE2 &sensor_pose) const
    {
        double x = sensor_pose.x();
        double y = sensor_pose.y();
        double theta = sensor_pose.theta();

        double dx = std::cos(theta);
        double dy = std::sin(theta);

        const double epsilon = 1e-9;

        // Calculate distance parameter t to each wall
        // Ray equation: point = (x, y) + t * (dx, dy)
        double min_t = std::numeric_limits<double>::infinity();
        Wall nearest_wall = Wall::NONE;

        // Check EAST wall
        if (std::abs(dx) > epsilon)
        {
            double t = (east_wall() - x) / dx;
            if (t > epsilon && t < min_t)
            {
                min_t = t;
                nearest_wall = Wall::EAST;
            }
        }

        // Check WEST wall
        if (std::abs(dx) > epsilon)
        {
            double t = (west_wall() - x) / dx;
            if (t > epsilon && t < min_t)
            {
                min_t = t;
                nearest_wall = Wall::WEST;
            }
        }

        // Check NORTH wall
        if (std::abs(dy) > epsilon)
        {
            double t = (north_wall() - y) / dy;
            if (t > epsilon && t < min_t)
            {
                min_t = t;
                nearest_wall = Wall::NORTH;
            }
        }

        // Check SOUTH wall
        if (std::abs(dy) > epsilon)
        {
            double t = (south_wall() - y) / dy;
            if (t > epsilon && t < min_t)
            {
                min_t = t;
                nearest_wall = Wall::SOUTH;
            }
        }

        return nearest_wall;
    }

}