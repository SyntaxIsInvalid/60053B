#pragma once

#include "abclib/units/units.hpp"
#include <cmath>
#include <algorithm>

namespace abclib::field
{
    /**
     * @brief Represents the VEX competition field geometry
     * 
     * Field is 12ft x 12ft (144" x 144") with center at origin (0, 0)
     * Coordinate system:
     *   +X = East
     *   +Y = North
     *   +Theta = Counter-clockwise from East (standard math convention)
     */
    class FieldMap
    {
    public:
        // Wall positions in inches (center of field at origin)
        static constexpr double NORTH_WALL = 72.0;
        static constexpr double SOUTH_WALL = -72.0;
        static constexpr double EAST_WALL = 72.0;
        static constexpr double WEST_WALL = -72.0;
        
        // Field dimensions
        static constexpr double FIELD_WIDTH = 144.0;   // inches
        static constexpr double FIELD_HEIGHT = 144.0;  // inches

        enum class Wall
        {
            NORTH,
            SOUTH,
            EAST,
            WEST,
            NONE  // For error cases
        };

        /**
         * @brief Compute the expected distance from a sensor to the wall it's pointing at
         * 
         * @param robot_x Robot x position (inches)
         * @param robot_y Robot y position (inches)
         * @param robot_theta Robot heading (radians, 0 = East, CCW+)
         * @param sensor_offset_forward Distance of sensor ahead of robot center (inches, + = forward)
         * @param sensor_offset_lateral Distance of sensor left of robot centerline (inches, + = left)
         * @param sensor_bearing Direction sensor points in body frame (radians, 0 = forward)
         * @return Expected distance reading (inches), or -1.0 if no wall intersection
         */
        static double compute_expected_distance(
            double robot_x,
            double robot_y,
            double robot_theta,
            double sensor_offset_forward,
            double sensor_offset_lateral,
            double sensor_bearing);

        /**
         * @brief Determine which wall a sensor is most likely pointing at
         * 
         * @param robot_x Robot x position (inches)
         * @param robot_y Robot y position (inches)
         * @param robot_theta Robot heading (radians)
         * @param sensor_bearing Direction sensor points in body frame (radians, 0 = forward)
         * @return The wall the sensor is pointing toward
         */
        static Wall get_nearest_wall(
            double robot_x,
            double robot_y,
            double robot_theta,
            double sensor_bearing);

        /**
         * @brief Compute distance to a specific wall from a point in a given direction
         * 
         * @param x Starting x position (inches)
         * @param y Starting y position (inches)
         * @param direction Heading direction (radians)
         * @param wall Which wall to compute distance to
         * @return Distance to wall (inches), or -1.0 if not pointing toward that wall
         */
        static double compute_distance_to_wall(
            double x,
            double y,
            double direction,
            Wall wall);

        /**
         * @brief Get the position of a wall
         * 
         * @param wall Which wall
         * @return Wall position in inches (x for EAST/WEST, y for NORTH/SOUTH)
         */
        static double get_wall_position(Wall wall);

        /**
         * @brief Check if a point is within the field boundaries
         * 
         * @param x X position (inches)
         * @param y Y position (inches)
         * @param margin Safety margin from walls (inches)
         * @return true if point is inside field, false otherwise
         */
        static bool is_inside_field(double x, double y, double margin = 0.0);

        /**
         * @brief Get wall name as string (for debugging)
         */
        static const char* wall_to_string(Wall wall);

    private:
        /**
         * @brief Helper to compute sensor position in global frame
         */
        static void compute_sensor_global_position(
            double robot_x,
            double robot_y,
            double robot_theta,
            double sensor_offset_forward,
            double sensor_offset_lateral,
            double& sensor_x_out,
            double& sensor_y_out);
    };
}