#pragma once

#include "abclib/estimation/pose.hpp"
#include "abclib/field/alliance.hpp"
#include "abclib/field/field_config.hpp"
#include "abclib/math/SE2.hpp"
#include "abclib/math/angles.hpp"
#include <cmath>

namespace abclib::field
{
    /**
     * @file coordinate_transforms.hpp
     * @brief Coordinate system transformations for VEX field
     * 
     * Coordinate System Definitions:
     * 
     * 1. Alliance Corner Frame:
     *    - Origin: Alliance starting corner
     *    - X-axis: Horizontal (east)
     *    - Y-axis: Vertical (north)
     *    - Heading: 0° = toward opponent
     *      * Red: 0° = north (toward blue)
     *      * Blue: 0° = south (toward red)
     *    - Red corner at field position: (-half_width, -half_height)
     *    - Blue corner at field position: (+half_width, +half_height)
     * 
     * 2. Standard Mathematical Frame (Field-Centered):
     *    - Origin: Field center
     *    - X-axis: Horizontal (east)
     *    - Y-axis: Vertical (north)
     *    - Heading: 0° = east, 90° = north, counterclockwise positive
     *    - 90 is always facing blue, -90 is always facing red
     */

    /**
     * Get the SE2 transformation from alliance corner frame to standard frame
     * 
     * This is a pure SE2 transformation (translation + rotation):
     * - Red: Translate from corner (-W/2, -H/2), rotate heading by +90°
     * - Blue: Translate from corner (+W/2, +H/2), rotate heading by -90°
     */
    inline math::SE2 get_corner_to_standard_transform(
        Alliance alliance,
        const FieldConfig& field_config)
    {
        double half_width = field_config.width.to_inches() / 2.0;
        double half_height = field_config.height.to_inches() / 2.0;
        
        if (alliance == Alliance::RED)
        {
            // Red corner at (-half_width, -half_height) in standard frame
            // Red's 0° (toward blue/north) = 90° in standard frame
            return math::SE2(-half_width, -half_height, M_PI_2);
        }
        else // BLUE
        {
            // Blue corner at (+half_width, +half_height) in standard frame
            // Blue's 0° (toward red/south) = -90° in standard frame
            return math::SE2(half_width, half_height, -M_PI_2);
        }
    }

    /**
     * Get the SE2 transformation from standard frame to alliance corner frame
     * (Inverse of corner-to-standard)
     */
    inline math::SE2 get_standard_to_corner_transform(
        Alliance alliance,
        const FieldConfig& field_config)
    {
        return get_corner_to_standard_transform(alliance, field_config).inverse();
    }

    /**
     * Convert pose from alliance corner frame to standard math frame
     * 
     * Pure SE2 composition - no axis reinterpretation needed!
     */
    inline estimation::Pose alliance_corner_to_standard(
        const estimation::Pose& corner_pose,
        Alliance alliance,
        const FieldConfig& field_config)
    {
        math::SE2 T_corner_to_standard = get_corner_to_standard_transform(alliance, field_config);
        math::SE2 pose_standard = T_corner_to_standard * corner_pose.se2;
        
        return estimation::Pose(pose_standard, corner_pose.v, corner_pose.omega);
    }

    /**
     * Convert pose from standard math frame to alliance corner frame
     * 
     * Pure SE2 composition - no axis reinterpretation needed!
     */
    inline estimation::Pose standard_to_alliance_corner(
        const estimation::Pose& standard_pose,
        Alliance alliance,
        const FieldConfig& field_config)
    {
        math::SE2 T_standard_to_corner = get_standard_to_corner_transform(alliance, field_config);
        math::SE2 pose_corner = T_standard_to_corner * standard_pose.se2;
        
        return estimation::Pose(pose_corner, standard_pose.v, standard_pose.omega);
    }

} // namespace abclib::field