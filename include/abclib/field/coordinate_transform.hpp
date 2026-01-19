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
     * Two implementations available:
     * 1. ALIGNED AXES (default, recommended)
     * 2. COUPLED AXES (alternative, pure SE2 composition)
     *
     * ===========================================================================
     * ALIGNED AXES VERSION (Currently Active)
     * ===========================================================================
     *
     * Coordinate System Definitions:
     *
     * 1. Alliance Corner Frame:
     *    - Origin: Alliance starting corner
     *    - X-axis: Horizontal (east) — SAME AS STANDARD
     *    - Y-axis: Vertical (north) — SAME AS STANDARD
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
     *    - 90° is always facing blue, -90° is always facing red
     *
     * Pros:
     *  + Intuitive: (12, 24) means "12 east, 24 north" in BOTH frames
     *  + Easy for users to understand positions
     * Cons:
     *  - Position and heading must be transformed separately
     *  - Cannot use pure SE2 composition for frame conversion
     *
     * ===========================================================================
     * COUPLED AXES VERSION (Alternative)
     * ===========================================================================
     *
     * 1. Alliance Corner Frame (Coupled):
     *    - Origin: Alliance starting corner
     *    - RED:
     *      * X-axis: North (toward blue) — ROTATED 90° from standard
     *      * Y-axis: West (90° CCW from X) — ROTATED 90° from standard
     *      * Heading: 0° = north
     *    - BLUE:
     *      * X-axis: South (toward red) — ROTATED -90° from standard
     *      * Y-axis: East (90° CCW from X) — ROTATED -90° from standard
     *      * Heading: 0° = south
     *
     * 2. Standard Mathematical Frame:
     *    - (same as aligned version)
     *
     * Pros:
     *  + Can use pure SE2 composition for all transformations
     *  + Mathematically elegant
     * Cons:
     *  - Confusing: Field center is (24, -12) in RED frame, (24, 12) in BLUE frame
     *  - Y-axis points WEST in RED, EAST in BLUE
     *  - Harder for users to reason about positions
     */

    // =======================================================================
    // ALIGNED AXES VERSION (DEFAULT - RECOMMENDED)
    // =======================================================================

    /**
     * Convert pose from alliance corner frame to standard math frame
     * (Aligned axes version)
     *
     * Transforms position by corner offset and heading by rotation offset.
     * Since axes are aligned, we translate position and rotate heading separately.
     */
    inline estimation::Pose alliance_corner_to_standard(
        const estimation::Pose &corner_pose,
        Alliance alliance,
        const FieldConfig &field_config)
    {
        using units::Angle;
        using units::Length;

        Length half_width = field_config.width / 2.0;
        Length half_height = field_config.height / 2.0;

        Length x_corner = corner_pose.x();
        Length y_corner = corner_pose.y();
        Angle theta_corner = corner_pose.theta();

        Length x_standard, y_standard;
        Angle theta_standard;

        if (alliance == Alliance::RED)
        {
            // Red corner at (-half_width, -half_height) in standard frame
            x_standard = x_corner - half_width;
            y_standard = y_corner - half_height;
            theta_standard = theta_corner + Angle::from_radians(M_PI_2);
        }
        else // BLUE
        {
            // Blue corner at (+half_width, +half_height) in standard frame
            // To go from corner to center: subtract the corner position
            x_standard = x_corner - half_width;  // CHANGED: was +
            y_standard = y_corner - half_height; // CHANGED: was +
            theta_standard = theta_corner + Angle::from_radians(-M_PI_2);
        }

        math::SE2 pose_standard(
            x_standard.to_inches(),
            y_standard.to_inches(),
            theta_standard.to_radians());

        return estimation::Pose(pose_standard, corner_pose.v, corner_pose.omega);
    }

    /**
     * Convert pose from standard math frame to alliance corner frame
     * (Aligned axes version)
     *
     * Inverse of alliance_corner_to_standard.
     */
    inline estimation::Pose standard_to_alliance_corner(
        const estimation::Pose &standard_pose,
        Alliance alliance,
        const FieldConfig &field_config)
    {
        using units::Angle;
        using units::Length;

        Length half_width = field_config.width / 2.0;
        Length half_height = field_config.height / 2.0;

        Length x_standard = standard_pose.x();
        Length y_standard = standard_pose.y();
        Angle theta_standard = standard_pose.theta();

        Length x_corner, y_corner;
        Angle theta_corner;

        if (alliance == Alliance::RED)
        {
            x_corner = x_standard + half_width;
            y_corner = y_standard + half_height;
            theta_corner = theta_standard - Angle::from_radians(M_PI_2);
        }
        else // BLUE
        {
            // Inverse: to go from center to corner, add the corner position
            x_corner = x_standard + half_width;  // CHANGED: was -
            y_corner = y_standard + half_height; // CHANGED: was -
            theta_corner = theta_standard - Angle::from_radians(-M_PI_2);
        }

        math::SE2 pose_corner(
            x_corner.to_inches(),
            y_corner.to_inches(),
            theta_corner.to_radians());

        return estimation::Pose(pose_corner, standard_pose.v, standard_pose.omega);
    }
    // =======================================================================
    // COUPLED AXES VERSION (ALTERNATIVE)
    // =======================================================================

    /**
     * Get the SE2 transformation from alliance corner frame to standard frame
     * (Coupled axes version - for pure SE2 composition)
     *
     * In this version, the corner frame axes are rotated to align with the
     * robot's "forward" direction:
     * - RED: X=north, Y=west, rotated +90° from standard
     * - BLUE: X=south, Y=east, rotated -90° from standard
     */
    inline math::SE2 get_corner_to_standard_transform_coupled(
        Alliance alliance,
        const FieldConfig &field_config)
    {
        double half_width = field_config.width.to_inches() / 2.0;
        double half_height = field_config.height.to_inches() / 2.0;

        if (alliance == Alliance::RED)
        {
            // Red corner at (-W/2, -H/2), frame rotated 90° (X=north, Y=west)
            return math::SE2(-half_width, -half_height, M_PI_2);
        }
        else // BLUE
        {
            // Blue corner at (+W/2, +H/2), frame rotated -90° (X=south, Y=east)
            return math::SE2(half_width, half_height, -M_PI_2);
        }
    }

    /**
     * Get the SE2 transformation from standard frame to alliance corner frame
     * (Coupled axes version)
     */
    inline math::SE2 get_standard_to_corner_transform_coupled(
        Alliance alliance,
        const FieldConfig &field_config)
    {
        return get_corner_to_standard_transform_coupled(alliance, field_config).inverse();
    }

    /**
     * Convert pose from alliance corner frame to standard math frame
     * (Coupled axes version - pure SE2 composition)
     *
     * WARNING: In this version, position coordinates have different meanings:
     * - RED corner frame: (x, y) = (north_offset, west_offset)
     * - BLUE corner frame: (x, y) = (south_offset, east_offset)
     *
     * Example: Field center on 24×48 field
     * - RED corner frame: (24, -12) — 24" north, 12" east (note: -12 in Y!)
     * - BLUE corner frame: (24, -12) — 24" south, 12" west (note: -12 in Y!)
     * - Standard frame: (0, 0)
     */
    inline estimation::Pose alliance_corner_to_standard_coupled(
        const estimation::Pose &corner_pose,
        Alliance alliance,
        const FieldConfig &field_config)
    {
        math::SE2 T_corner_to_standard = get_corner_to_standard_transform_coupled(
            alliance, field_config);

        // Pure SE2 composition - works because axes are coupled!
        math::SE2 pose_standard = T_corner_to_standard * corner_pose.se2;

        return estimation::Pose(pose_standard, corner_pose.v, corner_pose.omega);
    }

    /**
     * Convert pose from standard math frame to alliance corner frame
     * (Coupled axes version - pure SE2 composition)
     */
    inline estimation::Pose standard_to_alliance_corner_coupled(
        const estimation::Pose &standard_pose,
        Alliance alliance,
        const FieldConfig &field_config)
    {
        math::SE2 T_standard_to_corner = get_standard_to_corner_transform_coupled(
            alliance, field_config);

        // Pure SE2 composition - works because axes are coupled!
        math::SE2 pose_corner = T_standard_to_corner * standard_pose.se2;

        return estimation::Pose(pose_corner, standard_pose.v, standard_pose.omega);
    }

} // namespace abclib::field