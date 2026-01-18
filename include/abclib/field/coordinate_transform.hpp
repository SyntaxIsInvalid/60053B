#pragma once

#include "abclib/estimation/pose.hpp"
#include "abclib/field/alliance.hpp"
#include "abclib/field/field_config.hpp"
#include "abclib/units/units.hpp"
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
     * 1. Alliance Corner Frame (Display Coords):
     *    - Origin at alliance's starting corner
     *    - Y-axis: horizontal (positive = away from left wall)
     *    - X-axis: vertical (positive = toward opponent)
     *    - Heading: measured from +X axis (toward opponent/north)
     *    - Red: origin at bottom-left, Blue: origin at top-right
     * 
     * 2. Field Center Frame (Display Coords):
     *    - Origin at field center
     *    - Y-axis: horizontal (positive = toward east wall)
     *    - X-axis: vertical (positive = toward north/blue wall)
     *    - Heading: measured from +X axis (north)
     * 
     * 3. Standard Mathematical Frame:
     *    - Origin at field center
     *    - X-axis: horizontal (positive = toward east wall)
     *    - Y-axis: vertical (positive = toward north/blue wall)
     *    - Heading: measured counterclockwise from +X axis (east)
     */

    // ========================================================================
    // Alliance Corner <-> Field Center (both use Display coordinates)
    // ========================================================================

    /**
     * Convert from alliance corner frame to field center frame
     * Both frames use display coordinates (Y=horizontal, X=vertical)
     * 
     * Red alliance: simple translation
     * Blue alliance: 180° rotation + translation (corner at opposite end)
     */
    inline estimation::Pose alliance_corner_to_field_center(
        const estimation::Pose& corner_pose,
        Alliance alliance,
        const FieldConfig& field_config)
    {
        estimation::Pose center_pose;
        double half_width = field_config.width.to_inches() / 2.0;
        double half_height = field_config.height.to_inches() / 2.0;

        if (alliance == Alliance::BLUE)
        {
            // Blue corner at north - 180° rotation from center
            center_pose.set_x(-corner_pose.x_inches() + half_height);
            center_pose.set_y(-corner_pose.y_inches() + half_width);
            center_pose.set_theta(corner_pose.theta_rad() - M_PI);
        }
        else // RED
        {
            // Red corner at south - direct translation
            center_pose.set_x(corner_pose.x_inches() - half_height);
            center_pose.set_y(corner_pose.y_inches() - half_width);
            center_pose.set_theta(corner_pose.theta_rad());
        }

        center_pose.v = corner_pose.v;
        center_pose.omega = corner_pose.omega;
        return center_pose;
    }

    /**
     * Convert from field center frame to alliance corner frame
     * Inverse of alliance_corner_to_field_center
     */
    inline estimation::Pose field_center_to_alliance_corner(
        const estimation::Pose& center_pose,
        Alliance alliance,
        const FieldConfig& field_config)
    {
        estimation::Pose corner_pose;
        double half_width = field_config.width.to_inches() / 2.0;
        double half_height = field_config.height.to_inches() / 2.0;

        if (alliance == Alliance::BLUE)
        {
            corner_pose.set_x(-center_pose.x_inches() + half_height);
            corner_pose.set_y(-center_pose.y_inches() + half_width);
            corner_pose.set_theta(center_pose.theta_rad() + M_PI);
        }
        else // RED
        {
            corner_pose.set_x(center_pose.x_inches() + half_height);
            corner_pose.set_y(center_pose.y_inches() + half_width);
            corner_pose.set_theta(center_pose.theta_rad());
        }

        corner_pose.v = center_pose.v;
        corner_pose.omega = center_pose.omega;
        return corner_pose;
    }

    // ========================================================================
    // Display Coords <-> Standard Math Coords (both centered at field origin)
    // ========================================================================

    /**
     * Convert from display coordinates to standard mathematical coordinates
     * 
     * Display: Y=horizontal, X=vertical, heading from +X (north)
     * Standard: X=horizontal, Y=vertical, heading from +X (east)
     * 
     * Transformation: swap axes + rotate heading by +90°
     */
    inline estimation::Pose display_to_standard(const estimation::Pose& display_pose)
    {
        estimation::Pose standard_pose;
        
        // Swap axes: display (y,x) -> standard (x,y)
        standard_pose.set_x(display_pose.y_inches());  // horizontal
        standard_pose.set_y(display_pose.x_inches());  // vertical
        
        // Rotate heading: north (0°) becomes east (0°), so add 90°
        standard_pose.set_theta(math::normalize_angle(display_pose.theta_rad() + M_PI_2));
        
        // Velocity magnitudes are frame-independent
        standard_pose.v = display_pose.v;
        standard_pose.omega = display_pose.omega;
        
        return standard_pose;
    }

    /**
     * Convert from standard mathematical coordinates to display coordinates
     * Inverse of display_to_standard
     */
    inline estimation::Pose standard_to_display(const estimation::Pose& standard_pose)
    {
        estimation::Pose display_pose;
        
        // Swap axes: standard (x,y) -> display (y,x)
        display_pose.set_y(standard_pose.x_inches());  // horizontal
        display_pose.set_x(standard_pose.y_inches());  // vertical
        
        // Rotate heading: east (0°) becomes north (0°), so subtract 90°
        display_pose.set_theta(math::normalize_angle(standard_pose.theta_rad() - M_PI_2));
        
        // Velocity magnitudes are frame-independent
        display_pose.v = standard_pose.v;
        display_pose.omega = standard_pose.omega;
        
        return display_pose;
    }

    // ========================================================================
    // Convenience: Alliance Corner <-> Standard Math (composed transforms)
    // ========================================================================

    /**
     * Convert directly from alliance corner to standard math frame
     * Composed: corner -> center (display) -> standard
     */
    inline estimation::Pose alliance_corner_to_standard(
        const estimation::Pose& corner_pose,
        Alliance alliance,
        const FieldConfig& field_config)
    {
        auto center = alliance_corner_to_field_center(corner_pose, alliance, field_config);
        return display_to_standard(center);
    }

    /**
     * Convert directly from standard math to alliance corner frame
     * Composed: standard -> center (display) -> corner
     */
    inline estimation::Pose standard_to_alliance_corner(
        const estimation::Pose& standard_pose,
        Alliance alliance,
        const FieldConfig& field_config)
    {
        auto display = standard_to_display(standard_pose);
        return field_center_to_alliance_corner(display, alliance, field_config);
    }

} // namespace abclib::field