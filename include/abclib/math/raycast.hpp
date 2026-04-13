#pragma once

#include "abclib/field/field_map.hpp"
#include <cmath>
#include <limits>

namespace abclib::math
{
    /**
     * @brief Ray-AABB intersection against axis-aligned field walls
     * 
     * All inputs are floats for performance in particle filter loops.
     * Cast wall bounds once outside the loop using FieldMap::get_wall_bounds_f32().
     * 
     * @param sx sensor x position in world frame (inches)
     * @param sy sensor y position in world frame (inches)
     * @param dx ray direction x component (unit vector)
     * @param dy ray direction y component (unit vector)
     * @param bounds wall positions cached as floats
     * @return distance to nearest wall in inches, or -1.0f if no valid intersection
     */
    inline float raycast_aabb(
        float sx, float sy,
        float dx, float dy,
        const field::FieldMap::WallBounds& bounds)
    {
        const float epsilon = 1e-6f;
        float best_t = std::numeric_limits<float>::infinity();

        // Check east/west walls
        if (std::abs(dx) > epsilon)
        {
            float inv_dx = 1.0f / dx;

            float t = (bounds.east - sx) * inv_dx;
            if (t > epsilon)
            {
                float y = sy + t * dy;
                if (y >= bounds.south && y <= bounds.north)
                    best_t = std::min(best_t, t);
            }

            t = (bounds.west - sx) * inv_dx;
            if (t > epsilon)
            {
                float y = sy + t * dy;
                if (y >= bounds.south && y <= bounds.north)
                    best_t = std::min(best_t, t);
            }
        }

        // Check north/south walls
        if (std::abs(dy) > epsilon)
        {
            float inv_dy = 1.0f / dy;

            float t = (bounds.north - sy) * inv_dy;
            if (t > epsilon)
            {
                float x = sx + t * dx;
                if (x >= bounds.west && x <= bounds.east)
                    best_t = std::min(best_t, t);
            }

            t = (bounds.south - sy) * inv_dy;
            if (t > epsilon)
            {
                float x = sx + t * dx;
                if (x >= bounds.west && x <= bounds.east)
                    best_t = std::min(best_t, t);
            }
        }

        if (best_t == std::numeric_limits<float>::infinity())
            return -1.0f;

        // best_t is parameterized — multiply by ray length to get inches
        return best_t * std::hypot(dx, dy);
    }
}