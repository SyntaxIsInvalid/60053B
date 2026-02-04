#pragma once
#include <string>
#include <vector>
#include <cstdio>
#include "abclib/units/units.hpp"
#include "abclib/field/field_map.hpp"

namespace abclib::ui
{
    /**
     * @brief Format a vector of numeric values with validity flags
     * 
     * @param values Vector of values to format
     * @param valid Vector of validity flags (same size as values)
     * @param format Printf-style format string for each value (e.g. "%.0f" for integers)
     * @return Formatted string like "[1205, ---, 823, ---]"
     */
    template<typename T>
    inline std::string format_value_vector(
        const std::vector<T>& values,
        const std::vector<bool>& valid,
        const char* format)
    {
        std::string result = "[";
        for (size_t i = 0; i < values.size(); i++)
        {
            if (i > 0) result += ",";
            
            if (i < valid.size() && valid[i])
            {
                char buf[32];
                snprintf(buf, sizeof(buf), format, values[i]);
                result += buf;
            }
            else
            {
                result += "---";
            }
        }
        result += "]";
        return result;
    }
    
    /**
     * @brief Format a vector of walls
     * 
     * @param walls Vector of wall enums
     * @param valid Vector of validity flags
     * @return Formatted string like "[N, --, S, --]"
     */
    inline std::string format_wall_vector(
        const std::vector<field::FieldMap::Wall>& walls,
        const std::vector<bool>& valid)
    {
        std::string result = "[";
        for (size_t i = 0; i < walls.size(); i++)
        {
            if (i > 0) result += ",";
            
            if (i < valid.size() && valid[i])
            {
                // Use existing wall_to_string from ScreenManager
                switch (walls[i])
                {
                    case field::FieldMap::Wall::NORTH: result += "N"; break;
                    case field::FieldMap::Wall::SOUTH: result += "S"; break;
                    case field::FieldMap::Wall::EAST:  result += "E"; break;
                    case field::FieldMap::Wall::WEST:  result += "W"; break;
                    default: result += "-"; break;
                }
            }
            else
            {
                result += "-";
            }
        }
        result += "]";
        return result;
    }
    
    /**
     * @brief Format a vector of validity flags
     * 
     * @param valid Vector of validity flags
     * @return Formatted string like "[OK, --, OK, --]"
     */
    inline std::string format_valid_vector(const std::vector<bool>& valid)
    {
        std::string result = "[";
        for (size_t i = 0; i < valid.size(); i++)
        {
            if (i > 0) result += ",";
            result += valid[i] ? "OK" : "--";
        }
        result += "]";
        return result;
    }
    
    /**
     * @brief Format a vector of blend factors
     * 
     * @param factors Vector of blend factors (0.0 to 1.0)
     * @return Formatted string like "[0.2, 0.2, 0.2, 0.2]"
     */
    inline std::string format_blend_vector(const std::vector<double>& factors)
    {
        std::string result = "[";
        for (size_t i = 0; i < factors.size(); i++)
        {
            if (i > 0) result += ",";
            char buf[16];
            snprintf(buf, sizeof(buf), "%.1f", factors[i]);
            result += buf;
        }
        result += "]";
        return result;
    }
    
    /**
     * @brief Format pose as compact tuple
     * 
     * @param x X coordinate in inches
     * @param y Y coordinate in inches
     * @param theta_deg Heading in degrees
     * @return Formatted string like "(12.3, 24.5, 45°)"
     */
    inline std::string format_pose_tuple(double x, double y, double theta_deg)
    {
        char buf[64];
        snprintf(buf, sizeof(buf), "(%.1f,%.1f,%.0f°)", x, y, theta_deg);
        return buf;
    }
    
    /**
     * @brief Format correction vector
     * 
     * @param dx X correction in inches
     * @param dy Y correction in inches
     * @return Formatted string like "(+0.1",-0.3")"
     */
    inline std::string format_correction(double dx, double dy)
    {
        char buf[64];
        snprintf(buf, sizeof(buf), "(%+.1f\",%+.1f\")", dx, dy);
        return buf;
    }

} // namespace abclib::ui