// field_config.hpp
#pragma once

#include "abclib/units/units.hpp"

namespace abclib::field
{
    struct FieldConfig {
        units::Length width;
        units::Length height;
        
        // Factory methods
        static FieldConfig standard_vex() {
            return FieldConfig{
                .width = units::Length::from_inches(144),
                .height = units::Length::from_inches(144)
            };
        }
        
        static FieldConfig custom(units::Length w, units::Length h) {
            return FieldConfig{
                .width = w,
                .height = h
            };
        }
    };
}