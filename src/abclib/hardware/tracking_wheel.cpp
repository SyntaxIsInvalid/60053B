#include "abclib/hardware/tracking_wheel.hpp"
#include <cmath>

namespace abclib::hardware {
    TrackingWheel::TrackingWheel(pros::Rotation* rotation_sensor, 
                                units::Distance diameter, 
                                units::Distance offset)
        : rotation_sensor(rotation_sensor),
          diameter(diameter),
          radius(diameter / 2.0),
          offset(offset)
    {}

    units::Distance TrackingWheel::get_distance() {
        double angle_rad = (rotation_sensor->get_position() / 100.0) * M_PI / 180.0;
        return units::Distance::from_inches(radius.inches * angle_rad);
    }

    units::Distance TrackingWheel::get_offset() {
        return offset;
    }

    void TrackingWheel::reset() {
        rotation_sensor->reset();
    }
}