#include "abclib/hardware/tracking_wheel.hpp"

namespace abclib::hardware {
    TrackingWheel::TrackingWheel(pros::Rotation* rotation_sensor, 
                                units::Length diameter, 
                                units::Length offset)
        : rotation_sensor(rotation_sensor),
          diameter(diameter),
          radius(diameter / 2.0),  // No explicit construction needed!
          offset(offset)
    {}

    units::Length TrackingWheel::get_distance() {
        // Rotation sensor returns centidegrees
        double centidegrees = rotation_sensor->get_position();
        units::Angle angle = units::Angle::from_degrees(centidegrees / 100.0);
        

        return radius * angle.to_radians();
    }

    units::Length TrackingWheel::get_offset() {
        return offset;
    }

    void TrackingWheel::reset() {
        rotation_sensor->reset();
    }
}