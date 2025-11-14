#include "motor_tracking_wheel.hpp"

namespace abclib::hardware
{
    MotorTrackingWheel::MotorTrackingWheel(AdvancedMotorGroup *motors,
                                           units::Length diameter,
                                           units::Length wheel_offset)
        : motor_group_(motors),
          wheel_diameter_(diameter),
          offset_(wheel_offset)
    {
    }

    units::Length MotorTrackingWheel::get_distance()
    {
        // Get position returns the new Angle type
        units::Angle angle = motor_group_->get_position();
        
        // arc length = radius * angle (in radians)
        units::Length radius = wheel_diameter_ / 2.0;  // Clean!
        return radius * angle.to_radians();            // Clean!
    }

    units::Length MotorTrackingWheel::get_offset()
    {
        return offset_;
    }

    void MotorTrackingWheel::reset()
    {
        motor_group_->reset_position();
    }
}