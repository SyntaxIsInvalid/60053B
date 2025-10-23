#include "motor_tracking_wheel.hpp"

namespace abclib::hardware
{
    MotorTrackingWheel::MotorTrackingWheel(AdvancedMotorGroup *motors,
                                           units::Distance diameter,
                                           units::Distance wheel_offset)
        : motor_group_(motors),
          wheel_diameter_(diameter),
          offset_(wheel_offset)
    {
    }

    units::Distance MotorTrackingWheel::get_distance()
    {
        units::Degrees degrees = motor_group_->get_position();
        double angle_rad = degrees.to_radians().value;
        return units::Distance::from_inches((wheel_diameter_.inches / 2.0) * angle_rad);
    }

    units::Distance MotorTrackingWheel::get_offset()
    {
        return offset_;
    }

    void MotorTrackingWheel::reset()
    {
        motor_group_->reset_position();
    }
}
