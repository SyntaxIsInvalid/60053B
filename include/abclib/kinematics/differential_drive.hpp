#pragma once

#include "abclib/units/units.hpp"

namespace abclib::kinematics
{
    struct WheelVelocities
    {
        units::WheelLinearVelocity left;
        units::WheelLinearVelocity right;
    };

    struct BodyVelocities
    {
        units::BodyLinearVelocity v;
        units::BodyAngularVelocity omega;
    };

    inline WheelVelocities diff_drive_ik(
        units::BodyLinearVelocity v, 
        units::BodyAngularVelocity omega, 
        units::Distance track_width)
    {
        double half_track = track_width.inches / 2.0;
        return WheelVelocities{
            units::WheelLinearVelocity(v.inches_per_sec - half_track * omega.rad_per_sec),
            units::WheelLinearVelocity(v.inches_per_sec + half_track * omega.rad_per_sec)
        };
    }

    inline BodyVelocities diff_drive_fk(
        units::WheelLinearVelocity v_left,
        units::WheelLinearVelocity v_right,
        units::Distance track_width)
    {
        return BodyVelocities{
            units::BodyLinearVelocity((v_right.inches_per_sec + v_left.inches_per_sec) / 2.0),
            units::BodyAngularVelocity((v_right.inches_per_sec - v_left.inches_per_sec) / track_width.inches)
        };
    }
}