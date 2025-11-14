#pragma once

#include "abclib/units/units.hpp"

namespace abclib::kinematics
{
    struct WheelVelocities
    {
        units::Velocity left;
        units::Velocity right;
    };

    struct BodyVelocities
    {
        units::Velocity v;
        units::AngularVelocity omega;
    };

    inline WheelVelocities diff_drive_ik(
        units::Velocity v, 
        units::AngularVelocity omega, 
        units::Length track_width)
    {
        // Calculate half track width
        units::Length half_track = track_width / 2.0;
        
        // v_left = v - (track_width/2) * omega
        // v_right = v + (track_width/2) * omega
        return WheelVelocities{
            v - half_track * omega,
            v + half_track * omega
        };
    }

    inline BodyVelocities diff_drive_fk(
        units::Velocity v_left,
        units::Velocity v_right,
        units::Length track_width)
    {
        // v = (v_left + v_right) / 2
        // omega = (v_right - v_left) / track_width
        return BodyVelocities{
            (v_left + v_right) / 2.0,
            (v_right - v_left) / track_width
        };
    }
}