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
        double half_track_inches = track_width.to_inches() / 2.0;
        double omega_rad_per_sec = omega.to_rad_per_sec();
        double tangential_velocity = half_track_inches * omega_rad_per_sec;
        
        units::Velocity wheel_offset = units::Velocity::from_ips(tangential_velocity);
        
        return WheelVelocities{
            v - wheel_offset,
            v + wheel_offset
        };
    }

    inline BodyVelocities diff_drive_fk(
        units::Velocity v_left,
        units::Velocity v_right,
        units::Length track_width)
    {
        units::Velocity v = (v_left + v_right) / 2.0;
        
        units::Velocity delta_v(v_right - v_left);
        double omega_rad_per_sec = delta_v.to_ips() / track_width.to_inches();
        
        return BodyVelocities{
            v,
            units::AngularVelocity::from_rad_per_sec(omega_rad_per_sec)
        };
    }
}