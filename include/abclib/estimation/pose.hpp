#pragma once
#include "abclib/units/units.hpp"

namespace abclib::estimation
{
    struct Pose
    {
        units::Length x;
        units::Length y;
        units::Angle theta;
        units::Velocity v;
        units::AngularVelocity omega;

        Pose() 
            : x(units::Length::from_inches(0)),
              y(units::Length::from_inches(0)),
              theta(units::Angle::from_radians(0)),
              v(units::Velocity::from_ips(0)),
              omega(units::AngularVelocity::from_rad_per_sec(0)) {}

        Pose(double x_in, double y_in, double theta_radians)
            : x(units::Length::from_inches(x_in)),
              y(units::Length::from_inches(y_in)),
              theta(units::Angle::from_radians(theta_radians)),
              v(units::Velocity::from_ips(0)),
              omega(units::AngularVelocity::from_rad_per_sec(0)) {}

        Pose(units::Length x_pos, units::Length y_pos, units::Angle heading,
             units::Velocity vel, units::AngularVelocity ang_vel)
            : x(x_pos), y(y_pos), theta(heading), v(vel), omega(ang_vel) {}

        // Accessors that return raw values in your preferred units (inches/radians)
        double x_inches() const { return x.to_inches(); }
        double y_inches() const { return y.to_inches(); }
        double theta_rad() const { return theta.to_radians(); }
        double theta_deg() const { return theta.to_degrees(); }
        
        // Setters
        void set_x(double x_in) { x = units::Length::from_inches(x_in); }
        void set_y(double y_in) { y = units::Length::from_inches(y_in); }
        void set_theta(double theta_radians) { theta = units::Angle::from_radians(theta_radians); }
    };
}