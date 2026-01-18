#pragma once
#include "abclib/units/units.hpp"
#include "abclib/math/SE2.hpp"

namespace abclib::estimation
{
    struct Pose
    {
        math::SE2 se2;  // Position and orientation
        units::Velocity v;
        units::AngularVelocity omega;

        // ============= CONSTRUCTORS =============
        
        /** Default constructor - identity pose at origin */
        Pose() 
            : se2(math::SE2::Identity()),
              v(units::Velocity::from_ips(0)),
              omega(units::AngularVelocity::from_rad_per_sec(0)) {}

        /** Construct from raw doubles (inches, radians) */
        Pose(double x_in, double y_in, double theta_radians)
            : se2(x_in, y_in, theta_radians),
              v(units::Velocity::from_ips(0)),
              omega(units::AngularVelocity::from_rad_per_sec(0)) {}

        /** Construct from units */
        Pose(units::Length x_pos, units::Length y_pos, units::Angle heading,
             units::Velocity vel, units::AngularVelocity ang_vel)
            : se2(x_pos.to_inches(), y_pos.to_inches(), heading.to_radians()),
              v(vel), omega(ang_vel) {}

        /** Construct from SE2 + velocities */
        Pose(const math::SE2& transformation, 
             units::Velocity vel, 
             units::AngularVelocity ang_vel)
            : se2(transformation), v(vel), omega(ang_vel) {}

        /** Construct from SE2 only (zero velocities) */
        explicit Pose(const math::SE2& transformation)
            : se2(transformation),
              v(units::Velocity::from_ips(0)),
              omega(units::AngularVelocity::from_rad_per_sec(0)) {}

        // ============= ACCESSORS =============
        
        /** Get x position in inches */
        double x_inches() const { return se2.x(); }
        
        /** Get y position in inches */
        double y_inches() const { return se2.y(); }
        
        /** Get heading in radians */
        double theta_rad() const { return se2.theta(); }
        
        /** Get heading in degrees */
        double theta_deg() const { return se2.theta() * 180.0 / M_PI; }

        /** Get x position as Length */
        units::Length x() const { return units::Length::from_inches(se2.x()); }
        
        /** Get y position as Length */
        units::Length y() const { return units::Length::from_inches(se2.y()); }
        
        /** Get heading as Angle */
        units::Angle theta() const { return units::Angle::from_radians(se2.theta()); }

        /** Get underlying SE2 transform */
        const math::SE2& transform() const { return se2; }
        math::SE2& transform() { return se2; }
        
        // ============= SETTERS =============
        
        void set_x(double x_in) { se2 = math::SE2(x_in, se2.y(), se2.theta()); }
        void set_y(double y_in) { se2 = math::SE2(se2.x(), y_in, se2.theta()); }
        void set_theta(double theta_radians) { se2 = math::SE2(se2.x(), se2.y(), theta_radians); }

        void set_x(units::Length x_pos) { set_x(x_pos.to_inches()); }
        void set_y(units::Length y_pos) { set_y(y_pos.to_inches()); }
        void set_theta(units::Angle heading) { set_theta(heading.to_radians()); }

        // ============= SE2 OPERATIONS =============

        /** Transform this pose by another SE2 transformation */
        Pose operator*(const math::SE2& transform) const
        {
            return Pose(se2 * transform, v, omega);
        }

        /** Get relative transformation to another pose */
        math::SE2 relative_to(const Pose& other) const
        {
            return other.se2.inverse() * se2;
        }
    };
}