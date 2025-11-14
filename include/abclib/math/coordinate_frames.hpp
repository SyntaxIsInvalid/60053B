// abclib/math/frames.hpp
#pragma once
#include "abclib/units/units.hpp"
#include <cmath>

namespace abclib::math
{
    // Frame definitions:
    // MATH FRAME: Standard Cartesian (X=right, Y=forward, θ=0 points +X/right)
    // BODY FRAME: REP-103 (X=forward, Y=left, θ=0 points +X_body/forward)
    // Relationship: Body frame = Math frame rotated 90° CCW

    struct BodyPose
    {
        units::Length x;
        units::Length y;
        units::Angle theta;

        static BodyPose from_inches_radians(double x_in, double y_in, double theta_rad)
        {
            return BodyPose{
                units::Length::from_inches(x_in),
                units::Length::from_inches(y_in),
                units::Angle::from_radians(theta_rad)
            };
        }

        double x_inches() const { return x.to_inches(); }
        double y_inches() const { return y.to_inches(); }
        double theta_radians() const { return theta.to_radians(); }
    };

    // ============= POSE CONVERSIONS =============

    inline BodyPose math_to_body_frame(
        units::Length math_x, 
        units::Length math_y, 
        units::Angle math_theta)
    {
        // 90° CCW rotation matrix applied to position
        units::Length body_x = math_y;   // Math forward → Body forward
        units::Length body_y = -math_x;  // Math right → Body -left (right)
        units::Angle body_theta = math_theta - units::Angle::from_radians(M_PI / 2);
        
        return BodyPose{body_x, body_y, body_theta};
    }

    inline void body_to_math_frame(
        const BodyPose &body_pose,
        units::Length &math_x, 
        units::Length &math_y, 
        units::Angle &math_theta)
    {
        // 90° CW rotation matrix (inverse of above)
        math_x = -body_pose.y;  // Body left → Math left (-X)
        math_y = body_pose.x;   // Body forward → Math forward
        math_theta = body_pose.theta + units::Angle::from_radians(M_PI / 2);
    }

    // ============= VELOCITY CONVERSIONS =============

    inline void math_velocity_to_body(
        units::Velocity math_vx, 
        units::Velocity math_vy, 
        units::AngularVelocity math_omega,
        units::Angle body_heading, // Current robot heading in body frame
        units::Velocity &body_v,
        units::AngularVelocity &body_omega)
    {
        // Project global velocity onto robot's forward direction
        // Body forward direction in global frame is at angle (body_heading + π/2)
        double math_heading_rad = body_heading.to_radians() + M_PI / 2;

        // v_body = v_global · forward_direction
        double vx = math_vx.to_mps();
        double vy = math_vy.to_mps();
        double body_v_mps = vx * std::cos(math_heading_rad) + vy * std::sin(math_heading_rad);
        
        body_v = units::Velocity::from_mps(body_v_mps);
        body_omega = math_omega; // Angular velocity is frame-invariant
    }

    inline void body_velocity_to_math(
        units::Velocity body_v,
        units::AngularVelocity body_omega,
        units::Angle body_heading,
        units::Velocity &math_vx, 
        units::Velocity &math_vy, 
        units::AngularVelocity &math_omega)
    {
        // Convert body-frame forward velocity to global math frame
        double body_heading_rad = body_heading.to_radians();
        double math_heading_rad = body_heading_rad + M_PI / 2;
        double v_mps = body_v.to_mps();
        
        // Component in math X and Y
        math_vx = units::Velocity::from_mps(-v_mps * std::sin(body_heading_rad));
        math_vy = units::Velocity::from_mps(v_mps * std::cos(body_heading_rad));
        math_omega = body_omega; // Angular velocity is frame-invariant
    }

} // namespace abclib::math