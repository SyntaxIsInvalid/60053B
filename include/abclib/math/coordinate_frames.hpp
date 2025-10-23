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

    // ============= POSE CONVERSIONS =============

    inline units::BodyPose math_to_body_frame(double math_x, double math_y, double math_theta)
    {
        // 90° CCW rotation matrix applied to position
        double body_x = math_y;  // Math forward → Body forward
        double body_y = -math_x; // Math right → Body -left (right)
        double body_theta = math_theta - M_PI / 2;
        return units::BodyPose::from_radians(body_x, body_y, body_theta);
    }

    inline void body_to_math_frame(const units::BodyPose &body_pose,
                                   double &math_x, double &math_y, double &math_theta)
    {
        // 90° CW rotation matrix (inverse of above)
        math_x = -body_pose.y(); // Body left → Math left (-X)
        math_y = body_pose.x();  // Body forward → Math forward
        math_theta = body_pose.theta() + M_PI / 2;
    }

    // ============= VELOCITY CONVERSIONS =============

    inline void math_velocity_to_body(
        double math_vx, double math_vy, double math_omega,
        double body_heading_rad, // Current robot heading in body frame
        units::BodyLinearVelocity &body_v,
        units::BodyAngularVelocity &body_omega)
    {
        // Project global velocity onto robot's forward direction
        // Body forward direction in global frame is at angle (body_heading + π/2)
        double math_heading = body_heading_rad + M_PI / 2;

        // v_body = v_global · forward_direction
        body_v = units::BodyLinearVelocity(
            math_vx * std::cos(math_heading) + math_vy * std::sin(math_heading));

        // Angular velocity is frame-invariant
        body_omega = units::BodyAngularVelocity(math_omega);
    }

    inline void body_velocity_to_math(units::BodyLinearVelocity body_v,
                                      units::BodyAngularVelocity body_omega,
                                      double body_heading,
                                      double &math_vx, double &math_vy, double &math_omega)
    {
        // Convert body-frame forward velocity to global math frame
        double math_heading = body_heading + M_PI / 2;
        math_vx = -body_v.inches_per_sec * std::sin(body_heading); // Component in math X
        math_vy = body_v.inches_per_sec * std::cos(body_heading);  // Component in math Y
        math_omega = body_omega.rad_per_sec;
    }

} // namespace abclib::math