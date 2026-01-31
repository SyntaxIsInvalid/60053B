#pragma once
#include "abclib/units/units.hpp"
#include "abclib/math/SE2.hpp"
#include <optional>

/**
 * @file pose.hpp
 * @brief Robot pose representation with SE(2) transformations and uncertainty
 * 
 * COORDINATE FRAMES:
 * - Internal SE2 storage: inches for position, radians for orientation
 * - Covariance storage: meters² for position, radians² for orientation
 *   (follows robotics community standard)
 * 
 * BODY FRAME CONVENTION (VEX/PROS standard):
 * - X-axis = LATERAL (right is positive, left is negative)
 * - Y-axis = FORWARD (front is positive, back is negative)
 * - Z-axis = UP
 * 
 * VELOCITY FRAME:
 * - v: Forward velocity in BODY FRAME (body Y-axis direction)
 * - omega: Angular velocity (turn rate, CCW positive)
 * - For differential drive: lateral velocity is always zero
 * 
 * WORLD FRAME:
 * - X-axis = EAST (horizontal right)
 * - Y-axis = NORTH (vertical up)
 * - At heading=0° in corner frame (facing opponent), body Y aligns with world Y
 * 
 * UNCERTAINTY PROPAGATION:
 * - operator*: Uses Jacobian method for covariance transformation
 * - Frame conversions: Direct copy for aligned-axes transformations
 * - Reference: MRPT pose_cov_ops library, Blanco 2010 SE(3) tutorial
 */

namespace abclib::estimation
{
    struct Pose
    {
        math::SE2 se2; // Position and orientation (stored in inches, radians)

        // Velocities in BODY FRAME (forward speed + turn rate)
        // For differential drive: v = forward velocity, omega = angular velocity
        units::Velocity v;
        units::AngularVelocity omega;

        // Optional uncertainty estimate (populated by probabilistic estimators)
        // 3×3 covariance matrix for [x, y, θ] in GLOBAL STANDARD FRAME
        // Units: [meters², meters², radians²]
        //
        // NOTE: SE2 stores position in INCHES, but covariance is in METERS²
        //       This is the robotics community standard (meters for uncertainty)
        //       Use the uncertainty query methods to handle unit conversions
        //
        // nullopt indicates no uncertainty information available
        std::optional<Eigen::Matrix3d> covariance;

        // ============= CONSTRUCTORS =============

        /** Default constructor - identity pose at origin */
        Pose()
            : se2(math::SE2::Identity()),
              v(units::Velocity::from_ips(0)),
              omega(units::AngularVelocity::from_rad_per_sec(0)),
              covariance(std::nullopt) {}

        /** Construct from raw doubles (inches, radians) */
        Pose(double x_in, double y_in, double theta_radians)
            : se2(x_in, y_in, theta_radians),
              v(units::Velocity::from_ips(0)),
              omega(units::AngularVelocity::from_rad_per_sec(0)),
              covariance(std::nullopt) {}

        /** Construct from units */
        Pose(units::Length x_pos, units::Length y_pos, units::Angle heading,
             units::Velocity vel, units::AngularVelocity ang_vel)
            : se2(x_pos.to_inches(), y_pos.to_inches(), heading.to_radians()),
              v(vel), omega(ang_vel),
              covariance(std::nullopt) {}

        /** Construct from SE2 + velocities */
        Pose(const math::SE2 &transformation,
             units::Velocity vel,
             units::AngularVelocity ang_vel)
            : se2(transformation), v(vel), omega(ang_vel),
              covariance(std::nullopt) {}

        /** Construct from SE2 only (zero velocities) */
        explicit Pose(const math::SE2 &transformation)
            : se2(transformation),
              v(units::Velocity::from_ips(0)),
              omega(units::AngularVelocity::from_rad_per_sec(0)),
              covariance(std::nullopt) {}

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
        const math::SE2 &transform() const { return se2; }
        math::SE2 &transform() { return se2; }

        // ============= SETTERS =============

        void set_x(double x_in) { se2 = math::SE2(x_in, se2.y(), se2.theta()); }
        void set_y(double y_in) { se2 = math::SE2(se2.x(), y_in, se2.theta()); }
        void set_theta(double theta_radians) { se2 = math::SE2(se2.x(), se2.y(), theta_radians); }

        void set_x(units::Length x_pos) { set_x(x_pos.to_inches()); }
        void set_y(units::Length y_pos) { set_y(y_pos.to_inches()); }
        void set_theta(units::Angle heading) { set_theta(heading.to_radians()); }

        // ============= SE2 OPERATIONS =============

        /**
         * @brief Transform this pose by another SE2 transformation
         *
         * Computes: result = this ⊕ transform
         *
         * If this pose has uncertainty, propagates covariance using the Jacobian
         * of SE(2) composition. This is the standard method for uncertainty
         * propagation in robotics (see MRPT pose_cov_ops).
         *
         * @param transform SE2 transformation to apply
         * @return Transformed pose with propagated uncertainty
         */
        Pose operator*(const math::SE2 &transform) const
        {
            Pose result(se2 * transform, v, omega);

            if (covariance.has_value())
            {
                // Compute Jacobian of SE(2) composition: J = ∂(p1 ⊕ p2)/∂p1
                // Where p1 = this pose, p2 = transform
                const double c = std::cos(this->theta_rad());
                const double s = std::sin(this->theta_rad());
                const double tx = transform.x();
                const double ty = transform.y();

                Eigen::Matrix3d J;
                J << 1.0, 0.0, -tx * s - ty * c,
                    0.0, 1.0, tx * c - ty * s,
                    0.0, 0.0, 1.0;

                // Transform covariance: Σ_out = J * Σ_in * J^T
                result.covariance = J * (*covariance) * J.transpose();
            }

            return result;
        }

        /** Get relative transformation to another pose */
        math::SE2 relative_to(const Pose &other) const
        {
            return other.se2.inverse() * se2;
        }

        Eigen::Vector2d global_to_local(const Eigen::Vector2d &global_point) const
        {
            return se2.inverse() * global_point;
        }

        Eigen::Vector2d local_to_global(const Eigen::Vector2d &local_point) const
        {
            return se2 * local_point;
        }

        // ============= UNCERTAINTY QUERIES =============

        /**
         * @brief Check if uncertainty information is available
         * @return true if covariance is populated, false otherwise
         */
        bool has_uncertainty() const
        {
            return covariance.has_value();
        }

        /**
         * @brief Get 2D position uncertainty (standard deviation)
         * @return Combined x-y uncertainty in meters, or -1.0 if unavailable
         *
         * Computes: sqrt(σ_x² + σ_y²)
         * This gives a scalar measure of position confidence
         */
        double position_uncertainty_meters() const
        {
            if (!covariance)
                return -1.0;
            double sigma_x = std::sqrt((*covariance)(0, 0));
            double sigma_y = std::sqrt((*covariance)(1, 1));
            return std::sqrt(sigma_x * sigma_x + sigma_y * sigma_y);
        }

        /**
         * @brief Get 2D position uncertainty in inches
         * @return Combined x-y uncertainty in inches, or -1.0 if unavailable
         */
        units::Length position_uncertainty() const
        {
            double meters = position_uncertainty_meters();
            if (meters < 0.0)
                return units::Length::from_inches(-1.0);
            return units::Length::from_meters(meters);
        }

        /**
         * @brief Get heading uncertainty (standard deviation)
         * @return Heading uncertainty in radians, or -1.0 if unavailable
         */
        double heading_uncertainty_rad() const
        {
            if (!covariance)
                return -1.0;
            return std::sqrt((*covariance)(2, 2));
        }

        /**
         * @brief Get heading uncertainty as typed Angle
         * @return Heading uncertainty, or -1.0 radians if unavailable
         */
        units::Angle heading_uncertainty() const
        {
            double rad = heading_uncertainty_rad();
            if (rad < 0.0)
                return units::Angle::from_radians(-1.0);
            return units::Angle::from_radians(rad);
        }

        /**
         * @brief Get x-position uncertainty (standard deviation)
         * @return σ_x in meters, or -1.0 if unavailable
         */
        double x_uncertainty_meters() const
        {
            if (!covariance)
                return -1.0;
            return std::sqrt((*covariance)(0, 0));
        }

        /**
         * @brief Get y-position uncertainty (standard deviation)
         * @return σ_y in meters, or -1.0 if unavailable
         */
        double y_uncertainty_meters() const
        {
            if (!covariance)
                return -1.0;
            return std::sqrt((*covariance)(1, 1));
        }

        /**
         * @brief Get reference to full covariance matrix
         * @return const reference to 3×3 matrix, or throws if unavailable
         *
         * Matrix layout:
         * [[σ_x²,  σ_xy,  σ_xθ],
         *  [σ_xy,  σ_y²,  σ_yθ],
         *  [σ_xθ,  σ_yθ,  σ_θ²]]
         *
         * All position units in meters, angles in radians
         */
        const Eigen::Matrix3d &get_covariance() const
        {
            if (!covariance)
            {
                throw std::runtime_error("Pose has no uncertainty information");
            }
            return *covariance;
        }

        /**
         * @brief Set covariance matrix
         * @param cov 3×3 covariance matrix [meters², meters², radians²]
         */
        void set_covariance(const Eigen::Matrix3d &cov)
        {
            covariance = cov;
        }

        /**
         * @brief Clear uncertainty information
         */
        void clear_covariance()
        {
            covariance = std::nullopt;
        }

        // ============= MOTION INTEGRATION =============

        /**
         * @brief Propagate pose forward using body-frame velocities
         * @param dt Time step
         * @return New pose after time dt assuming constant velocities
         *
         * Uses SE(2) exponential map for accurate motion integration.
         * Useful for prediction step in filters between sensor updates.
         *
         * COORDINATE CONVENTION (VEX/PROS standard):
         * - Body X-axis = LATERAL (right is positive)
         * - Body Y-axis = FORWARD (front is positive)
         * - For differential drive: lateral velocity is zero
         */
        Pose propagate(units::Time dt) const
        {
            // Body-frame velocity vector [vx_body, vy_body, omega]
            // VEX convention: Y is forward, X is lateral
            Eigen::Vector3d xi(
                0.0,                                     // X = lateral (zero for diff drive)
                v.to_mps() * dt.to_seconds(),            // Y = forward displacement
                omega.to_rad_per_sec() * dt.to_seconds() // angular displacement
            );

            // Convert meters to inches for SE2
            xi(0) *= 39.3701; // lateral
            xi(1) *= 39.3701; // forward

            // Convert body displacement to world frame via exponential map
            math::SE2 delta = math::SE2::exp(xi);

            // Compose with current pose (uses operator* with Jacobian!)
            Pose result = (*this) * delta;

            // Velocities remain the same (constant velocity model)
            result.v = v;
            result.omega = omega;

            return result;
        }

        /**
         * @brief Compose two poses with full uncertainty propagation
         * @param other Second pose to compose
         * @return Composed pose with combined uncertainty
         *
         * Implements: p_out = this ⊕ other
         *
         * This is more general than operator* because it handles uncertainty
         * in BOTH poses. Use this when combining two uncertain pose estimates
         * (e.g., fusing odometry with vision).
         *
         * If only the first pose has uncertainty, operator* is equivalent.
         */
        Pose compose_with_uncertainty(const Pose &other) const
        {
            Pose result(se2 * other.se2, v, omega);

            if (!covariance.has_value() && !other.covariance.has_value())
            {
                // No uncertainty from either pose
                return result;
            }

            // Jacobians for composition: p_out = p1 ⊕ p2
            const double c1 = std::cos(this->theta_rad());
            const double s1 = std::sin(this->theta_rad());
            const double x2 = other.se2.x() / 39.3701; // inches to meters
            const double y2 = other.se2.y() / 39.3701; // inches to meters

            // J1 = ∂(p1 ⊕ p2)/∂p1
            Eigen::Matrix3d J1;
            J1 << 1.0, 0.0, -x2 * s1 - y2 * c1,
                0.0, 1.0, x2 * c1 - y2 * s1,
                0.0, 0.0, 1.0;

            // J2 = ∂(p1 ⊕ p2)/∂p2 (rotation by theta1)
            Eigen::Matrix3d J2;
            J2 << c1, -s1, 0.0,
                s1, c1, 0.0,
                0.0, 0.0, 1.0;

            // Combine uncertainties
            Eigen::Matrix3d cov_out = Eigen::Matrix3d::Zero();

            if (covariance.has_value())
            {
                cov_out += J1 * (*covariance) * J1.transpose();
            }

            if (other.covariance.has_value())
            {
                cov_out += J2 * (*other.covariance) * J2.transpose();
            }

            result.covariance = cov_out;
            return result;
        }
    };
}