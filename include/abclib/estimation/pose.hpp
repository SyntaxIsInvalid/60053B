#pragma once
#include "abclib/units/units.hpp"
#include "abclib/math/SE2.hpp"
#include <optional>  // NEW

namespace abclib::estimation
{
    struct Pose
    {
        math::SE2 se2; // Position and orientation
        units::Velocity v;
        units::AngularVelocity omega;
        
        // NEW: Optional uncertainty estimate (populated by probabilistic estimators)
        // 3×3 covariance matrix for [x, y, θ] in [meters², meters², radians²]
        // nullopt indicates no uncertainty information available
        std::optional<Eigen::Matrix3d> covariance;

        // ============= CONSTRUCTORS =============

        /** Default constructor - identity pose at origin */
        Pose()
            : se2(math::SE2::Identity()),
              v(units::Velocity::from_ips(0)),
              omega(units::AngularVelocity::from_rad_per_sec(0)),
              covariance(std::nullopt) {}  // MODIFIED

        /** Construct from raw doubles (inches, radians) */
        Pose(double x_in, double y_in, double theta_radians)
            : se2(x_in, y_in, theta_radians),
              v(units::Velocity::from_ips(0)),
              omega(units::AngularVelocity::from_rad_per_sec(0)),
              covariance(std::nullopt) {}  // MODIFIED

        /** Construct from units */
        Pose(units::Length x_pos, units::Length y_pos, units::Angle heading,
             units::Velocity vel, units::AngularVelocity ang_vel)
            : se2(x_pos.to_inches(), y_pos.to_inches(), heading.to_radians()),
              v(vel), omega(ang_vel),
              covariance(std::nullopt) {}  // MODIFIED

        /** Construct from SE2 + velocities */
        Pose(const math::SE2 &transformation,
             units::Velocity vel,
             units::AngularVelocity ang_vel)
            : se2(transformation), v(vel), omega(ang_vel),
              covariance(std::nullopt) {}  // MODIFIED

        /** Construct from SE2 only (zero velocities) */
        explicit Pose(const math::SE2 &transformation)
            : se2(transformation),
              v(units::Velocity::from_ips(0)),
              omega(units::AngularVelocity::from_rad_per_sec(0)),
              covariance(std::nullopt) {}  // MODIFIED

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

        /** Transform this pose by another SE2 transformation */
        Pose operator*(const math::SE2 &transform) const
        {
            Pose result(se2 * transform, v, omega);
            result.covariance = covariance;  // NEW: Preserve covariance
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

        // ============= UNCERTAINTY QUERIES (NEW) =============

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
            if (!covariance) return -1.0;
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
            if (meters < 0.0) return units::Length::from_inches(-1.0);
            return units::Length::from_meters(meters);
        }

        /**
         * @brief Get heading uncertainty (standard deviation)
         * @return Heading uncertainty in radians, or -1.0 if unavailable
         */
        double heading_uncertainty_rad() const
        {
            if (!covariance) return -1.0;
            return std::sqrt((*covariance)(2, 2));
        }

        /**
         * @brief Get heading uncertainty as typed Angle
         * @return Heading uncertainty, or -1.0 radians if unavailable
         */
        units::Angle heading_uncertainty() const
        {
            double rad = heading_uncertainty_rad();
            if (rad < 0.0) return units::Angle::from_radians(-1.0);
            return units::Angle::from_radians(rad);
        }

        /**
         * @brief Get x-position uncertainty (standard deviation)
         * @return σ_x in meters, or -1.0 if unavailable
         */
        double x_uncertainty_meters() const
        {
            if (!covariance) return -1.0;
            return std::sqrt((*covariance)(0, 0));
        }

        /**
         * @brief Get y-position uncertainty (standard deviation)
         * @return σ_y in meters, or -1.0 if unavailable
         */
        double y_uncertainty_meters() const
        {
            if (!covariance) return -1.0;
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
        const Eigen::Matrix3d& get_covariance() const
        {
            if (!covariance) {
                throw std::runtime_error("Pose has no uncertainty information");
            }
            return *covariance;
        }

        /**
         * @brief Set covariance matrix
         * @param cov 3×3 covariance matrix [meters², meters², radians²]
         */
        void set_covariance(const Eigen::Matrix3d& cov)
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
    };
}