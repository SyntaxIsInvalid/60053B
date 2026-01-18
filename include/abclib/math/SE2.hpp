#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

namespace abclib::math
{
    /**
     * @brief SE(2) - Special Euclidean Group in 2D
     * 
     * Represents rigid transformations (rotation + translation) in the plane.
     * Wraps Eigen::Transform with Lie group operations.
     * 
     * Coordinate convention: Standard mathematical frame
     * - X-axis: horizontal (positive = east)
     * - Y-axis: vertical (positive = north)
     * - Theta: counterclockwise from +X axis
     */
    class SE2
    {
    private:
        Eigen::Transform<double, 2, Eigen::Isometry> T_;

    public:
        // ============= CONSTRUCTORS =============
        
        /** Default constructor - identity transformation */
        SE2() : T_(Eigen::Transform<double, 2, Eigen::Isometry>::Identity()) {}

        /** Construct from position and orientation */
        SE2(double x, double y, double theta)
            : T_(Eigen::Translation2d(x, y) * Eigen::Rotation2Dd(theta)) {}

        /** Construct from Eigen transform (explicit to avoid accidents) */
        explicit SE2(const Eigen::Transform<double, 2, Eigen::Isometry>& T) : T_(T) {}

        /** Construct from rotation and translation */
        SE2(const Eigen::Rotation2Dd& rotation, const Eigen::Vector2d& translation)
            : T_(Eigen::Translation2d(translation) * rotation) {}

        // ============= STATIC FACTORIES =============

        /** Identity transformation */
        static SE2 Identity() { return SE2(); }

        /** Pure translation */
        static SE2 Translation(double x, double y)
        {
            return SE2(x, y, 0.0);
        }

        /** Pure rotation around origin */
        static SE2 Rotation(double theta)
        {
            return SE2(0.0, 0.0, theta);
        }

        // ============= ACCESSORS =============

        /** Get x position */
        double x() const { return T_.translation().x(); }

        /** Get y position */
        double y() const { return T_.translation().y(); }

        /** Get orientation (radians) */
        double theta() const
        {
            // Extract angle from rotation matrix
            return std::atan2(T_.linear()(1, 0), T_.linear()(0, 0));
        }

        /** Get position vector */
        Eigen::Vector2d translation() const { return T_.translation(); }

        /** Get rotation matrix (2x2) */
        Eigen::Matrix2d rotation() const { return T_.linear(); }

        /** Get underlying Eigen transform (const) */
        const Eigen::Transform<double, 2, Eigen::Isometry>& transform() const { return T_; }

        /** Get underlying Eigen transform (mutable) */
        Eigen::Transform<double, 2, Eigen::Isometry>& transform() { return T_; }

        /** Get homogeneous matrix (3x3) */
        Eigen::Matrix3d matrix() const { return T_.matrix(); }

        // ============= OPERATIONS =============

        /** Compose transformations: this * other */
        SE2 operator*(const SE2& other) const
        {
            return SE2(T_ * other.T_);
        }

        /** Compound assignment */
        SE2& operator*=(const SE2& other)
        {
            T_ = T_ * other.T_;
            return *this;
        }

        /** Transform a point */
        Eigen::Vector2d operator*(const Eigen::Vector2d& point) const
        {
            return T_ * point;
        }

        /** Inverse transformation */
        SE2 inverse() const
        {
            return SE2(T_.inverse());
        }

        // ============= LIE GROUP OPERATIONS =============

        /**
         * @brief Exponential map: se(2) → SE(2)
         * 
         * Maps a tangent vector (velocity) to the manifold.
         * 
         * @param xi Tangent vector [v_x, v_y, omega]^T
         *           - v_x, v_y: linear velocity components
         *           - omega: angular velocity
         * @return SE2 transformation
         * 
         * Formula:
         * - If omega ≈ 0: pure translation
         * - Otherwise: rotation with "curved" translation
         */
        static SE2 exp(const Eigen::Vector3d& xi)
        {
            const double v_x = xi(0);
            const double v_y = xi(1);
            const double omega = xi(2);

            constexpr double epsilon = 1e-10;

            if (std::abs(omega) < epsilon)
            {
                // Small angle approximation: just translation
                return SE2(v_x, v_y, 0.0);
            }

            // Rotation part
            const double cos_omega = std::cos(omega);
            const double sin_omega = std::sin(omega);
            Eigen::Rotation2Dd R(omega);

            // Left Jacobian of SO(2) - converts body velocity to spatial displacement
            Eigen::Matrix2d V;
            V << sin_omega / omega, -(1.0 - cos_omega) / omega,
                 (1.0 - cos_omega) / omega, sin_omega / omega;

            // Translation part
            Eigen::Vector2d v(v_x, v_y);
            Eigen::Vector2d t = V * v;

            return SE2(R, t);
        }

        /**
         * @brief Logarithmic map: SE(2) → se(2)
         * 
         * Maps a transformation to its tangent vector.
         * 
         * @return Tangent vector [v_x, v_y, omega]^T
         * 
         * Inverse of exp(). Useful for:
         * - Computing velocity from pose change
         * - Interpolation
         * - Optimization
         */
        Eigen::Vector3d log() const
        {
            const double theta = this->theta();
            const Eigen::Vector2d t = T_.translation();

            constexpr double epsilon = 1e-10;

            if (std::abs(theta) < epsilon)
            {
                // Small angle: just return translation
                return Eigen::Vector3d(t.x(), t.y(), 0.0);
            }

            // Inverse of left Jacobian
            const double cos_theta = std::cos(theta);
            const double sin_theta = std::sin(theta);
            const double half_theta = theta / 2.0;

            Eigen::Matrix2d V_inv;
            V_inv << half_theta * sin_theta / (1.0 - cos_theta), half_theta,
                     -half_theta, half_theta * sin_theta / (1.0 - cos_theta);

            Eigen::Vector2d v = V_inv * t;

            return Eigen::Vector3d(v.x(), v.y(), theta);
        }

        /**
         * @brief Geodesic interpolation between two SE(2) elements
         * 
         * @param a Start transformation
         * @param b End transformation
         * @param t Interpolation parameter [0, 1]
         * @return Interpolated transformation
         * 
         * Computes: a * exp(t * log(a^{-1} * b))
         * This gives the "shortest path" on the manifold.
         */
        static SE2 interpolate(const SE2& a, const SE2& b, double t)
        {
            // Compute relative transformation
            SE2 delta = a.inverse() * b;
            
            // Get tangent vector
            Eigen::Vector3d xi = delta.log();
            
            // Scale by interpolation parameter
            Eigen::Vector3d xi_scaled = t * xi;
            
            // Map back to manifold and compose with start
            return a * exp(xi_scaled);
        }

        /**
         * @brief Adjoint matrix - transforms velocities between frames
         * 
         * Given velocity xi_b in frame b, computes velocity xi_a in frame a:
         * xi_a = Ad(T_ab) * xi_b
         * 
         * @return 3x3 Adjoint matrix
         */
        Eigen::Matrix3d Adjoint() const
        {
            const Eigen::Matrix2d R = T_.linear();
            const Eigen::Vector2d t = T_.translation();

            Eigen::Matrix3d Ad;
            Ad.block<2, 2>(0, 0) = R;
            Ad.block<2, 1>(0, 2) << -t.y(), t.x();  // [0, -1; 1, 0] * t
            Ad.block<1, 2>(2, 0).setZero();
            Ad(2, 2) = 1.0;

            return Ad;
        }

        // ============= COMPARISON =============

        /** Approximate equality */
        bool isApprox(const SE2& other, double epsilon = 1e-9) const
        {
            return T_.isApprox(other.T_, epsilon);
        }

        // ============= UTILITY =============

        /** Set to identity */
        void setIdentity()
        {
            T_.setIdentity();
        }
    };

} // namespace abclib::math