#pragma once

#include "path_segment_interface.hpp"
#include <optional>
#include <Eigen/Dense>

namespace abclib::path
{
    class QuinticHermiteSegment : public IPathSegment
    {
    public:
        /**
         * @brief Constructs a quintic Hermite spline segment with heuristic control vectors.
         * 
         * Uses the 1.2 × distance heuristic for velocity magnitude:
         * - Velocity magnitude = 1.2 × ||end - start||
         * - Velocity direction aligned with heading
         * - Zero acceleration at both endpoints
         * 
         * This provides G2 continuity (continuous position, tangent, and curvature).
         * 
         * @param start_pose Starting pose [x, y, theta]
         * @param end_pose Ending pose [x, y, theta]
         */
        QuinticHermiteSegment(const Pose& start_pose, const Pose& end_pose);

        /**
         * @brief Constructs a quintic Hermite spline with explicit control vectors.
         * 
         * Allows full control over velocity and acceleration at endpoints.
         * 
         * @param start_pose Starting pose [x, y, theta]
         * @param end_pose Ending pose [x, y, theta]
         * @param start_vel Velocity vector at start [vx, vy]
         * @param start_accel Acceleration vector at start [ax, ay]
         * @param end_vel Velocity vector at end [vx, vy]
         * @param end_accel Acceleration vector at end [ax, ay]
         */
        QuinticHermiteSegment(const Pose& start_pose, const Pose& end_pose,
                            const Point& start_vel, const Point& start_accel,
                            const Point& end_vel, const Point& end_accel);

        // IPathSegment interface
        void calc_point(double u, double& x, double& y) const override;
        Point calc_first_deriv(double u) const override;
        Point calc_second_deriv(double u) const override;
        double calc_curvature(double u) const override;
        double get_segment_length() const override;
        const Pose& get_start_pose() const override;
        const Pose& get_end_pose() const override;

        // Boundary conditions for path stitching
        double get_start_curvature() const override;
        double get_end_curvature() const override;
        double get_start_curvature_derivative() const override;
        double get_end_curvature_derivative() const override;

    private:
        Pose start_pose_;
        Pose end_pose_;

        // Quintic polynomial coefficients: p(u) = c0 + c1*u + c2*u^2 + c3*u^3 + c4*u^4 + c5*u^5
        Eigen::Matrix<double, 6, 1> x_coeffs_;
        Eigen::Matrix<double, 6, 1> y_coeffs_;

        double segment_length_;

        /**
         * @brief Calculates quintic polynomial coefficients from boundary conditions.
         * 
         * Uses closed-form Hermite interpolation formulas.
         * 
         * @param pos_a Start position [x, y]
         * @param vel_a Start velocity [vx, vy]
         * @param accel_a Start acceleration [ax, ay]
         * @param pos_b End position [x, y]
         * @param vel_b End velocity [vx, vy]
         * @param accel_b End acceleration [ax, ay]
         */
        void calculate_coefficients(
            const Point& pos_a, const Point& vel_a, const Point& accel_a,
            const Point& pos_b, const Point& vel_b, const Point& accel_b);

        /**
         * @brief Integrates arc length using Gauss-Legendre quadrature.
         */
        double integrate_arc_length();

        /**
         * @brief Generates heuristic control vectors from poses.
         * 
         * Returns [velocity, acceleration] vectors for a given pose using:
         * - velocity magnitude = scalar × cos/sin(theta)
         * - acceleration = [0, 0]
         */
        static std::pair<Point, Point> calculate_heuristic_control_vectors(
            const Pose& pose, double scalar);
    };

} // namespace abclib::path