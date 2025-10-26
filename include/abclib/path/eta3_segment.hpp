#pragma once

#include <optional>
#include <stdexcept>
#include <cmath>
#include <Eigen/Dense>
#include "path_segment_interface.hpp"

namespace abclib::path
{
    class Eta3PathSegment : public IPathSegment
    {
    public:
        // Type aliases
        using EtaVec = Eigen::Matrix<double, 6, 1>;
        using KappaVec = Eigen::Matrix<double, 4, 1>;

        /**
         * @brief Constructs an Eta3PathSegment.
         *
         * This constructor provides a flexible interface:
         * - If 'eta' is not provided, a heuristic is used to generate it based on
         *   the start and end poses.
         * - If 'kappa' is not provided, it defaults to a vector of all zeros.
         *
         * @param start_pose The starting pose [x, y, theta].
         * @param end_pose The ending pose [x, y, theta].
         * @param eta Optional user-provided 6-element shaping vector.
         * @param kappa Optional user-provided 4-element curvature vector.
         */
        Eta3PathSegment(const Pose &start_pose, const Pose &end_pose,
                        const std::optional<EtaVec> &eta = std::nullopt,
                        const std::optional<KappaVec> &kappa = std::nullopt);

        void calc_point(double u, double &x, double &y) const override;
        Point calc_first_deriv(double u) const override;
        Point calc_second_deriv(double u) const override;
        double calc_curvature(double u) const override;
        double get_segment_length() const override;
        const Pose &get_start_pose() const override;
        const Pose &get_end_pose() const override;

        double get_start_curvature() const;
        double get_end_curvature() const;
        double get_start_curvature_derivative() const;
        double get_end_curvature_derivative() const;

    private:
        static EtaVec calculate_heuristic_eta(const Pose &start_pose, const Pose &end_pose);
        void calculate_coefficients(const EtaVec &eta, const KappaVec &kappa);
        double s_dot(double u) const;
        double integrate_arc_length();
        KappaVec kappa_;

        Pose start_pose_;
        Pose end_pose_;
        Eigen::Matrix<double, 2, 8> coeffs_;
        double segment_length_;
    };
}
