#include "eta3_segment.hpp"
#include <algorithm>
#include "abclib/math/GL15.hpp"
#include "main.h"
namespace abclib::path
{

    using Point = IPathSegment::Point;
    using Pose = IPathSegment::Pose;

    Eta3PathSegment::Eta3PathSegment(const Pose &start_pose, const Pose &end_pose,
                                     const std::optional<EtaVec> &opt_eta,
                                     const std::optional<KappaVec> &opt_kappa)
        : start_pose_(start_pose),
          end_pose_(end_pose)
    {
        // 1. Resolve eta: use the provided value, otherwise calculate the heuristic.
        const EtaVec final_eta = opt_eta.has_value() ? opt_eta.value() : calculate_heuristic_eta(start_pose, end_pose);

        // 2. Resolve kappa: use the provided value, otherwise default to a zero vector.
        const KappaVec final_kappa = opt_kappa.value_or(KappaVec::Zero());

        // 3. With the final parameters determined, compute the polynomial coefficients.
        calculate_coefficients(final_eta, final_kappa);

        // 4. Finally, calculate the segment's arc length for future use.
        segment_length_ = integrate_arc_length();
    }

    void Eta3PathSegment::calc_point(double u, double &x, double &y) const
    {
        u = std::clamp(u, 0.0, 1.0);

        x = coeffs_(0, 7);
        x = x * u + coeffs_(0, 6);
        x = x * u + coeffs_(0, 5);
        x = x * u + coeffs_(0, 4);
        x = x * u + coeffs_(0, 3);
        x = x * u + coeffs_(0, 2);
        x = x * u + coeffs_(0, 1);
        x = x * u + coeffs_(0, 0);

        y = coeffs_(1, 7);
        y = y * u + coeffs_(1, 6);
        y = y * u + coeffs_(1, 5);
        y = y * u + coeffs_(1, 4);
        y = y * u + coeffs_(1, 3);
        y = y * u + coeffs_(1, 2);
        y = y * u + coeffs_(1, 1);
        y = y * u + coeffs_(1, 0);
    }

    abclib::path::Point Eta3PathSegment::calc_first_deriv(double u) const
    {
        u = std::clamp(u, 0.0, 1.0);

        double x = 7.0 * coeffs_(0, 7);
        x = x * u + 6.0 * coeffs_(0, 6);
        x = x * u + 5.0 * coeffs_(0, 5);
        x = x * u + 4.0 * coeffs_(0, 4);
        x = x * u + 3.0 * coeffs_(0, 3);
        x = x * u + 2.0 * coeffs_(0, 2);
        x = x * u + coeffs_(0, 1);

        double y = 7.0 * coeffs_(1, 7);
        y = y * u + 6.0 * coeffs_(1, 6);
        y = y * u + 5.0 * coeffs_(1, 5);
        y = y * u + 4.0 * coeffs_(1, 4);
        y = y * u + 3.0 * coeffs_(1, 3);
        y = y * u + 2.0 * coeffs_(1, 2);
        y = y * u + coeffs_(1, 1);

        return abclib::path::Point(x, y);
    }

    abclib::path::Point Eta3PathSegment::calc_second_deriv(double u) const
    {
        u = std::clamp(u, 0.0, 1.0);

        double x = 42.0 * coeffs_(0, 7);
        x = x * u + 30.0 * coeffs_(0, 6);
        x = x * u + 20.0 * coeffs_(0, 5);
        x = x * u + 12.0 * coeffs_(0, 4);
        x = x * u + 6.0 * coeffs_(0, 3);
        x = x * u + 2.0 * coeffs_(0, 2);

        double y = 42.0 * coeffs_(1, 7);
        y = y * u + 30.0 * coeffs_(1, 6);
        y = y * u + 20.0 * coeffs_(1, 5);
        y = y * u + 12.0 * coeffs_(1, 4);
        y = y * u + 6.0 * coeffs_(1, 3);
        y = y * u + 2.0 * coeffs_(1, 2);

        return abclib::path::Point(x, y);
    }

    double Eta3PathSegment::calc_curvature(double u) const
    {
        const Point d1 = calc_first_deriv(u);  // First derivative (x', y')
        const Point d2 = calc_second_deriv(u); // Second derivative (x'', y'')

        const double numerator = std::abs(d1.x() * d2.y() - d1.y() * d2.x());
        const double denominator = std::pow(d1.squaredNorm(), 1.5);

        // Avoid division by zero if the path has zero velocity at u
        if (std::abs(denominator) < 1e-9)
        {
            return 0.0;
        }

        return numerator / denominator;
    }

    double Eta3PathSegment::get_segment_length() const
    {
        return segment_length_;
    }

    const Pose &Eta3PathSegment::get_start_pose() const
    {
        return start_pose_;
    }

    const Pose &Eta3PathSegment::get_end_pose() const
    {
        return end_pose_;
    }

    void Eta3PathSegment::calculate_coefficients(const EtaVec &eta, const KappaVec &kappa)
    {
        // Precompute trig values
        const double cos_start = std::cos(start_pose_(2));
        const double sin_start = std::sin(start_pose_(2));
        const double cos_end = std::cos(end_pose_(2));
        const double sin_end = std::sin(end_pose_(2));

        // Precompute displacement
        const double dx = end_pose_(0) - start_pose_(0);
        const double dy = end_pose_(1) - start_pose_(1);

        // Precompute powers (major optimization)
        const double eta0_squared = eta(0) * eta(0);
        const double eta0_cubed = eta0_squared * eta(0);
        const double eta1_squared = eta(1) * eta(1);
        const double eta1_cubed = eta1_squared * eta(1);

        // Precompute common products
        const double eta0_eta2_kappa0 = eta(0) * eta(2) * kappa(0);
        const double eta1_eta3_kappa2 = eta(1) * eta(3) * kappa(2);

        // Quadratic coefficients
        const double quadratic_eta_term = 0.5 * eta(2);
        const double quadratic_curvature_term = 0.5 * eta0_squared * kappa(0);

        // Cubic coefficients
        const double cubic_eta_term = eta(4) / 6.0;
        const double cubic_curvature_term = (eta0_cubed * kappa(1) + 3.0 * eta0_eta2_kappa0) / 6.0;

        // Quartic polynomial combinations
        const double start_side_quartic_combination = 20 * eta(0) + 5 * eta(2) + (2.0 / 3.0) * eta(4);
        const double end_side_quartic_combination = 15 * eta(1) - 2.5 * eta(3) + (1.0 / 6.0) * eta(5);

        // Quintic polynomial combinations
        const double start_side_quintic_combination = 45 * eta(0) + 10 * eta(2) + eta(4);
        const double end_side_quintic_combination = 39 * eta(1) - 7 * eta(3) + 0.5 * eta(5);

        // Sextic polynomial combinations
        const double start_side_sextic_combination = 36 * eta(0) + 7.5 * eta(2) + (2.0 / 3.0) * eta(4);
        const double end_side_sextic_combination = 34 * eta(1) - 6.5 * eta(3) + 0.5 * eta(5);

        // Septic polynomial combinations
        const double start_side_septic_combination = 10 * eta(0) + 2 * eta(2) + (1.0 / 6.0) * eta(4);
        const double end_side_septic_combination = 10 * eta(1) - 2 * eta(3) + (1.0 / 6.0) * eta(5);

        // Curvature terms for higher-order polynomials
        const double start_curvature_quartic = 5 * eta0_squared * kappa(0) + (2.0 / 3.0) * eta0_cubed * kappa(1) + 2 * eta0_eta2_kappa0;
        const double end_curvature_quartic = 2.5 * eta1_squared * kappa(2) - (1.0 / 6.0) * eta1_cubed * kappa(3) - 0.5 * eta1_eta3_kappa2;

        const double start_curvature_quintic = 10 * eta0_squared * kappa(0) + eta0_cubed * kappa(1) + 3 * eta0_eta2_kappa0;
        const double end_curvature_quintic = 7 * eta1_squared * kappa(2) - 0.5 * eta1_cubed * kappa(3) - 1.5 * eta1_eta3_kappa2;

        const double start_curvature_sextic = 7.5 * eta0_squared * kappa(0) + (2.0 / 3.0) * eta0_cubed * kappa(1) + 2 * eta0_eta2_kappa0;
        const double end_curvature_sextic = 6.5 * eta1_squared * kappa(2) - 0.5 * eta1_cubed * kappa(3) - 1.5 * eta1_eta3_kappa2;

        const double start_curvature_septic = 2 * eta0_squared * kappa(0) + (1.0 / 6.0) * eta0_cubed * kappa(1) + 0.5 * eta0_eta2_kappa0;
        const double end_curvature_septic = 2 * eta1_squared * kappa(2) - (1.0 / 6.0) * eta1_cubed * kappa(3) - 0.5 * eta1_eta3_kappa2;

        // Now compute coefficients efficiently
        // Constant & linear terms
        coeffs_(0, 0) = start_pose_(0);
        coeffs_(1, 0) = start_pose_(1);
        coeffs_(0, 1) = eta(0) * cos_start;
        coeffs_(1, 1) = eta(0) * sin_start;

        // Quadratic terms
        coeffs_(0, 2) = quadratic_eta_term * cos_start - quadratic_curvature_term * sin_start;
        coeffs_(1, 2) = quadratic_eta_term * sin_start + quadratic_curvature_term * cos_start;

        // Cubic terms
        coeffs_(0, 3) = cubic_eta_term * cos_start - cubic_curvature_term * sin_start;
        coeffs_(1, 3) = cubic_eta_term * sin_start + cubic_curvature_term * cos_start;

        // Quartic terms
        coeffs_(0, 4) = 35 * dx - start_side_quartic_combination * cos_start + start_curvature_quartic * sin_start - end_side_quartic_combination * cos_end - end_curvature_quartic * sin_end;
        coeffs_(1, 4) = 35 * dy - start_side_quartic_combination * sin_start - start_curvature_quartic * cos_start - end_side_quartic_combination * sin_end + end_curvature_quartic * cos_end;

        // Quintic terms
        coeffs_(0, 5) = -84 * dx + start_side_quintic_combination * cos_start - start_curvature_quintic * sin_start + end_side_quintic_combination * cos_end + end_curvature_quintic * sin_end;
        coeffs_(1, 5) = -84 * dy + start_side_quintic_combination * sin_start + start_curvature_quintic * cos_start + end_side_quintic_combination * sin_end - end_curvature_quintic * cos_end;

        // Sextic terms
        coeffs_(0, 6) = 70 * dx - start_side_sextic_combination * cos_start + start_curvature_sextic * sin_start - end_side_sextic_combination * cos_end + end_curvature_sextic * sin_end;
        coeffs_(1, 6) = 70 * dy - start_side_sextic_combination * sin_start - start_curvature_sextic * cos_start - end_side_sextic_combination * sin_end + end_curvature_sextic * cos_end;

        // Septic terms
        coeffs_(0, 7) = -20 * dx + start_side_septic_combination * cos_start - start_curvature_septic * sin_start + end_side_septic_combination * cos_end + end_curvature_septic * sin_end;
        coeffs_(1, 7) = -20 * dy + start_side_septic_combination * sin_start + start_curvature_septic * cos_start + end_side_septic_combination * sin_end - end_curvature_septic * cos_end;
    }

    Eta3PathSegment::EtaVec Eta3PathSegment::calculate_heuristic_eta(const Pose &start_pose, const Pose &end_pose)
    {
        EtaVec heuristic_eta = EtaVec::Zero();
        double distance = (end_pose.head<2>() - start_pose.head<2>()).norm();

        heuristic_eta(0) = distance;
        heuristic_eta(1) = distance;

        return heuristic_eta;
    }

    double Eta3PathSegment::s_dot(double u) const
    {
        // s_dot is the magnitude (norm) of the first derivative vector.
        // It represents the "speed" of the curve with respect to the parameter u.
        const double norm = calc_first_deriv(u).norm();
        return std::max(norm, 1e-6);
    }
    double Eta3PathSegment::integrate_arc_length()
    {
        return integrate_gauss15([this](double u)
                                 {
        const double norm = calc_first_deriv(u).norm();
        return std::max(norm, 1e-6); });
    }
}