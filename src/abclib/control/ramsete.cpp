#include "ramsete.hpp"
#include "abclib/math/angles.hpp"
#include <cmath>

namespace abclib::control
{
    Ramsete::Ramsete(const RamseteConstants &constants)
        : constants_(constants)
    {
        if (constants_.b <= 0.0)
        {
            throw std::invalid_argument("RAMSETE: b must be positive");
        }
        if (constants_.zeta <= 0.0 || constants_.zeta >= 1.0)
        {
            throw std::invalid_argument("RAMSETE: zeta must be in (0, 1)");
        }
    }

    RamseteOutput Ramsete::compute(
        const estimation::Pose &current_pose,
        const trajectory::TrajectoryState &reference_state) const
    {
        RamseteOutput output;

        // Create reference pose from trajectory state
        math::SE2 ref_se2(reference_state.x, reference_state.y, reference_state.theta);

        // Compute error in body frame using SE2 (like WPILib's relativeTo)
        math::SE2 pose_error = current_pose.se2.inverse() * ref_se2;

        // Extract errors (already in body frame!)
        const double e_x_body = pose_error.x();
        const double e_y_body = pose_error.y();
        const double e_theta = pose_error.theta();

        // Get reference velocities
        const double v_ref = reference_state.arc_velocity.to_ips();
        const double omega_ref = reference_state.omega;

        // Compute time-varying gain k
        const double k = 2.0 * constants_.zeta *
                         std::sqrt(omega_ref * omega_ref +
                                   constants_.b * v_ref * v_ref);

        // RAMSETE control law
        const double v_command = v_ref * std::cos(e_theta) + k * e_x_body;
        const double sinc_e_theta = math::sinc(e_theta);
        const double omega_command = omega_ref + k * e_theta +
                                     constants_.b * v_ref * sinc_e_theta * e_y_body;

        // Wrap results
        output.v = units::Velocity::from_ips(v_command);
        output.omega = units::AngularVelocity::from_rad_per_sec(omega_command);
        output.e_x = units::Length::from_inches(e_x_body);
        output.e_y = units::Length::from_inches(e_y_body);
        output.e_theta = units::Angle::from_radians(e_theta);

        return output;
    }

    void Ramsete::set_constants(const RamseteConstants &constants)
    {
        constants_ = constants;
    }

    RamseteConstants Ramsete::get_constants() const
    {
        return constants_;
    }

} // namespace abclib::control