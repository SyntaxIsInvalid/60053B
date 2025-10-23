#include "ramsete.hpp"
#include "abclib/math/angles.hpp"
#include <cmath>
#include <algorithm>
#include "abclib/math/coordinate_frames.hpp"
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

        // Compute pose error in global (body) frame - DIRECT, no conversion
        const double e_x_global = reference_state.x - current_pose.x();
        const double e_y_global = reference_state.y - current_pose.y();
        const double e_theta = math::normalize_angle(reference_state.theta - current_pose.theta());

        // Transform position error to robot's body frame
        // Use current_pose.theta() (current heading in body frame) for the rotation
        const double cos_theta = std::cos(current_pose.theta());
        const double sin_theta = std::sin(current_pose.theta());

        double e_x_body = cos_theta * e_x_global + sin_theta * e_y_global;
        double e_y_body = -sin_theta * e_x_global + cos_theta * e_y_global;

        // Get reference velocity (already in body frame)
        const double v_ref = reference_state.arc_velocity.inches_per_sec;
        const double omega_ref = reference_state.omega; // Use directly, already in body frame

        // Compute time-varying gain k
        // k = 2 * zeta * sqrt(omega_ref^2 + b * v_ref^2)
        const double k = 2.0 * constants_.zeta *
                         std::sqrt(omega_ref * omega_ref +
                                   constants_.b * v_ref * v_ref);

        // RAMSETE control law
        // v = v_ref * cos(e_theta) + k * e_x
        double v_command = v_ref * std::cos(e_theta) + k * e_x_body;

        // omega = omega_ref + k * e_theta + b * v_ref * sinc(e_theta) * e_y
        // where sinc(e_theta) = sin(e_theta) / e_theta
        const double sinc_e_theta = math::sinc(e_theta);
        double omega_command = omega_ref + k * e_theta +
                               constants_.b * v_ref * sinc_e_theta * e_y_body;

        // Wrap results in typed units
        output.v = units::BodyLinearVelocity(v_command);
        output.omega = units::BodyAngularVelocity(omega_command);
        output.e_x = units::Distance::from_inches(e_x_body);
        output.e_y = units::Distance::from_inches(e_y_body);
        output.e_theta = units::Radians(e_theta);

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