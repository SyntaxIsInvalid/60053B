#pragma once

#include "abclib/estimation/pose.hpp"
#include "abclib/trajectory/trajectory.hpp"
#include "abclib/units/units.hpp"

namespace abclib::control
{
    struct RamseteConstants
    {
        double b;
        double zeta;
    };

    struct RamseteOutput
    {
        units::BodyLinearVelocity v;
        units::BodyAngularVelocity omega;
        
        units::Distance e_x;
        units::Distance e_y;
        units::Radians e_theta;
    };

    class Ramsete
    {
    public:
        explicit Ramsete(const RamseteConstants& constants);

        RamseteOutput compute(
            const estimation::Pose& current_pose,
            const trajectory::TrajectoryState& reference_state) const;

        void set_constants(const RamseteConstants& constants);

        RamseteConstants get_constants() const;

    private:
        RamseteConstants constants_;
    };

}