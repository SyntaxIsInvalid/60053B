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
        units::Velocity v;
        units::AngularVelocity omega;
        
        units::Length e_x;
        units::Length e_y;
        units::Angle e_theta;
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