#pragma once

#include "abclib/units/units.hpp"
#include <string>

namespace abclib::builder {

class PathBuilder;

class ProfileBuilder {
public:
    ProfileBuilder(PathBuilder& parent, std::string name);
    
    PathBuilder& trapezoidal(units::Velocity max_vel, units::Acceleration max_accel);
    
    // Future:
    // PathBuilder& s_curve(units::Velocity max_vel, units::Acceleration max_accel, units::Jerk max_jerk);

private:
    PathBuilder& parent_;
    std::string name_;
};

} // namespace abclib::builder