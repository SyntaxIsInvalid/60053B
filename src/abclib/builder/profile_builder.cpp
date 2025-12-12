#include "abclib/builder/profile_builder.hpp"
#include "abclib/builder/path_builder.hpp"

namespace abclib::builder {

ProfileBuilder::ProfileBuilder(PathBuilder& parent, std::string name)
    : parent_(parent)
    , name_(std::move(name))
{
}

PathBuilder& ProfileBuilder::trapezoidal(units::Velocity max_vel, units::Acceleration max_accel) {
    parent_.create_profile_group(name_, max_vel, max_accel, path::ProfileType::Trapezoidal);
    return parent_;
}

} // namespace abclib::builder