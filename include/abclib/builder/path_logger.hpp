#pragma once

#include "abclib/builder/path.hpp"
#include <string>

namespace abclib::builder {

class PathLogger {
public:
    struct Config {
        double geometry_ds = 0.25; // 0.25 inches 
        double trajectory_dt = 0.010; // 10 ms
        int precision = 6; // decimal places
    };

    PathLogger(const std::string& base_name, units::Length track_width);
    PathLogger(const std::string& base_name, units::Length track_width, Config config);

    void log_geometry(const path::Path& path);
    void log_trajectory(const path::Path& path);
    void log_all(const path::Path& path);

private:
    std::string base_name_;
    units::Length track_width_;
    Config config_;

    void write_geometry_header(std::ostream& os);
    void write_trajectory_header(std::ostream& os);
    
    double compute_curvature_derivative(const path::ProfileGroup& group, double s, double ds = 0.01);
    double compute_omega(double v, double kappa);
    std::pair<double, double> compute_wheel_velocities(double v, double omega);
};

} // namespace abclib::builder