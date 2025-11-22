#include "abclib/builder/path_logger.hpp"
#include <fstream>
#include <iomanip>
#include <cmath>

namespace abclib::builder {

PathLogger::PathLogger(const std::string& base_name, units::Length track_width)
    : PathLogger(base_name, track_width, Config{})
{
}

PathLogger::PathLogger(const std::string& base_name, units::Length track_width, Config config)
    : base_name_(base_name)
    , track_width_(track_width)
    , config_(config)
{
}

void PathLogger::log_geometry(const path::Path& path)
{
    std::ofstream file(base_name_ + "_geometry.csv");
    if (!file.is_open()) {
        throw std::runtime_error("PathLogger: failed to open " + base_name_ + "_geometry.csv");
    }
    
    file << std::fixed << std::setprecision(config_.precision);
    write_geometry_header(file);
    
    const auto& groups = path.get_profile_groups();
    double global_s = 0.0;
    
    for (size_t group_idx = 0; group_idx < groups.size(); ++group_idx) {
        const auto& group = groups[group_idx];
        double group_length = group.total_arc_length.to_inches();
        
        // Sample at regular intervals, ensuring we hit the endpoint
        int num_samples = static_cast<int>(std::ceil(group_length / config_.geometry_ds)) + 1;
        
        for (int i = 0; i < num_samples; ++i) {
            double local_s = std::min(i * config_.geometry_ds, group_length);
            
            // Get segment info
            auto [seg_idx, u] = group.arc_length_to_segment_u(local_s);
            const auto& segment = group.segments[seg_idx];
            
            // Query position
            double x, y;
            segment->calc_point(u, x, y);
            
            // Query heading (special handling for turn-in-place)
            double theta;
            if (segment->is_turn_in_place()) {
                auto start = segment->get_start_pose();
                auto end = segment->get_end_pose();
                theta = start(2) + u * (end(2) - start(2));
            } else {
                auto deriv = segment->calc_first_deriv(u);
                theta = std::atan2(deriv.y(), deriv.x());
            }
            
            // Query curvature
            double kappa = segment->calc_curvature(u);
            
            // Curvature derivative
            double dkappa_ds = compute_curvature_derivative(group, local_s);
            
            file << global_s + local_s << ","
                 << local_s << ","
                 << x << ","
                 << y << ","
                 << theta << ","
                 << kappa << ","
                 << dkappa_ds << ","
                 << group_idx << ","
                 << group.name << ","
                 << seg_idx << ","
                 << segment->get_type_name() << "\n";
        }
        
        global_s += group_length;
    }
}

void PathLogger::log_trajectory(const path::Path& path)
{
    std::ofstream file(base_name_ + "_trajectory.csv");
    if (!file.is_open()) {
        throw std::runtime_error("PathLogger: failed to open " + base_name_ + "_trajectory.csv");
    }
    
    file << std::fixed << std::setprecision(config_.precision);
    write_trajectory_header(file);
    
    const auto& groups = path.get_profile_groups();
    double global_t = 0.0;
    double global_s = 0.0;
    
    for (size_t group_idx = 0; group_idx < groups.size(); ++group_idx) {
        const auto& group = groups[group_idx];
        
        if (!group.profile) {
            continue;
        }
        
        double group_time = group.get_total_time().to_seconds();
        double group_length = group.total_arc_length.to_inches();
        
        // Sample at regular intervals, ensuring we hit the endpoint
        int num_samples = static_cast<int>(std::ceil(group_time / config_.trajectory_dt)) + 1;
        
        double v_prev = 0.0;
        
        for (int i = 0; i < num_samples; ++i) {
            double local_t = std::min(i * config_.trajectory_dt, group_time);
            units::Time t = units::Time::from_seconds(local_t);
            
            // Get arc length at this time
            double local_s = group.profile->get_position(t).to_inches();
            
            // Get segment for turn-in-place handling
            auto [seg_idx, u] = group.arc_length_to_segment_u(local_s);
            const auto& segment = group.segments[seg_idx];
            
            // Query position
            double x, y;
            segment->calc_point(u, x, y);
            
            // Query heading
            double theta;
            if (segment->is_turn_in_place()) {
                auto start = segment->get_start_pose();
                auto end = segment->get_end_pose();
                theta = start(2) + u * (end(2) - start(2));
            } else {
                auto deriv = segment->calc_first_deriv(u);
                theta = std::atan2(deriv.y(), deriv.x());
            }
            
            // Get velocity
            double v = group.get_velocity_at_time(t).to_ips();
            
            // Get curvature
            double kappa = segment->calc_curvature(u);
            
            // Compute omega
            double omega = compute_omega(v, kappa);
            
            // Compute acceleration via finite difference
            double a = 0.0;
            if (i > 0) {
                a = (v - v_prev) / config_.trajectory_dt;
            }
            v_prev = v;
            
            // Compute wheel velocities
            auto [v_left, v_right] = compute_wheel_velocities(v, omega);
            
            file << global_t + local_t << ","
                 << global_s + local_s << ","
                 << local_s << ","
                 << x << ","
                 << y << ","
                 << theta << ","
                 << v << ","
                 << omega << ","
                 << a << ","
                 << kappa << ","
                 << v_left << ","
                 << v_right << ","
                 << group_idx << ","
                 << group.name << "\n";
        }
        
        global_t += group_time;
        global_s += group_length;
    }
}

void PathLogger::log_all(const path::Path& path)
{
    log_geometry(path);
    log_trajectory(path);
}

void PathLogger::write_geometry_header(std::ostream& os)
{
    os << "s,local_s,x,y,theta,kappa,dkappa_ds,group_idx,group_name,segment_idx,segment_type\n";
}

void PathLogger::write_trajectory_header(std::ostream& os)
{
    os << "t,s,local_s,x,y,theta,v,omega,a,kappa,v_left,v_right,group_idx,group_name\n";
}

double PathLogger::compute_curvature_derivative(const path::ProfileGroup& group, double s, double ds)
{
    double total = group.total_arc_length.to_inches();
    
    if (total < 1e-9) {
        return 0.0;
    }
    
    // Central difference where possible, forward/backward at boundaries
    double s_minus = std::max(0.0, s - ds);
    double s_plus = std::min(total, s + ds);
    
    double kappa_minus = group.query_curvature(s_minus);
    double kappa_plus = group.query_curvature(s_plus);
    
    double actual_ds = s_plus - s_minus;
    if (actual_ds < 1e-9) {
        return 0.0;
    }
    
    return (kappa_plus - kappa_minus) / actual_ds;
}

double PathLogger::compute_omega(double v, double kappa)
{
    return v * kappa;
}

std::pair<double, double> PathLogger::compute_wheel_velocities(double v, double omega)
{
    double track = track_width_.to_inches();
    double v_left = v - (omega * track / 2.0);
    double v_right = v + (omega * track / 2.0);
    return {v_left, v_right};
}

} // namespace abclib::builder