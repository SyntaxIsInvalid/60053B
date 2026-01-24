#include "abclib/telemetry/logger.hpp"
#include "abclib/telemetry/telemetry.hpp"
#include "api.h"
#include <mutex>

namespace abclib::telemetry
{
    Logger::Logger(const std::string &name, bool auto_segment, const LogFields &fields)
        : base_name_(name),
          auto_segment_(auto_segment),
          fields_(fields),
          file_(nullptr),
          start_time_(pros::millis()),
          segment_counter_(0),
          current_event_marker_("")
    {
        // If not auto-segmenting, open initial file immediately
        if (!auto_segment_)
        {
            open_new_file();
        }
        // If auto-segmenting, wait for first mark() call to open file
    }

    Logger::~Logger()
    {
        close_current_file();
    }

    void Logger::log()
    {
        // If no file is open yet (can happen in segmented mode before first mark)
        if (!file_)
        {
            open_new_file();
        }

        uint32_t absolute_time = pros::millis();
        double elapsed_time = (absolute_time - start_time_) / 1000.0;

        write_row(absolute_time, elapsed_time);
    }

    void Logger::mark(const std::string &segment_name)
    {
        if (auto_segment_)
        {
            // Close current file and open new one
            close_current_file();

            // Use provided name or auto-generate
            std::string actual_name = segment_name;
            if (actual_name.empty())
            {
                actual_name = "segment_" + std::to_string(segment_counter_);
                segment_counter_++;
            }

            open_new_file(actual_name);
        }
        else
        {
            // Just update the event marker for next log() call
            current_event_marker_ = segment_name;
        }
    }

    void Logger::open_new_file(const std::string &segment_name)
    {
        // Close any existing file first
        close_current_file();

        std::string filename = generate_filename(segment_name);
        file_ = fopen(filename.c_str(), "w");

        if (!file_)
        {
            // Log error but don't crash - robot should keep running
            return;
        }

        write_header();
    }

    void Logger::close_current_file()
    {
        if (file_)
        {
            fclose(file_);
            file_ = nullptr;
        }
    }

    void Logger::write_header()
    {
        if (!file_)
            return;

        // Timestamp columns (always included)
        fprintf(file_, "absolute_time_ms,elapsed_time_s");

        // Event marker column (only in non-segmented mode)
        if (!auto_segment_)
        {
            fprintf(file_, ",event_marker");
        }

        // Pose data
        if (fields_.pose)
        {
            fprintf(file_, ",x_inches,y_inches,theta_deg");
        }

        // Velocity data
        if (fields_.velocity)
        {
            fprintf(file_, ",v_inches_per_sec,omega_rad_per_sec");
            fprintf(file_, ",v_raw_inches_per_sec,omega_raw_rad_per_sec");
        }

        // Lateral control
        if (fields_.lateral_pid)
        {
            fprintf(file_, ",lateral_error_inches,lateral_output_volts");
            fprintf(file_, ",lateral_target_inches,lateral_actual_inches");
            fprintf(file_, ",lateral_p_term,lateral_i_term,lateral_d_term");
            fprintf(file_, ",max_lateral_error_inches,cumulative_lateral_error_inches");
        }

        // Angular control
        if (fields_.angular_pid)
        {
            fprintf(file_, ",angular_error_rad,angular_output_volts");
            fprintf(file_, ",angular_target_rad,angular_actual_rad");
            fprintf(file_, ",angular_p_term,angular_i_term,angular_d_term");
            fprintf(file_, ",max_angular_error_rad,cumulative_angular_error_rad");
        }

        // Settlement tracking
        if (fields_.settlement)
        {
            fprintf(file_, ",is_settled,settle_count,settlement_reason");
            fprintf(file_, ",time_to_settle_s");
        }

        // Motor voltages
        if (fields_.motors)
        {
            fprintf(file_, ",left_motor_voltage_volts,right_motor_voltage_volts");
        }

        // Path tracking errors
        if (fields_.path_tracking)
        {
            fprintf(file_, ",cross_track_error_inches,along_track_error_inches");
            fprintf(file_, ",max_cross_track_error_inches,cumulative_xte_inches");
            fprintf(file_, ",max_along_track_error_inches,cumulative_ate_inches");
            fprintf(file_, ",final_pose_error_x_inches,final_pose_error_y_inches");
            fprintf(file_, ",final_pose_error_theta_rad");
        }

        // Path follower status
        if (fields_.path_status)
        {
            fprintf(file_, ",path_status,trajectory_time_s,trajectory_progress");
            fprintf(file_, ",trajectory_total_time_s");
            fprintf(file_, ",reference_velocity_inches_per_sec,reference_arc_position_inches");
        }

        // Individual wheel velocities
        if (fields_.wheels)
        {
            fprintf(file_, ",left_wheel_velocity_rad_per_sec,right_wheel_velocity_rad_per_sec");
        }

        // Turn-in-place tracking
        if (fields_.turn_in_place)
        {
            fprintf(file_, ",omega_reference_rad_per_sec,omega_error_rad_per_sec");
            fprintf(file_, ",omega_pid_output,omega_commanded_rad_per_sec");
            fprintf(file_, ",left_wheel_cmd_inches_per_sec,right_wheel_cmd_inches_per_sec");
        }

        // Loop timing metrics
        if (fields_.timing)
        {
            fprintf(file_, ",loop_time_ms,max_loop_time_ms,min_loop_time_ms");
            fprintf(file_, ",avg_loop_time_ms,timing_violations,total_loop_count");
            fprintf(file_, ",target_loop_time_ms");
        }

        // Battery monitoring
        if (fields_.battery)
        {
            fprintf(file_, ",battery_voltage_volts,battery_capacity_percent");
            fprintf(file_, ",voltage_comp_scale,voltage_comp_active");
        }

        if (fields_.ekf)
        {
            fprintf(file_, ",ekf_x_inches,ekf_y_inches,ekf_theta_deg");
            fprintf(file_, ",ekf_x_std_inches,ekf_y_std_inches,ekf_theta_std_rad");
            fprintf(file_, ",front_dist_measured_inches,front_dist_expected_inches");
            fprintf(file_, ",front_dist_valid,front_wall");
            fprintf(file_, ",back_dist_measured_inches,back_dist_expected_inches");
            fprintf(file_, ",back_dist_valid,back_wall");
            fprintf(file_, ",front_innovation_inches,back_innovation_inches");
        }

        if (fields_.ramsete)
        {
            fprintf(file_, ",x_ref_in,y_ref_in,theta_ref_deg");
            fprintf(file_, ",v_ref_ips,omega_ref_rps,curvature");
            fprintf(file_, ",e_x_in,e_y_in,e_theta_deg"); // Body frame errors!
            fprintf(file_, ",v_cmd_ips,omega_cmd_rps");
            fprintf(file_, ",x_err_global_in,y_err_global_in"); // For plotting
            fprintf(file_, ",k_gain,b_param,zeta_param");
        }

        fprintf(file_, "\n");
        fflush(file_); // Ensure header is written immediately
    }

    void Logger::write_row(uint32_t absolute_time, double elapsed_time)
    {
        if (!file_)
            return;

        // Thread-safe copy of telemetry data
        const TelemetryData &data = g_telemetry.get_read_buffer();

        // Timestamp columns (always included)
        fprintf(file_, "%u,%.3f", absolute_time, elapsed_time);

        // Event marker column (only in non-segmented mode)
        if (!auto_segment_)
        {
            fprintf(file_, ",%s", current_event_marker_.c_str());
            current_event_marker_ = ""; // Clear after writing
        }

        // Pose data
        if (fields_.pose)
        {
            fprintf(file_, ",%.3f,%.3f,%.3f",
                    data.pose_corner.x_inches(), data.pose_corner.y_inches(), data.pose_corner.theta_deg());
        }

        // Velocity data
        if (fields_.velocity)
        {
            fprintf(file_, ",%.3f,%.3f",
                    data.pose_corner.v.to_ips(), data.pose_corner.omega.to_rad_per_sec());
            fprintf(file_, ",%.3f,%.3f",
                    data.pose_v_raw.to_ips(), data.pose_omega_raw.to_rad_per_sec());
        }

        // Lateral control
        if (fields_.lateral_pid)
        {
            fprintf(file_, ",%.3f,%.3f",
                    data.lateral_error.to_inches(), data.lateral_output.to_volts());
            fprintf(file_, ",%.3f,%.3f",
                    data.lateral_target.to_inches(), data.lateral_actual.to_inches());
            fprintf(file_, ",%.3f,%.3f,%.3f",
                    data.lateral_p_term, data.lateral_i_term, data.lateral_d_term);
            fprintf(file_, ",%.3f,%.3f",
                    data.max_lateral_error.to_inches(), data.cumulative_lateral_error.to_inches());
        }

        // Angular control
        if (fields_.angular_pid)
        {
            fprintf(file_, ",%.3f,%.3f",
                    data.angular_error.to_radians(), data.angular_output.to_volts());
            fprintf(file_, ",%.3f,%.3f",
                    data.angular_target.to_radians(), data.angular_actual.to_radians());
            fprintf(file_, ",%.3f,%.3f,%.3f",
                    data.angular_p_term, data.angular_i_term, data.angular_d_term);
            fprintf(file_, ",%.3f,%.3f",
                    data.max_angular_error.to_radians(), data.cumulative_angular_error.to_radians());
        }

        // Settlement tracking
        if (fields_.settlement)
        {
            fprintf(file_, ",%d,%d,%s",
                    data.is_settled ? 1 : 0,
                    data.settle_count,
                    settlement_reason_to_string(data.settlement_reason));
            fprintf(file_, ",%.3f", data.time_to_settle.to_seconds());
        }

        // Motor voltages
        if (fields_.motors)
        {
            fprintf(file_, ",%.3f,%.3f",
                    data.left_motor_voltage.to_volts(), data.right_motor_voltage.to_volts());
        }

        // Path tracking errors
        if (fields_.path_tracking)
        {
            fprintf(file_, ",%.3f,%.3f",
                    data.cross_track_error.to_inches(), data.along_track_error.to_inches());
            fprintf(file_, ",%.3f,%.3f",
                    data.max_cross_track_error.to_inches(), data.cumulative_xte.to_inches());
            fprintf(file_, ",%.3f,%.3f",
                    data.max_along_track_error.to_inches(), data.cumulative_ate.to_inches());
            fprintf(file_, ",%.3f,%.3f,%.3f",
                    data.final_pose_error_x.to_inches(),
                    data.final_pose_error_y.to_inches(),
                    data.final_pose_error_theta.to_radians());
        }

        // Path follower status
        if (fields_.path_status)
        {
            fprintf(file_, ",%s,%.3f,%.3f",
                    path_status_to_string(data.path_status),
                    data.trajectory_time.to_seconds(),
                    data.trajectory_progress);
            fprintf(file_, ",%.3f",
                    data.trajectory_total_time.to_seconds());
            fprintf(file_, ",%.3f,%.3f",
                    data.reference_velocity.to_ips(),
                    data.reference_arc_position.to_inches());
        }

        // Individual wheel velocities
        if (fields_.wheels)
        {
            fprintf(file_, ",%.3f,%.3f",
                    data.left_wheel_velocity.to_rad_per_sec(),
                    data.right_wheel_velocity.to_rad_per_sec());
        }

        // Turn-in-place tracking
        if (fields_.turn_in_place)
        {
            fprintf(file_, ",%.3f,%.3f",
                    data.omega_reference.to_rad_per_sec(),
                    data.omega_error.to_rad_per_sec());
            fprintf(file_, ",%.3f,%.3f",
                    data.omega_pid_output,
                    data.omega_commanded.to_rad_per_sec());
            fprintf(file_, ",%.3f,%.3f",
                    data.left_wheel_cmd.to_ips(),
                    data.right_wheel_cmd.to_ips());
        }

        // Loop timing metrics
        if (fields_.timing)
        {
            fprintf(file_, ",%.3f,%.3f,%.3f",
                    data.loop_time.to_milliseconds(),
                    data.max_loop_time.to_milliseconds(),
                    data.min_loop_time.to_milliseconds());
            fprintf(file_, ",%.3f,%u,%u",
                    data.avg_loop_time.to_milliseconds(),
                    data.timing_violations,
                    data.total_loop_count);
            fprintf(file_, ",%.3f", data.target_loop_time.to_milliseconds());
        }

        // Battery monitoring
        if (fields_.battery)
        {
            fprintf(file_, ",%.3f,%.1f,%.2f,%d",
                    data.battery_voltage.to_volts(),
                    data.battery_capacity_percent,
                    data.voltage_compensation_scale,
                    data.voltage_compensation_active ? 1 : 0);
        }

        if (fields_.ekf)
        {
            fprintf(file_, ",%.3f,%.3f,%.3f",
                    data.ekf_x.to_inches(), data.ekf_y.to_inches(),
                    data.ekf_theta.to_degrees());
            fprintf(file_, ",%.3f,%.3f,%.3f",
                    data.ekf_x_std * 39.3701, // convert meters to inches
                    data.ekf_y_std * 39.3701,
                    data.ekf_theta_std);
            fprintf(file_, ",%.3f,%.3f,%d,%s",
                    data.front_distance_measured.to_inches(),
                    data.front_distance_expected.to_inches(),
                    data.front_distance_valid ? 1 : 0,
                    field::FieldMap::wall_to_string(data.front_wall));
            fprintf(file_, ",%.3f,%.3f,%d,%s",
                    data.back_distance_measured.to_inches(),
                    data.back_distance_expected.to_inches(),
                    data.back_distance_valid ? 1 : 0,
                    field::FieldMap::wall_to_string(data.back_wall));
            fprintf(file_, ",%.3f,%.3f",
                    data.front_innovation.to_inches(),
                    data.back_innovation.to_inches());
        }

        if (fields_.ramsete)
        {
            fprintf(file_, ",%.3f,%.3f,%.3f",
                    data.ramsete.x_ref.to_inches(),
                    data.ramsete.y_ref.to_inches(),
                    data.ramsete.theta_ref.to_degrees());
            fprintf(file_, ",%.3f,%.3f,%.6f",
                    data.ramsete.v_ref.to_ips(),
                    data.ramsete.omega_ref.to_rad_per_sec(),
                    data.ramsete.curvature);
            fprintf(file_, ",%.3f,%.3f,%.3f",
                    data.ramsete.e_x.to_inches(),
                    data.ramsete.e_y.to_inches(),
                    data.ramsete.e_theta.to_degrees());
            fprintf(file_, ",%.3f,%.3f",
                    data.ramsete.v_cmd.to_ips(),
                    data.ramsete.omega_cmd.to_rad_per_sec());
            fprintf(file_, ",%.3f,%.3f",
                    data.ramsete.x_error.to_inches(),
                    data.ramsete.y_error.to_inches());
            fprintf(file_, ",%.3f,%.3f,%.3f",
                    data.ramsete.k_gain,
                    data.ramsete.b_param,
                    data.ramsete.zeta_param);
        }

        fprintf(file_, "\n");
        fflush(file_); // Flush after each row for safety (data persists even if crash)
    }

    std::string Logger::generate_filename(const std::string &segment_name) const
    {
        std::string filename = "/usd/" + base_name_;

        if (!segment_name.empty())
        {
            filename += "_" + segment_name;
        }
        else if (auto_segment_)
        {
            // First file in segmented mode with no name
            filename += "_segment_0";
        }

        filename += ".csv";
        return filename;
    }

} // namespace abclib::telemetry