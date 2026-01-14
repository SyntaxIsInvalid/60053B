#pragma once
#include "abclib/hardware/motor_group.hpp"
#include "abclib/hardware/motor_tracking_wheel.hpp"
#include "abclib/units/units.hpp"
#include "api.h"
#include "abclib/control/ramsete.hpp"
#include "abclib/estimation/estimator_config.hpp"
namespace abclib::robot_config
{

    // Motor ports
    inline const std::vector<int8_t> LEFT_MOTOR_PORTS = {-10, 4};
    inline const std::vector<int8_t> RIGHT_MOTOR_PORTS = {6, -11};

    // Sensor ports
    constexpr int8_t IMU_PORT = 9;

    // Physical dimensions (using typed units)
    inline constexpr units::Length WHEEL_DIAMETER = units::Length::from_inches(3.25);
    inline constexpr units::Length TRACK_WIDTH = units::Length::from_inches(14.0);
    inline constexpr units::Length Y_TRACKER_OFFSET = units::Length::from_inches(7.0);

    // Motor configurations
    inline hardware::motor_group_config get_left_motor_config()
    {
        return hardware::motor_group_config{
            .kS = 0.535278,
            .kV = 0.158462,
            .kA = 0.012848,
            .kPv = 0.0,
            .kIv = 0.25,
            .kDv = 0.00,
            .enable_voltage_compensation = true,
            .compensation_nominal = units::Voltage::from_volts(12.0),
            .compensation_min_battery = units::Voltage::from_volts(11.5)};
    }

    inline hardware::motor_group_config get_right_motor_config()
    {
        return hardware::motor_group_config{
            .kS = 0.535278,
            .kV = 0.158462,
            .kA = 0.012848,
            .kPv = 0.0,
            .kIv = 0.0,
            .kDv = 0.00,
            .enable_voltage_compensation = true,
            .compensation_nominal = units::Voltage::from_volts(12.0),
            .compensation_min_battery = units::Voltage::from_volts(11.5)};
    }

    // PID constants
    inline control::PIDConstants get_lateral_pid()
    {
        return control::PIDConstants(0.6, 5, 0);
    }

    inline control::PIDConstants get_angular_pid()
    {
        return control::PIDConstants(5, 50, 0);
    }

    // Turn-in-place feedforward constants (for turn_to_heading_profiled)
    constexpr double TURN_IN_PLACE_KS = 1.278592;
    constexpr double TURN_IN_PLACE_KV = 0.170242;
    constexpr double TURN_IN_PLACE_KA = 0.012877;

    constexpr double LATERAL_KS = 0.535278;
    constexpr double LATERAL_KV = 0.158462;
    constexpr double LATERAL_KA = 0.012848;

    // profiled turn pid constants
    inline control::PIDConstants get_profiled_turn_pid()
    {
        return control::PIDConstants(25, 0.0, 0.0); // Tune these values
    }

    // profiled lateral pid constants
    inline control::PIDConstants get_profiled_lateral_pid()
    {
        return control::PIDConstants(2.0, 0.0, 0.0); // Tune these values
    }

    // Ramsete controller constants (for path following)
    inline control::RamseteConstants get_ramsete_config()
    {
        return control::RamseteConstants{2.0, 0.7}; // b, zeta
    }

    inline estimation::EstimatorConfig get_estimator_config()
    {
        return estimation::EstimatorConfig{
            .type = estimation::FilterType::EKF,  // Change to EKF when ready to test
            .vertical_offset = Y_TRACKER_OFFSET,
            .horizontal_offset = units::Length::from_inches(0.0),
            .ekf = {
                .process_noise_x = 0.01,
                .process_noise_y = 0.01,
                .process_noise_theta = 0.01
            }
        };
    }

}