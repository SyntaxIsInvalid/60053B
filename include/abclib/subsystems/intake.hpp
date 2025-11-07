// intake.hpp
#pragma once

#include "abclib/hardware/motor_group.hpp"
#include "abclib/units/units.hpp"
#include "api.h"

namespace abclib::subsystems
{
    enum class IntakeState
    {
        IDLE,
        INTAKING,
        OUTTAKING
    };

    class Intake
    {
    private:
        hardware::AdvancedMotorGroup *motors;
        std::unique_ptr<hardware::AdvancedMotorGroup> owned_motors;
        IntakeState current_state;
        units::Voltage intake_voltage;
        units::Voltage outtake_voltage;

        std::optional<pros::Task> timed_task_;
        pros::Mutex task_mutex_;

    public:
        // Constructor accepting pointer to motor group
        Intake(hardware::AdvancedMotorGroup *motor_group,
               units::Voltage intake_v = units::Voltage::from_volts(12.0),
               units::Voltage outtake_v = units::Voltage::from_volts(-12.0));

        // Constructor that owns the motors
        Intake(std::vector<int8_t> ports,
               pros::MotorGearset gearset,
               const hardware::motor_group_config &config = hardware::motor_group_config{},
               units::Voltage intake_v = units::Voltage::from_volts(12.0),
               units::Voltage outtake_v = units::Voltage::from_volts(-12.0));

        // Basic control
        void set_intake();
        void set_outtake();
        void set_idle(); // Yes, this stops the motors
        void toggle_intake();

        // Timed control - runs for a duration then stops
        void intake_for(units::Time duration);
        void outtake_for(units::Time duration);

        void intake_for_voltage(units::Voltage voltage, units::Time duration);
        void outtake_for_voltage(units::Voltage voltage, units::Time duration);

        // Velocity control with PROS built-in controller
        void intake_at_velocity(units::RPM target_rpm);
        void outtake_at_velocity(units::RPM target_rpm);
        void intake_at_velocity_for(units::RPM target_rpm, units::Time duration);
        void outtake_at_velocity_for(units::RPM target_rpm, units::Time duration);

        // Direct voltage control (advanced)
        void set_voltage(units::Voltage voltage);

        // State queries
        IntakeState get_state() const { return current_state; }
        bool is_intaking() const { return current_state == IntakeState::INTAKING; }
        bool is_outtaking() const { return current_state == IntakeState::OUTTAKING; }
        bool is_idle() const { return current_state == IntakeState::IDLE; }
    };

} // namespace abclib::subsystems