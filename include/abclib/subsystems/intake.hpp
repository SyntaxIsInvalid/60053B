// intake.hpp
#pragma once

#include "abclib/hardware/motor_group.hpp"
#include "abclib/units/units.hpp"

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
        IntakeState current_state;
        units::Voltage intake_voltage;
        units::Voltage outtake_voltage;

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

        void set_intake();
        void set_outtake();
        void set_idle();
        void toggle_intake();

        void set_voltage(units::Voltage voltage);
        IntakeState get_state() const { return current_state; }
    };

} // namespace abclib::subsystems