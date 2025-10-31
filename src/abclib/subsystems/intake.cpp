// intake.cpp
#include "abclib/subsystems/intake.hpp"

namespace abclib::subsystems
{
    Intake::Intake(hardware::AdvancedMotorGroup* motor_group,
                   units::Voltage intake_v,
                   units::Voltage outtake_v)
        : motors(motor_group),
          current_state(IntakeState::IDLE),
          intake_voltage(intake_v),
          outtake_voltage(outtake_v)
    {
    }

    Intake::Intake(std::vector<int8_t> ports,
                   pros::MotorGearset gearset,
                   const hardware::motor_group_config& config,
                   units::Voltage intake_v,
                   units::Voltage outtake_v)
        : motors(nullptr),
          current_state(IntakeState::IDLE),
          intake_voltage(intake_v),
          outtake_voltage(outtake_v)
    {
        // Create owned motor group
        static hardware::AdvancedMotorGroup owned_motors(ports, gearset, config);
        motors = &owned_motors;
    }

    void Intake::set_intake()
    {
        current_state = IntakeState::INTAKING;
        motors->move_voltage(intake_voltage);
    }

    void Intake::set_outtake()
    {
        current_state = IntakeState::OUTTAKING;
        motors->move_voltage(outtake_voltage);
    }

    void Intake::set_idle()
    {
        current_state = IntakeState::IDLE;
        motors->brake();
    }

    void Intake::toggle_intake()
    {
        if (current_state == IntakeState::INTAKING)
        {
            set_idle();
        }
        else
        {
            set_intake();
        }
    }

    void Intake::set_voltage(units::Voltage voltage)
    {
        motors->move_voltage(voltage);
    }

} // namespace abclib::subsystems