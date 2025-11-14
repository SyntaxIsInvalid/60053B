#pragma once

#include "intake.hpp"

namespace abclib::subsystems
{
    class DummyIntake : public Intake
    {
    public:
        // Use a dummy port that won't actually be initialized
        DummyIntake() : Intake(
            {1},  // fake port number
            pros::MotorGearset::blue,
            hardware::motor_group_config{},
            units::Voltage::from_volts(0),
            units::Voltage::from_volts(0)) 
        {}

        // Override all methods to do nothing
        void set_intake() override {}
        void set_outtake() override {}
        void set_idle() override {}
        void toggle_intake() override {}
        
        void intake_for(units::Time duration) override {}
        void outtake_for(units::Time duration) override {}
        
        void intake_for_voltage(units::Voltage voltage, units::Time duration) override {}
        void outtake_for_voltage(units::Voltage voltage, units::Time duration) override {}
        
        void intake_at_velocity(units::AngularVelocity target_rpm) override {}
        void outtake_at_velocity(units::AngularVelocity target_rpm) override {}
        void intake_at_velocity_for(units::AngularVelocity target_rpm, units::Time duration) override {}
        void outtake_at_velocity_for(units::AngularVelocity target_rpm, units::Time duration) override {}
        
        void set_voltage(units::Voltage voltage) override {}
    };

} // namespace abclib::subsystems