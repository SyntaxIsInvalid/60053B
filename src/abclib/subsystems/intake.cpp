// intake.cpp
#include "abclib/subsystems/intake.hpp"
#include <mutex>
namespace abclib::subsystems
{
    Intake::Intake(hardware::AdvancedMotorGroup *motor_group,
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
                   const hardware::motor_group_config &config,
                   units::Voltage intake_v,
                   units::Voltage outtake_v)
        : current_state(IntakeState::IDLE),
          intake_voltage(intake_v),
          outtake_voltage(outtake_v)
    {
        owned_motors = std::make_unique<hardware::AdvancedMotorGroup>(ports, gearset, config);
        motors = owned_motors.get();
    }

    void Intake::set_intake()
    {
        // Cancel any running timed task
        {
            std::lock_guard<pros::Mutex> lock(task_mutex_);
            if (timed_task_.has_value())
            {
                timed_task_->notify();
                timed_task_ = std::nullopt;
            }
        }

        current_state = IntakeState::INTAKING;
        motors->move_voltage(intake_voltage);
    }

    void Intake::set_outtake()
    {
        // Cancel any running timed task
        {
            std::lock_guard<pros::Mutex> lock(task_mutex_);
            if (timed_task_.has_value())
            {
                timed_task_->notify();
                timed_task_ = std::nullopt;
            }
        }

        current_state = IntakeState::OUTTAKING;
        motors->move_voltage(outtake_voltage);
    }

    void Intake::set_idle()
    {
        // Cancel any running timed task
        {
            std::lock_guard<pros::Mutex> lock(task_mutex_);
            if (timed_task_.has_value())
            {
                timed_task_->notify();
                timed_task_ = std::nullopt;
            }
        }

        current_state = IntakeState::IDLE;
        motors->brake(); // Yes, this stops the motors
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

    void Intake::intake_for(units::Time duration)
    {
        // Cancel any existing timed task
        {
            std::lock_guard<pros::Mutex> lock(task_mutex_);
            if (timed_task_.has_value())
            {
                timed_task_->notify();
            }
        }

        // Start intake immediately
        current_state = IntakeState::INTAKING;
        motors->move_voltage(intake_voltage);

        // Create task to stop after duration
        {
            std::lock_guard<pros::Mutex> lock(task_mutex_);
            timed_task_ = pros::Task([this, duration]()
                                     {
                uint32_t start = pros::millis();
                while ((pros::millis() - start) < duration.to_millis_uint()) {
                    if (pros::Task::notify_take(true, 10) > 0) {
                        return; // Task was cancelled
                    }
                }
                this->set_idle(); });
        }
    }

    void Intake::outtake_for(units::Time duration)
    {
        // Cancel any existing timed task
        {
            std::lock_guard<pros::Mutex> lock(task_mutex_);
            if (timed_task_.has_value())
            {
                timed_task_->notify();
            }
        }

        // Start outtake immediately
        current_state = IntakeState::OUTTAKING;
        motors->move_voltage(outtake_voltage);

        // Create task to stop after duration
        {
            std::lock_guard<pros::Mutex> lock(task_mutex_);
            timed_task_ = pros::Task([this, duration]()
                                     {
                uint32_t start = pros::millis();
                while ((pros::millis() - start) < duration.to_millis_uint()) {
                    if (pros::Task::notify_take(true, 10) > 0) {
                        return; // Task was cancelled
                    }
                }
                this->set_idle(); });
        }
    }

    void Intake::intake_at_velocity(units::RPM target_rpm)
    {
        // Cancel any running timed task
        {
            std::lock_guard<pros::Mutex> lock(task_mutex_);
            if (timed_task_.has_value())
            {
                timed_task_->notify();
                timed_task_ = std::nullopt;
            }
        }

        current_state = IntakeState::INTAKING;
        motors->move_velocity_pros(units::MotorAngularVelocity::from_rpm(target_rpm.value));
    }

    void Intake::outtake_at_velocity(units::RPM target_rpm)
    {
        // Cancel any running timed task
        {
            std::lock_guard<pros::Mutex> lock(task_mutex_);
            if (timed_task_.has_value())
            {
                timed_task_->notify();
                timed_task_ = std::nullopt;
            }
        }

        current_state = IntakeState::OUTTAKING;
        // Negative RPM for outtake
        motors->move_velocity_pros(units::MotorAngularVelocity::from_rpm(-target_rpm.value));
    }

    void Intake::intake_at_velocity_for(units::RPM target_rpm, units::Time duration)
    {
        // Cancel any existing timed task
        {
            std::lock_guard<pros::Mutex> lock(task_mutex_);
            if (timed_task_.has_value())
            {
                timed_task_->notify();
            }
        }

        // Start intake at velocity immediately
        current_state = IntakeState::INTAKING;
        motors->move_velocity_pros(units::MotorAngularVelocity::from_rpm(target_rpm.value));

        // Create task to stop after duration
        {
            std::lock_guard<pros::Mutex> lock(task_mutex_);
            timed_task_ = pros::Task([this, duration]()
                                     {
            uint32_t start = pros::millis();
            while ((pros::millis() - start) < duration.to_millis_uint()) {
                if (pros::Task::notify_take(true, 10) > 0) {
                    return;
                }
            }
            this->set_idle(); });
        }
    }

    void Intake::outtake_at_velocity_for(units::RPM target_rpm, units::Time duration)
    {
        // Cancel any existing timed task
        {
            std::lock_guard<pros::Mutex> lock(task_mutex_);
            if (timed_task_.has_value())
            {
                timed_task_->notify();
            }
        }

        // Start outtake at velocity immediately
        current_state = IntakeState::OUTTAKING;
        motors->move_velocity_pros(units::MotorAngularVelocity::from_rpm(-target_rpm.value));

        // Create task to stop after duration
        {
            std::lock_guard<pros::Mutex> lock(task_mutex_);
            timed_task_ = pros::Task([this, duration]()
                                     {
            uint32_t start = pros::millis();
            while ((pros::millis() - start) < duration.to_millis_uint()) {
                if (pros::Task::notify_take(true, 10) > 0) {
                    return;
                }
            }
            this->set_idle(); });
        }
    }

    void Intake::set_voltage(units::Voltage voltage)
    {
        motors->move_voltage(voltage);
    }

} // namespace abclib::subsystems