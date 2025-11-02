#pragma once

#include "measurement_model.hpp"
#include "api.h"
#include "abclib/units/units.hpp"
#include <cmath>

namespace abclib::estimation
{
    class IMUMeasurementModel : public IMeasurementModel<units::Radians>
    {
    private:
        pros::IMU* imu_;
        units::Radians prev_reading_;
        
    public:
        explicit IMUMeasurementModel(pros::IMU* imu)
            : imu_(imu), prev_reading_(units::Radians(0.0))
        {
            if (imu_) {
                double imu_reading = imu_->get_rotation();
                prev_reading_ = std::isnan(imu_reading) ? 
                    units::Radians(0.0) : 
                    units::Degrees(-imu_reading).to_radians();
            }
        }
        
        units::Radians get_measurement() override
        {
            if (!imu_) return units::Radians(0.0);
            
            double imu_reading = imu_->get_rotation();
            units::Radians current = std::isnan(imu_reading) ? 
                units::Radians(0.0) : 
                units::Degrees(-imu_reading).to_radians();
            
            units::Radians delta = units::Radians(current.value - prev_reading_.value);
            prev_reading_ = current;
            return delta;
        }
        
        void reset() override
        {
            if (imu_) {
                double imu_reading = imu_->get_rotation();
                prev_reading_ = std::isnan(imu_reading) ? 
                    units::Radians(0.0) : 
                    units::Degrees(-imu_reading).to_radians();
            }
        }
    };
}