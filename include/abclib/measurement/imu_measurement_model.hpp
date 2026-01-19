#pragma once

#include "measurement_model.hpp"
#include "api.h"
#include "abclib/units/units.hpp"
#include <cmath>

namespace abclib::estimation
{
    class IMUMeasurementModel : public IMeasurementModel<units::Angle>
    {
    private:
        pros::IMU* imu_;
        units::Angle prev_reading_;
        
    public:
        explicit IMUMeasurementModel(pros::IMU* imu)
            : imu_(imu), prev_reading_(units::Angle::from_radians(0.0))
        {
            if (imu_) {
                double imu_reading = imu_->get_rotation();
                prev_reading_ = std::isnan(imu_reading) ? 
                    units::Angle::from_radians(0.0) : 
                    units::Angle::from_degrees(-imu_reading);
            }
        }
        
        units::Angle get_measurement() override
        {
            if (!imu_) return units::Angle::from_radians(0.0);
            
            double imu_reading = imu_->get_rotation();
            units::Angle current = std::isnan(imu_reading) ? 
                units::Angle::from_radians(0.0) : 
                units::Angle::from_degrees(-imu_reading);
            
            units::Angle delta = current - prev_reading_;
            prev_reading_ = current;
            return delta;
        }
        
        void reset() override
        {
            if (imu_) {
                double imu_reading = imu_->get_rotation();
                prev_reading_ = std::isnan(imu_reading) ? 
                    units::Angle::from_radians(0.0) : 
                    units::Angle::from_degrees(-imu_reading);
            }
        }
    };
}