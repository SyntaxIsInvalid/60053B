#pragma once

#include "measurement_model.hpp"
#include "abclib/hardware/tracking_wheel_interface.hpp"
#include "abclib/units/units.hpp"

namespace abclib::estimation
{
    class WheelMeasurementModel : public IMeasurementModel<units::Distance>
    {
    private:
        hardware::ITrackingWheel* wheel_;
        units::Distance prev_reading_;
        
    public:
        explicit WheelMeasurementModel(hardware::ITrackingWheel* wheel)
            : wheel_(wheel), prev_reading_(units::Distance::from_inches(0.0))
        {
            if (wheel_) {
                prev_reading_ = wheel_->get_distance();
            }
        }
        
        units::Distance get_measurement() override
        {
            if (!wheel_) return units::Distance::from_inches(0.0);
            
            units::Distance current = wheel_->get_distance();
            units::Distance delta = current - prev_reading_;
            prev_reading_ = current;
            return delta;
        }
        
        void reset() override
        {
            if (wheel_) {
                prev_reading_ = wheel_->get_distance();
            }
        }
    };
}