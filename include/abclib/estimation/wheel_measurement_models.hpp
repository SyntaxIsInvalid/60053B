#pragma once

#include "measurement_model.hpp"
#include "abclib/hardware/tracking_wheel_interface.hpp"
#include "abclib/units/units.hpp"

namespace abclib::estimation
{
    class WheelMeasurementModel : public IMeasurementModel<units::Length>
    {
    private:
        hardware::ITrackingWheel* wheel_;
        units::Length prev_reading_;
        
    public:
        explicit WheelMeasurementModel(hardware::ITrackingWheel* wheel)
            : wheel_(wheel), prev_reading_(units::Length::from_inches(0.0))
        {
            if (wheel_) {
                prev_reading_ = wheel_->get_distance();
            }
        }
        
        units::Length get_measurement() override
        {
            if (!wheel_) return units::Length::from_inches(0.0);
            
            units::Length current = wheel_->get_distance();
            units::Length delta = current - prev_reading_;
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