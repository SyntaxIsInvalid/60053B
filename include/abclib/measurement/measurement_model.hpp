#pragma once

namespace abclib::estimation
{
    template<typename T>
    class IMeasurementModel
    {
    public:
        virtual ~IMeasurementModel() = default;
        
        virtual T get_measurement() = 0;
        
        virtual double get_uncertainty() { return 0.0; }
        
        virtual void reset() = 0;
    };
}