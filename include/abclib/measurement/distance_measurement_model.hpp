#pragma once

#include "measurement_model.hpp"
#include "api.h"
#include "abclib/units/units.hpp"

namespace abclib::estimation
{
    /**
     * @brief Measurement model for V5 Distance Sensor
     * 
     * Specs:
     * - Range: 20mm to 2000mm
     * - Accuracy: ±15mm below 200mm, ±5% above 200mm
     * - Update rate: ~30Hz
     * 
     * NOTE: Uses rate limiting to prevent applying stale measurements
     * when running at higher rates (e.g., 100Hz EKF with 30Hz sensors)
     */
    class DistanceSensorMeasurementModel : public IMeasurementModel<units::Length>
    {
    private:
        pros::Distance* sensor_;
        units::Length last_reading_;
        bool last_reading_valid_;
        uint32_t last_update_time_ms_;  // NEW: When we last got new data
        
        // Sensor specs
        static constexpr units::Length MIN_RANGE = units::Length::from_mm(20.0);
        static constexpr units::Length MAX_RANGE = units::Length::from_mm(2000.0);
        static constexpr units::Length ACCURACY_THRESHOLD = units::Length::from_mm(200.0);
        static constexpr units::Length ACCURACY_CLOSE = units::Length::from_mm(15.0);
        static constexpr double ACCURACY_FAR_PERCENT = 0.05;
        
        // NEW: Sensor update rate (30Hz = ~33ms between updates)
        static constexpr uint32_t MIN_UPDATE_INTERVAL_MS = 33;
        
    public:
        explicit DistanceSensorMeasurementModel(pros::Distance* sensor)
            : sensor_(sensor), 
              last_reading_(units::Length::from_mm(0.0)),
              last_reading_valid_(false),
              last_update_time_ms_(0)  // NEW
        {
        }
        
        units::Length get_measurement() override
        {
            if (!sensor_) {
                last_reading_valid_ = false;
                return units::Length::from_mm(0.0);
            }
            
            // Get raw reading from PROS API (returns int32_t in mm)
            int32_t distance_mm = sensor_->get();
            units::Length distance = units::Length::from_mm(distance_mm);
            
            // Check if reading is in valid range
            last_reading_valid_ = (distance >= MIN_RANGE && distance <= MAX_RANGE);
            
            if (last_reading_valid_) {
                last_reading_ = distance;
                // NEW: Mark when we consumed this data
                last_update_time_ms_ = pros::millis();
            } else {
                last_reading_ = units::Length::from_mm(0.0);
            }
            
            return last_reading_;
        }
        
        double get_uncertainty() override
        {
            if (!last_reading_valid_) {
                return 1.0; // 1 meter uncertainty for invalid readings
            }
            
            if (last_reading_ < ACCURACY_THRESHOLD) {
                // Below 200mm: ±15mm accuracy
                return ACCURACY_CLOSE.to_meters();
            } else {
                // Above 200mm: ±5% accuracy
                return ACCURACY_FAR_PERCENT * last_reading_.to_meters();
            }
        }
        
        bool is_valid() const
        {
            // Check basic validity first
            if (!sensor_ || !last_reading_valid_) {
                return false;
            }
            
            // NEW: Only return true if enough time has passed since last update
            // This prevents applying the same measurement multiple times when
            // EKF runs at 100Hz but sensor updates at 30Hz
            uint32_t now = pros::millis();
            uint32_t time_since_last_update = now - last_update_time_ms_;
            
            return time_since_last_update >= MIN_UPDATE_INTERVAL_MS;
        }
        
        void reset() override
        {
            last_reading_ = units::Length::from_mm(0.0);
            last_reading_valid_ = false;
            last_update_time_ms_ = 0;  // NEW
        }
        
        // NEW: Diagnostic methods for telemetry
        uint32_t get_time_since_last_update_ms() const
        {
            return pros::millis() - last_update_time_ms_;
        }
        
        uint32_t get_last_update_time() const
        {
            return last_update_time_ms_;
        }
    };
}