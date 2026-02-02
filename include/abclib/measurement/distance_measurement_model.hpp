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
     */
    class DistanceSensorMeasurementModel : public IMeasurementModel<units::Length>
    {
    private:
        pros::Distance* sensor_;
        
        // Current state
        units::Length last_reading_;
        bool last_reading_valid_;
        
        // NEW: Track hardware updates
        int32_t last_raw_value_mm_;
        uint32_t last_read_time_ms_;
        
        // Sensor specs
        static constexpr units::Length MIN_RANGE = units::Length::from_mm(20.0);
        static constexpr units::Length MAX_RANGE = units::Length::from_mm(2000.0);
        static constexpr units::Length ACCURACY_THRESHOLD = units::Length::from_mm(200.0);
        static constexpr units::Length ACCURACY_CLOSE = units::Length::from_mm(15.0);
        static constexpr double ACCURACY_FAR_PERCENT = 0.05;
        
        // Sensor update rate (30Hz = ~33ms between updates)
        static constexpr uint32_t SENSOR_UPDATE_INTERVAL_MS = 33;
        
        // Confidence threshold (for readings > 200mm)
        static constexpr int MIN_CONFIDENCE = 20;  // Conservative: clearly bad readings
        
    public:
        explicit DistanceSensorMeasurementModel(pros::Distance* sensor)
            : sensor_(sensor), 
              last_reading_(units::Length::from_mm(0.0)),
              last_reading_valid_(false),
              last_raw_value_mm_(-1),
              last_read_time_ms_(0)
        {
        }
        
        /**
         * @brief Check if sensor has new data available
         * 
         * Returns true if:
         * 1. Reading value changed, OR
         * 2. 33ms passed (30Hz hardware update rate)
         * 
         * This is a HARDWARE check - does sensor have fresh data?
         * The FILTER decides if this data is useful (robot moved, etc.)
         */
        bool has_new_reading()
        {
            if (!sensor_) return false;
            
            uint32_t now = pros::millis();
            int32_t current_value = sensor_->get();
            
            bool value_changed = (current_value != last_raw_value_mm_);
            bool time_passed = (now - last_read_time_ms_) >= SENSOR_UPDATE_INTERVAL_MS;
            
            if (value_changed || time_passed) {
                last_raw_value_mm_ = current_value;
                last_read_time_ms_ = now;
                return true;
            }
            
            return false;
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
            
            // Additional check: confidence for far readings
            if (last_reading_valid_ && distance > ACCURACY_THRESHOLD) {
                int confidence = sensor_->get_confidence();
                if (confidence < MIN_CONFIDENCE) {
                    last_reading_valid_ = false;  // Low confidence, reject
                }
            }
            
            if (last_reading_valid_) {
                last_reading_ = distance;
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
            return sensor_ && last_reading_valid_;
        }
        
        void reset() override
        {
            last_reading_ = units::Length::from_mm(0.0);
            last_reading_valid_ = false;
            last_raw_value_mm_ = -1;
            last_read_time_ms_ = 0;
        }
        
        // Diagnostics
        uint32_t get_time_since_last_reading_ms() const
        {
            return pros::millis() - last_read_time_ms_;
        }
    };
}