#include "abclib/estimation/estimator_factory.hpp"
#include "abclib/estimation/geometric_odometry_estimator.hpp"
#include "abclib/estimation/blended_geometric_estimator.hpp"
#include "abclib/estimation/particle_filter_estimator.hpp"
#include "abclib/measurement/distance_measurement_model.hpp"
#include "abclib/configs/robot_selection.hpp"
#include <algorithm>
#include <vector>

#ifdef ROBOT_TEST_DRIVE
#include "abclib/configs/test_robot.hpp"
#elif defined(ROBOT_COMPETITION)
#include "abclib/configs/competition_robot.hpp"
#else
#error "No robot configuration selected!"
#endif

namespace abclib::estimation
{
    std::unique_ptr<IStateEstimator> create_estimator(
        const EstimatorConfig &config,
        IMeasurementModel<units::Length> *vertical_model,
        IMeasurementModel<units::Length> *horizontal_model,
        IMeasurementModel<units::Angle> *imu_model)
    {
        auto build_sensor_array = [&]() -> std::vector<DistanceSensorConfig>
        {
            auto hardware_sensors = robot_config::get_distance_sensors();
            auto sensor_configs = robot_config::get_distance_sensor_configs();

            size_t num_sensors = std::min(hardware_sensors.size(), sensor_configs.size());
            for (size_t i = 0; i < num_sensors; i++)
            {
                if (hardware_sensors[i])
                {
                    sensor_configs[i].sensor = new DistanceSensorMeasurementModel(hardware_sensors[i]);
                }
            }

            return sensor_configs;
        };

        switch (config.type)
        {
        case FilterType::GEOMETRIC:
        {
            auto sensor_configs = build_sensor_array();
            return std::make_unique<GeometricOdometryEstimator>(
                vertical_model,
                horizontal_model,
                imu_model,
                config.vertical_offset,
                config.horizontal_offset,
                config.field_config,
                sensor_configs);
        }

        case FilterType::BLENDED_GEOMETRIC:
        {
            auto sensor_configs = build_sensor_array();
            return std::make_unique<BlendedGeometricEstimator>(
                vertical_model,
                horizontal_model,
                imu_model,
                sensor_configs,
                config);
        }

        case FilterType::PARTICLE_FILTER:  // add this case
        {
            auto sensor_configs = build_sensor_array();
            return std::make_unique<ParticleFilterEstimator>(
                vertical_model,
                horizontal_model,
                imu_model,
                sensor_configs,
                config);
        }

        default:
        {
            auto sensor_configs = build_sensor_array();
            return std::make_unique<GeometricOdometryEstimator>(
                vertical_model,
                horizontal_model,
                imu_model,
                config.vertical_offset,
                config.horizontal_offset,
                config.field_config,
                sensor_configs);
        }
        }
    }
}