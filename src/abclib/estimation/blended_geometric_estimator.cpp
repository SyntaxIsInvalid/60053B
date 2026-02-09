#include "abclib/estimation/blended_geometric_estimator.hpp"
#include <cmath>
#include <mutex>
#include "abclib/math/angles.hpp"
#include "abclib/telemetry/telemetry.hpp"
#include "abclib/measurement/distance_measurement_model.hpp"
#include "abclib/field/field_map.hpp"
#include "abclib/field/coordinate_transform.hpp"
#include "abclib/math/mahalanobis.hpp"

namespace abclib::estimation
{
    BlendedGeometricEstimator::BlendedGeometricEstimator(
        IMeasurementModel<units::Length> *vertical_model,
        IMeasurementModel<units::Length> *horizontal_model,
        IMeasurementModel<units::Angle> *imu_model,
        units::Length vertical_offset,
        units::Length horizontal_offset,
        const field::FieldConfig &field_config,
        const std::vector<DistanceSensorConfig> &distance_sensors,
        const BlendingConfig &blend_config)
        : vertical_model_(vertical_model),
          horizontal_model_(horizontal_model),
          imu_model_(imu_model),
          vertical_offset_(vertical_offset),
          horizontal_offset_(horizontal_offset),
          distance_sensors_(distance_sensors),
          field_map_(field_config),
          blend_config_(blend_config)
    {
    }

    BlendedGeometricEstimator::~BlendedGeometricEstimator()
    {
        std::lock_guard<pros::Mutex> lock(task_mutex_);
        if (tracking_task_.has_value())
        {
            tracking_task_->notify();
        }
    }

    void BlendedGeometricEstimator::init()
    {
        std::lock_guard<pros::Mutex> lock(task_mutex_);
        if (!tracking_task_.has_value())
        {
            tracking_task_ = pros::Task([this]
                                        {
                while (pros::Task::notify_take(true, 0) == 0) {
                    this->update();
                    pros::delay(10);
                } });
        }
    }

    void BlendedGeometricEstimator::stop()
    {
        std::lock_guard<pros::Mutex> lock(task_mutex_);
        if (tracking_task_.has_value())
        {
            tracking_task_->notify();
            tracking_task_ = std::nullopt;
        }
    }

    void BlendedGeometricEstimator::reset()
    {
        std::lock_guard<pros::Mutex> lock(pose_mutex_);
        current_pose_ = Pose();

        if (vertical_model_)
            vertical_model_->reset();
        if (horizontal_model_)
            horizontal_model_->reset();
        if (imu_model_)
            imu_model_->reset();

        // Reset cumulative correction tracking
        auto &telem = abclib::telemetry::g_telemetry.get_write_buffer();
        telem.correction_x_total = units::Length::from_inches(0);
        telem.correction_y_total = units::Length::from_inches(0);
        abclib::telemetry::g_telemetry.swap();
    }

    void BlendedGeometricEstimator::calibrate()
    {
        if (vertical_model_)
            vertical_model_->reset();
        if (horizontal_model_)
            horizontal_model_->reset();
        if (imu_model_)
            imu_model_->reset();

        std::lock_guard<pros::Mutex> lock(pose_mutex_);
        current_pose_ = Pose();

        // Reset cumulative correction tracking
        auto &telem = abclib::telemetry::g_telemetry.get_write_buffer();
        telem.correction_x_total = units::Length::from_inches(0);
        telem.correction_y_total = units::Length::from_inches(0);
        abclib::telemetry::g_telemetry.swap();
    }

    Pose BlendedGeometricEstimator::get_pose() const
    {
        std::lock_guard<pros::Mutex> lock(pose_mutex_);
        return current_pose_;
    }

    void BlendedGeometricEstimator::set_pose(const Pose &pose)
    {
        std::lock_guard<pros::Mutex> lock(pose_mutex_);
        current_pose_ = pose;

        current_pose_.v = units::Velocity::from_ips(0.0);
        current_pose_.omega = units::AngularVelocity::from_rad_per_sec(0.0);
    }

    double BlendedGeometricEstimator::get_sensor_uncertainty(
        const DistanceSensorMeasurementModel *sensor) const
    {
        if (!sensor)
        {
            return 1.0; // 1 meter default for invalid sensor
        }

        return sensor->get_uncertainty();
    }

    double BlendedGeometricEstimator::get_pose_uncertainty(
        const Pose &robot_pose,
        field::FieldMap::Wall wall) const
    {
        // Phase 1: No pose uncertainty tracking yet
        return 0.0;

        // Phase 2: TODO - Options for future implementation:
        //
        // Option A: Time-based heuristic
        //   double time_since_correction = ...;
        //   return k_drift * time_since_correction;
        //
        // Option B: Covariance-based (if robot_pose.has_uncertainty())
        //   Project covariance to measurement direction based on wall:
        //   if (wall == Wall::NORTH || wall == Wall::SOUTH)
        //       return robot_pose.y_uncertainty_meters();
        //   else
        //       return robot_pose.x_uncertainty_meters();
    }

    void BlendedGeometricEstimator::update()
    {
        update_odometry();
        update_sensor_corrections();

        // Swap telemetry once after both updates complete
        abclib::telemetry::g_telemetry.swap();
    }

    void BlendedGeometricEstimator::update_odometry()
    {
        // Get sensor deltas
        units::Length delta_vertical = vertical_model_ ? vertical_model_->get_measurement() : units::Length::from_inches(0.0);
        units::Length delta_horizontal = horizontal_model_ ? horizontal_model_->get_measurement() : units::Length::from_inches(0.0);
        units::Angle delta_imu = imu_model_ ? imu_model_->get_measurement() : units::Angle::from_radians(0.0);

        // Compute local transformation (body frame)
        math::SE2 local_transform = ArcLengthDifferentialDrive::compute_local_transformation(
            delta_vertical,
            delta_horizontal,
            delta_imu,
            vertical_offset_,
            horizontal_offset_);

        std::lock_guard<pros::Mutex> lock(pose_mutex_);

        // Update pose: new_pose = old_pose * local_transform
        current_pose_.se2 = current_pose_.se2 * local_transform;

        // Compute velocities using the logarithm map
        const double dt = 0.01; // 10ms update rate

        // Get the tangent vector (body-frame velocity integrated over dt)
        Eigen::Vector3d xi = local_transform.log(); // [v_x, v_y, omega]^T

        // Extract velocity components
        double v_y_body = xi(1) / dt;  // Forward velocity (body frame)
        double omega_raw = xi(2) / dt; // Angular velocity

        // For non-holonomic robot, use forward velocity only
        double v_raw = v_y_body;

        // Exponential moving average filter
        const double alpha = 0.3;
        current_pose_.v = units::Velocity::from_ips(
            alpha * v_raw + (1.0 - alpha) * current_pose_.v.to_ips());
        current_pose_.omega = units::AngularVelocity::from_rad_per_sec(
            alpha * omega_raw + (1.0 - alpha) * current_pose_.omega.to_rad_per_sec());

        // Telemetry (same as GeometricOdometryEstimator)
        {
            auto &telem = abclib::telemetry::g_telemetry.get_write_buffer();
            telem.pose_standard = current_pose_;
            telem.pose_corner = field::standard_to_alliance_corner(
                current_pose_,
                telem.current_alliance,
                field_map_.get_field_config());

            telem.heading_wall = field_map_.get_nearest_wall(
                current_pose_,
                units::Angle::from_radians(0.0));

            // Compute sensor pose for distance calculation
            math::SE2 sensor_pose = field_map_.compute_sensor_global_pose(
                current_pose_,
                units::Length::from_inches(0.0),
                units::Length::from_inches(0.0),
                units::Angle::from_radians(0.0));

            units::Length distance = field_map_.compute_distance_to_wall(
                sensor_pose,
                telem.heading_wall);

            telem.heading_wall_valid = (distance.to_inches() >= 0.0);
            telem.heading_distance_to_wall = telem.heading_wall_valid ? distance : units::Length::from_inches(-1.0);

            telem.pose_v_raw = units::Velocity::from_ips(v_raw);
            telem.pose_omega_raw = units::AngularVelocity::from_rad_per_sec(omega_raw);
        }
    }

    void BlendedGeometricEstimator::update_sensor_corrections()
    {
        // Get current pose for validation
        Pose robot_pose;
        {
            std::lock_guard<pros::Mutex> lock(pose_mutex_);
            robot_pose = current_pose_;
        }

        // Check if safe to apply corrections
        bool can_apply_corrections = is_safe_to_blend();

        // Initialize telemetry vectors (only resize if needed - preserves values)
        auto &telem = abclib::telemetry::g_telemetry.get_write_buffer();
        size_t num_sensors = distance_sensors_.size();

        if (telem.distance_measured.size() != num_sensors)
        {
            telem.distance_measured.resize(num_sensors);
            telem.distance_expected.resize(num_sensors);
            telem.distance_innovation.resize(num_sensors);
            telem.distance_walls.resize(num_sensors);
            telem.distance_valid.resize(num_sensors);
            telem.distance_blend_factors.resize(num_sensors);
            telem.distance_mahalanobis.resize(num_sensors); // NEW
        }

        telem.num_total_sensors = num_sensors;
        telem.blending_enabled = blend_config_.enable_blending;
        telem.blending_safe = can_apply_corrections;

        // Reset per-frame correction tracking
        units::Length total_dx = units::Length::from_inches(0);
        units::Length total_dy = units::Length::from_inches(0);

        // Iterate through all sensors
        for (size_t i = 0; i < distance_sensors_.size(); i++)
        {
            auto &sensor_config = distance_sensors_[i];

            // Store blend factor (always available from config)
            telem.distance_blend_factors[i] = sensor_config.blend_factor;

            // Cast to DistanceSensorMeasurementModel to access has_new_reading()
            auto *distance_model = dynamic_cast<DistanceSensorMeasurementModel *>(
                sensor_config.sensor);

            if (!distance_model || !distance_model->has_new_reading())
            {
                // No new data - keep previous telemetry values (prevents flicker)
                continue;
            }

            // Get measurement
            units::Length measured = distance_model->get_measurement();
            telem.distance_measured[i] = measured;

            if (!distance_model->is_valid())
            {
                // Invalid reading
                telem.distance_valid[i] = false;
                telem.distance_expected[i] = units::Length::from_mm(0);
                telem.distance_innovation[i] = units::Length::from_mm(0);
                telem.distance_walls[i] = field::FieldMap::Wall::NONE;
                telem.distance_mahalanobis[i] = -1.0; // NEW: sentinel for invalid
                continue;
            }

            // Validate against expected distance
            units::Length expected;
            field::FieldMap::Wall wall;
            double mahal_dist; // NEW: out-param

            if (validate_sensor_reading(sensor_config, measured, robot_pose,
                                        expected, wall, mahal_dist)) // NEW: added mahal_dist
            {
                // Valid reading
                telem.distance_expected[i] = expected;
                telem.distance_innovation[i] = measured - expected;
                telem.distance_walls[i] = wall;
                telem.distance_valid[i] = true;
                telem.distance_mahalanobis[i] = mahal_dist; // NEW

                // Only apply correction if safe to blend
                if (can_apply_corrections)
                {
                    // Track correction before applying
                    units::Length dx_before, dy_before;
                    {
                        std::lock_guard<pros::Mutex> lock(pose_mutex_);
                        dx_before = current_pose_.x();
                        dy_before = current_pose_.y();
                    }

                    apply_sensor_correction(sensor_config, measured, expected, wall);

                    // Track correction after applying
                    {
                        std::lock_guard<pros::Mutex> lock(pose_mutex_);
                        units::Length dx_after = current_pose_.x();
                        units::Length dy_after = current_pose_.y();

                        total_dx = total_dx + (dx_after - dx_before);
                        total_dy = total_dy + (dy_after - dy_before);
                    }
                }
            }
            else
            {
                // Validation failed - still show innovation for debugging
                telem.distance_expected[i] = expected;
                telem.distance_innovation[i] = measured - expected; // CHANGED: show actual innovation
                telem.distance_walls[i] = wall;
                telem.distance_valid[i] = false;
                telem.distance_mahalanobis[i] = mahal_dist;
            }
        }

        // Update summary stats
        telem.num_active_sensors = 0;
        for (size_t i = 0; i < telem.distance_valid.size(); i++)
        {
            if (telem.distance_valid[i])
            {
                telem.num_active_sensors++;
            }
        }

        // Update correction tracking
        telem.correction_x_frame = total_dx;
        telem.correction_y_frame = total_dy;
        telem.correction_x_total = telem.correction_x_total + total_dx;
        telem.correction_y_total = telem.correction_y_total + total_dy;
    }

    bool BlendedGeometricEstimator::is_safe_to_blend() const
    {
        if (!blend_config_.enable_blending)
            return false;

        if (blend_config_.require_stationary)
        {
            // Robot must be nearly stopped
            return current_pose_.v.to_ips() < 0.1;
        }

        // Check velocity threshold
        return current_pose_.v < blend_config_.max_blend_velocity;
    }

    bool BlendedGeometricEstimator::validate_sensor_reading(
        const DistanceSensorConfig &sensor_config,
        units::Length reading,
        const Pose &robot_pose,
        units::Length &expected_distance,
        field::FieldMap::Wall &detected_wall,
        double &mahalanobis_distance) const
    {
        // Initialize out-param
        mahalanobis_distance = -1.0;

        // 1. Check if sensor is enabled
        if (!sensor_config.enabled)
        {
            return false;
        }

        // 2. Compute sensor's global pose
        math::SE2 sensor_pose = field_map_.compute_sensor_global_pose(
            robot_pose,
            sensor_config.offset_forward,
            sensor_config.offset_lateral,
            sensor_config.bearing);

        // 3. Find which wall sensor ray intersects
        detected_wall = field_map_.find_wall_intersection(sensor_pose);

        // 4. Compute expected distance to that wall
        expected_distance = field_map_.compute_distance_to_wall(sensor_pose, detected_wall);

        // 5. Check if expected distance is valid
        if (expected_distance.to_inches() < 0.0)
        {
            return false;
        }

        // 6. Check if sensor reading is in valid range
        if (reading > blend_config_.max_sensor_reading)
        {
            return false;
        }

        // 7. Outlier rejection - choose method based on config
        if (blend_config_.outlier_mode == BlendingConfig::OutlierRejectionMode::MAHALANOBIS)
        {
            // Statistical outlier rejection using Mahalanobis distance
            auto *distance_model = dynamic_cast<DistanceSensorMeasurementModel *>(
                sensor_config.sensor);

            if (!distance_model)
            {
                return false; // Can't get uncertainty without sensor model
            }

            // Get sensor uncertainty (standard deviation in meters)
            double sigma_sensor = get_sensor_uncertainty(distance_model);

            // Get pose uncertainty (Phase 1: returns 0.0)
            double sigma_pose = get_pose_uncertainty(robot_pose, detected_wall);

            // Combine uncertainties (independent error sources)
            double sigma_total = std::sqrt(
                sigma_sensor * sigma_sensor +
                sigma_pose * sigma_pose);

            if (sigma_total <= 0.0)
            {
                return false; // Invalid uncertainty
            }

            // Compute Mahalanobis distance
            mahalanobis_distance = math::mahalanobis_distance_scalar(
                reading.to_meters(),
                expected_distance.to_meters(),
                sigma_total);

            // Test against threshold (default 3.0 sigma = 99.7% confidence)
            return math::is_inlier_scalar(
                mahalanobis_distance,
                blend_config_.mahalanobis_sigma_threshold);
        }
        else
        {
            // Simple threshold-based rejection (original method)
            units::Length error = Qabs(reading - expected_distance);

            // Still compute Mahalanobis for telemetry if possible
            auto *distance_model = dynamic_cast<DistanceSensorMeasurementModel *>(
                sensor_config.sensor);
            if (distance_model)
            {
                double sigma = get_sensor_uncertainty(distance_model);
                if (sigma > 0.0)
                {
                    mahalanobis_distance = std::abs(error.to_meters()) / sigma;
                }
            }

            return error <= blend_config_.max_expected_error;
        }
    }

    void BlendedGeometricEstimator::apply_sensor_correction(
        const DistanceSensorConfig &sensor_config,
        units::Length measured_distance,
        units::Length expected_distance,
        field::FieldMap::Wall wall)
    {
        // Compute error (innovation)
        // If positive: robot is farther from wall than estimated
        // If negative: robot is closer to wall than estimated
        units::Length innovation = measured_distance - expected_distance;

        // Apply blend factor for complementary filtering
        // blend_factor = 0.2 means: trust sensor 20%, trust odometry 80%
        units::Length correction_magnitude = innovation * sensor_config.blend_factor;

        // Determine correction direction based on wall
        // Correction is perpendicular to the wall
        double dx_inches = 0.0;
        double dy_inches = 0.0;

        switch (wall)
        {
        case field::FieldMap::Wall::NORTH:
            // North wall: if measured > expected, robot is farther south
            // Need to move pose south (decrease Y)
            dy_inches = -correction_magnitude.to_inches();
            break;

        case field::FieldMap::Wall::SOUTH:
            // South wall: if measured > expected, robot is farther north
            // Need to move pose north (increase Y)
            dy_inches = correction_magnitude.to_inches();
            break;

        case field::FieldMap::Wall::EAST:
            // East wall: if measured > expected, robot is farther west
            // Need to move pose west (decrease X)
            dx_inches = -correction_magnitude.to_inches();
            break;

        case field::FieldMap::Wall::WEST:
            // West wall: if measured > expected, robot is farther east
            // Need to move pose east (increase X)
            dx_inches = correction_magnitude.to_inches();
            break;

        case field::FieldMap::Wall::NONE:
        default:
            return; // No valid wall, skip correction
        }

        // Apply correction to current pose (thread-safe)
        {
            std::lock_guard<pros::Mutex> lock(pose_mutex_);

            double new_x = current_pose_.x_inches() + dx_inches;
            double new_y = current_pose_.y_inches() + dy_inches;
            double theta = current_pose_.theta_rad(); // Heading unchanged

            current_pose_.se2 = math::SE2(new_x, new_y, theta);
        }
    }

    void BlendedGeometricEstimator::set_blend_config(const BlendingConfig &config)
    {
        blend_config_ = config;
    }

    void BlendedGeometricEstimator::set_blending_enabled(bool enabled)
    {
        blend_config_.enable_blending = enabled;
    }

} // namespace abclib::estimation