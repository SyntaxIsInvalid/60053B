#include "abclib/estimation/ekf_odometry_estimator.hpp"
#include <cmath>
#include <mutex>
#include "abclib/math/angles.hpp"
#include "abclib/telemetry/telemetry.hpp"
#include "abclib/field/field_map.hpp"
#include "abclib/field/coordinate_transform.hpp"

namespace abclib::estimation
{
    EKFOdometryEstimator::EKFOdometryEstimator(
        IMeasurementModel<units::Length> *vertical_model,
        IMeasurementModel<units::Length> *horizontal_model,
        IMeasurementModel<units::Angle> *imu_model,
        units::Length vertical_offset,
        units::Length horizontal_offset,
        const field::FieldConfig &field_config,
        const std::vector<DistanceSensorConfig> &distance_sensors,
        FilterMode mode,
        double measurement_noise)
        : vertical_model_(vertical_model),
          horizontal_model_(horizontal_model),
          imu_model_(imu_model),
          distance_sensors_(distance_sensors), // Store sensor array
          vertical_offset_(vertical_offset),
          horizontal_offset_(horizontal_offset),
          field_map_(field_config),
          mode_(mode),
          measurement_noise_(measurement_noise),
          prev_vertical_total_(units::Length::from_inches(0)),
          prev_horizontal_total_(units::Length::from_inches(0)),
          prev_imu_total_(units::Angle::from_radians(0)),
          first_update_(true)
    {
        initialize_ekf();
    }

    void EKFOdometryEstimator::initialize_ekf()
    {
        Eigen::Vector3d initial_state = Eigen::Vector3d::Zero();
        Eigen::Matrix3d initial_covariance = Eigen::Matrix3d::Identity() * 0.1;
        Eigen::Matrix3d process_noise;
        process_noise << 0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01;

        ekf_ = filters::ExtendedKalmanFilter<3>(
            initial_state,
            initial_covariance,
            process_noise);
    }

    void EKFOdometryEstimator::init()
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

    void EKFOdometryEstimator::stop()
    {
        std::lock_guard<pros::Mutex> lock(task_mutex_);
        if (tracking_task_.has_value())
        {
            tracking_task_->notify();
            tracking_task_ = std::nullopt;
        }
    }

    EKFOdometryEstimator::~EKFOdometryEstimator()
    {
        std::lock_guard<pros::Mutex> lock(task_mutex_);
        if (tracking_task_.has_value())
        {
            tracking_task_->notify();
        }
    }

    void EKFOdometryEstimator::reset()
    {
        current_pose_ = Pose();
        ekf_.reset();
        first_update_ = true;

        if (vertical_model_)
            vertical_model_->reset();
        if (horizontal_model_)
            horizontal_model_->reset();
        if (imu_model_)
            imu_model_->reset();

        prev_vertical_total_ = units::Length::from_inches(0);
        prev_horizontal_total_ = units::Length::from_inches(0);
        prev_imu_total_ = units::Angle::from_radians(0);
    }

    Pose EKFOdometryEstimator::get_pose() const
    {
        std::lock_guard<pros::Mutex> lock(pose_mutex_);

        // Create result with current pose
        Pose result = current_pose_;

        // Populate covariance from EKF
        result.set_covariance(ekf_.get_covariance());

        return result;
    }

    double EKFOdometryEstimator::predict_sensor_distance(
        const Eigen::Vector3d &state,
        const DistanceSensorConfig &sensor_config) const
    {
        // Convert state from meters to inches for field_map (which works in inches)
        double x_inches = units::Length::from_meters(state(0)).to_inches();
        double y_inches = units::Length::from_meters(state(1)).to_inches();
        double theta_rad = state(2);

        // Create temporary pose for sensor pose computation
        Pose robot_pose(x_inches, y_inches, theta_rad);

        // Compute expected distance using field_map
        units::Length expected_distance = field_map_.compute_expected_distance(
            robot_pose,
            sensor_config.offset_forward,
            sensor_config.offset_lateral,
            sensor_config.bearing);

        // Return in meters (EKF internal units)
        return expected_distance.to_meters();
    }

    Eigen::RowVector3d EKFOdometryEstimator::compute_sensor_jacobian_numerical(
        const Eigen::Vector3d &state,
        const DistanceSensorConfig &sensor_config) const
    {
        const double epsilon = 1e-6;

        // Compute nominal measurement
        double h_nominal = predict_sensor_distance(state, sensor_config);

        Eigen::RowVector3d jacobian;

        // Finite difference for each state variable
        for (int i = 0; i < 3; i++)
        {
            Eigen::Vector3d state_perturbed = state;
            state_perturbed(i) += epsilon;

            double h_perturbed = predict_sensor_distance(state_perturbed, sensor_config);

            jacobian(i) = (h_perturbed - h_nominal) / epsilon;
        }

        return jacobian;
    }

    int EKFOdometryEstimator::build_measurement_batch(
        const Eigen::Vector3d &state,
        Eigen::VectorXd &z_measured,
        Eigen::MatrixXd &H_jacobian) const
    {
        // First pass: count active sensors
        std::vector<size_t> active_indices;

        for (size_t i = 0; i < distance_sensors_.size(); i++)
        {
            const auto &sensor_config = distance_sensors_[i];

            // Skip if disabled or no sensor
            if (!sensor_config.enabled || !sensor_config.sensor)
                continue;

            // Check if sensor reading is valid
            auto *distance_model = dynamic_cast<DistanceSensorMeasurementModel *>(
                sensor_config.sensor);

            if (!distance_model)
                continue;

            // NEW: Check validity BEFORE consuming measurement
            if (!distance_model->is_valid())
                continue;

            // This sensor is active
            active_indices.push_back(i);
        }

        int N = active_indices.size();

        if (N == 0)
        {
            // No active sensors - return empty
            z_measured.resize(0);
            H_jacobian.resize(0, 3);
            return 0;
        }

        // Resize output matrices
        z_measured.resize(N);
        H_jacobian.resize(N, 3);

        // Second pass: fill measurement vector and Jacobian
        for (int k = 0; k < N; k++)
        {
            size_t sensor_idx = active_indices[k];
            const auto &sensor_config = distance_sensors_[sensor_idx];

            // NOW get the actual measurement (marks timestamp as consumed)
            units::Length measured_distance = sensor_config.sensor->get_measurement();
            z_measured(k) = measured_distance.to_meters();

            // Compute Jacobian row for this sensor
            H_jacobian.row(k) = compute_sensor_jacobian_numerical(state, sensor_config);
        }

        return N;
    }

    void EKFOdometryEstimator::update()
    {
        const double dt = 0.01;

        // Get delta measurements from odometry
        units::Length delta_vertical = vertical_model_ ? vertical_model_->get_measurement() : units::Length::from_inches(0.0);
        units::Length delta_horizontal = horizontal_model_ ? horizontal_model_->get_measurement() : units::Length::from_inches(0.0);
        units::Angle delta_imu = imu_model_ ? imu_model_->get_measurement() : units::Angle::from_radians(0.0);

        // Compute local SE2 transformation (body frame)
        math::SE2 local_transform = ArcLengthDifferentialDrive::compute_local_transformation(
            delta_vertical,
            delta_horizontal,
            delta_imu,
            vertical_offset_,
            horizontal_offset_);

        // Define prediction function using SE2 composition
        auto prediction_function = [&](const Eigen::Vector3d &state, double dt_unused) -> Eigen::Vector3d
        {
            (void)dt_unused;

            // Convert state vector to SE2 (state is in meters, SE2 uses inches)
            double x_inches = units::Length::from_meters(state(0)).to_inches();
            double y_inches = units::Length::from_meters(state(1)).to_inches();
            math::SE2 current_pose(x_inches, y_inches, state(2));

            // Compose: new_pose = current_pose * local_transform
            math::SE2 new_pose = current_pose * local_transform;

            // Convert back to state vector (meters)
            Eigen::Vector3d new_state;
            new_state(0) = units::Length::from_inches(new_pose.x()).to_meters();
            new_state(1) = units::Length::from_inches(new_pose.y()).to_meters();
            new_state(2) = new_pose.theta();

            return new_state;
        };

        auto prediction_jacobian = [&](const Eigen::Vector3d &state, double dt_unused) -> Eigen::Matrix3d
        {
            (void)dt_unused;

            double theta = state(2); // Current heading in radians

            // Get local transform components (in inches from SE2)
            double dx_local_in = local_transform.x();
            double dy_local_in = local_transform.y();

            // Compute Jacobian of composition: ∂(state ⊕ delta)/∂state
            // For SE(2) composition: new_pose = current_pose * local_transform
            //
            // new_x = x + cos(θ)·Δx - sin(θ)·Δy
            // new_y = y + sin(θ)·Δx + cos(θ)·Δy
            // new_θ = θ + Δθ
            //
            // Therefore:
            // ∂new_x/∂θ = -sin(θ)·Δx - cos(θ)·Δy
            // ∂new_y/∂θ =  cos(θ)·Δx - sin(θ)·Δy

            double sin_theta = std::sin(theta);
            double cos_theta = std::cos(theta);

            double dx_dtheta_in = -sin_theta * dx_local_in - cos_theta * dy_local_in;
            double dy_dtheta_in = cos_theta * dx_local_in - sin_theta * dy_local_in;

            // Convert inches to meters for EKF state units
            double dx_dtheta_m = units::Length::from_inches(dx_dtheta_in).to_meters();
            double dy_dtheta_m = units::Length::from_inches(dy_dtheta_in).to_meters();

            Eigen::Matrix3d F;
            F << 1.0, 0.0, dx_dtheta_m,
                0.0, 1.0, dy_dtheta_m,
                0.0, 0.0, 1.0;

            return F;
        };
        Eigen::Matrix3d Q_continuous = ekf_.get_process_noise();
        Eigen::Matrix3d Q_discrete = Q_continuous * dt;
        ekf_.set_process_noise(Q_discrete);
        // EKF prediction step
        ekf_.predict_only(prediction_function, prediction_jacobian, dt);
        ekf_.set_process_noise(Q_continuous);
        // Measurement update (only if in FULL mode and we have sensors)
        if (mode_ == FilterMode::FULL && !distance_sensors_.empty())
        {
            Eigen::Vector3d current_state = ekf_.get_state();

            // Build batch measurement
            Eigen::VectorXd z_measured;
            Eigen::MatrixXd H_jacobian;
            int N = build_measurement_batch(current_state, z_measured, H_jacobian);

            if (N > 0)
            {
                // Build measurement noise matrix R (diagonal)
                Eigen::MatrixXd R = Eigen::MatrixXd::Identity(N, N) * (measurement_noise_ * measurement_noise_);

                // Measurement function (returns predicted measurements)
                auto measurement_function = [&](const Eigen::Vector3d &state) -> Eigen::VectorXd
                {
                    Eigen::VectorXd z_predicted(N);

                    // Recompute predictions for all active sensors
                    // (Could optimize by caching from build_measurement_batch, but cleaner this way)
                    Eigen::VectorXd z_temp;
                    Eigen::MatrixXd H_temp;
                    build_measurement_batch(state, z_temp, H_temp);

                    // z_temp should equal z_predicted from the batch
                    // We need to compute h(state) for the measurement function
                    // Actually, we need to return the expected measurements given state
                    // This is a bit redundant with build_measurement_batch...

                    // Let me reconsider: h(x) should return expected measurements
                    // But build_measurement_batch gets actual measurements
                    // We need a separate function that just computes h(x) predictions

                    // For now, loop through active sensors again
                    int k = 0;
                    for (size_t i = 0; i < distance_sensors_.size() && k < N; i++)
                    {
                        const auto &sensor_config = distance_sensors_[i];

                        if (!sensor_config.enabled || !sensor_config.sensor)
                            continue;

                        auto *distance_model = dynamic_cast<DistanceSensorMeasurementModel *>(
                            sensor_config.sensor);

                        if (!distance_model || !distance_model->is_valid())
                            continue;

                        // Predict distance for this sensor
                        z_predicted(k) = predict_sensor_distance(state, sensor_config);
                        k++;
                    }

                    return z_predicted;
                };

                // Jacobian function (returns H matrix)
                auto measurement_jacobian = [&](const Eigen::Vector3d &state) -> Eigen::MatrixXd
                {
                    // Return the precomputed Jacobian
                    // Note: H_jacobian is captured from outer scope
                    return H_jacobian;
                };

                // Perform batch EKF update
                ekf_.update(z_measured, measurement_function, measurement_jacobian, R);
            }
        }

        // Get final state and update pose
        Eigen::Vector3d state = ekf_.get_state();

        static double prev_x = 0.0;
        static double prev_y = 0.0;
        static double prev_theta = 0.0;

        std::lock_guard<pros::Mutex> lock(pose_mutex_);

        // Update current pose from EKF state (convert meters back to inches)
        current_pose_.set_x(units::Length::from_meters(state(0)).to_inches());
        current_pose_.set_y(units::Length::from_meters(state(1)).to_inches());
        current_pose_.set_theta(state(2));
        current_pose_.set_covariance(ekf_.get_covariance());

        // Velocity estimation using finite difference
        if (!first_update_)
        {
            double dx = current_pose_.x_inches() - prev_x;
            double dy = current_pose_.y_inches() - prev_y;
            double dtheta = current_pose_.theta_rad() - prev_theta;

            dtheta = math::normalize_angle(dtheta);

            // Project velocity onto robot heading
            double v_raw = (dx * std::cos(current_pose_.theta_rad()) +
                            dy * std::sin(current_pose_.theta_rad())) /
                           dt;
            double omega_raw = dtheta / dt;

            // Exponential moving average filter
            const double alpha = 0.3;
            current_pose_.v = units::Velocity::from_ips(
                alpha * v_raw + (1.0 - alpha) * current_pose_.v.to_ips());
            current_pose_.omega = units::AngularVelocity::from_rad_per_sec(
                alpha * omega_raw + (1.0 - alpha) * current_pose_.omega.to_rad_per_sec());
            /*
            // Telemetry
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

                math::SE2 sensor_pose = field_map_.compute_sensor_global_pose(
                    current_pose_,
                    units::Length::from_inches(0.0), // Robot center
                    units::Length::from_inches(0.0),
                    units::Angle::from_radians(0.0));

                units::Length distance = field_map_.compute_distance_to_wall(
                    sensor_pose,
                    telem.heading_wall);

                telem.heading_wall_valid = (distance >= units::Length::from_inches(0.0));
                telem.heading_distance_to_wall = telem.heading_wall_valid ? distance : units::Length::from_inches(-1.0);

                telem.pose_v_raw = units::Velocity::from_ips(v_raw);
                telem.pose_omega_raw = units::AngularVelocity::from_rad_per_sec(omega_raw);

                // EKF-specific telemetry
                telem.ekf_x = units::Length::from_meters(state(0));
                telem.ekf_y = units::Length::from_meters(state(1));
                telem.ekf_theta = units::Angle::from_radians(state(2));

                // Uncertainty (standard deviations)
                telem.ekf_x_std = ekf_.get_state_uncertainty(0);     // meters
                telem.ekf_y_std = ekf_.get_state_uncertainty(1);     // meters
                telem.ekf_theta_std = ekf_.get_state_uncertainty(2); // radians

                // NEW: Covariance information from pose
                telem.has_covariance = current_pose_.has_uncertainty();
                if (telem.has_covariance)
                {
                    telem.position_uncertainty = current_pose_.position_uncertainty();
                    telem.heading_uncertainty = current_pose_.heading_uncertainty();
                    telem.x_uncertainty = units::Length::from_meters(current_pose_.x_uncertainty_meters());
                    telem.y_uncertainty = units::Length::from_meters(current_pose_.y_uncertainty_meters());
                }

                // NEW: Distance sensor diagnostics (front sensor)
                
                if (!distance_sensors_.empty() && distance_sensors_[0].enabled && distance_sensors_[0].sensor)
                {
                    auto *front_sensor = dynamic_cast<DistanceSensorMeasurementModel *>(
                        distance_sensors_[0].sensor);

                    if (front_sensor)
                    {
                        telem.front_distance_measured = front_sensor->get_measurement();
                        telem.front_distance_valid = front_sensor->is_valid();

                        telem.front_wall = field_map_.get_nearest_wall(
                            current_pose_,
                            distance_sensors_[0].bearing);

                        // Expected distance based on current EKF state
                        telem.front_distance_expected = field_map_.compute_expected_distance(
                            current_pose_,
                            distance_sensors_[0].offset_forward,
                            distance_sensors_[0].offset_lateral,
                            distance_sensors_[0].bearing);

                        // Innovation (measurement - prediction) - key diagnostic metric!
                        if (telem.front_distance_valid)
                        {
                            telem.front_innovation =
                                telem.front_distance_measured - telem.front_distance_expected;
                        }
                        else
                        {
                            telem.front_innovation = units::Length::from_inches(-999.0);
                        }
                    }
                    else
                    {
                        // Sensor not available
                        telem.front_distance_valid = false;
                        telem.front_distance_measured = units::Length::from_inches(-1.0);
                        telem.front_distance_expected = units::Length::from_inches(-1.0);
                        telem.front_innovation = units::Length::from_inches(-999.0);
                        telem.front_wall = field::FieldMap::Wall::NONE;
                    }
                }
                else
                {
                    // Sensor disabled or not configured
                    telem.front_distance_valid = false;
                    telem.front_distance_measured = units::Length::from_inches(-1.0);
                    telem.front_distance_expected = units::Length::from_inches(-1.0);
                    telem.front_innovation = units::Length::from_inches(-999.0);
                    telem.front_wall = field::FieldMap::Wall::NONE;
                }

                // NEW: Back sensor diagnostics (if you want to track it even when disabled)
                if (distance_sensors_.size() > 1 && distance_sensors_[1].enabled && distance_sensors_[1].sensor)
                {
                    auto *back_sensor = dynamic_cast<DistanceSensorMeasurementModel *>(
                        distance_sensors_[1].sensor);

                    if (back_sensor)
                    {
                        telem.back_distance_measured = back_sensor->get_measurement();
                        telem.back_distance_valid = back_sensor->is_valid();

                        telem.back_wall = field_map_.get_nearest_wall(
                            current_pose_,
                            distance_sensors_[1].bearing);

                        telem.back_distance_expected = field_map_.compute_expected_distance(
                            current_pose_,
                            distance_sensors_[1].offset_forward,
                            distance_sensors_[1].offset_lateral,
                            distance_sensors_[1].bearing);

                        if (telem.back_distance_valid)
                        {
                            telem.back_innovation =
                                telem.back_distance_measured - telem.back_distance_expected;
                        }
                        else
                        {
                            telem.back_innovation = units::Length::from_inches(-999.0);
                        }
                    }
                    else
                    {
                        telem.back_distance_valid = false;
                        telem.back_distance_measured = units::Length::from_inches(-1.0);
                        telem.back_distance_expected = units::Length::from_inches(-1.0);
                        telem.back_innovation = units::Length::from_inches(-999.0);
                        telem.back_wall = field::FieldMap::Wall::NONE;
                    }
                }
                else
                {
                    // Back sensor disabled or not configured
                    telem.back_distance_valid = false;
                    telem.back_distance_measured = units::Length::from_inches(-1.0);
                    telem.back_distance_expected = units::Length::from_inches(-1.0);
                    telem.back_innovation = units::Length::from_inches(-999.0);
                    telem.back_wall = field::FieldMap::Wall::NONE;
                }

                abclib::telemetry::g_telemetry.swap();
            }
            */
        }
        else
        {
            current_pose_.v = units::Velocity::from_ips(0.0);
            current_pose_.omega = units::AngularVelocity::from_rad_per_sec(0.0);
            first_update_ = false;
        }

        prev_x = current_pose_.x_inches();
        prev_y = current_pose_.y_inches();
        prev_theta = current_pose_.theta_rad();
    }

    void EKFOdometryEstimator::calibrate()
    {
        if (vertical_model_)
            vertical_model_->reset();
        if (horizontal_model_)
            horizontal_model_->reset();
        if (imu_model_)
            imu_model_->reset();

        current_pose_ = Pose();
        ekf_.reset();
        first_update_ = true;

        prev_vertical_total_ = units::Length::from_inches(0);
        prev_horizontal_total_ = units::Length::from_inches(0);
        prev_imu_total_ = units::Angle::from_radians(0);
    }

    void EKFOdometryEstimator::set_pose(const Pose &pose)
    {
        std::lock_guard<pros::Mutex> lock(pose_mutex_);
        current_pose_ = pose;

        // Reset velocities
        current_pose_.v = units::Velocity::from_ips(0.0);
        current_pose_.omega = units::AngularVelocity::from_rad_per_sec(0.0);

        // Update EKF state to match the new pose
        Eigen::Vector3d new_state;
        new_state(0) = units::Length::from_inches(pose.x_inches()).to_meters();
        new_state(1) = units::Length::from_inches(pose.y_inches()).to_meters();
        new_state(2) = pose.theta_rad();

        ekf_.set_state(new_state);

        // Reset the first_update flag since we're setting a new pose
        first_update_ = true;
    }
}