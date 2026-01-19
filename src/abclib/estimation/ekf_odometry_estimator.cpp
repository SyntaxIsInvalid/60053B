#include "abclib/estimation/ekf_odometry_estimator.hpp"
#include <cmath>
#include <mutex>
#include "abclib/math/angles.hpp"
#include "abclib/telemetry/telemetry.hpp"
#include "abclib/field/field_map.hpp"

namespace abclib::estimation
{
    EKFOdometryEstimator::EKFOdometryEstimator(
        IMeasurementModel<units::Length> *vertical_model,
        IMeasurementModel<units::Length> *horizontal_model,
        IMeasurementModel<units::Angle> *imu_model,
        IMeasurementModel<units::Length> *front_distance_model, // ADD THIS PARAMETER
        units::Length vertical_offset,
        units::Length horizontal_offset,
        const field::FieldConfig &field_config,
        FilterMode mode)
        : vertical_model_(vertical_model),
          horizontal_model_(horizontal_model),
          imu_model_(imu_model),
          front_distance_model_(front_distance_model), // ADD THIS INITIALIZATION
          vertical_offset_(vertical_offset),
          horizontal_offset_(horizontal_offset),
          field_map_(field_config),
          mode_(mode),
          prev_vertical_total_(units::Length::from_inches(0)),
          prev_horizontal_total_(units::Length::from_inches(0)),
          prev_imu_total_(units::Angle::from_radians(0)),
          first_update_(true)
    {
        initialize_ekf();
    }

    void EKFOdometryEstimator::initialize_ekf()
    {
        // Initial state: [x, y, theta] = [0, 0, 0]
        Eigen::Vector3d initial_state;
        initial_state << 0.0, 0.0, 0.0;

        // Initial covariance (high uncertainty)
        Eigen::Matrix3d initial_covariance = Eigen::Matrix3d::Identity() * 0.1;

        // Process noise (tune these based on your system)
        Eigen::Matrix3d process_noise;
        process_noise << 0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01;

        // Measurement noise (2x2 for two distance sensors)
        // UPDATED: Convert distance_sensor_noise_ from inches to meters
        Eigen::Matrix<double, 1, 1> measurement_noise;
        measurement_noise << 0.05 * 0.05; // Variance in meters^2 (std = 0.05m)

        ekf_ = filters::ExtendedKalmanFilter<3, 1>(
            initial_state,
            initial_covariance,
            process_noise,
            measurement_noise);
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
        return current_pose_;
    }

    void EKFOdometryEstimator::update()
    {
        const double dt = 0.01;

        // Get delta measurements
        units::Length delta_vertical = vertical_model_ ? vertical_model_->get_measurement() : units::Length::from_inches(0.0);
        units::Length delta_horizontal = horizontal_model_ ? horizontal_model_->get_measurement() : units::Length::from_inches(0.0);
        units::Angle delta_imu = imu_model_ ? imu_model_->get_measurement() : units::Angle::from_radians(0.0);

        // Compute local SE2 transformation (this is already correct!)
        math::SE2 local_transform = ArcLengthDifferentialDrive::compute_local_transformation(
            delta_vertical,
            delta_horizontal,
            delta_imu,
            vertical_offset_,
            horizontal_offset_);

        // Define prediction using SE2 composition
        auto prediction_function = [&](const Eigen::Vector3d &state, double dt_unused) -> Eigen::Vector3d
        {
            (void)dt_unused;

            // Convert state vector to SE2
            math::SE2 current_pose(state(0), state(1), state(2));

            // Compose: new_pose = current_pose * local_transform
            math::SE2 new_pose = current_pose * local_transform;

            // Convert back to state vector
            Eigen::Vector3d new_state;
            new_state << new_pose.x(), new_pose.y(), new_pose.theta();

            return new_state;
        };

        // Define Jacobian using SE2's Adjoint
        auto prediction_jacobian = [&](const Eigen::Vector3d &state, double dt_unused) -> Eigen::Matrix3d
        {
            (void)dt_unused;

            // The Jacobian for SE2 composition is the Adjoint matrix
            math::SE2 current_pose(state(0), state(1), state(2));
            return current_pose.Adjoint();
        };

        // EKF prediction step
        ekf_.predict_only(prediction_function, prediction_jacobian, dt);
        // Measurement update (only if in FULL mode)
        if (mode_ == FilterMode::FULL && front_distance_model_)
        {

            // Cast to concrete type to access is_valid()
            auto *distance_sensor = dynamic_cast<DistanceSensorMeasurementModel *>(front_distance_model_);

            if (distance_sensor)
            { // Check cast succeeded

                // Measurement function: h(x) = expected sensor reading
                auto measurement_function = [&](const Eigen::Vector3d &state) -> Eigen::Matrix<double, 1, 1>
                {
                    double x_m = state(0);
                    double y_m = state(1);
                    double theta_rad = state(2);

                    double x_in = units::Length::from_meters(x_m).to_inches();
                    double y_in = units::Length::from_meters(y_m).to_inches();
                    #if 0
                    double expected_distance_in = field_map_.compute_expected_distance(
                        x_in, y_in, theta_rad,
                        SENSOR_FORWARD_OFFSET, SENSOR_LATERAL_OFFSET, SENSOR_BEARING);

                    Eigen::Matrix<double, 1, 1> z_predicted;
                    z_predicted(0) = units::Length::from_inches(expected_distance_in).to_meters();
                    // return z_predicted;
                    #endif
                    Eigen::Matrix<double, 1, 1> z_predicted;
                    z_predicted << 0;
                    return z_predicted;
                };

                // Numerical Jacobian: H = ∂h/∂x
                auto measurement_jacobian = [&](const Eigen::Vector3d &state) -> Eigen::Matrix<double, 1, 3>
                {
                    const double epsilon = 1e-6;
                    Eigen::Matrix<double, 1, 3> H;
                    double h_nominal = measurement_function(state)(0);

                    for (int i = 0; i < 3; i++)
                    {
                        Eigen::Vector3d state_perturbed = state;
                        state_perturbed(i) += epsilon;
                        double h_perturbed = measurement_function(state_perturbed)(0);
                        H(0, i) = (h_perturbed - h_nominal) / epsilon;
                    }
                    return H;
                };

                // Get actual measurement from sensor
                units::Length measured_distance = distance_sensor->get_measurement();

                // Only correct if sensor reading is valid
                if (distance_sensor->is_valid())
                {
                    Eigen::Matrix<double, 1, 1> z_measured;
                    z_measured(0) = measured_distance.to_meters();

                    // Perform EKF correction step
                    ekf_.update(z_measured, measurement_function, measurement_jacobian);
                }
            }
        }
        // ============================================================
        // GET FINAL STATE AND UPDATE POSE
        // ============================================================
        Eigen::Vector3d state = ekf_.get_state();

        static double prev_x = 0.0;
        static double prev_y = 0.0;
        static double prev_theta = 0.0;

        std::lock_guard<pros::Mutex> lock(pose_mutex_);

        // Update current pose from EKF state
        current_pose_.set_x(units::Length::from_meters(state(0)).to_inches());
        current_pose_.set_y(units::Length::from_meters(state(1)).to_inches());
        current_pose_.set_theta(state(2));

        // Velocity estimation (same as geometric)
        if (!first_update_)
        {
            double dx = current_pose_.x_inches() - prev_x;
            double dy = current_pose_.y_inches() - prev_y;
            double dtheta = current_pose_.theta_rad() - prev_theta;

            dtheta = math::normalize_angle(dtheta);

            double v_raw = (dx * std::cos(current_pose_.theta_rad()) +
                            dy * std::sin(current_pose_.theta_rad())) /
                           dt;
            double omega_raw = dtheta / dt;

            const double alpha = 0.3;
            current_pose_.v = units::Velocity::from_ips(
                alpha * v_raw + (1.0 - alpha) * current_pose_.v.to_ips());
            current_pose_.omega = units::AngularVelocity::from_rad_per_sec(
                alpha * omega_raw + (1.0 - alpha) * current_pose_.omega.to_rad_per_sec());

            // ============================================================
            // LOG EKF STATE AND UNCERTAINTY
            // ============================================================
            {
                auto &telem = abclib::telemetry::g_telemetry.get_write_buffer();

                // Basic pose (this was already here)
                telem.pose_corner = current_pose_;
                telem.pose_v_raw = units::Velocity::from_ips(v_raw);
                telem.pose_omega_raw = units::AngularVelocity::from_rad_per_sec(omega_raw);
                #if 0
                // ADD THIS: Wall information (same as geometric)
                telem.heading_wall = field_map_.get_nearest_wall(
                    current_pose_.x_inches(),
                    current_pose_.y_inches(),
                    current_pose_.theta_rad(),
                    0.0 // sensor_bearing = 0 for pure robot heading
                );

                // How far to that wall?
                double distance = field_map_.compute_distance_to_wall(
                    current_pose_.x_inches(),
                    current_pose_.y_inches(),
                    current_pose_.theta_rad(),
                    telem.heading_wall);

                // Check if valid (distance >= 0 means ray hits wall)
                telem.heading_wall_valid = (distance >= 0.0);

                if (telem.heading_wall_valid)
                {
                    telem.heading_distance_to_wall = units::Length::from_inches(distance);
                }
                else
                {
                    telem.heading_distance_to_wall = units::Length::from_inches(-1.0);
                }

                // EKF-specific state (this was already here)
                telem.ekf_x = units::Length::from_meters(state(0));
                telem.ekf_y = units::Length::from_meters(state(1));
                telem.ekf_theta = units::Angle::from_radians(state(2));

                // EKF uncertainty (standard deviations) (this was already here)
                telem.ekf_x_std = ekf_.get_state_uncertainty(0);     // in meters
                telem.ekf_y_std = ekf_.get_state_uncertainty(1);     // in meters
                telem.ekf_theta_std = ekf_.get_state_uncertainty(2); // in radians
                #endif
                abclib::telemetry::g_telemetry.swap();
            }
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
        new_state(2) = pose.theta_rad(); // Already in radians

        ekf_.set_state(new_state);

        // Reset the first_update flag since we're setting a new pose
        first_update_ = true;
    }
}