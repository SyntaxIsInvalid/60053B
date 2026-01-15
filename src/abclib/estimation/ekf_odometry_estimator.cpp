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
        const double dt = 0.01; // 10ms update rate

        // Get delta measurements (same as geometric)
        units::Length delta_vertical = vertical_model_ ? vertical_model_->get_measurement() : units::Length::from_inches(0.0);
        units::Length delta_horizontal = horizontal_model_ ? horizontal_model_->get_measurement() : units::Length::from_inches(0.0);
        units::Angle delta_imu = imu_model_ ? imu_model_->get_measurement() : units::Angle::from_radians(0.0);

        // Compute local motion using the same arc-length model
        LocalMotion local_motion = ArcLengthDifferentialDrive::compute_local_motion(
            delta_vertical,
            delta_horizontal,
            delta_imu,
            vertical_offset_,
            horizontal_offset_);

        // Define the prediction function: f(x, dt)
        auto prediction_function = [&](const Eigen::Vector3d &state, double dt_unused) -> Eigen::Vector3d
        {
            (void)dt_unused; // dt is already baked into the deltas

            double x = state(0);
            double y = state(1);
            double theta = state(2);

            // Average heading for integration
            double avg_heading = theta + delta_imu.value() / 2.0;

            // New state
            Eigen::Vector3d new_state;
            new_state(0) = x + local_motion.y.value() * std::cos(avg_heading) +
                           local_motion.x.value() * std::sin(avg_heading);
            new_state(1) = y + local_motion.y.value() * std::sin(avg_heading) -
                           local_motion.x.value() * std::cos(avg_heading);
            new_state(2) = theta + delta_imu.value();

            return new_state;
        };

        // Define the Jacobian of the prediction function: F = ∂f/∂x
        auto prediction_jacobian = [&](const Eigen::Vector3d &state, double dt_unused) -> Eigen::Matrix3d
        {
            (void)dt_unused;

            double theta = state(2);
            double avg_heading = theta + delta_imu.value() / 2.0;

            double dx_local = local_motion.y.value();
            double dy_local = local_motion.x.value();

            // Jacobian matrix
            Eigen::Matrix3d F;
            F << 1.0, 0.0, -dx_local * std::sin(avg_heading) + dy_local * std::cos(avg_heading),
                0.0, 1.0, dx_local * std::cos(avg_heading) + dy_local * std::sin(avg_heading),
                0.0, 0.0, 1.0;

            return F;
        };

        // EKF prediction step
        ekf_.predict_only(prediction_function, prediction_jacobian, dt);

        if (mode_ == FilterMode::FULL && front_distance_model_)
        {
            // Get actual distance measurement from sensor
            units::Length measured_distance = front_distance_model_->get_measurement();

            // Only update if measurement is valid (sensor returns 0 for invalid)
            if (measured_distance.to_meters() > 0.02) // > 20mm (sensor min range)
            {
                // Get current state estimate
                Eigen::Vector3d state = ekf_.get_state();

                // Create measurement vector (1x1 for single distance sensor)
                Eigen::Matrix<double, 1, 1> z;
                z << measured_distance.to_meters();

                // Define measurement function h(x): state -> expected distance
                auto measurement_function = [&](const Eigen::Vector3d &state) -> Eigen::Matrix<double, 1, 1>
                {
                    double x = state(0);     // meters
                    double y = state(1);     // meters
                    double theta = state(2); // radians

                    // Convert to inches for FieldMap (it works in inches)
                    double x_inches = x * 39.3701;
                    double y_inches = y * 39.3701;

                    // Compute expected distance using field map
                    double expected_distance_inches = field_map_.compute_expected_distance(
                        x_inches, y_inches, theta,
                        SENSOR_FORWARD_OFFSET,
                        SENSOR_LATERAL_OFFSET,
                        SENSOR_BEARING);

                    Eigen::Matrix<double, 1, 1> h_x;
                    h_x << expected_distance_inches * 0.0254; // Convert back to meters
                    return h_x;
                };

                // Define measurement Jacobian H = ∂h/∂x
                // For now, we'll use NUMERICAL differentiation (we'll optimize later)
                auto measurement_jacobian = [&](const Eigen::Vector3d &state) -> Eigen::Matrix<double, 1, 3>
                {
                    const double epsilon = 1e-6; // Small perturbation
                    Eigen::Matrix<double, 1, 3> H;

                    // Numerical derivative: (h(x+ε) - h(x)) / ε
                    Eigen::Matrix<double, 1, 1> h_nominal = measurement_function(state);

                    for (int i = 0; i < 3; i++)
                    {
                        Eigen::Vector3d state_perturbed = state;
                        state_perturbed(i) += epsilon;
                        Eigen::Matrix<double, 1, 1> h_perturbed = measurement_function(state_perturbed);
                        H(0, i) = (h_perturbed(0) - h_nominal(0)) / epsilon;
                    }

                    return H;
                };

                // Perform EKF update step
                ekf_.update(z, measurement_function, measurement_jacobian);
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
                telem.pose = current_pose_;
                telem.pose_v_raw = units::Velocity::from_ips(v_raw);
                telem.pose_omega_raw = units::AngularVelocity::from_rad_per_sec(omega_raw);

                // EKF-specific state
                telem.ekf_x = units::Length::from_meters(state(0));
                telem.ekf_y = units::Length::from_meters(state(1));
                telem.ekf_theta = units::Angle::from_radians(state(2));

                // EKF uncertainty (standard deviations)
                telem.ekf_x_std = ekf_.get_state_uncertainty(0);     // in meters
                telem.ekf_y_std = ekf_.get_state_uncertainty(1);     // in meters
                telem.ekf_theta_std = ekf_.get_state_uncertainty(2); // in radians

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