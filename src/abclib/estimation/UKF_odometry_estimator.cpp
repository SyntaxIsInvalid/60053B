#if 0
#include "abclib/estimation/UKF_odometry_estimator.hpp"
#include "abclib/math/angles.hpp"
#include "abclib/telemetry/telemetry.hpp"
#include <cmath>
#include <mutex>
namespace abclib::estimation
{
    UKFEstimator::UKFEstimator(
        IMeasurementModel<units::Distance> *vertical_model,
        IMeasurementModel<units::Distance> *horizontal_model,
        IMeasurementModel<units::Radians> *imu_model,
        units::Distance vertical_offset,
        units::Distance horizontal_offset,
        double process_noise_scale)
        : vertical_model_(vertical_model),
          horizontal_model_(horizontal_model),
          imu_model_(imu_model),
          vertical_offset_(vertical_offset),
          horizontal_offset_(horizontal_offset),
          last_update_time_ms_(0)
    {
        // Initialize UKF with default unscented transform parameters
        // alpha=0.001 (tight sigma points), beta=2.0 (Gaussian), kappa=0.0
        ukf_ = filters::UnscentedKalmanFilter<STATE_DIM, MEAS_DIM>(0.001, 2.0, 0.0);

        // Initialize state: [x, y, theta, v, omega] = [0, 0, 0, 0, 0]
        Eigen::Matrix<double, STATE_DIM, 1> x_init;
        x_init.setZero();

        // Initialize covariance with small uncertainty (robot starts at known pose)
        Eigen::Matrix<double, STATE_DIM, STATE_DIM> P_init;
        P_init.setIdentity();
        P_init(0, 0) = 0.01;  // 0.1 inch std in x
        P_init(1, 1) = 0.01;  // 0.1 inch std in y
        P_init(2, 2) = 0.001; // ~1.8 degree std in theta
        P_init(3, 3) = 0.01;  // 0.1 in/s std in v
        P_init(4, 4) = 0.001; // ~1.8 deg/s std in omega

        ukf_.set_state(x_init, P_init);

        // Initialize process noise
        initialize_process_noise(process_noise_scale);

        // Initialize current pose
        current_pose_ = Pose();

        // Initialize time
        last_update_time_ms_ = pros::millis();
    }

    UKFEstimator::~UKFEstimator()
    {
        stop();
    }

    void UKFEstimator::init()
    {
        std::lock_guard<pros::Mutex> lock(task_mutex_);
        if (!tracking_task_.has_value())
        {
            tracking_task_ = pros::Task([this]
                                        {
                while (pros::Task::notify_take(true, 0) == 0) {
                    this->update();
                    pros::delay(10);  // 100Hz update rate
                } });
        }
    }

    void UKFEstimator::stop()
    {
        std::lock_guard<pros::Mutex> lock(task_mutex_);
        if (tracking_task_.has_value())
        {
            tracking_task_->notify();
            tracking_task_ = std::nullopt;
        }
    }

    void UKFEstimator::reset()
    {
        // Reset measurement models
        if (vertical_model_)
            vertical_model_->reset();
        if (horizontal_model_)
            horizontal_model_->reset();
        if (imu_model_)
            imu_model_->reset();

        // Reset UKF state to zero
        Eigen::Matrix<double, STATE_DIM, 1> x_reset;
        x_reset.setZero();

        Eigen::Matrix<double, STATE_DIM, STATE_DIM> P_reset;
        P_reset.setIdentity();
        P_reset(0, 0) = 0.01;
        P_reset(1, 1) = 0.01;
        P_reset(2, 2) = 0.001;
        P_reset(3, 3) = 0.01;
        P_reset(4, 4) = 0.001;

        ukf_.set_state(x_reset, P_reset);

        // Reset pose
        std::lock_guard<pros::Mutex> lock(pose_mutex_);
        current_pose_ = Pose();

        // Reset time
        last_update_time_ms_ = pros::millis();
    }

    void UKFEstimator::calibrate()
    {
        // Same as reset but keep pose
        if (vertical_model_)
            vertical_model_->reset();
        if (horizontal_model_)
            horizontal_model_->reset();
        if (imu_model_)
            imu_model_->reset();

        last_update_time_ms_ = pros::millis();
    }

    Pose UKFEstimator::get_pose() const
    {
        std::lock_guard<pros::Mutex> lock(pose_mutex_);
        return current_pose_;
    }

    void UKFEstimator::set_pose(const Pose &pose)
    {
        std::lock_guard<pros::Mutex> lock(pose_mutex_);

        // Update internal pose
        current_pose_ = pose;

        // Update UKF state
        Eigen::Matrix<double, STATE_DIM, 1> x;
        x << pose.x(),
            pose.y(),
            pose.theta(),
            pose.v.inches_per_sec,
            pose.omega.rad_per_sec;

        // Keep existing covariance
        ukf_.set_state(x, ukf_.get_covariance());
    }

    void UKFEstimator::update()
    {
        // Calculate dt
        uint32_t current_time = pros::millis();
        double dt = (current_time - last_update_time_ms_) / 1000.0;
        last_update_time_ms_ = current_time;

        // Clamp dt to reasonable range (prevent instability on first call or long delays)
        if (dt < 0.001 || dt > 0.1)
        {
            dt = 0.01; // Default to 10ms
        }

        // ==================== PREDICTION STEP ====================

        // Define process model: f(x, dt) -> x_next
        // NOTE: Must capture dt AND include double parameter to match ProcessModel signature
        auto process_model = [](const Eigen::Matrix<double, STATE_DIM, 1> &x, double dt) -> Eigen::Matrix<double, STATE_DIM, 1>
        {
            double x_pos = x(0);
            double y_pos = x(1);
            double theta = x(2);
            double v = x(3);
            double omega = x(4);

            Eigen::Matrix<double, STATE_DIM, 1> x_next;
            x_next(0) = x_pos + v * std::cos(theta) * dt;
            x_next(1) = y_pos + v * std::sin(theta) * dt;
            x_next(2) = theta + omega * dt;
            x_next(3) = v;     // Constant velocity model
            x_next(4) = omega; // Constant angular velocity model

            return x_next;
        };

        // Build process noise Q
        Eigen::Matrix<double, STATE_DIM, STATE_DIM> Q;
        Q.setIdentity();
        Q(0, 0) = 0.01 * dt * dt; // Position noise: ~0.1 in std per sqrt(second)
        Q(1, 1) = 0.01 * dt * dt;
        Q(2, 2) = 0.0001 * dt * dt; // Heading noise: ~0.01 rad std per sqrt(second)
        Q(3, 3) = 0.1 * dt;         // Velocity noise: ~0.3 in/s std per sqrt(second)
        Q(4, 4) = 0.01 * dt;        // Angular velocity noise: ~0.1 rad/s std per sqrt(second)

        // Perform prediction
        ukf_.predict(process_model, Q, dt);

        // ==================== UPDATE STEP ====================

        // Get raw sensor measurements
        units::Distance delta_vertical = vertical_model_ ? vertical_model_->get_measurement() : units::Distance::from_inches(0.0);
        units::Distance delta_horizontal = horizontal_model_ ? horizontal_model_->get_measurement() : units::Distance::from_inches(0.0);
        units::Radians delta_theta = imu_model_ ? imu_model_->get_measurement() : units::Radians(0.0);

        // Convert raw sensor deltas to meaningful body-frame motion using arc-length kinematics
        LocalMotion local_motion = ArcLengthDifferentialDrive::compute_local_motion(
            delta_vertical,
            delta_horizontal,
            delta_theta,
            vertical_offset_,
            horizontal_offset_);

        // Build measurement vector z = [forward_motion, lateral_motion, heading_change]
        Eigen::Matrix<double, MEAS_DIM, 1> z;
        z << local_motion.y.inches,   // Forward distance in body frame
            local_motion.x.inches,    // Lateral distance in body frame
            local_motion.theta.value; // Heading change

        // Define measurement model: h(x) -> expected measurement
        auto measurement_model = [dt](const Eigen::Matrix<double, STATE_DIM, 1> &x) -> Eigen::Matrix<double, MEAS_DIM, 1>
        {
            double v = x(3);
            double omega = x(4);

            // For small rotations, body-frame motion is approximately:
            // - Forward: v * dt (along body y-axis)
            // - Lateral: 0 (no slip)
            // - Rotation: omega * dt

            Eigen::Matrix<double, MEAS_DIM, 1> z_expected;
            z_expected(0) = v * dt;     // Expected forward motion in body frame
            z_expected(1) = 0.0;        // Expected lateral motion (assume no slip)
            z_expected(2) = omega * dt; // Expected heading change

            return z_expected;
        };

        // Build measurement covariance R
        Eigen::Matrix<double, MEAS_DIM, MEAS_DIM> R = build_measurement_covariance();

        // Perform update
        ukf_.update(measurement_model, z, R);

        // ==================== EXTRACT STATE ====================

        Eigen::Matrix<double, STATE_DIM, 1> x_updated = ukf_.get_state();

        // Update current pose
        {
            std::lock_guard<pros::Mutex> lock(pose_mutex_);

            current_pose_.set_x(x_updated(0));
            current_pose_.set_y(x_updated(1));
            current_pose_.set_theta(math::normalize_angle(x_updated(2)));
            current_pose_.v = units::BodyLinearVelocity(x_updated(3));
            current_pose_.omega = units::BodyAngularVelocity(x_updated(4));
        }

        // Update telemetry
        {
            std::lock_guard<pros::Mutex> telem_lock(abclib::telemetry_mutex);
            abclib::telemetry.pose = current_pose_.pose;
            abclib::telemetry.pose_v = current_pose_.v;
            abclib::telemetry.pose_omega = current_pose_.omega;
        }
    }

    void UKFEstimator::initialize_process_noise(double scale)
    {
        // Process noise Q represents uncertainty in the process model
        // For our constant velocity model, we expect:
        // - Small position drift (from unmodeled accelerations)
        // - Small heading drift (from gyro bias)
        // - Moderate velocity changes (accelerations do occur)
        // - Moderate angular velocity changes

        // These are per-second values, scaled by dt in update()
        // Tuned for 100Hz (dt=0.01s) operation

        // Note: Actual Q is computed in update() and scaled by dt
        // This function exists for architectural consistency
        // but Q construction is inline in update() for clarity

        (void)scale; // Suppress unused warning, scale applied in update()
    }

    Eigen::Matrix<double, 3, 3> UKFEstimator::build_measurement_covariance()
    {
        Eigen::Matrix<double, MEAS_DIM, MEAS_DIM> R;
        R.setZero();

        // Get uncertainties from measurement models
        double vertical_variance = vertical_model_ ? vertical_model_->get_uncertainty() : 1e-6;
        double horizontal_variance = horizontal_model_ ? horizontal_model_->get_uncertainty() : 1e-6;
        double imu_variance = imu_model_ ? imu_model_->get_uncertainty() : 1e-6;

        // Diagonal measurement covariance (assuming independent sensor noise)
        R(0, 0) = vertical_variance;
        R(1, 1) = horizontal_variance;
        R(2, 2) = imu_variance;

        // Add small regularization to prevent numerical issues
        R(0, 0) += 1e-8;
        R(1, 1) += 1e-8;
        R(2, 2) += 1e-8;

        return R;
    }

} // namespace abclib::estimation
#endif