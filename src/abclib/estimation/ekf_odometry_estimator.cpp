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
        units::Length vertical_offset,
        units::Length horizontal_offset,
        DistanceSensorMeasurementModel *front_distance_sensor,
        DistanceSensorMeasurementModel *back_distance_sensor,
        units::Length front_sensor_offset_forward,
        units::Length front_sensor_offset_lateral,
        double front_sensor_bearing,
        units::Length back_sensor_offset_forward,
        units::Length back_sensor_offset_lateral,
        double back_sensor_bearing,
        double distance_sensor_noise)
        : vertical_model_(vertical_model),
          horizontal_model_(horizontal_model),
          imu_model_(imu_model),
          vertical_offset_(vertical_offset),
          horizontal_offset_(horizontal_offset),
          front_distance_sensor_(front_distance_sensor),
          back_distance_sensor_(back_distance_sensor),
          front_sensor_offset_forward_(front_sensor_offset_forward),
          front_sensor_offset_lateral_(front_sensor_offset_lateral),
          front_sensor_bearing_(front_sensor_bearing),
          back_sensor_offset_forward_(back_sensor_offset_forward),
          back_sensor_offset_lateral_(back_sensor_offset_lateral),
          back_sensor_bearing_(back_sensor_bearing),
          distance_sensor_noise_(distance_sensor_noise),
          prev_vertical_total_(units::Length::from_inches(0)),
          prev_horizontal_total_(units::Length::from_inches(0)),
          prev_imu_total_(units::Angle::from_radians(0)),
          first_update_(true),
          update_count_(0),                                       // ADD THIS
          prev_front_measurement_(units::Length::from_meters(0)), // ADD THIS
          prev_back_measurement_(units::Length::from_meters(0))   // ADD THIS
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
        Eigen::Matrix2d measurement_noise;
        double noise_meters = distance_sensor_noise_ * 0.0254; // inches to meters
        double variance = noise_meters * noise_meters;
        measurement_noise << variance, 0.0,
            0.0, variance;

        ekf_ = filters::ExtendedKalmanFilter<3, 2>(
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
    update_count_ = 0;  // ADD THIS
    prev_front_measurement_ = units::Length::from_meters(0);  // ADD THIS
    prev_back_measurement_ = units::Length::from_meters(0);   // ADD THIS

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

    // ============================================================
    // LOGGING PREPARATION - Capture state BEFORE correction
    // ============================================================
    Eigen::Vector3d state_before_correction = ekf_.get_state();
    
    // Convert to inches for field map calculations
    double x_inches = state_before_correction(0) * 39.3701;
    double y_inches = state_before_correction(1) * 39.3701;
    double theta = state_before_correction(2);

    // ============================================================
    // DISTANCE SENSOR MEASUREMENTS
    // ============================================================
    bool use_front_sensor = false;
    bool use_back_sensor = false;
    units::Length front_measurement = units::Length::from_meters(0.0);
    units::Length back_measurement = units::Length::from_meters(0.0);

    if (front_distance_sensor_) {
        front_measurement = front_distance_sensor_->get_measurement();
        use_front_sensor = front_distance_sensor_->is_valid();
    }

    if (back_distance_sensor_) {
        back_measurement = back_distance_sensor_->get_measurement();
        use_back_sensor = back_distance_sensor_->is_valid();
    }

    // ============================================================
    // COMPUTE EXPECTED MEASUREMENTS FOR LOGGING
    // ============================================================
    double front_expected = field::FieldMap::compute_expected_distance(
        x_inches, y_inches, theta,
        front_sensor_offset_forward_.to_inches(),
        front_sensor_offset_lateral_.to_inches(),
        front_sensor_bearing_
    );
    
    double back_expected = field::FieldMap::compute_expected_distance(
        x_inches, y_inches, theta,
        back_sensor_offset_forward_.to_inches(),
        back_sensor_offset_lateral_.to_inches(),
        back_sensor_bearing_
    );
    
    auto front_wall = field::FieldMap::get_nearest_wall(
        x_inches, y_inches, theta, front_sensor_bearing_);
    auto back_wall = field::FieldMap::get_nearest_wall(
        x_inches, y_inches, theta, back_sensor_bearing_);

    // ============================================================
    // EKF CORRECTION STEP - Distance Sensor Updates
    // ============================================================
    Eigen::Vector2d innovation = Eigen::Vector2d::Zero();  // Initialize for logging
    
    // Only perform correction if at least one sensor is valid
    if (use_front_sensor || use_back_sensor) {
        
        // Define measurement function: h(state) -> expected measurements
        auto measurement_function = [&](const Eigen::Vector3d& state) -> Eigen::Vector2d {
            double x = state(0);      // meters
            double y = state(1);      // meters  
            double theta = state(2);  // radians
            
            // Convert to inches for FieldMap (it uses inches internally)
            double x_inches = x * 39.3701;
            double y_inches = y * 39.3701;
            
            Eigen::Vector2d expected_measurements;
            
            // Front sensor expected distance
            double front_expected = field::FieldMap::compute_expected_distance(
                x_inches, 
                y_inches, 
                theta,
                front_sensor_offset_forward_.to_inches(),
                front_sensor_offset_lateral_.to_inches(),
                front_sensor_bearing_
            );
            
            // Back sensor expected distance  
            double back_expected = field::FieldMap::compute_expected_distance(
                x_inches,
                y_inches, 
                theta,
                back_sensor_offset_forward_.to_inches(),
                back_sensor_offset_lateral_.to_inches(),
                back_sensor_bearing_
            );
            
            // Convert back to meters for EKF
            expected_measurements(0) = front_expected / 39.3701;  // inches to meters
            expected_measurements(1) = back_expected / 39.3701;
            
            return expected_measurements;
        };
        
        // Define measurement Jacobian: H = ∂h/∂x (numerical approximation)
        auto measurement_jacobian = [&](const Eigen::Vector3d& state) -> Eigen::Matrix<double, 2, 3> {
            const double epsilon = 1e-6;  // Small perturbation for numerical derivative
            
            Eigen::Matrix<double, 2, 3> H;
            Eigen::Vector2d h0 = measurement_function(state);
            
            // Compute ∂h/∂x
            Eigen::Vector3d state_dx = state;
            state_dx(0) += epsilon;
            Eigen::Vector2d h_dx = measurement_function(state_dx);
            H.col(0) = (h_dx - h0) / epsilon;
            
            // Compute ∂h/∂y
            Eigen::Vector3d state_dy = state;
            state_dy(1) += epsilon;
            Eigen::Vector2d h_dy = measurement_function(state_dy);
            H.col(1) = (h_dy - h0) / epsilon;
            
            // Compute ∂h/∂θ
            Eigen::Vector3d state_dtheta = state;
            state_dtheta(2) += epsilon;
            Eigen::Vector2d h_dtheta = measurement_function(state_dtheta);
            H.col(2) = (h_dtheta - h0) / epsilon;
            
            return H;
        };
        
        // Build measurement vector from actual sensor readings
        Eigen::Vector2d measurement;
        measurement(0) = front_measurement.to_meters();  // Convert to meters
        measurement(1) = back_measurement.to_meters();
        
        // ============================================================
        // COMPUTE INNOVATION FOR LOGGING (before update changes state)
        // ============================================================
        Eigen::Vector2d expected_measurements = measurement_function(state_before_correction);
        innovation = measurement - expected_measurements;
        
        // Perform EKF update/correction step
        ekf_.update(measurement, measurement_function, measurement_jacobian);
    }

    // ============================================================
    // LOG DISTANCE SENSOR DATA
    // ============================================================
    {
        auto &telem = abclib::telemetry::g_telemetry.get_write_buffer();
        
        // Distance sensor measurements
        telem.front_distance_measured = front_measurement;
        telem.front_distance_expected = units::Length::from_inches(front_expected);
        telem.front_distance_valid = use_front_sensor;
        telem.front_wall = front_wall;
        
        telem.back_distance_measured = back_measurement;
        telem.back_distance_expected = units::Length::from_inches(back_expected);
        telem.back_distance_valid = use_back_sensor;
        telem.back_wall = back_wall;
        
        // Innovation (difference between measured and expected)
        telem.front_innovation = units::Length::from_meters(innovation(0));
        telem.back_innovation = units::Length::from_meters(innovation(1));
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
    current_pose_.set_x(state(0) * 39.3701); // Convert meters to inches
    current_pose_.set_y(state(1) * 39.3701);
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
            telem.ekf_x_std = ekf_.get_state_uncertainty(0);  // in meters
            telem.ekf_y_std = ekf_.get_state_uncertainty(1);  // in meters
            telem.ekf_theta_std = ekf_.get_state_uncertainty(2);  // in radians
            
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
    update_count_ = 0;  // ADD THIS
    prev_front_measurement_ = units::Length::from_meters(0);  // ADD THIS
    prev_back_measurement_ = units::Length::from_meters(0);   // ADD THIS

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
    new_state(0) = pose.x_inches() / 39.3701;  // Convert inches to meters
    new_state(1) = pose.y_inches() / 39.3701;  // Convert inches to meters
    new_state(2) = pose.theta_rad();            // Already in radians
    
    ekf_.set_state(new_state);
    
    // Reset the first_update flag since we're setting a new pose
    first_update_ = true;
}
}