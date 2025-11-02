#if 0
#include <abclib/estimation/odometry.hpp>
#include <abclib/hardware/tracking_wheel.hpp>
#include <cmath>
#include <mutex>
#include <abclib/math/angles.hpp>
#include <abclib/telemetry/telemetry.hpp>

namespace abclib::estimation
{
    Odometry::Odometry(hardware::ITrackingWheel *vertical_wheel, hardware::ITrackingWheel *horizontal_wheel, pros::IMU *imu_sensor)
        : vertical(vertical_wheel), horizontal(horizontal_wheel), imu(imu_sensor)
    {
        // Initialize previous values
        if (vertical)
            prev_vertical = vertical->get_distance().inches; // Extract .inches
        if (horizontal)
            prev_horizontal = horizontal->get_distance().inches; // Extract .inches
        if (imu)
        {
            double imu_reading = imu->get_rotation();
            // Convert degrees to radians using the units system
            prev_imu = std::isnan(imu_reading) ? 0.0 : -units::Degrees(imu_reading).to_radians().value;
        }
    }

    void Odometry::init()
    {
        std::lock_guard<pros::Mutex> lock(task_mutex_);
        if (!tracking_task.has_value())
        {
            tracking_task = pros::Task([this]
                                       {
                while (pros::Task::notify_take(true, 0) == 0) {
                    this->update();
                    pros::delay(10);
                } });
        }
    }

    void Odometry::stop()
    {
        std::lock_guard<pros::Mutex> lock(task_mutex_);
        if (tracking_task.has_value())
        {
            tracking_task->notify();
            tracking_task = std::nullopt;
        }
    }

    Odometry::~Odometry()
    {
        std::lock_guard<pros::Mutex> lock(task_mutex_); // ← Add this line
        if (tracking_task.has_value())
        {
            tracking_task->notify();
        }
    }

    void Odometry::reset()
    {
        // Reset the pose to origin
        current_pose = Pose(); // Defaults to (0, 0, 0)

        // Store current values as previous values
        if (vertical)
            prev_vertical = vertical->get_distance().inches;
        if (horizontal)
            prev_horizontal = horizontal->get_distance().inches;
        if (imu)
            prev_imu = -units::Degrees(imu->get_rotation()).to_radians().value;
    }

    Pose Odometry::get_pose() const
    {
        std::lock_guard<pros::Mutex> lock(pose_mutex);
        return current_pose;
    }

    void Odometry::update()
    {
        // Get current sensor values
        double verticalRaw = vertical ? vertical->get_distance().inches : 0.0;       // Extract .inches
        double horizontalRaw = horizontal ? horizontal->get_distance().inches : 0.0; // Extract .inches
        // double imuRaw = imu ? (imu->get_rotation() * M_PI / 180.0) : 0.0; // Convert to radians
        double imuRaw = imu ? (-imu->get_rotation() * M_PI / 180.0) : 0.0; // Convert to radians
        // Calculate sensor value changes
        double deltaVertical = verticalRaw - prev_vertical;
        double deltaHorizontal = horizontalRaw - prev_horizontal;
        double deltaImu = imuRaw - prev_imu;

        // Update previous values
        prev_vertical = verticalRaw;
        prev_horizontal = horizontalRaw;
        prev_imu = imuRaw;

        // Calculate heading using IMU
        double heading = current_pose.theta() + deltaImu;
        double deltaHeading = deltaImu;
        double avgHeading = current_pose.theta() + deltaHeading / 2;

        // Get offsets
        double verticalOffset = vertical ? vertical->get_offset().inches : 0.0;       // Extract .inches
        double horizontalOffset = horizontal ? horizontal->get_offset().inches : 0.0; // Extract .inches

        // Calculate local x and y
        double localX = 0.0;
        double localY = 0.0;
        if (std::abs(deltaHeading) < 1e-6)
        { // prevent divide by 0
            localX = deltaHorizontal;
            localY = deltaVertical;
        }
        else
        {
            localX = 2 * std::sin(deltaHeading / 2) * (deltaHorizontal / deltaHeading + horizontalOffset);
            localY = 2 * std::sin(deltaHeading / 2) * (deltaVertical / deltaHeading + verticalOffset);
        }

        static double prev_x = 0.0;
        static double prev_y = 0.0;
        static double prev_theta = 0.0;
        static bool first_update = true;

        // Calculate global x and y
        std::lock_guard<pros::Mutex> lock(pose_mutex);
        // current_pose.x -= localX * std::cos(avgHeading) - localY * std::sin(avgHeading);
        // current_pose.y += localX * std::sin(avgHeading) + localY * std::cos(avgHeading);
        current_pose.set_x(current_pose.x() + localY * std::cos(avgHeading) + localX * std::sin(avgHeading));
        current_pose.set_y(current_pose.y() + localY * std::sin(avgHeading) - localX * std::cos(avgHeading));
        current_pose.set_theta(heading);

        if (!first_update)
        {
            const double dt = 0.01; // 10ms update rate

            // Calculate global frame velocities
            double dx = current_pose.x() - prev_x;
            double dy = current_pose.y() - prev_y;
            double dtheta = current_pose.theta() - prev_theta;

            // Normalize angle difference
            dtheta = math::normalize_angle(dtheta);

            // Transform global velocity to body frame
            // v = dx*cos(theta) + dy*sin(theta)
            double v_global_x = dx / dt;
            double v_global_y = dy / dt;
            double v_raw = (dx * std::cos(current_pose.theta()) +
                            dy * std::sin(current_pose.theta())) /
                           dt;
            double omega_raw = dtheta / dt;

            // Filter - works fine from the start
            const double alpha = 0.3;
            current_pose.v = units::BodyLinearVelocity(alpha * v_raw + (1.0 - alpha) * current_pose.v.inches_per_sec);
            current_pose.omega = units::BodyAngularVelocity(alpha * omega_raw + (1.0 - alpha) * current_pose.omega.rad_per_sec);
            /*
            double omega_imu = imu->get_gyro_rate().z * (-M_PI / 180.0);

            // Apply light filtering (gyros can be noisy)
            current_pose.omega = units::BodyAngularVelocity(
                alpha * omega_imu + (1.0 - alpha) * current_pose.omega.rad_per_sec);
            */
            {
                std::lock_guard<pros::Mutex> telem_lock(abclib::telemetry_mutex);
                abclib::telemetry.pose = current_pose.pose;                               // Assign the BodyPose directly
                abclib::telemetry.pose_v = current_pose.v;                                // Already BodyLinearVelocity
                abclib::telemetry.pose_omega = current_pose.omega;                        // Already BodyAngularVelocity
                abclib::telemetry.pose_v_raw = units::BodyLinearVelocity(v_raw);          // Wrap raw value
                abclib::telemetry.pose_omega_raw = units::BodyAngularVelocity(omega_raw); // Wrap raw value
            }
        }
        else
        {
            // First update - no velocity yet
            current_pose.v = units::BodyLinearVelocity(0.0);
            current_pose.omega = units::BodyAngularVelocity(0.0);
            first_update = false;
        }

        // Store current pose for next iteration
        prev_x = current_pose.x();
        prev_y = current_pose.y();
        prev_theta = current_pose.theta();
    }

    void Odometry::calibrate_sensors()
    {
        // Reset the tracking wheels themselves
        if (vertical)
            vertical->reset();
        if (horizontal)
            horizontal->reset();

        // Update previous values to current (now zeroed) readings
        if (vertical)
            prev_vertical = vertical->get_distance().inches; // Extract .inches
        if (horizontal)
            prev_horizontal = horizontal->get_distance().inches; // Extract .inches
        if (imu)
            prev_imu = -units::Degrees(imu->get_rotation()).to_radians().value; // Use units conversion
    }

    void Odometry::set_pose(const Pose &pose)
    {
        std::lock_guard<pros::Mutex> lock(pose_mutex);
        current_pose = pose;

        // Reset velocities
        current_pose.v = units::BodyLinearVelocity(0.0);
        current_pose.omega = units::BodyAngularVelocity(0.0);
    }

}
/*
void Odometry::update() {
    // Sensors
    const double vRaw = vertical ? vertical->get_distance() : 0.0;
    const double hRaw = horizontal ? horizontal->get_distance() : 0.0;
    const double imuRaw = imu ? (imu->get_rotation() * M_PI / 180.0) : 0.0;

    // Deltas
    const double dV = vRaw - prev_vertical;
    const double dH = hRaw - prev_horizontal;
    const double dTh = imuRaw - prev_imu;

    prev_vertical = vRaw;
    prev_horizontal = hRaw;
    prev_imu = imuRaw;

    // Snapshot heading θ_k
    double theta_k;
    {
        std::lock_guard<pros::Mutex> g(pose_mutex);
        theta_k = current_pose.theta;
    }

    // Sensor offsets (r_x for vertical wheel, r_y for horizontal wheel)
    const double r_x = vertical ? vertical->get_offset() : 0.0;     // offset along x_b of the V wheel
    const double r_y = horizontal ? horizontal->get_offset() : 0.0; // offset along y_b of the H wheel

    // ρ = body-frame integrated twist at the body origin from wheel deltas + offset terms
    // Δx_b, Δy_b before curvature correction
    const double rho_x = dH + r_y * dTh;
    const double rho_y = dV - r_x * dTh;

    // V(Δθ) (a.k.a. left Jacobian J)
    double J11, J12, J21, J22;
    if (std::abs(dTh) < 1e-6) {
        J11 = 1.0;          J12 = -0.5 * dTh;
        J21 = 0.5 * dTh;    J22 = 1.0;
    } else {
        const double s = std::sin(dTh), c = std::cos(dTh);
        const double a = s / dTh;
        const double b = (1.0 - c) / dTh;
        J11 = a;  J12 = -b;
        J21 = b;  J22 =  a;
    }

    // Body-frame displacement of the body origin over the step
    const double dxb = J11 * rho_x + J12 * rho_y;
    const double dyb = J21 * rho_x + J22 * rho_y;

    // World-frame increment: R(θ_k) * [dxb; dyb]
    const double cth = std::cos(theta_k), sth = std::sin(theta_k);
    const double dx =  cth * dxb - sth * dyb;
    const double dy =  sth * dxb + cth * dyb;

    // Commit
    {
        std::lock_guard<pros::Mutex> g(pose_mutex);
        current_pose.x += dx;
        current_pose.y += dy;
        current_pose.theta = theta_k + dTh;
    }
}
*/
#endif