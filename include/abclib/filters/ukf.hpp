#pragma once

#include <Eigen/Dense>
#include <functional>
#include <cmath>

namespace abclib::filters
{
    /**
     * @brief Generic Unscented Kalman Filter
     * 
     * @tparam N_STATE Dimension of state vector
     * @tparam N_MEAS Dimension of measurement vector
     */
    template<int N_STATE, int N_MEAS>
    class UnscentedKalmanFilter
    {
    public:
        using StateVector = Eigen::Matrix<double, N_STATE, 1>;
        using StateMatrix = Eigen::Matrix<double, N_STATE, N_STATE>;
        using MeasVector = Eigen::Matrix<double, N_MEAS, 1>;
        using MeasMatrix = Eigen::Matrix<double, N_MEAS, N_MEAS>;
        using CrossCovMatrix = Eigen::Matrix<double, N_STATE, N_MEAS>;
        
        // Function types for process and measurement models
        using ProcessModel = std::function<StateVector(const StateVector&, double)>;
        using MeasurementModel = std::function<MeasVector(const StateVector&)>;
        
    private:
        static constexpr int N_SIGMA = 2 * N_STATE + 1;
        
        // State estimate
        StateVector x_;           // State mean
        StateMatrix P_;           // State covariance
        
        // Unscented transform parameters
        double alpha_;            // Spread of sigma points (0.001 - 1)
        double beta_;             // Distribution shape (2 for Gaussian)
        double kappa_;            // Secondary scaling (0 or 3-N)
        double lambda_;           // Computed: alpha^2 * (N + kappa) - N
        
        // Sigma point weights
        double wm0_;              // Weight for mean (first sigma point)
        double wc0_;              // Weight for covariance (first sigma point)
        double wi_;               // Weight for other sigma points (same for mean and cov)
        
    public:
        /**
         * @brief Construct UKF with default unscented transform parameters
         */
        UnscentedKalmanFilter()
            : alpha_(0.001),
              beta_(2.0),
              kappa_(0.0)
        {
            x_.setZero();
            P_.setIdentity();
            compute_weights();
        }
        
        /**
         * @brief Construct UKF with custom unscented transform parameters
         */
        UnscentedKalmanFilter(double alpha, double beta, double kappa)
            : alpha_(alpha),
              beta_(beta),
              kappa_(kappa)
        {
            x_.setZero();
            P_.setIdentity();
            compute_weights();
        }
        
        /**
         * @brief Initialize state estimate
         */
        void set_state(const StateVector& x, const StateMatrix& P)
        {
            x_ = x;
            P_ = P;
        }
        
        /**
         * @brief Get current state estimate
         */
        StateVector get_state() const { return x_; }
        
        /**
         * @brief Get current covariance
         */
        StateMatrix get_covariance() const { return P_; }
        
        /**
         * @brief Prediction step
         * 
         * @param process_model Function: f(state, dt) -> predicted_state
         * @param Q Process noise covariance
         * @param dt Time step
         */
        void predict(const ProcessModel& process_model, const StateMatrix& Q, double dt)
        {
            // 1. Generate sigma points
            auto sigma_points = generate_sigma_points(x_, P_);
            
            // 2. Propagate sigma points through process model
            Eigen::Matrix<double, N_STATE, N_SIGMA> predicted_sigma;
            for (int i = 0; i < N_SIGMA; ++i)
            {
                predicted_sigma.col(i) = process_model(sigma_points.col(i), dt);
            }
            
            // 3. Compute predicted mean
            x_ = wm0_ * predicted_sigma.col(0);
            for (int i = 1; i < N_SIGMA; ++i)
            {
                x_ += wi_ * predicted_sigma.col(i);
            }
            
            // 4. Compute predicted covariance
            StateVector diff = predicted_sigma.col(0) - x_;
            P_ = wc0_ * (diff * diff.transpose());
            
            for (int i = 1; i < N_SIGMA; ++i)
            {
                diff = predicted_sigma.col(i) - x_;
                P_ += wi_ * (diff * diff.transpose());
            }
            
            P_ += Q;  // Add process noise
        }
        
        /**
         * @brief Update step (measurement correction)
         * 
         * @param measurement_model Function: h(state) -> expected_measurement
         * @param z Actual measurement
         * @param R Measurement noise covariance
         */
        void update(const MeasurementModel& measurement_model, 
                   const MeasVector& z, 
                   const MeasMatrix& R)
        {
            // 1. Generate sigma points from predicted state
            auto sigma_points = generate_sigma_points(x_, P_);
            
            // 2. Propagate sigma points through measurement model
            Eigen::Matrix<double, N_MEAS, N_SIGMA> predicted_meas;
            for (int i = 0; i < N_SIGMA; ++i)
            {
                predicted_meas.col(i) = measurement_model(sigma_points.col(i));
            }
            
            // 3. Compute predicted measurement mean
            MeasVector z_pred = wm0_ * predicted_meas.col(0);
            for (int i = 1; i < N_SIGMA; ++i)
            {
                z_pred += wi_ * predicted_meas.col(i);
            }
            
            // 4. Compute innovation covariance S
            MeasVector meas_diff = predicted_meas.col(0) - z_pred;
            MeasMatrix S = wc0_ * (meas_diff * meas_diff.transpose());
            
            for (int i = 1; i < N_SIGMA; ++i)
            {
                meas_diff = predicted_meas.col(i) - z_pred;
                S += wi_ * (meas_diff * meas_diff.transpose());
            }
            
            S += R;  // Add measurement noise
            
            // 5. Compute cross-covariance between state and measurement
            StateVector state_diff = sigma_points.col(0) - x_;
            meas_diff = predicted_meas.col(0) - z_pred;
            CrossCovMatrix Pxz = wc0_ * (state_diff * meas_diff.transpose());
            
            for (int i = 1; i < N_SIGMA; ++i)
            {
                state_diff = sigma_points.col(i) - x_;
                meas_diff = predicted_meas.col(i) - z_pred;
                Pxz += wi_ * (state_diff * meas_diff.transpose());
            }
            
            // 6. Compute Kalman gain
            Eigen::Matrix<double, N_STATE, N_MEAS> K = Pxz * S.inverse();
            
            // 7. Update state
            MeasVector innovation = z - z_pred;
            x_ = x_ + K * innovation;
            
            // 8. Update covariance
            P_ = P_ - K * S * K.transpose();
        }
        
    private:
        /**
         * @brief Compute sigma point weights
         */
        void compute_weights()
        {
            lambda_ = alpha_ * alpha_ * (N_STATE + kappa_) - N_STATE;
            
            wm0_ = lambda_ / (N_STATE + lambda_);
            wc0_ = wm0_ + (1.0 - alpha_ * alpha_ + beta_);
            wi_ = 1.0 / (2.0 * (N_STATE + lambda_));
        }
        
        /**
         * @brief Generate sigma points using Cholesky decomposition
         * 
         * @param x State mean
         * @param P State covariance
         * @return Matrix where each column is a sigma point
         */
        Eigen::Matrix<double, N_STATE, N_SIGMA> generate_sigma_points(
            const StateVector& x, 
            const StateMatrix& P) const
        {
            Eigen::Matrix<double, N_STATE, N_SIGMA> sigma_points;
            
            // Compute matrix square root: S such that S * S^T = (N + lambda) * P
            StateMatrix A = (N_STATE + lambda_) * P;
            Eigen::LLT<StateMatrix> llt(A);
            StateMatrix L = llt.matrixL();
            
            // First sigma point is the mean
            sigma_points.col(0) = x;
            
            // Generate remaining sigma points
            for (int i = 0; i < N_STATE; ++i)
            {
                sigma_points.col(i + 1) = x + L.col(i);
                sigma_points.col(i + 1 + N_STATE) = x - L.col(i);
            }
            
            return sigma_points;
        }
    };

} // namespace abclib::filters