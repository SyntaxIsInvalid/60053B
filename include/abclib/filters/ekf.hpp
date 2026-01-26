#pragma once

#include <Eigen/Dense>
#include <functional>

namespace abclib::filters
{
    /**
     * @brief General-purpose Extended Kalman Filter implementation
     * @tparam StateDim Dimension of the state vector (compile-time fixed)
     * 
     * Measurements are runtime-variable to support dynamic sensor configurations.
     */
    template <int StateDim>
    class ExtendedKalmanFilter
    {
    public:
        // Type aliases for cleaner code
        using StateVector = Eigen::Matrix<double, StateDim, 1>;
        using StateMatrix = Eigen::Matrix<double, StateDim, StateDim>;
        
        // Dynamic measurement types (runtime-variable size)
        using MeasurementVector = Eigen::VectorXd;
        using MeasurementMatrix = Eigen::MatrixXd;
        using KalmanGain = Eigen::MatrixXd;  // StateDim × MeasurementDim (runtime)
        using MeasurementJacobian = Eigen::MatrixXd;  // MeasurementDim × StateDim (runtime)
        
        // Function types for models
        using PredictionFunction = std::function<StateVector(const StateVector&, double dt)>;
        using PredictionJacobianFunction = std::function<StateMatrix(const StateVector&, double dt)>;
        using MeasurementFunction = std::function<MeasurementVector(const StateVector&)>;
        using MeasurementJacobianFunction = std::function<MeasurementJacobian(const StateVector&)>;

    private:
        StateVector state_;              // Current state estimate (x)
        StateMatrix covariance_;         // State covariance (P)
        StateMatrix process_noise_;      // Process noise covariance (Q)

    public:
        /**
         * @brief Default constructor - initializes with identity matrices
         */
        ExtendedKalmanFilter()
            : state_(StateVector::Zero()),
              covariance_(StateMatrix::Identity()),
              process_noise_(StateMatrix::Identity() * 0.01)
        {
        }

        /**
         * @brief Constructor with initial state and covariances
         */
        ExtendedKalmanFilter(
            const StateVector& initial_state,
            const StateMatrix& initial_covariance,
            const StateMatrix& process_noise)
            : state_(initial_state),
              covariance_(initial_covariance),
              process_noise_(process_noise)
        {
        }

        // === Getters and Setters ===
        
        void set_state(const StateVector& state) { state_ = state; }
        StateVector get_state() const { return state_; }
        
        void set_covariance(const StateMatrix& covariance) { covariance_ = covariance; }
        StateMatrix get_covariance() const { return covariance_; }
        
        void set_process_noise(const StateMatrix& Q) { process_noise_ = Q; }
        StateMatrix get_process_noise() const { return process_noise_; }

        /**
         * @brief Prediction step of the EKF
         * @param f State transition function: x_k = f(x_{k-1}, dt)
         * @param F Jacobian of f: ∂f/∂x
         * @param dt Time step
         */
        void predict(
            PredictionFunction f,
            PredictionJacobianFunction F,
            double dt)
        {
            // Predict state: x_k|k-1 = f(x_k-1|k-1, dt)
            state_ = f(state_, dt);
            
            // Compute Jacobian at predicted state
            StateMatrix F_jacobian = F(state_, dt);
            
            // Predict covariance: P_k|k-1 = F*P_k-1|k-1*F^T + Q
            covariance_ = F_jacobian * covariance_ * F_jacobian.transpose() + process_noise_;
        }

        /**
         * @brief Update/Correction step of the EKF
         * @param measurement Actual measurement z_k (runtime-sized)
         * @param h Measurement function: z = h(x)
         * @param H Jacobian of h: ∂h/∂x (MeasurementDim × StateDim, runtime-sized)
         * @param R Measurement noise covariance (MeasurementDim × MeasurementDim)
         */
        void update(
            const MeasurementVector& measurement,
            MeasurementFunction h,
            MeasurementJacobianFunction H,
            const MeasurementMatrix& R)
        {
            // Get measurement dimension from actual measurement
            int measurement_dim = measurement.size();
            
            // Compute measurement Jacobian at current state
            MeasurementJacobian H_jacobian = H(state_);
            
            // Validate dimensions
            if (H_jacobian.rows() != measurement_dim || H_jacobian.cols() != StateDim) {
                // Dimension mismatch - skip update
                return;
            }
            
            if (R.rows() != measurement_dim || R.cols() != measurement_dim) {
                // R dimension mismatch - skip update
                return;
            }
            
            // Innovation (measurement residual): y = z - h(x_k|k-1)
            MeasurementVector innovation = measurement - h(state_);
            
            // Innovation covariance: S = H*P_k|k-1*H^T + R
            MeasurementMatrix S = H_jacobian * covariance_ * H_jacobian.transpose() + R;
            
            // Kalman gain: K = P_k|k-1*H^T*S^-1
            KalmanGain K = covariance_ * H_jacobian.transpose() * S.inverse();
            
            // Update state: x_k|k = x_k|k-1 + K*y
            state_ = state_ + K * innovation;
            
            // Update covariance: P_k|k = (I - K*H)*P_k|k-1
            // Using Joseph form for numerical stability
            StateMatrix I = StateMatrix::Identity();
            StateMatrix temp = I - K * H_jacobian;
            covariance_ = temp * covariance_ * temp.transpose() + 
                         K * R * K.transpose();
        }

        /**
         * @brief Predict-only step (no measurement correction)
         * Useful for testing and when measurements are unavailable
         */
        void predict_only(
            PredictionFunction f,
            PredictionJacobianFunction F,
            double dt)
        {
            predict(f, F, dt);
        }

        /**
         * @brief Reset filter to zero state with identity covariance
         */
        void reset()
        {
            state_ = StateVector::Zero();
            covariance_ = StateMatrix::Identity();
        }

        /**
         * @brief Reset filter with specific initial conditions
         */
        void reset(const StateVector& initial_state, const StateMatrix& initial_covariance)
        {
            state_ = initial_state;
            covariance_ = initial_covariance;
        }

        /**
         * @brief Get individual state element by index
         */
        double get_state_element(int index) const
        {
            return state_(index);
        }

        /**
         * @brief Set individual state element by index
         */
        void set_state_element(int index, double value)
        {
            state_(index) = value;
        }

        /**
         * @brief Get uncertainty (standard deviation) for a state element
         */
        double get_state_uncertainty(int index) const
        {
            return std::sqrt(covariance_(index, index));
        }
    };
}