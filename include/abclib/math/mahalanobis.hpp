// math/statistics.hpp
#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace abclib::math
{
    /**
     * @brief Compute Mahalanobis distance for scalar measurement
     * 
     * Measures how many standard deviations a measurement is from expected value.
     * 
     * Formula: d_M = |x - μ| / σ
     * 
     * @param measurement Observed value
     * @param expected Expected value (mean)
     * @param std_dev Standard deviation (uncertainty)
     * @return Mahalanobis distance (dimensionless), or -1.0 if invalid
     * 
     * Interpretation:
     * - d_M < 1.0: Within 1-sigma (68% confidence)
     * - d_M < 2.0: Within 2-sigma (95% confidence)
     * - d_M < 3.0: Within 3-sigma (99.7% confidence)
     * - d_M > 3.0: Likely outlier
     */
    inline double mahalanobis_distance_scalar(
        double measurement,
        double expected,
        double std_dev)
    {
        if (std_dev <= 0.0) {
            return -1.0;  // Invalid uncertainty
        }
        
        double innovation = measurement - expected;
        return std::abs(innovation) / std_dev;
    }
    
    /**
     * @brief Compute Mahalanobis distance for vector measurement
     * 
     * Generalized distance accounting for correlation between dimensions.
     * 
     * Formula: d_M = sqrt((x - μ)ᵀ Σ⁻¹ (x - μ))
     * 
     * @param measurement Observed vector
     * @param expected Expected vector (mean)
     * @param covariance Covariance matrix (uncertainty)
     * @return Mahalanobis distance (dimensionless), or -1.0 if invalid
     * 
     * For uncorrelated measurements with equal variance σ²:
     * d_M = ||x - μ|| / σ  (Euclidean distance normalized by std dev)
     * 
     * Example: 2D pose measurement [x, y]
     * - measurement = [10.0, 5.0]
     * - expected = [10.5, 4.8]
     * - covariance = [[0.01, 0], [0, 0.01]]  (1cm std dev, uncorrelated)
     * - d_M ≈ 2.06 (within 95% confidence)
     */
    inline double mahalanobis_distance_vector(
        const Eigen::VectorXd& measurement,
        const Eigen::VectorXd& expected,
        const Eigen::MatrixXd& covariance)
    {
        if (measurement.size() != expected.size() ||
            covariance.rows() != covariance.cols() ||
            covariance.rows() != measurement.size()) {
            return -1.0;  // Dimension mismatch
        }
        
        // Compute innovation (residual)
        Eigen::VectorXd innovation = measurement - expected;
        
        // Use LLT decomposition (Cholesky) for numerical stability
        // This is faster and more stable than computing inverse explicitly
        Eigen::LLT<Eigen::MatrixXd> llt(covariance);
        
        if (llt.info() != Eigen::Success) {
            return -1.0;  // Covariance not positive definite
        }
        
        // Solve: cov_inv * innovation = llt.solve(innovation)
        // Compute quadratic form: innovation^T * cov_inv * innovation
        //                      = innovation^T * llt.solve(innovation)
        Eigen::VectorXd solved = llt.solve(innovation);
        double mahalanobis_squared = innovation.dot(solved);
        
        if (mahalanobis_squared < 0.0) {
            return -1.0;  // Numerical issue (shouldn't happen with valid covariance)
        }
        
        return std::sqrt(mahalanobis_squared);
    }
    
    /**
     * @brief Outlier test for scalar measurements (1 DOF)
     * 
     * Tests whether a scalar measurement is an inlier based on standard deviations.
     * 
     * @param mahalanobis_distance Computed Mahalanobis distance
     * @param sigma_threshold Rejection threshold in standard deviations (default: 3.0)
     * @return true if measurement is inlier, false if outlier
     * 
     * Common thresholds:
     * - 2.0: ~95% confidence (2-sigma rule)
     * - 3.0: ~99.7% confidence (3-sigma rule) [DEFAULT]
     * - 4.0: ~99.99% confidence (4-sigma rule)
     */
    inline bool is_inlier_scalar(double mahalanobis_distance, double sigma_threshold = 3.0)
    {
        if (mahalanobis_distance < 0.0) {
            return false;  // Invalid distance
        }
        
        return mahalanobis_distance <= sigma_threshold;
    }
    
    /**
     * @brief Outlier test for vector measurements (multi-DOF)
     * 
     * Tests whether a measurement is an inlier using chi-squared test.
     * 
     * @param mahalanobis_distance Computed Mahalanobis distance
     * @param chi_squared_threshold Chi-squared threshold (depends on degrees of freedom)
     * @return true if measurement is inlier, false if outlier
     * 
     * The test checks: d_M² ≤ χ²_threshold
     * 
     * Common chi-squared thresholds at 95% confidence:
     * - 1 DOF: 3.84  (use is_inlier_scalar instead with sigma=2.0)
     * - 2 DOF: 5.99
     * - 3 DOF: 7.81
     * - 4 DOF: 9.49
     * - 6 DOF: 12.59
     * 
     * Common chi-squared thresholds at 99% confidence:
     * - 2 DOF: 9.21
     * - 3 DOF: 11.34
     * - 4 DOF: 13.28
     * - 6 DOF: 16.81
     * 
     * See chi-squared distribution tables for other DOF values.
     */
    inline bool is_inlier_vector(double mahalanobis_distance, double chi_squared_threshold)
    {
        if (mahalanobis_distance < 0.0) {
            return false;  // Invalid distance
        }
        
        double mahalanobis_squared = mahalanobis_distance * mahalanobis_distance;
        return mahalanobis_squared <= chi_squared_threshold;
    }

} // namespace abclib::math