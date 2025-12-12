#pragma once

#include <string>

namespace abclib::path {

enum class ContinuityLevel {
    G0,  // Position only
    G1,  // + Tangent
    G2,  // + Curvature
    G3   // + Curvature derivative
};

struct ContinuityResult {
    ContinuityLevel achieved = ContinuityLevel::G3;
    bool meets_minimum = true;
    
    double position_error = 0.0;
    double heading_error = 0.0;
    double curvature_error = 0.0;
    double curvature_deriv_error = 0.0;
};

inline std::string to_string(ContinuityLevel level) {
    switch (level) {
        case ContinuityLevel::G0: return "G0";
        case ContinuityLevel::G1: return "G1";
        case ContinuityLevel::G2: return "G2";
        case ContinuityLevel::G3: return "G3";
    }
    return "Unknown";
}

} // namespace abclib::path