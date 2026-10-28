#include "tuning/parameters/parameter_validator.h"
#include <cmath>

bool ParameterValidator::validate(const PIDParameters& params) {
    if (params.Kp < 0.0 || std::isnan(params.Kp) || std::isinf(params.Kp)) {
        return false;
    }
    
    if (params.Ti <= 0.0 || std::isnan(params.Ti) || std::isinf(params.Ti)) {
        return false;
    }
    
    if (params.Td < 0.0 || std::isnan(params.Td) || std::isinf(params.Td)) {
        return false;
    }
    
    return true;
}
