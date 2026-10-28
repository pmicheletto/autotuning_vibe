#ifndef TUNING_PARAMETER_VALIDATOR_H_
#define TUNING_PARAMETER_VALIDATOR_H_

#include "../models.h"

class ParameterValidator {
public:
    static bool validate(const PIDParameters& params);
};

#endif
