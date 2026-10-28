#include "ModelicaWrapper.h"
#include <fmi4cpp/fmi2/fmi2TypesPlatform.h>
#include <stdexcept>

ModelicaWrapper::ModelicaWrapper(const std::string& fmuPath) {
    _m_fmu = std::make_unique<fmi4cpp::fmi2::fmu>(fmuPath);
    
    _m_slave = _m_fmu->as_cs_fmu()->new_instance();
}

void ModelicaWrapper::init(double startTime) {
    _m_slave->setup_experiment(startTime);
    _m_slave->enter_initialization_mode();
    _m_slave->exit_initialization_mode();
}

void ModelicaWrapper::stepFMU(double stepSize) {
    if (!_m_slave->step(stepSize)) {
        throw std::runtime_error("FMU step failed");
    }
}

double ModelicaWrapper::getReal(const std::string& varName) {
    const auto& var = _m_fmu->get_model_description()->get_variable_by_name(varName);
    const auto vr = var.value_reference;
    
    if (var.is_boolean()) {
        fmi2Boolean boolValue = 0;
        if (!_m_slave->read_boolean(vr, boolValue)) {
            throw std::runtime_error("Failed to read boolean variable: " + varName);
        }
        return boolValue ? 1.0 : 0.0;
    } else {
        double value = 0.0;
        if (!_m_slave->read_real(vr, value)) {
            throw std::runtime_error("Failed to read real variable: " + varName);
        }
        return value;
    }
}

// Writes Real or Boolean (coerced from non-zero) variables by name.
void ModelicaWrapper::setReal(const std::string& varName, double value) {
    const auto& var = _m_fmu->get_model_description()->get_variable_by_name(varName);
    const auto vr = var.value_reference;

    if (var.is_boolean()) {
        if (!_m_slave->write_boolean(vr, value != 0.0)) {
            throw std::runtime_error("Failed to write boolean variable: " + varName);
        }
    } else {
        if (!_m_slave->write_real(vr, value)) {
            throw std::runtime_error("Failed to write real variable: " + varName);
        }
    }
}