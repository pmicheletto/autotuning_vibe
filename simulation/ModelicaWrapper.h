#ifndef SIMULATION_MODELICAWRAPPER_H_
#define SIMULATION_MODELICAWRAPPER_H_

#include <string>
#include <memory>
#include <fmi4cpp/fmi4cpp.hpp>

class ModelicaWrapper
{
public:
    explicit ModelicaWrapper(const std::string& fmuPath);

    void init(double startTime);
    void stepFMU(double stepSize);

    double getReal(const std::string& varName);
    void setReal(const std::string& varName, double value);

private:
    std::unique_ptr<fmi4cpp::fmi2::fmu> _m_fmu;
    std::unique_ptr<fmi4cpp::fmi2::cs_slave> _m_slave;
};

#endif  // SIMULATION_MODELICAWRAPPER_H_