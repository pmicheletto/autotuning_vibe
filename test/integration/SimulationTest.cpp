#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <thread>

#include "simulation/ModelicaWrapper.h"
#include "simulation/UDPSender.h"
#include "statistics/WelfordAlgorithm.h"

// UDP sender integration for streaming simulation variables (time, rpm, step.y)
// to PlotJuggler.
TEST(SimulationTest, SendsUdpMessagesDuringSimulation) {
    ModelicaWrapper model("../simulation/modelica_models/SimplesPID.fmu");
    UDPSender udp("127.0.0.1", 9870);

    EXPECT_NO_THROW(model.init(0.0));

    double stepSize = 0.001;
    double stepHeight = 0.25;

    EXPECT_NO_THROW(model.setReal("step.height", stepHeight));

    for (int i = 0; i < 5000; ++i) {
        if (i == 5000 / 2) {
            EXPECT_NO_THROW(model.setReal("step.height", 2 * stepHeight));
        }
        EXPECT_NO_THROW(model.stepFMU(stepSize));
        double t = i * stepSize;
        double speed = model.getReal("speedSensor.w");
        double stepY = model.getReal("step.y");

        if (i % 100 == 0) {
            std::string json = "{\"time\": " + std::to_string(t) +
                               ", \"rpm\": " + std::to_string(speed) +
                               ", \"step.y\": " + std::to_string(stepY) + "}";
            EXPECT_NO_THROW(udp.send(json));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    usleep(1000);
    double speed = model.getReal("speedSensor.w");
    double stepY = model.getReal("step.y");
    EXPECT_TRUE(std::isfinite(speed));
    EXPECT_TRUE(std::isfinite(stepY));
}

TEST(SimulationTest, SimulationWithWelford) {
    ModelicaWrapper model("../simulation/modelica_models/TransferFunction.fmu");
    UDPSender udp("127.0.0.1", 9870);

    ASSERT_NO_THROW(model.init(0.0));

    double stepSize = 0.001;
    double stepHeight = 10;
    int simulation_steps = 10000; 

    EXPECT_NO_THROW(model.setReal("step.height", stepHeight));

    WelfordAlgorithm welford;

    for (int i = 0; i < simulation_steps; ++i) {
        
        if (i == simulation_steps / 2) {
            EXPECT_NO_THROW(model.setReal("step.height", 2 * stepHeight));
        }

        EXPECT_NO_THROW(model.stepFMU(stepSize));
        
        double t = i * stepSize;
        double speed = model.getReal("transferFunction.y");
        double stepY = model.getReal("step.y");

        welford.AddSample(speed);
        double deviation = welford.GetDeviation();

        if (i % 100 == 0) {
            std::string json = "{\"time\": " + std::to_string(t) +
                               ", \"rpm\": " + std::to_string(speed) +
                               ", \"deviation\": " + std::to_string(deviation) +
                               ", \"step.y\": " + std::to_string(stepY) + "}";
            udp.send(json);
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    double speed = model.getReal("transferFunction.y");
    EXPECT_TRUE(std::isfinite(speed));
}