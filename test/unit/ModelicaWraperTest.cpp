#include <gtest/gtest.h>

#include <iostream>

#include <chrono>
#include <cmath>
#include <thread>

#include "simulation/ModelicaWrapper.h"
#include "simulation/UDPSender.h"

TEST(ModelicaWraperTest, StepHeightAppliedAndOutputsFinite) {
    ModelicaWrapper model("../simulation/modelica_models/SimplesPID.fmu");
 
    EXPECT_NO_THROW(model.init(0.0));
    double stepSize = 0.001;
    double stepHeight = 0.5;

    EXPECT_NO_THROW(model.setReal("step.height", stepHeight));

    for (int i = 0; i < 5000; ++i) {
        EXPECT_NO_THROW(model.stepFMU(stepSize));
    }
    double speed = model.getReal("speedSensor.w");
    double stepY = model.getReal("step.y");

    EXPECT_NEAR(speed, 0.5, 0.01); // the speed is equal to the step height
}

TEST(ModelicaWraperTest, IncreasesInput) {
    ModelicaWrapper model("../simulation/modelica_models/SimplesPID.fmu");
 
    EXPECT_NO_THROW(model.init(0.0));
    double stepSize = 0.001;
    double stepHeight = 0.5;

    EXPECT_NO_THROW(model.setReal("step.height", stepHeight));

    for (int i = 0; i < 5000; ++i) {
        if (i == 5000/2){
            EXPECT_NO_THROW(model.setReal("step.height", 2*stepHeight));
        }
        EXPECT_NO_THROW(model.stepFMU(stepSize));
    }

    double speed = model.getReal("speedSensor.w");
    double stepY = model.getReal("step.y");
    std::cout << "aqui" << speed;
    EXPECT_NEAR(speed, 1, 0.01); // the speed is equal to the step height
}
