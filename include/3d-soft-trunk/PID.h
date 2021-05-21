#pragma once

#include "ControllerPCC.h"

class PID: public ControllerPCC 
{
public:
    PID(CurvatureCalculator::SensorType sensor_type);

private:
    void control_loop();

    // parameters for PID controller
    std::array<double, 3> Ku = {3000, 2500, 2000}; /** @brief ultimate gain for each segment, used in Ziegler-Nichols method */
    std::array<double, 3> Tu = {0.7, 0.7, 0.7}; /** @brief oscillation period for each segment, used in Ziegler-Nichols method */
    std::vector<MiniPID> miniPIDs;

    /** @brief ziegler nichols method of tuning */
    MiniPID ZieglerNichols(double Ku, double period, double control_period);

};